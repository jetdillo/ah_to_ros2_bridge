// amazinghand_hardware/src/scs0009_driver.cpp
//
// SCS0009 serial bus driver implementation.
//
// All serial I/O is delegated to the SCServo_Linux SCS0009 class — termios
// configuration, half-duplex direction switching, packet framing, and
// checksum are handled there.  This file owns:
//   - The FingerBus list and its lifecycle (open / close)
//   - The poll loop (sequential FeedBack() per servo → ServoState)
//   - The write path (mix semantic commands → SyncWritePos for healthy pairs)
//   - Unit conversion (raw <-> radians) via the inline helpers in the header
//   - Health tracking (reachable flag per servo, warn callback to ROS layer)

#include "amazinghand_hardware/scs0009_driver.hpp"
#include "amazinghand_hardware/finger_types.hpp"

#include <algorithm>
#include <chrono>
#include <sstream>

namespace amazinghand {

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
SCS0009Driver::SCS0009Driver(const std::string& port,
                             int                baud_rate,
                             uint16_t           min_raw,
                             uint16_t           max_raw)
  : port_(port)
  , baud_rate_(baud_rate)
  , min_raw_(min_raw)
  , max_raw_(max_raw)
{}

SCS0009Driver::~SCS0009Driver()
{
  if (open_) close();
}

// ---------------------------------------------------------------------------
// open
//
// Hands the serial port to SCServo_Linux, then builds a FingerBus for each
// FingerConfig and does an initial Ping to establish reachability.
// A missing servo is a warning, not a fatal error — the hand is still usable
// with fewer fingers.
// ---------------------------------------------------------------------------
bool SCS0009Driver::open(const std::vector<FingerConfig>& fingers)
{
  // SCServo_Linux begin() opens the serial port and configures termios.
  // Returns true on success.
  if (!servo_.begin(baud_rate_, port_.c_str())) {
    warn("Failed to open serial port: " + port_);
    return false;
  }

  // Build FingerBus entries and seed servo IDs
  fingers_.clear();
  fingers_.reserve(fingers.size());

  for (const auto& cfg : fingers) {
    FingerBus fb;
    fb.config          = cfg;
    fb.leader.id       = cfg.leader_id;
    fb.follower.id     = cfg.follower_id;
    fb.leader.reachable   = false;
    fb.follower.reachable = false;

    // Initial ping to establish reachability
    if (servo_.Ping(cfg.leader_id) != -1) {
      fb.leader.reachable = true;
    } else {
      std::ostringstream oss;
      oss << "Finger '" << cfg.name << "': leader servo "
          << static_cast<int>(cfg.leader_id) << " did not respond to Ping";
      warn(oss.str());
    }

    if (servo_.Ping(cfg.follower_id) != -1) {
      fb.follower.reachable = true;
    } else {
      std::ostringstream oss;
      oss << "Finger '" << cfg.name << "': follower servo "
          << static_cast<int>(cfg.follower_id) << " did not respond to Ping";
      warn(oss.str());
    }

    fingers_.push_back(std::move(fb));
  }

  open_ = true;
  return true;
}

// ---------------------------------------------------------------------------
// close
// ---------------------------------------------------------------------------
void SCS0009Driver::close()
{
  if (!open_) return;
  enable_torque_all(false);
  servo_.end();
  open_ = false;
}

// ---------------------------------------------------------------------------
// poll_all
//
// Issue a Feedback() call sequentially per servo. Unlike other bus servo
// protocols, SCS does not do bulk-reads so we have to do this. 
// At 1 Mbps though, this is totally doable.  

// On success  : ServoState fields updated, reachable = true, last_seen stamped
// On failure  : reachable = false, existing state values left unchanged
// ---------------------------------------------------------------------------

void SCS0009Driver::poll_all()
{
  for (auto& finger : fingers_) {
    update_servo_state(finger.leader);
    update_servo_state(finger.follower);
  }
}

// ---------------------------------------------------------------------------
// update_servo_state  (called for each servo in poll_all)
// ---------------------------------------------------------------------------
void SCS0009Driver::update_servo_state(ServoState& state)
{
  if (servo_.FeedBack(state.id) != 1) {
    state.reachable = false;
    std::ostringstream oss;
    oss << "No feedback from servo " << static_cast<int>(state.id);
    warn(oss.str());
    return;
  }

  // FeedBack() caches all registers — use Read*(-1) to pull from cache
  state.pos_raw     = static_cast<uint16_t>(servo_.ReadPos(-1));
  state.speed_raw   = static_cast<int16_t>(servo_.ReadSpeed(-1));
  state.load_raw    = static_cast<int16_t>(servo_.ReadLoad(-1));
  state.voltage_raw = static_cast<uint8_t>(servo_.ReadVoltage(-1));
  state.temperature = static_cast<uint8_t>(servo_.ReadTemper(-1));
  state.current_raw = static_cast<int16_t>(servo_.ReadCurrent(-1));
  state.reachable   = true;
  state.last_seen   = std::chrono::steady_clock::now();
}

// ---------------------------------------------------------------------------
// write_all
//
// Builds arrays of IDs and positions for every healthy finger pair, then
// fires them all in a single SyncWritePos packet.
//
// Fingers where healthy() == false are skipped — they hold whatever position
// the servos last received.  This is the correct behaviour: a partially
// responsive hand should keep moving the fingers it can reach rather than
// stalling the entire hand.
// ---------------------------------------------------------------------------
void SCS0009Driver::write_all()
{
  // Stack-allocate worst-case arrays (8 servos for a 4-finger hand)
  // SyncWritePos takes raw C arrays, not vectors
  const size_t max_servos = fingers_.size() * 2;
  std::vector<uint8_t>  ids(max_servos);
  std::vector<uint16_t> positions(max_servos);
  std::vector<uint16_t> times(max_servos, 0);     // 0 = use speed register
  std::vector<uint16_t> speeds(max_servos, 0);    // 0 = maximum speed

  uint8_t count = 0;

  for (const auto& finger : fingers_) {
    if (!finger.healthy()) continue;

    // Mix semantic commands into raw motor positions
    MotorCommand mc = mix(finger.config,
                          finger.cmd_flexion,
                          finger.cmd_abduction);

    uint16_t leader_raw   = rad_to_raw(mc.leader_pos_rad,   min_raw_, max_raw_);
    uint16_t follower_raw = rad_to_raw(mc.follower_pos_rad, min_raw_, max_raw_);

    ids[count]       = finger.leader.id;
    positions[count] = leader_raw;
    ++count;

    ids[count]       = finger.follower.id;
    positions[count] = follower_raw;
    ++count;
  }

  if (count == 0) return;   // nothing healthy to write

  // Single bus transaction for all healthy servos
  servo_.SyncWritePos(ids.data(), count,
                      positions.data(),
                      times.data(),
                      speeds.data());
}

// ---------------------------------------------------------------------------
// set_command
// ---------------------------------------------------------------------------
bool SCS0009Driver::set_command(const std::string& finger_name,
                                double             flexion_rad,
                                double             abduction_rad)
{
  for (auto& finger : fingers_) {
    if (finger.config.name == finger_name) {
      finger.cmd_flexion   = flexion_rad;
      finger.cmd_abduction = abduction_rad;
      return true;
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// get_finger
// ---------------------------------------------------------------------------
const FingerBus* SCS0009Driver::get_finger(const std::string& finger_name) const
{
  for (const auto& finger : fingers_) {
    if (finger.config.name == finger_name) return &finger;
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// enable_torque_all / enable_torque
// ---------------------------------------------------------------------------
void SCS0009Driver::enable_torque_all(bool enable)
{
  for (const auto& finger : fingers_) {
    servo_.EnableTorque(finger.leader.id,   enable ? 1 : 0);
    servo_.EnableTorque(finger.follower.id, enable ? 1 : 0);
  }
}

void SCS0009Driver::enable_torque(uint8_t servo_id, bool enable)
{
  servo_.EnableTorque(servo_id, enable ? 1 : 0);
}

// ---------------------------------------------------------------------------
// healthy_finger_count
// ---------------------------------------------------------------------------
int SCS0009Driver::healthy_finger_count() const
{
  return static_cast<int>(
    std::count_if(fingers_.begin(), fingers_.end(),
      [](const FingerBus& f) { return f.healthy(); }));
}

// ---------------------------------------------------------------------------
// warn
// ---------------------------------------------------------------------------
void SCS0009Driver::warn(const std::string& msg) const
{
  if (warn_fn_) warn_fn_(msg);
}

}  // namespace amazinghand
