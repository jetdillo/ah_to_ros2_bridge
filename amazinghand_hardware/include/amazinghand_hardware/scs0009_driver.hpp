#pragma once

// amazinghand_hardware/include/amazinghand_hardware/scs0009_driver.hpp
//
// SCS0009 serial bus driver for the AmazingHand.
//
// This layer wraps the SCServo_Linux SCS0009 class and owns the FingerBus
// abstraction — a servo pair (leader + follower) plus their live health and
// feedback state.
//
// Design notes:
//   - One SCS0009 object, one serial port, all servos on a single bus.
//   - "Open" means the port was initialised by SCServo_Linux and at least one
//     Ping succeeded.  There is no OS-level fd exposed here; SCServo_Linux
//     owns the termios / ioctl layer entirely.
//   - "Reachable" is a per-servo boolean updated every poll cycle via
//     FeedBack().  It is the domain equivalent of FD_ISSET() — it gates
//     writes so that only healthy servo pairs receive position commands.
//   - poll_all() sequences FeedBack() calls for every servo and updates
//     ServoState.  It is the only place reachable flags change.
//   - write_all() builds a single SyncWritePos packet containing only
//     healthy finger pairs and fires it in one bus transaction.
//   - Unit conversion (raw 0-1023 <-> radians) lives here.  Everything above
//     this layer speaks radians through the finger_types.hpp mixing matrix.

#include <string>
#include <vector>
#include <functional>
#include <cstdint>
#include <chrono>

#include "amazinghand_hardware/finger_types.hpp"
#include "SCS0009.h"   // SCServo_Linux application layer

namespace amazinghand {

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
constexpr uint16_t SCS_CENTER_POS   = 512;
constexpr uint16_t SCS_MAX_POS      = 1023;
constexpr double   SCS_RAD_PER_UNIT = (2.0 * 3.14159265358979323846) / 1023.0;

// ---------------------------------------------------------------------------
// Unit conversion
// ---------------------------------------------------------------------------
inline uint16_t rad_to_raw(double rad, uint16_t min_raw, uint16_t max_raw)
{
  int raw = static_cast<int>(SCS_CENTER_POS + rad / SCS_RAD_PER_UNIT);
  if (raw < static_cast<int>(min_raw)) raw = static_cast<int>(min_raw);
  if (raw > static_cast<int>(max_raw)) raw = static_cast<int>(max_raw);
  return static_cast<uint16_t>(raw);
}

inline double raw_to_rad(uint16_t raw)
{
  return (static_cast<double>(raw) - SCS_CENTER_POS) * SCS_RAD_PER_UNIT;
}

// ---------------------------------------------------------------------------
// ServoState
// Per-servo live state, updated every poll cycle by poll_all().
// ---------------------------------------------------------------------------
struct ServoState {
  uint8_t  id          = 0;
  bool     reachable   = false;       // last FeedBack() succeeded
  uint16_t pos_raw     = SCS_CENTER_POS;
  int16_t  speed_raw   = 0;           // signed — bit15 = direction
  int16_t  load_raw    = 0;           // signed — bit15 = direction
  uint8_t  voltage_raw = 0;           // ×0.1 V
  uint8_t  temperature = 0;           // °C
  int16_t  current_raw = 0;           // mA

  double pos_rad()     const { return raw_to_rad(pos_raw); }
  double load_effort() const { return static_cast<double>(load_raw) / 1000.0; }

  // Timestamp of last successful FeedBack() — for stale-data detection
  std::chrono::steady_clock::time_point last_seen{};
};

// ---------------------------------------------------------------------------
// FingerBus
// The atomic control unit: one finger = one leader + one follower servo.
//
// Commands are stored here in semantic axes (flexion, abduction) and mixed
// into raw motor positions by write_all() using the finger_types mix()
// function.  The hardware interface writes to cmd_flexion / cmd_abduction;
// it never touches servo IDs or raw positions directly.
// ---------------------------------------------------------------------------
struct FingerBus {
  FingerConfig config;      // from finger_types.hpp (name, chirality, IDs)
  ServoState   leader;
  ServoState   follower;

  // Commanded positions in semantic axes — set by the hardware interface
  double cmd_flexion   = 0.0;   // rad, + = curl
  double cmd_abduction = 0.0;   // rad, + = spread (chirality-adjusted)

  // Both servos must have responded to their last FeedBack() for a write
  // to be sent to this finger.
  bool healthy() const { return leader.reachable && follower.reachable; }
};

// ---------------------------------------------------------------------------
// SCS0009Driver
// ---------------------------------------------------------------------------
class SCS0009Driver {
public:
  // port      : e.g. "/dev/ttyACM0"
  // baud_rate : 1000000 is the SCS0009 default
  // min/max   : raw position clamp limits (from YAML — protect mechanical stops)
  SCS0009Driver(const std::string& port,
                int                baud_rate,
                uint16_t           min_raw = 200,
                uint16_t           max_raw = 824);
  ~SCS0009Driver();

  // Non-copyable — owns the servo bus
  SCS0009Driver(const SCS0009Driver&)            = delete;
  SCS0009Driver& operator=(const SCS0009Driver&) = delete;

  // -------------------------------------------------------------------------
  // Lifecycle
  // -------------------------------------------------------------------------

  // Initialise the serial port via SCServo_Linux and build the FingerBus list
  // from the provided finger configs.  Pings every servo; logs unreachable
  // ones as warnings but does not fail — partial hands are usable.
  // Returns false only if the serial port itself could not be opened.
  bool open(const std::vector<FingerConfig>& fingers);

  // Disable torque on all servos and close the serial port.
  void close();

  // True if the serial port is open (independent of per-servo reachability).
  bool is_open() const { return open_; }

  // -------------------------------------------------------------------------
  // Control cycle — called from hardware interface read() / write()
  // -------------------------------------------------------------------------

  // Poll every servo sequentially via FeedBack().
  // Updates ServoState for each servo and sets its reachable flag.
  // Unreachable servos emit a throttled warn via the warn callback.
  void poll_all();

  // Build a SyncWritePos packet for all healthy() finger pairs and send it
  // in a single bus transaction.  Fingers where healthy() == false are
  // skipped — they hold their last physical position.
  void write_all();

  // -------------------------------------------------------------------------
  // Command interface — called by the hardware interface write() path
  // -------------------------------------------------------------------------

  // Set the commanded semantic position for a named finger.
  // Returns false if finger_name is not in the configured finger list.
  bool set_command(const std::string& finger_name,
                   double             flexion_rad,
                   double             abduction_rad);

  // -------------------------------------------------------------------------
  // State readback — called by the hardware interface read() path
  // -------------------------------------------------------------------------

  // Access the full FingerBus state for a named finger (read-only).
  // Returns nullptr if not found.
  const FingerBus* get_finger(const std::string& finger_name) const;

  // Full finger list — used by the hardware interface to iterate state
  const std::vector<FingerBus>& fingers() const { return fingers_; }

  // -------------------------------------------------------------------------
  // Torque control
  // -------------------------------------------------------------------------
  void enable_torque_all(bool enable);
  void enable_torque(uint8_t servo_id, bool enable);

  // -------------------------------------------------------------------------
  // Diagnostics
  // -------------------------------------------------------------------------
  int  healthy_finger_count() const;

  // Warn callback — set by the hardware interface so the driver can emit
  // ROS log messages without a direct rclcpp dependency.
  using WarnFn = std::function<void(const std::string&)>;
  void set_warn_callback(WarnFn fn) { warn_fn_ = std::move(fn); }

private:
  void update_servo_state(ServoState& state);
  void warn(const std::string& msg) const;

  // SCServo_Linux handle — one object owns the entire bus
  SCS0009  servo_;
  bool     open_     = false;

  std::string port_;
  int         baud_rate_;
  uint16_t    min_raw_;
  uint16_t    max_raw_;

  // Index-stable after open() — the hardware interface holds pointers into
  // FingerBus entries for the lifetime of the driver
  std::vector<FingerBus> fingers_;

  WarnFn warn_fn_;
};

}  // namespace amazinghand
