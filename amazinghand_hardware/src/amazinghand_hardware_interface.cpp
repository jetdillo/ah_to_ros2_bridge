// amazinghand_hardware/src/amazinghand_hardware_interface.cpp

#include "amazinghand_hardware/amazinghand_hardware_interface.hpp"

#include <algorithm>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace amazinghand {

static const char* TAG = "AmazingHandHW";

// ---------------------------------------------------------------------------
// on_init  — parse parameters, build FingerConfig list
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
AmazingHandHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    hand_name_     = info.hardware_parameters.at("hand_name");
    serial_port_   = info.hardware_parameters.at("serial_port");
    baud_rate_     = std::stoi(info.hardware_parameters.at("baud_rate"));
    servo_min_pos_ = std::stoi(info.hardware_parameters.at("servo_min_pos"));
    servo_max_pos_ = std::stoi(info.hardware_parameters.at("servo_max_pos"));
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger(TAG), "Missing hardware parameter: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  load_finger_configs(info);

  // Allocate flat state/command buffers (2 semantic axes per finger)
  const size_t n = fingers_.size() * 2;
  pos_state_.assign(n, 0.0);
  vel_state_.assign(n, 0.0);
  eff_state_.assign(n, 0.0);
  pos_cmd_.assign(n,   0.0);

  RCLCPP_INFO(rclcpp::get_logger(TAG),
    "Initialised '%s' with %zu finger(s) on %s @ %d baud",
    hand_name_.c_str(), fingers_.size(), serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_configure  — create driver and open serial port
//
// FIX: driver constructor now takes min/max raw limits as well as port/baud.
//      open() now takes the finger config list — it builds FingerBus entries
//      and pings each servo internally.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
AmazingHandHardwareInterface::on_configure(const rclcpp_lifecycle::State&)
{
  driver_ = std::make_unique<SCS0009Driver>(
    serial_port_,
    baud_rate_,
    static_cast<uint16_t>(servo_min_pos_),
    static_cast<uint16_t>(servo_max_pos_));

  // Wire up the warn callback so the driver can emit ROS log messages
  driver_->set_warn_callback([](const std::string& msg) {
    RCLCPP_WARN(rclcpp::get_logger(TAG), "%s", msg.c_str());
  });

  if (!driver_->open(fingers_)) {
    RCLCPP_ERROR(rclcpp::get_logger(TAG),
      "Failed to open serial port %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger(TAG),
    "Serial port opened: %s  (%d/%zu finger(s) healthy)",
    serial_port_.c_str(),
    driver_->healthy_finger_count(),
    fingers_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_activate  — enable torque on all servos
//
// FIX: enable_torque_all() now takes only a bool — the driver iterates its
//      own FingerBus list internally.  No servo ID vector needed here.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
AmazingHandHardwareInterface::on_activate(const rclcpp_lifecycle::State&)
{
  driver_->enable_torque_all(true);
  RCLCPP_INFO(rclcpp::get_logger(TAG), "Torque enabled on all servos");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate  — disable torque (hand goes compliant)
//
// FIX: same enable_torque_all() signature change as on_activate.
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
AmazingHandHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&)
{
  driver_->enable_torque_all(false);
  RCLCPP_INFO(rclcpp::get_logger(TAG), "Torque disabled — hand is compliant");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_cleanup  — close serial port
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
AmazingHandHardwareInterface::on_cleanup(const rclcpp_lifecycle::State&)
{
  if (driver_) driver_->close();
  driver_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// export_state_interfaces
// ---------------------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
AmazingHandHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(fingers_.size() * 6);  // pos+vel+eff × 2 axes

  for (size_t i = 0; i < fingers_.size(); ++i) {
    const size_t base = i * 2;
    const auto& f = fingers_[i];

    interfaces.emplace_back(f.flexion_joint_name,
      hardware_interface::HW_IF_POSITION, &pos_state_[base]);
    interfaces.emplace_back(f.flexion_joint_name,
      hardware_interface::HW_IF_VELOCITY, &vel_state_[base]);
    interfaces.emplace_back(f.flexion_joint_name,
      hardware_interface::HW_IF_EFFORT,   &eff_state_[base]);

    interfaces.emplace_back(f.abduction_joint_name,
      hardware_interface::HW_IF_POSITION, &pos_state_[base + 1]);
    interfaces.emplace_back(f.abduction_joint_name,
      hardware_interface::HW_IF_VELOCITY, &vel_state_[base + 1]);
    interfaces.emplace_back(f.abduction_joint_name,
      hardware_interface::HW_IF_EFFORT,   &eff_state_[base + 1]);
  }
  return interfaces;
}

// ---------------------------------------------------------------------------
// export_command_interfaces
// ---------------------------------------------------------------------------
std::vector<hardware_interface::CommandInterface>
AmazingHandHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(fingers_.size() * 2);

  for (size_t i = 0; i < fingers_.size(); ++i) {
    const size_t base = i * 2;
    const auto& f = fingers_[i];

    interfaces.emplace_back(f.flexion_joint_name,
      hardware_interface::HW_IF_POSITION, &pos_cmd_[base]);
    interfaces.emplace_back(f.abduction_joint_name,
      hardware_interface::HW_IF_POSITION, &pos_cmd_[base + 1]);
  }
  return interfaces;
}

// ---------------------------------------------------------------------------
// read  — poll all servos, unmix feedback into state buffers
//
// FIX: replaced bulk_read_feedback(all_servo_ids_) with driver_->poll_all().
//      State is read back per finger via driver_->fingers() which returns
//      the FingerBus list.  unmix() and SCS_RAD_PER_UNIT replace the old
//      per-feedback-struct approach.
// ---------------------------------------------------------------------------
hardware_interface::return_type
AmazingHandHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&)
{
  driver_->poll_all();

  const auto& finger_buses = driver_->fingers();

  for (size_t i = 0; i < fingers_.size(); ++i) {
    const auto& fb = finger_buses[i];

    if (!fb.healthy()) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger(TAG),
        *rclcpp::Clock::make_shared(), 1000,
        "Stale feedback for finger '%s'", fingers_[i].name.c_str());
      continue;
    }

    const size_t base = i * 2;

    // Unmix raw servo positions into semantic axes
    double flexion_rad, abduction_rad;
    unmix(fingers_[i],
          fb.leader.pos_rad(),
          fb.follower.pos_rad(),
          flexion_rad, abduction_rad);

    pos_state_[base]     = flexion_rad;
    pos_state_[base + 1] = abduction_rad;

    // Velocity — unmix raw speed readings the same way
    double flex_vel, abd_vel;
    unmix(fingers_[i],
          raw_to_rad(static_cast<uint16_t>(std::abs(fb.leader.speed_raw))),
          raw_to_rad(static_cast<uint16_t>(std::abs(fb.follower.speed_raw))),
          flex_vel, abd_vel);

    vel_state_[base]     = flex_vel;
    vel_state_[base + 1] = abd_vel;

    // Effort — normalised load as a proxy
    eff_state_[base]     = fb.leader.load_effort();
    eff_state_[base + 1] = fb.follower.load_effort();
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write  — push commands to driver, fire sync write
//
// FIX: replaced hand-rolled mix+rad_to_raw+sync_write_positions with
//      driver_->set_command() + driver_->write_all().  The driver now owns
//      the mixing, clamping, and SyncWritePos call internally.
// ---------------------------------------------------------------------------
hardware_interface::return_type
AmazingHandHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  for (size_t i = 0; i < fingers_.size(); ++i) {
    const size_t base = i * 2;
    driver_->set_command(fingers_[i].name,
                         pos_cmd_[base],       // flexion
                         pos_cmd_[base + 1]);  // abduction
  }

  driver_->write_all();

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// load_finger_configs  — parse from HardwareInfo parameters
// ---------------------------------------------------------------------------
void AmazingHandHardwareInterface::load_finger_configs(
  const hardware_interface::HardwareInfo& info)
{
  size_t idx = 0;
  while (true) {
    const std::string pfx = "finger." + std::to_string(idx) + ".";
    auto it = info.hardware_parameters.find(pfx + "name");
    if (it == info.hardware_parameters.end()) break;

    FingerConfig fc;
    fc.id          = static_cast<int>(idx);
    fc.name        = it->second;
    fc.chirality   = (info.hardware_parameters.at(pfx + "chirality") == "left")
                     ? Chirality::LEFT : Chirality::RIGHT;
    fc.leader_id   = static_cast<uint8_t>(
                       std::stoi(info.hardware_parameters.at(pfx + "leader_id")));
    fc.follower_id = static_cast<uint8_t>(
                       std::stoi(info.hardware_parameters.at(pfx + "follower_id")));

    fc.flexion_joint_name   = hand_name_ + "_" + fc.name + "_flexion";
    fc.abduction_joint_name = hand_name_ + "_" + fc.name + "_abduction";

    fingers_.push_back(fc);
    ++idx;
  }

  if (fingers_.empty()) {
    RCLCPP_WARN(rclcpp::get_logger(TAG),
      "No finger configs found in hardware parameters — is the URDF correct?");
  }
}

}  // namespace amazinghand

// ---------------------------------------------------------------------------
// Plugin export
// ---------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  amazinghand::AmazingHandHardwareInterface,
  hardware_interface::SystemInterface)
