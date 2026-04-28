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
    hand_name_    = info.hardware_parameters.at("hand_name");
    serial_port_  = info.hardware_parameters.at("serial_port");
    baud_rate_    = std::stoi(info.hardware_parameters.at("baud_rate"));
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
  states_.resize(fingers_.size());

  // Build ordered servo ID list for bulk I/O
  for (const auto& f : fingers_) {
    all_servo_ids_.push_back(f.leader_id);
    all_servo_ids_.push_back(f.follower_id);
  }

  RCLCPP_INFO(rclcpp::get_logger(TAG),
    "Initialised '%s' with %zu finger(s) on %s @ %d baud",
    hand_name_.c_str(), fingers_.size(), serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_configure  — open serial port
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
AmazingHandHardwareInterface::on_configure(const rclcpp_lifecycle::State&)
{
  driver_ = std::make_unique<SCS0009Driver>(serial_port_, baud_rate_);
  if (!driver_->open()) {
    RCLCPP_ERROR(rclcpp::get_logger(TAG),
      "Failed to open serial port %s", serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger(TAG), "Serial port opened: %s", serial_port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_activate  — enable torque on all servos
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
AmazingHandHardwareInterface::on_activate(const rclcpp_lifecycle::State&)
{
  if (!driver_->set_torque_all(all_servo_ids_, true)) {
    RCLCPP_ERROR(rclcpp::get_logger(TAG), "Failed to enable torque");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger(TAG), "Torque enabled on all servos");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate  — disable torque (hand goes compliant)
// ---------------------------------------------------------------------------
hardware_interface::CallbackReturn
AmazingHandHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&)
{
  driver_->set_torque_all(all_servo_ids_, false);
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

    // Flexion (index base+0)
    interfaces.emplace_back(f.flexion_joint_name,
      hardware_interface::HW_IF_POSITION, &pos_state_[base]);
    interfaces.emplace_back(f.flexion_joint_name,
      hardware_interface::HW_IF_VELOCITY, &vel_state_[base]);
    interfaces.emplace_back(f.flexion_joint_name,
      hardware_interface::HW_IF_EFFORT,   &eff_state_[base]);

    // Abduction (index base+1)
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
// read  — bulk-read servos, unmix into semantic feedback
// ---------------------------------------------------------------------------
hardware_interface::return_type
AmazingHandHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&)
{
  auto feedbacks = driver_->bulk_read_feedback(all_servo_ids_);
  // feedbacks layout mirrors all_servo_ids_: [leader0, follower0, leader1, ...]

  for (size_t i = 0; i < fingers_.size(); ++i) {
    const size_t servo_base = i * 2;
    const auto& leader_fb   = feedbacks[servo_base];
    const auto& follower_fb = feedbacks[servo_base + 1];

    if (!leader_fb.valid || !follower_fb.valid) {
      RCLCPP_WARN_THROTTLE(rclcpp::get_logger(TAG),
        *rclcpp::Clock::make_shared(), 1000,
        "Stale feedback for finger '%s'", fingers_[i].name.c_str());
      continue;
    }

    double leader_rad   = raw_to_rad(leader_fb.position);
    double follower_rad = raw_to_rad(follower_fb.position);

    double flexion_rad, abduction_rad;
    unmix(fingers_[i], leader_rad, follower_rad, flexion_rad, abduction_rad);

    const size_t state_base = i * 2;
    pos_state_[state_base]     = flexion_rad;
    pos_state_[state_base + 1] = abduction_rad;

    // Velocity: same unmix applied to velocity readings
    double leader_vel   = raw_to_rad(leader_fb.velocity);
    double follower_vel = raw_to_rad(follower_fb.velocity);
    double flex_vel, abd_vel;
    unmix(fingers_[i], leader_vel, follower_vel, flex_vel, abd_vel);
    vel_state_[state_base]     = flex_vel;
    vel_state_[state_base + 1] = abd_vel;

    // Effort: approximate from servo load (average of abs values)
    eff_state_[state_base]     = std::abs(leader_fb.load) * SCS0009_RAD_PER_UNIT;
    eff_state_[state_base + 1] = std::abs(follower_fb.load) * SCS0009_RAD_PER_UNIT;
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// write  — mix semantic commands → raw positions, sync-write all servos
// ---------------------------------------------------------------------------
hardware_interface::return_type
AmazingHandHardwareInterface::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  std::vector<uint8_t>  ids;
  std::vector<uint16_t> raw_positions;
  ids.reserve(all_servo_ids_.size());
  raw_positions.reserve(all_servo_ids_.size());

  for (size_t i = 0; i < fingers_.size(); ++i) {
    const size_t base = i * 2;
    const double flexion_cmd   = pos_cmd_[base];
    const double abduction_cmd = pos_cmd_[base + 1];

    MotorCommand mc = mix(fingers_[i], flexion_cmd, abduction_cmd);

    uint16_t leader_raw   = rad_to_raw(mc.leader_pos_rad);
    uint16_t follower_raw = rad_to_raw(mc.follower_pos_rad);

    // Clamp to configured mechanical limits
    leader_raw   = std::clamp(leader_raw,
      static_cast<uint16_t>(servo_min_pos_),
      static_cast<uint16_t>(servo_max_pos_));
    follower_raw = std::clamp(follower_raw,
      static_cast<uint16_t>(servo_min_pos_),
      static_cast<uint16_t>(servo_max_pos_));

    ids.push_back(fingers_[i].leader_id);
    ids.push_back(fingers_[i].follower_id);
    raw_positions.push_back(leader_raw);
    raw_positions.push_back(follower_raw);
  }

  if (!driver_->sync_write_positions(ids, raw_positions)) {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger(TAG),
      *rclcpp::Clock::make_shared(), 1000, "sync_write failed");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

// ---------------------------------------------------------------------------
// load_finger_configs  — parse from HardwareInfo parameters
// ---------------------------------------------------------------------------
void AmazingHandHardwareInterface::load_finger_configs(
  const hardware_interface::HardwareInfo& info)
{
  // Finger parameters are encoded in the URDF <hardware> tag as:
  //   finger.<idx>.name, finger.<idx>.chirality,
  //   finger.<idx>.leader_id, finger.<idx>.follower_id
  // The Python launch script or xacro macro generates these from the YAML.

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

    // Build canonical joint names
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
