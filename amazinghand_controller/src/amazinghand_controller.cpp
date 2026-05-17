// amazinghand_controller/src/amazinghand_controller.cpp
//
// ROS2 Humble compatible. Key changes from original Rolling version:
//   - Base class: ControllerInterface (not ChainableControllerInterface)
//   - update_reference_from_subscribers() + update_and_write_commands() merged
//     into a single update() — the only update method on Humble
//   - on_export_reference_interfaces() removed (ChainableControllerInterface only)
//   - PLUGINLIB_EXPORT_CLASS base corrected to controller_interface::ControllerInterface

#include "amazinghand_controller/amazinghand_controller.hpp"

#include <algorithm>
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

namespace amazinghand_controller {

static const char* TAG = "AmazingHandCtrl";

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
std::string AmazingHandController::flexion_joint_name(const std::string& f) const
{
  return hand_name_ + "_" + f + "_flexion";
}

std::string AmazingHandController::abduction_joint_name(const std::string& f) const
{
  return hand_name_ + "_" + f + "_abduction";
}

void AmazingHandController::clamp_command(double& flexion, double& abduction) const
{
  flexion   = std::clamp(flexion,   flexion_min_rad_,   flexion_max_rad_);
  abduction = std::clamp(abduction, abduction_min_rad_, abduction_max_rad_);
}

// ---------------------------------------------------------------------------
// on_init
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn AmazingHandController::on_init()
{
  try {
    auto_declare<std::string>("hand_name", "right_hand");
    auto_declare<std::vector<std::string>>("finger_names", {});
    auto_declare<double>("flexion_limit_rad.min",   -1.5708);
    auto_declare<double>("flexion_limit_rad.max",    1.5708);
    auto_declare<double>("abduction_limit_rad.min", -0.5236);
    auto_declare<double>("abduction_limit_rad.max",  0.5236);
    auto_declare<bool>("publish_joint_states", true);
    auto_declare<double>("joint_state_publish_rate_hz", 50.0);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_configure
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn AmazingHandController::on_configure(
  const rclcpp_lifecycle::State&)
{
  hand_name_    = get_node()->get_parameter("hand_name").as_string();
  finger_names_ = get_node()->get_parameter("finger_names").as_string_array();

  flexion_min_rad_    = get_node()->get_parameter("flexion_limit_rad.min").as_double();
  flexion_max_rad_    = get_node()->get_parameter("flexion_limit_rad.max").as_double();
  abduction_min_rad_  = get_node()->get_parameter("abduction_limit_rad.min").as_double();
  abduction_max_rad_  = get_node()->get_parameter("abduction_limit_rad.max").as_double();
  publish_joint_states_ = get_node()->get_parameter("publish_joint_states").as_bool();
  joint_state_publish_rate_hz_ =
    get_node()->get_parameter("joint_state_publish_rate_hz").as_double();

  if (finger_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'finger_names' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialise command storage to zero (neutral / straight)
  current_cmds_.assign(finger_names_.size(), {0.0, 0.0});

  // Subscribe to hand commands (non-realtime; buffered via RealtimeBuffer)
  cmd_sub_ = get_node()->create_subscription<HandCommandMsg>(
    "~/command", rclcpp::SystemDefaultsQoS(),
    [this](HandCommandMsg::SharedPtr msg) {
      command_buffer_.writeFromNonRT(msg);
    });

  // Joint state publisher
  if (publish_joint_states_) {
    auto js_pub_raw = get_node()->create_publisher<JointStateMsg>(
      "~/joint_states", rclcpp::SystemDefaultsQoS());
    js_pub_ = std::make_shared<realtime_tools::RealtimePublisher<JointStateMsg>>(
      js_pub_raw);

    js_publish_period_ = rclcpp::Duration::from_seconds(
      1.0 / joint_state_publish_rate_hz_);

    js_pub_->msg_.name.reserve(finger_names_.size() * 2);
    for (const auto& f : finger_names_) {
      js_pub_->msg_.name.push_back(flexion_joint_name(f));
      js_pub_->msg_.name.push_back(abduction_joint_name(f));
    }
    js_pub_->msg_.position.resize(finger_names_.size() * 2, 0.0);
    js_pub_->msg_.velocity.resize(finger_names_.size() * 2, 0.0);
    js_pub_->msg_.effort.resize(finger_names_.size() * 2,   0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(),
    "Configured for hand '%s' with %zu finger(s)",
    hand_name_.c_str(), finger_names_.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_activate
// Seed commands from current hardware state to avoid a startup position jump.
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn AmazingHandController::on_activate(
  const rclcpp_lifecycle::State&)
{
  last_js_publish_time_ = get_node()->now();

  for (size_t i = 0; i < finger_names_.size(); ++i) {
    const std::string flex_name = flexion_joint_name(finger_names_[i]);
    const std::string abd_name  = abduction_joint_name(finger_names_[i]);

    auto flex_it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
      [&](const auto& si) {
        return si.get_prefix_name() == flex_name &&
               si.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });
    auto abd_it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
      [&](const auto& si) {
        return si.get_prefix_name() == abd_name &&
               si.get_interface_name() == hardware_interface::HW_IF_POSITION;
      });

    current_cmds_[i][0] = (flex_it != state_interfaces_.end())
                           ? flex_it->get_value() : 0.0;
    current_cmds_[i][1] = (abd_it  != state_interfaces_.end())
                           ? abd_it->get_value()  : 0.0;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// on_deactivate
// ---------------------------------------------------------------------------
controller_interface::CallbackReturn AmazingHandController::on_deactivate(
  const rclcpp_lifecycle::State&)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

// ---------------------------------------------------------------------------
// command_interface_configuration
// ---------------------------------------------------------------------------
controller_interface::InterfaceConfiguration
AmazingHandController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& f : finger_names_) {
    cfg.names.push_back(flexion_joint_name(f)   + "/" + hardware_interface::HW_IF_POSITION);
    cfg.names.push_back(abduction_joint_name(f) + "/" + hardware_interface::HW_IF_POSITION);
  }
  return cfg;
}

// ---------------------------------------------------------------------------
// state_interface_configuration
// ---------------------------------------------------------------------------
controller_interface::InterfaceConfiguration
AmazingHandController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& f : finger_names_) {
    for (const auto& iface : {hardware_interface::HW_IF_POSITION,
                               hardware_interface::HW_IF_VELOCITY,
                               hardware_interface::HW_IF_EFFORT}) {
      cfg.names.push_back(flexion_joint_name(f)   + "/" + iface);
      cfg.names.push_back(abduction_joint_name(f) + "/" + iface);
    }
  }
  return cfg;
}

// ---------------------------------------------------------------------------
// update  (single RT method on Humble ControllerInterface)
//
// Replaces both update_reference_from_subscribers() and
// update_and_write_commands() from the Rolling ChainableControllerInterface.
// Subscriber handling and command writes happen in sequence each cycle.
// ---------------------------------------------------------------------------
controller_interface::return_type AmazingHandController::update(
  const rclcpp::Time& time, const rclcpp::Duration&)
{
  // --- Pull latest command from the non-RT subscriber buffer ---------------
  auto msg = *command_buffer_.readFromRT();
  if (msg) {
    for (const auto& fc : msg->fingers) {
      auto it = std::find(finger_names_.begin(), finger_names_.end(), fc.finger_name);
      if (it == finger_names_.end()) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(),
          *get_node()->get_clock(), 2000,
          "Unknown finger '%s' in command — ignored", fc.finger_name.c_str());
        continue;
      }
      size_t i = static_cast<size_t>(std::distance(finger_names_.begin(), it));
      double flex = fc.flexion_rad;
      double abd  = fc.abduction_rad;
      clamp_command(flex, abd);
      current_cmds_[i][0] = flex;
      current_cmds_[i][1] = abd;
    }
  }

  // --- Write to hardware command interfaces --------------------------------
  for (size_t i = 0; i < finger_names_.size(); ++i) {
    command_interfaces_[i * 2].set_value(current_cmds_[i][0]);      // flexion
    command_interfaces_[i * 2 + 1].set_value(current_cmds_[i][1]);  // abduction
  }

  // --- Publish joint states at configured rate -----------------------------
  if (publish_joint_states_ && js_pub_ &&
      (time - last_js_publish_time_) >= js_publish_period_) {
    if (js_pub_->trylock()) {
      js_pub_->msg_.header.stamp = time;
      for (size_t i = 0; i < finger_names_.size(); ++i) {
        const size_t base = i * 2;
        js_pub_->msg_.position[base]     = state_interfaces_[base * 3].get_value();
        js_pub_->msg_.position[base + 1] = state_interfaces_[base * 3 + 3].get_value();
        js_pub_->msg_.velocity[base]     = state_interfaces_[base * 3 + 1].get_value();
        js_pub_->msg_.velocity[base + 1] = state_interfaces_[base * 3 + 4].get_value();
        js_pub_->msg_.effort[base]       = state_interfaces_[base * 3 + 2].get_value();
        js_pub_->msg_.effort[base + 1]   = state_interfaces_[base * 3 + 5].get_value();
      }
      js_pub_->unlockAndPublish();
    }
    last_js_publish_time_ = time;
  }

  return controller_interface::return_type::OK;
}

}  // namespace amazinghand_controller

// ---------------------------------------------------------------------------
// Plugin export
// Base class corrected to ControllerInterface for Humble compatibility.
// ---------------------------------------------------------------------------
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  amazinghand_controller::AmazingHandController,
  controller_interface::ControllerInterface)
