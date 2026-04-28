#pragma once

// amazinghand_controller/include/amazinghand_controller/amazinghand_controller.hpp
//
// ChainableController that bridges semantic finger commands to the
// ros2_control hardware interface command interfaces.
//
// Subscription  : ~/command  [amazinghand_msgs/msg/HandCommand]
// Publication   : ~/joint_states [sensor_msgs/msg/JointState]  (optional)
//
// Chain mode reference interfaces (for upstream controllers e.g. MoveIt):
//   <controller_name>/<hand>_<finger>_flexion/position
//   <controller_name>/<hand>_<finger>_abduction/position
//
// The controller owns no mixing logic — that lives in the hardware interface.
// This controller's job is:
//   1. Validate and clamp incoming commands.
//   2. Write them to the hardware command interfaces.
//   3. Publish JointState feedback for visualisation / MoveIt state.

#include <string>
#include <vector>
#include <unordered_map>
#include <memory>

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

#include "amazinghand_msgs/msg/hand_command.hpp"

namespace amazinghand_controller {

class AmazingHandController : public controller_interface::ChainableControllerInterface {
public:
  AmazingHandController() = default;

  // -------------------------------------------------------------------------
  // Lifecycle
  // -------------------------------------------------------------------------
  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  // -------------------------------------------------------------------------
  // Interface declarations
  // -------------------------------------------------------------------------
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration()   const override;

  // -------------------------------------------------------------------------
  // Update
  // -------------------------------------------------------------------------
  controller_interface::return_type update_reference_from_subscribers(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  // ChainableControllerInterface: export reference interfaces for chaining
  std::vector<hardware_interface::CommandInterface>
    on_export_reference_interfaces() override;

private:
  // Parameters
  std::string hand_name_;
  std::vector<std::string> finger_names_;
  double flexion_min_rad_,  flexion_max_rad_;
  double abduction_min_rad_, abduction_max_rad_;
  bool   publish_joint_states_;
  double joint_state_publish_rate_hz_;

  // Per-finger command storage (index-aligned with finger_names_)
  // Each entry: [flexion_cmd, abduction_cmd]
  std::vector<std::array<double, 2>> current_cmds_;

  // Realtime-safe subscriber
  using HandCommandMsg = amazinghand_msgs::msg::HandCommand;
  realtime_tools::RealtimeBuffer<HandCommandMsg::SharedPtr> command_buffer_;
  rclcpp::Subscription<HandCommandMsg>::SharedPtr cmd_sub_;

  // Joint state publisher
  using JointStateMsg = sensor_msgs::msg::JointState;
  std::shared_ptr<realtime_tools::RealtimePublisher<JointStateMsg>> js_pub_;
  rclcpp::Duration js_publish_period_{0, 0};
  rclcpp::Time     last_js_publish_time_;

  // Reference interfaces (for chained mode) — flat buffer
  // Layout: [finger0_flexion, finger0_abduction, finger1_flexion, ...]
  std::vector<double> reference_interfaces_buffer_;

  // Helpers
  std::string flexion_joint_name(const std::string& finger) const;
  std::string abduction_joint_name(const std::string& finger) const;
  void clamp_command(double& flexion, double& abduction) const;
};

}  // namespace amazinghand_controller
