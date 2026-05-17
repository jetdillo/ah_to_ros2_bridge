#pragma once

// amazinghand_hardware/include/amazinghand_hardware/amazinghand_hardware_interface.hpp
//
// ros2_control SystemInterface for the AmazingHand.
//
// Advertised interfaces (per finger, per semantic axis):
//   Command : <hand>_<finger>_flexion/position
//             <hand>_<finger>_abduction/position
//   State   : <hand>_<finger>_flexion/position
//             <hand>_<finger>_flexion/velocity
//             <hand>_<finger>_flexion/effort
//             <hand>_<finger>_abduction/position
//             <hand>_<finger>_abduction/velocity
//             <hand>_<finger>_abduction/effort
//
// Internally the interface:
//   1. Reads the YAML finger config and builds FingerConfig objects.
//   2. On write(): writes semantic commands to the driver via set_command(),
//      then calls write_all() which handles mixing and SyncWritePos internally.
//   3. On read(): calls poll_all() which sequences FeedBack() per servo,
//      then reads back FingerBus state and applies unmix() to update the
//      ros2_control state interface buffers.

#include <vector>
#include <string>
#include <memory>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "amazinghand_hardware/finger_types.hpp"
#include "amazinghand_hardware/scs0009_driver.hpp"

namespace amazinghand {

class AmazingHandHardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AmazingHandHardwareInterface)

  // -------------------------------------------------------------------------
  // Lifecycle
  // -------------------------------------------------------------------------
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State& previous_state) override;

  // -------------------------------------------------------------------------
  // Interface export (called once after on_init)
  // -------------------------------------------------------------------------
  std::vector<hardware_interface::StateInterface>   export_state_interfaces()   override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // -------------------------------------------------------------------------
  // Control loop
  // -------------------------------------------------------------------------
  hardware_interface::return_type read(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

  hardware_interface::return_type write(
    const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // Parsed from YAML / ros2_control URDF <hardware> tag parameters
  std::string  hand_name_;
  std::string  serial_port_;
  int          baud_rate_;
  int          servo_min_pos_;
  int          servo_max_pos_;

  // Finger configs parsed from URDF <hardware> params — index-stable after on_init
  std::vector<FingerConfig> fingers_;

  // Driver owns the bus, the FingerBus list, and all servo I/O
  std::unique_ptr<SCS0009Driver> driver_;

  // Flat double buffers that ros2_control StateInterface / CommandInterface
  // hold references into.  Layout: [finger0_flex, finger0_abd, finger1_flex, ...]
  std::vector<double> pos_state_;
  std::vector<double> vel_state_;
  std::vector<double> eff_state_;
  std::vector<double> pos_cmd_;

  // Helpers
  void load_finger_configs(const hardware_interface::HardwareInfo& info);
};

}  // namespace amazinghand
