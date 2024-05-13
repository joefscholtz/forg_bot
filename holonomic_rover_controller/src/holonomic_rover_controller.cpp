#include <pluginlib/class_list_macros.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "holonomic_rover_controller/holonomic_rover_controller.hpp"

namespace holonomic_rover_controller {

HolonomicRoverController::HolonomicRoverController()
    : controller_interface::ChainableControllerInterface() {}

controller_interface::CallbackReturn HolonomicRoverController::on_init(void) {
  try {
    param_listener_ =
        std::make_shared<holonomic_rover_controller::ParamListener>(get_node());
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(
        get_node()->get_logger(),
        "Exception thrown during controller's init with message: " << e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // reference_interfaces_ is declared in the base class
  reference_interfaces_.resize(2); // TODO: Check if is really 2 in my case
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HolonomicRoverController::on_configure(
    const rclcpp_lifecycle::State &previous_state) {

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HolonomicRoverController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HolonomicRoverController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
HolonomicRoverController::command_interface_configuration(void) const {

  controller_interface::InterfaceConfiguration command_interfaces_config;
  // IMPLEMENT!!
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
HolonomicRoverController::state_interface_configuration(void) const {

  controller_interface::InterfaceConfiguration state_interfaces_config;
  // IMPLEMENT!!
  return state_interfaces_config;
}

controller_interface::return_type
HolonomicRoverController::update_reference_from_subscribers(void) {

  return controller_interface::return_type::OK;
}

controller_interface::return_type
HolonomicRoverController::update_and_write_commands(
    const rclcpp::Time &time, const rclcpp::Duration &period) {

  return controller_interface::return_type::OK;
}

bool HolonomicRoverController::on_set_chained_mode(bool chained_mode) {

  // Always accept switch to/from chained mode
  return true || chained_mode;
}

void HolonomicRoverController::reference_callback(
    const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {}

void HolonomicRoverController::reference_callback_unstamped(
    const std::shared_ptr<geometry_msgs::msg::Twist> msg) {}
} // namespace holonomic_rover_controller
