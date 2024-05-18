#include <limits>

#include <pluginlib/class_list_macros.hpp>

#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include "holonomic_rover_controller/holonomic_rover_controller.hpp"

void reset_controller_reference_msg(
    const std::shared_ptr<geometry_msgs::msg::TwistStamped> &msg,
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node) {
  msg->header.stamp = node->now();
  msg->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.linear.z = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.x = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.y = std::numeric_limits<double>::quiet_NaN();
  msg->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
}

namespace holonomic_rover_controller {

controller_interface::CallbackReturn HolonomicRoverController::on_init() {
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

  params_ = param_listener_->get_params();

  rear_wheels_steering_names_ = params_.rear_wheels_steering_names;
  middle_wheels_steering_names_ = params_.middle_wheels_steering_names;
  front_wheels_steering_names_ = params_.front_wheels_steering_names;

  rear_wheels_names_ = params_.rear_wheels_names;
  middle_wheels_names_ = params_.middle_wheels_names;
  front_wheels_names_ = params_.front_wheels_names;

  // topics QoS
  auto subscribers_qos = rclcpp::SystemDefaultsQoS();
  subscribers_qos.keep_last(1);
  subscribers_qos.best_effort();

  // Reference Subscriber
  reference_subscriber_ =
      get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
          "~/reference", subscribers_qos,
          std::bind(&HolonomicRoverController::reference_callback, this,
                    std::placeholders::_1));

  std::shared_ptr<geometry_msgs::msg::TwistStamped> msg =
      std::make_shared<geometry_msgs::msg::TwistStamped>();
  reset_controller_reference_msg(msg, get_node());
  reference_.writeFromNonRT(msg);

  // TODO: Odometry and TF2

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HolonomicRoverController::on_activate(
    const rclcpp_lifecycle::State &previous_state) {

  reset_controller_reference_msg(*(reference_.readFromRT()), get_node());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn HolonomicRoverController::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {

  // command_interfaces_ is inherited from ChainableControllerInterface from
  // ControllerInterface from ControllerInterfaceBase and is of type
  // std::vector<hardware_interface::LoanedCommandInterface>
  for (auto &interface : command_interfaces_)
    interface.set_value(std::numeric_limits<double>::quiet_NaN());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
HolonomicRoverController::command_interface_configuration(void) const {

  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type =
      controller_interface::interface_configuration_type::INDIVIDUAL;

  for (auto &rear_wheel : rear_wheels_names_) {
    command_interfaces_config.names.push_back(
        rear_wheel + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (auto &middle_wheel : middle_wheels_names_) {
    command_interfaces_config.names.push_back(
        middle_wheel + "/" + hardware_interface::HW_IF_VELOCITY);
  }
  for (auto &front_wheel : front_wheels_names_) {
    command_interfaces_config.names.push_back(
        front_wheel + "/" + hardware_interface::HW_IF_VELOCITY);
  }

  for (auto &rear_wheel_steering : rear_wheels_steering_names_) {
    command_interfaces_config.names.push_back(
        rear_wheel_steering + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (auto &middle_wheel_steering : middle_wheels_steering_names_) {
    command_interfaces_config.names.push_back(
        middle_wheel_steering + "/" + hardware_interface::HW_IF_POSITION);
  }
  for (auto &front_wheel_steering : front_wheels_steering_names_) {
    command_interfaces_config.names.push_back(
        front_wheel_steering + "/" + hardware_interface::HW_IF_POSITION);
  }

  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration
HolonomicRoverController::state_interface_configuration(void) const {

  controller_interface::InterfaceConfiguration state_interfaces_config;
  // IMPLEMENT!!
  return state_interfaces_config;
}

std::vector<hardware_interface::CommandInterface>
HolonomicRoverController::on_export_reference_interfaces(void) {

  std::vector<hardware_interface::CommandInterface> interfaces;
  // IMPLEMENT!!
  return interfaces;
}

bool HolonomicRoverController::on_set_chained_mode(bool chained_mode) {

  // Always accept switch to/from chained mode
  return true || chained_mode;
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

void HolonomicRoverController::reference_callback(
    const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {}

} // namespace holonomic_rover_controller

PLUGINLIB_EXPORT_CLASS(holonomic_rover_controller::HolonomicRoverController,
                       controller_interface::ChainableControllerInterface)
