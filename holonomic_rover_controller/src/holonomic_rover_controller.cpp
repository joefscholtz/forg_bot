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
  reference_interfaces_.resize(3); // vx, vy, wz
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

  controller_interface::InterfaceConfiguration state_interface_configuration;
  // TODO: implement state interfaces if there are states that need to be
  // accessed
  state_interface_configuration.type =
      controller_interface::interface_configuration_type::NONE;
  return state_interface_configuration;
}

std::vector<hardware_interface::CommandInterface>
HolonomicRoverController::on_export_reference_interfaces(void) {

  std::vector<hardware_interface::CommandInterface> reference_interfaces;

  reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name(), std::string("vx"), &reference_interfaces_[0]));
  reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name(), std::string("vy"), &reference_interfaces_[1]));
  reference_interfaces.push_back(hardware_interface::CommandInterface(
      get_node()->get_name(), std::string("wz"), &reference_interfaces_[2]));

  return reference_interfaces;
}

bool HolonomicRoverController::on_set_chained_mode(bool chained_mode) {

  // Always accept switch to/from chained mode
  return true || chained_mode;
}

controller_interface::return_type
HolonomicRoverController::update_reference_from_subscribers() {

  // Move functionality to the `update_and_write_commands` because of the
  // missing arguments in humble - otherwise issues with multiple time-sources
  // might happen when working with simulators

  return controller_interface::return_type::OK;
}

controller_interface::return_type
HolonomicRoverController::update_and_write_commands(
    const rclcpp::Time &time, const rclcpp::Duration &period) {

  if (!is_in_chained_mode()) {
    auto current_ref = *(reference_.readFromRT());
    const auto age_of_last_command = time - (current_ref)->header.stamp;

    // send message only if there is no timeout
    if (age_of_last_command <= ref_timeout_ ||
        ref_timeout_ == rclcpp::Duration::from_seconds(0.0)) {
      if (!std::isnan(current_ref->twist.linear.x) &&
          !std::isnan(current_ref->twist.linear.y) &&
          !std::isnan(current_ref->twist.angular.z)) {

        reference_interfaces_[0] = current_ref->twist.linear.x;
        reference_interfaces_[1] = current_ref->twist.linear.y;
        reference_interfaces_[2] = current_ref->twist.angular.z;
      }
    } else {
      if (!std::isnan(current_ref->twist.linear.x) &&
          !std::isnan(current_ref->twist.linear.y) &&
          !std::isnan(current_ref->twist.angular.z)) {

        reference_interfaces_[0] = 0.0;
        reference_interfaces_[1] = 0.0;
        reference_interfaces_[2] = 0.0;

        current_ref->twist.linear.x = std::numeric_limits<double>::quiet_NaN();
        current_ref->twist.linear.y = std::numeric_limits<double>::quiet_NaN();
        current_ref->twist.angular.z = std::numeric_limits<double>::quiet_NaN();
      }
    }
  }
  // MOVE ROBOT
  // TODO: Implement Odometry
  // TODO: limit velocities

  return controller_interface::return_type::OK;
}

void HolonomicRoverController::reference_callback(
    const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {

  // if no timestamp provided use current time for command timestamp
  if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0u) {
    RCLCPP_WARN(get_node()->get_logger(),
                "Timestamp in the reference's header is missing, using current "
                "time as command "
                "timestamp.");
    msg->header.stamp = get_node()->now();
  }
  const auto age_of_last_command = get_node()->now() - msg->header.stamp;

  if (ref_timeout_ == rclcpp::Duration::from_seconds(0) ||
      age_of_last_command <= ref_timeout_) {
    reference_.writeFromNonRT(msg);
  } else {
    RCLCPP_ERROR(
        get_node()->get_logger(),
        "Received reference message has timestamp %.10f older for %.10f which "
        "is more then allowed timeout "
        "(%.4f).",
        rclcpp::Time(msg->header.stamp).seconds(),
        age_of_last_command.seconds(), ref_timeout_.seconds());
  }
}

} // namespace holonomic_rover_controller

PLUGINLIB_EXPORT_CLASS(holonomic_rover_controller::HolonomicRoverController,
                       controller_interface::ChainableControllerInterface)
