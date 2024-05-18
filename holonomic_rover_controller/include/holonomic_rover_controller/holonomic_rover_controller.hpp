#ifndef HOLONOMIC_ROVER_CONTROLLER__HOLONOMIC_ROVER_CONTROLLER_HPP_
#define HOLONOMIC_ROVER_CONTROLLER__HOLONOMIC_ROVER_CONTROLLER_HPP_

#include <controller_interface/chainable_controller_interface.hpp>
/* #include <hardware_interface/handle.hpp> */
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "holonomic_rover_controller/visibility_control.h"
#include "holonomic_rover_controller_parameters.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

namespace holonomic_rover_controller {
class HolonomicRoverController
    : public controller_interface::ChainableControllerInterface {

public:
  HOLONOMIC_ROVER_CONTROLLER__VISIBILITY_PUBLIC HolonomicRoverController();

  HOLONOMIC_ROVER_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  HOLONOMIC_ROVER_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  HOLONOMIC_ROVER_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  HOLONOMIC_ROVER_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  // NOTE:it is not possible to override the update method from the
  // controller_interface class bc chainable_controller_interface has the final
  // implementation
  //
  /* HOLONOMIC_ROVER_CONTROLLER__VISIBILITY_PUBLIC */
  /* controller_interface::return_type */
  /* update(const rclcpp::Time &time, const rclcpp::Duration &period) override;
   */

  HOLONOMIC_ROVER_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration
  command_interface_configuration(void) const override;

  HOLONOMIC_ROVER_CONTROLLER__VISIBILITY_PUBLIC
  controller_interface::InterfaceConfiguration
  state_interface_configuration(void) const override;

protected:
  // NOTE:it is not possible to override the update method from the
  // controller_interface class bc chainable_controller_interface has the final
  // implementation

  /// Virtual method that each chainable controller should implement to export
  /// its chainable interfaces.
  /**
   * Each chainable controller implements this methods where all input (command)
   * interfaces are exported. The method has the same meaning as
   * `export_command_interface` method from hardware_interface::SystemInterface
   * or hardware_interface::ActuatorInterface.
   *
   * \returns list of CommandInterfaces that other controller can use as their
   * outputs.
   */
  std::vector<hardware_interface::CommandInterface>
  on_export_reference_interfaces(void) override;

  /// Virtual method that each chainable controller should implement to switch
  /// chained mode.
  /**
   * Each chainable controller implements this methods to switch between
   * "chained" and "external" mode. In "chained" mode all external interfaces
   * like subscriber and service servers are disabled to avoid potential
   * concurrency in input commands.
   *
   * \param[in] flag marking a switch to or from chained mode.
   *
   * \returns true if controller successfully switched between "chained" and
   * "external" mode. \default returns true so the method don't have to be
   * overridden if controller can always switch chained mode.
   */
  bool on_set_chained_mode(bool chained_mode) override;

  /// Update reference from input topics when not in chained mode.
  /**
   * Each chainable controller implements this method to update reference from
   * subscribers when not in chained mode.
   *
   * \returns return_type::OK if update is successfully, otherwise
   * return_type::ERROR.
   */
  controller_interface::return_type
  // in Humble:
  update_reference_from_subscribers() override;
  // Since Iron:
  /* update_reference_from_subscribers(const rclcpp::Time &time, */
  /*                                   const rclcpp::Duration &period) override;
   */

  /// Execute calculations of the controller and update command interfaces.
  /**
   * Update method for chainable controllers.
   * In this method is valid to assume that \reference_interfaces_ hold the
   * values for calculation of the commands in the current control step. This
   * means that this method is called after \update_reference_from_subscribers
   * if controller is not in chained mode.
   *
   * \returns return_type::OK if calculation and writing of interface is
   * successfully, otherwise return_type::ERROR.
   */
  controller_interface::return_type
  update_and_write_commands(const rclcpp::Time &time,
                            const rclcpp::Duration &period) override;

private:
  std::shared_ptr<holonomic_rover_controller::ParamListener> param_listener_;
  holonomic_rover_controller::Params params_;

  std::vector<std::string> rear_wheels_names_, middle_wheels_names_,
      front_wheels_names_;

  std::vector<std::string> rear_wheels_steering_names_,
      middle_wheels_steering_names_, front_wheels_steering_names_;

  realtime_tools::RealtimeBuffer<
      std::shared_ptr<geometry_msgs::msg::TwistStamped>>
      reference_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      reference_subscriber_;

  rclcpp::Duration ref_timeout_ = rclcpp::Duration::from_seconds(0.0);

  void reference_callback(
      const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg);
};
} // namespace holonomic_rover_controller

#endif // HOLONOMIC_ROVER_CONTROLLER__HOLONOMIC_ROVER_CONTROLLER_HPP_
