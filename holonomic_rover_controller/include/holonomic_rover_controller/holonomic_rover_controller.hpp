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

#include <geometry_msgs/msg/twist.hpp>
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

  /* HOLONOMIC_ROVER_CONTROLLER__VISIBILITY_PUBLIC */
  /* controller_interface::return_type */
  /* update_reference_from_subscribers(const rclcpp::Time &time, */
  /*                                   const rclcpp::Duration &period) override;
   */
  /* HOLONOMIC_ROVER_CONTROLLER__VISIBILITY_PUBLIC */
  /* controller_interface::return_type */
  /* update_and_write_commands(const rclcpp::Time &time, */
  /*                           const rclcpp::Duration &period) override; */

protected:
  // NOTE:it is not possible to override the update
  // on_export_reference_interfaces from the controller_interface class bc
  // chainable_controller_interface has the final implementation
  //
  /* std::vector<hardware_interface::CommandInterface> */
  /* on_export_reference_interfaces(void) override; */

  bool on_set_chained_mode(bool chained_mode) override;

  controller_interface::return_type
  update_reference_from_subscribers(void) override;
  /*(const rclcpp::Time &time, const rclcpp::Duration &period) override;*/

  controller_interface::return_type
  update_and_write_commands(const rclcpp::Time &time,
                            const rclcpp::Duration &period) override;

private:
  std::shared_ptr<holonomic_rover_controller::ParamListener> param_listener_;
  holonomic_rover_controller::Params params_;

  void reference_callback(
      const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg);
  void reference_callback_unstamped(
      const std::shared_ptr<geometry_msgs::msg::Twist> msg);
};
} // namespace holonomic_rover_controller

#endif // HOLONOMIC_ROVER_CONTROLLER__HOLONOMIC_ROVER_CONTROLLER_HPP_
