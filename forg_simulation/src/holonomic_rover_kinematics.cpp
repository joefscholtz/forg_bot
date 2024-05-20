#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class HolonomicRoverKinematics : public rclcpp::Node {
public:
  HolonomicRoverKinematics() : Node("holonomic_rover_kinematics") {
    joint_states_input_topic_name = this->declare_parameter<std::string>(
        "joint_states_input_topic_name", "input_joint_states");

    joint_states_output_topic_name = this->declare_parameter<std::string>(
        "joint_states_output_topic_name", "joint_states");

    twist_input_topic_name = this->declare_parameter<std::string>(
        "twist_input_topic_name", "cmd_vel");

    front_steering_wheel = this->declare_parameter<std::string>(
        "front_steering_wheel", "front_steering_wheel");

    rear_steering_wheel = this->declare_parameter<std::string>(
        "rear_steering_wheel", "rear_steering_wheel");

    joint_states_publisher =
        this->create_publisher<sensor_msgs::msg::JointState>(
            joint_states_output_topic_name, 10);

    joint_states_subscriber =
        this->create_subscription<sensor_msgs::msg::JointState>(
            joint_states_input_topic_name, 10,
            std::bind(&HolonomicRoverKinematics::joint_states_callback, this,
                      std::placeholders::_1));

    twist_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        twist_input_topic_name, 10,
        std::bind(&HolonomicRoverKinematics::twist_callback, this,
                  std::placeholders::_1));
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_publisher;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber;

  sensor_msgs::msg::JointState joint_states;

  std::string joint_states_input_topic_name, joint_states_output_topic_name,
      twist_input_topic_name;

  std::string front_steering_wheel, rear_steering_wheel;
  double deltaF{0}, deltaR{0};

  void joint_states_callback(
      const std::shared_ptr<sensor_msgs::msg::JointState> msg) {

    joint_states = *msg;
    for (unsigned int i = 0; i < joint_states.name.size(); i++) {
      if (joint_states.name[i] == front_steering_wheel) {
        joint_states.position[i] = deltaF;
        continue;
      }
      if (joint_states.name[i] == rear_steering_wheel) {
        joint_states.position[i] = deltaR;
        continue;
      }
    }
    joint_states_publisher->publish(joint_states);
  }

  void twist_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg) {}
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HolonomicRoverKinematics>());
  rclcpp::shutdown();
  return 0;
}
