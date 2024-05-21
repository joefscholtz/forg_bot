#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class HolonomicRoverKinematics : public rclcpp::Node {
public:
  HolonomicRoverKinematics() : Node("holonomic_rover_kinematics") {
    joint_states_input_topic_name = this->declare_parameter<std::string>(
        "joint_states_input_topic_name", "input_joint_states");

    joint_states_output_topic_name = this->declare_parameter<std::string>(
        "joint_states_output_topic_name", "joint_states");

    twist_input_topic_name = this->declare_parameter<std::string>(
        "twist_input_topic_name", "cmd_vel");

    markers_topic_name = this->declare_parameter<std::string>(
        "markers_topic_name", "normal_vectors_to_wheel");

    front_wheel =
        this->declare_parameter<std::string>("front_wheel", "front_wheel");
    front_steering_wheel = this->declare_parameter<std::string>(
        "front_steering_wheel", "front_steering_wheel");

    rear_wheel =
        this->declare_parameter<std::string>("rear_wheel", "rear_wheel");
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

    marker_array_publisher =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            markers_topic_name, 10);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_publisher;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher;

  sensor_msgs::msg::JointState joint_states;

  std::string joint_states_input_topic_name, joint_states_output_topic_name,
      twist_input_topic_name, markers_topic_name;

  std::string front_wheel, front_steering_wheel, rear_wheel,
      rear_steering_wheel;
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

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    geometry_msgs::msg::Point point;

    marker.header.stamp = this->get_clock()->now();
    marker.header.frame_id = rear_wheel;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = 0;
    marker.scale.x = 0.05;
    // lavender
    marker.color.r = 0.588;
    marker.color.g = 0.482;
    marker.color.b = 0.714;
    marker.color.a = 1.0;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    marker.points.push_back(point);
    point.x = 10;
    point.y = 0;
    point.z = 0;
    marker.points.push_back(point);
    markers.markers.push_back(marker);

    marker.header.frame_id = front_wheel;
    marker.id = 1;
    markers.markers.push_back(marker);

    marker_array_publisher->publish(markers);
  }

  void twist_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg) {}
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HolonomicRoverKinematics>());
  rclcpp::shutdown();
  return 0;
}
