#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <unsupported/Eigen/EulerAngles>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class HolonomicRoverKinematics : public rclcpp::Node {
public:
  HolonomicRoverKinematics() : Node("holonomic_rover_kinematics") {

    debug = this->declare_parameter<bool>("debug", false);

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

    middle_wheel =
        this->declare_parameter<std::string>("middle_wheel", "middle_wheel");
    middle_steering_wheel = this->declare_parameter<std::string>(
        "middle_steering_wheel", "middle_steering_wheel");

    rear_wheel =
        this->declare_parameter<std::string>("rear_wheel", "rear_wheel");
    rear_steering_wheel = this->declare_parameter<std::string>(
        "rear_steering_wheel", "rear_steering_wheel");

    front_to_middle_wheel_distance =
        this->declare_parameter<double>("front_to_middle_wheel_distance", 1.0);
    middle_to_base_link_distance =
        this->declare_parameter<double>("middle_to_base_link_distance", 1.0);
    middle_to_back_wheel_distance =
        this->declare_parameter<double>("middle_to_back_wheel_distance", 1.0);
    wheel_track = this->declare_parameter<double>("wheel_track", 1.0);

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

    timer_ = this->create_wall_timer(
        1s, std::bind(&HolonomicRoverKinematics::timer_callback, this));
  }

private:
  bool debug;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_publisher;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_states_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_publisher;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::JointState joint_states;

  std::string joint_states_input_topic_name, joint_states_output_topic_name,
      twist_input_topic_name, markers_topic_name;

  std::string front_wheel, front_steering_wheel, middle_wheel,
      middle_steering_wheel, rear_wheel, rear_steering_wheel;
  double front_to_middle_wheel_distance, middle_to_base_link_distance,
      middle_to_back_wheel_distance, wheel_track;
  double deltaF{0}, deltaM{0}, deltaR{0};

  void timer_callback() {

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    geometry_msgs::msg::Point point;

    marker.header.stamp = this->get_clock()->now();
    marker.header.frame_id = rear_wheel;
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = 0;
    marker.scale.x = 0.02;
    // lavender
    marker.color.r = 0.588;
    marker.color.g = 0.482;
    marker.color.b = 0.714;
    marker.color.a = 1.0;
    marker.frame_locked = true;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    marker.points.push_back(point);
    point.x = 10;
    point.y = 0;
    point.z = 0;
    marker.points.push_back(point);
    markers.markers.push_back(marker);

    marker.header.frame_id = middle_wheel;
    marker.id = 1;
    markers.markers.push_back(marker);

    marker.header.frame_id = front_wheel;
    marker.id = 2;
    markers.markers.push_back(marker);

    // red
    marker.header.frame_id = rear_wheel;
    marker.id = 3;
    marker.color.r = 0.9;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    point.x = 0;
    point.y = 0;
    point.z = 0;
    marker.points[0] = point;
    point.x = -10;
    point.y = 0;
    point.z = 0;
    marker.points[1] = point;
    markers.markers.push_back(marker);

    marker.header.frame_id = middle_wheel;
    marker.id = 4;
    markers.markers.push_back(marker);

    marker.header.frame_id = front_wheel;
    marker.id = 5;
    markers.markers.push_back(marker);

    marker_array_publisher->publish(markers);
  }

  void joint_states_callback(
      const std::shared_ptr<sensor_msgs::msg::JointState> msg) {

    joint_states = *msg;
    for (unsigned int i = 0; i < joint_states.name.size(); i++) {
      if (joint_states.name[i] == front_steering_wheel) {
        joint_states.position[i] = deltaF;
        continue;
      }
      if (joint_states.name[i] == middle_steering_wheel) {
        joint_states.position[i] = deltaM;
        continue;
      }
      if (joint_states.name[i] == rear_steering_wheel) {
        joint_states.position[i] = deltaR;
        continue;
      }
    }
    joint_states_publisher->publish(joint_states);
  }

  void twist_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
    auto twist = *msg;

    Eigen::Vector3d lf;
    lf(0) = -front_to_middle_wheel_distance;
    lf(1) = 0;
    lf(2) = 0;

    Eigen::Vector3d lm;
    lm(0) = -middle_to_base_link_distance;
    lm(1) = 0;
    lm(2) = 0;

    Eigen::Vector3d lr;
    lr(0) = middle_to_back_wheel_distance;
    lr(1) = 0;
    lr(2) = 0;

    Eigen::Vector3d w;
    w(0) = 0;
    w(1) = -wheel_track / 2.0;
    w(2) = 0;

    Eigen::Vector3d v;
    v(0) = twist.linear.x;
    v(1) = twist.linear.y;
    v(2) = 0;

    Eigen::Vector3d omega;
    omega(0) = 0;
    omega(1) = 0;
    omega(2) = twist.angular.z;

    Eigen::Vector3d r, rf, rm, rr;

    if (v.squaredNorm() == 0 && omega.squaredNorm() == 0) {
      deltaF = 0;
      deltaM = 0;
      deltaR = 0;
    } else {
      if (omega.squaredNorm() != 0) {
        r = omega.cross(v) / (omega.squaredNorm());
        rf = r + w + lm + lf;
        rm = r + w + lm;
        rr = r + w + lm + lr;
      } else {
        omega(0) = 0;
        omega(1) = 0;
        omega(2) = 1;
        r = omega.cross(v);
        rf = r;
        rm = r;
        rr = r;
      }
      deltaF = std::atan2(rf(1), rf(0)) - M_PI_2;
      deltaM = std::atan2(rm(1), rm(0)) - M_PI_2;
      deltaR = std::atan2(rr(1), rr(0)) - M_PI_2;
    }

    if (debug) {
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "holonomic_rover_kinematics debug: \n"
              << "twist.x: " << twist.linear.x << "\n"
              << "twist.y: " << twist.linear.y << "\n"
              << "twist.z: " << twist.angular.z << "\n"
              << "r: " << r << "\n"
              << "rr: " << rr << "\n"
              << "rm: " << rm << "\n"
              << "rf: " << rf
              << "\n"
              /* << "V: " << V << "\n" */
              /* << "angle_v : " << angle_v * 180 / M_PI << "\n" */
              /* << "normal_v : " << normal_v * 180 / M_PI << "\n" */
              /* << "r: " << r << "\n" */
              << "deltaF: " << deltaF * 180 / M_PI << "\n"
              << "deltaR: " << deltaR * 180 / M_PI << "\n");
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HolonomicRoverKinematics>());
  rclcpp::shutdown();
  return 0;
}
