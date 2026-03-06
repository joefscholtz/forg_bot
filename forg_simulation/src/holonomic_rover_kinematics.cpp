#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

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
    compute = this->declare_parameter<bool>("compute", true);

    // Topic Names
    joint_states_input_topic_name = this->declare_parameter<std::string>("joint_states_input_topic_name", "input_joint_states");
    joint_states_output_topic_name = this->declare_parameter<std::string>("joint_states_output_topic_name", "joint_states");
    twist_input_topic_name = this->declare_parameter<std::string>("twist_input_topic_name", "cmd_vel");
    markers_topic_name = this->declare_parameter<std::string>("markers_topic_name", "normal_vectors_to_wheel");

    // Frame and Joint Names (Left)
    l_front_w = this->declare_parameter<std::string>("left_front_wheel", "left_front_wheel");
    l_front_s = this->declare_parameter<std::string>("left_front_steering", "left_front_steering_joint");
    l_mid_w   = this->declare_parameter<std::string>("left_middle_wheel", "left_middle_wheel");
    l_mid_s   = this->declare_parameter<std::string>("left_middle_steering", "left_middle_steering_joint");
    l_rear_w  = this->declare_parameter<std::string>("left_rear_wheel", "left_rear_wheel");
    l_rear_s  = this->declare_parameter<std::string>("left_rear_steering", "left_rear_steering_joint");

    // Frame and Joint Names (Right)
    r_front_w = this->declare_parameter<std::string>("right_front_wheel", "right_front_wheel");
    r_front_s = this->declare_parameter<std::string>("right_front_steering", "right_front_steering_joint");
    r_mid_w   = this->declare_parameter<std::string>("right_middle_wheel", "right_middle_wheel");
    r_mid_s   = this->declare_parameter<std::string>("right_middle_steering", "right_middle_steering_joint");
    r_rear_w  = this->declare_parameter<std::string>("right_rear_wheel", "right_rear_wheel");
    r_rear_s  = this->declare_parameter<std::string>("right_rear_steering", "right_rear_steering_joint");

    // Geometry (Symmetric)
    dist_f = this->declare_parameter<double>("front_to_middle_distance", 1.0);
    dist_m = this->declare_parameter<double>("middle_to_base_distance", 1.0);
    dist_r = this->declare_parameter<double>("rear_to_middle_distance", 1.0);
    wheel_track = this->declare_parameter<double>("wheel_track", 1.0);

    if (compute) {
      joint_states_publisher = this->create_publisher<sensor_msgs::msg::JointState>(joint_states_output_topic_name, 10);
      joint_states_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
          joint_states_input_topic_name, 10, std::bind(&HolonomicRoverKinematics::joint_states_callback, this, std::placeholders::_1));
      twist_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
          twist_input_topic_name, 10, std::bind(&HolonomicRoverKinematics::twist_callback, this, std::placeholders::_1));
    }

    marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_name, 10);
    timer_ = this->create_wall_timer(1s, std::bind(&HolonomicRoverKinematics::timer_callback, this));
  }

private:
  bool debug, compute;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string joint_states_input_topic_name, joint_states_output_topic_name, twist_input_topic_name, markers_topic_name;
  
  // Frame/Joint strings
  std::string l_front_w, l_front_s, l_mid_w, l_mid_s, l_rear_w, l_rear_s;
  std::string r_front_w, r_front_s, r_mid_w, r_mid_s, r_rear_w, r_rear_s;

  double dist_f, dist_m, dist_r, wheel_track;
  double deltaLF{0}, deltaLM{0}, deltaLR{0};
  double deltaRF{0}, deltaRM{0}, deltaRR{0};

void timer_callback() {
    visualization_msgs::msg::MarkerArray markers;
    auto now = this->get_clock()->now();

    auto add_marker = [&](const std::string& frame, int id, float r, float g, float b, bool forward) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = now;
      m.header.frame_id = frame;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::LINE_LIST;
      m.scale.x = 0.02;
      m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.0;
      m.frame_locked = true;

      geometry_msgs::msg::Point p1, p2;
      p1.x = 0; p1.y = 0; p1.z = 0;
      p2.x = forward ? 10.0 : -10.0;
      p2.y = 0; p2.z = 0;

      m.points.push_back(p1);
      m.points.push_back(p2);
      markers.markers.push_back(m);
    };

    // --- LEFT SIDE ---
    // Lavender (Forward vector)
    add_marker(l_rear_w, 0, 0.588, 0.482, 0.714, true);
    add_marker(l_mid_w, 1, 0.588, 0.482, 0.714, true);
    add_marker(l_front_w, 2, 0.588, 0.482, 0.714, true);
    // Red (Backward vector)
    add_marker(l_rear_w, 3, 0.9, 0.0, 0.0, false);
    add_marker(l_mid_w, 4, 0.9, 0.0, 0.0, false);
    add_marker(l_front_w, 5, 0.9, 0.0, 0.0, false);

    // --- RIGHT SIDE ---
    // Cyan (Forward vector)
    add_marker(r_rear_w, 6, 0.0, 0.8, 0.8, true);
    add_marker(r_mid_w, 7, 0.0, 0.8, 0.8, true);
    add_marker(r_front_w, 8, 0.0, 0.8, 0.8, true);
    // Orange (Backward vector)
    add_marker(r_rear_w, 9, 1.0, 0.5, 0.0, false);
    add_marker(r_mid_w, 10, 1.0, 0.5, 0.0, false);
    add_marker(r_front_w, 11, 1.0, 0.5, 0.0, false);

    marker_array_publisher->publish(markers);
  }

  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    auto out_msg = *msg;
    for (size_t i = 0; i < out_msg.name.size(); i++) {
      if (out_msg.name[i] == l_front_s) out_msg.position[i] = deltaLF;
      else if (out_msg.name[i] == l_mid_s)   out_msg.position[i] = deltaLM;
      else if (out_msg.name[i] == l_rear_s)  out_msg.position[i] = deltaLR;
      else if (out_msg.name[i] == r_front_s) out_msg.position[i] = deltaRF;
      else if (out_msg.name[i] == r_mid_s)   out_msg.position[i] = deltaRM;
      else if (out_msg.name[i] == r_rear_s)  out_msg.position[i] = deltaRR;
    }
    joint_states_publisher->publish(out_msg);
  }

void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    auto twist = *msg;

    Eigen::Vector3d lf(-dist_f, 0, 0);
    Eigen::Vector3d lm(-dist_m, 0, 0);
    Eigen::Vector3d lr(dist_r, 0, 0);
    Eigen::Vector3d w(0, -wheel_track / 2.0, 0);

    Eigen::Vector3d v(twist.linear.x, twist.linear.y, 0);
    Eigen::Vector3d omega(0, 0, twist.angular.z);
    Eigen::Vector3d r, left_rf, left_rm, left_rr, right_rf, right_rm, right_rr;

    deltaLF = deltaLM = deltaLR = deltaRF = deltaRM = deltaRR = 0;
    if (v.squaredNorm() != 0 || omega.squaredNorm() != 0) {
      if (omega.squaredNorm() != 0) {
        r = omega.cross(v) / (omega.squaredNorm());
        left_rf  = r + w + lm + lf;
        left_rm  = r + w + lm;
        left_rr  = r + w + lm + lr;

        right_rf = r - w + lm + lf;
        right_rm = r - w + lm;
        right_rr = r - w + lm + lr;
      } else {
        omega << 0, 0, 1;
        r = omega.cross(v);
        left_rf = left_rm = left_rr = right_rf = right_rm = right_rr = r;
      }

      deltaLF = std::atan2(left_rf(1), left_rf(0)) - M_PI_2;
      deltaLM = std::atan2(left_rm(1), left_rm(0)) - M_PI_2;
      deltaLR = std::atan2(left_rr(1), left_rr(0)) - M_PI_2;
      deltaRF = std::atan2(right_rf(1), right_rf(0)) - M_PI_2;
      deltaRM = std::atan2(right_rm(1), right_rm(0)) - M_PI_2;
      deltaRR = std::atan2(right_rr(1), right_rr(0)) - M_PI_2;
    }

    if (debug) {
    RCLCPP_WARN_STREAM(
        this->get_logger(),
        "holonomic_rover_kinematics debug: \n"
            << "twist.x: " << msg->linear.x << "\n"
            << "twist.y: " << msg->linear.y << "\n"
            << "twist.z: " << msg->angular.z << "\n"
            << "r: " << r << "\n"
            << "w: " << w << "\n"
            << "lf: " << lf << "\n"
            << "lm: " << lm << "\n"
            << "lr: " << lr << "\n"
            << "left_rf: " << left_rf << "\n"
            << "left_rm: " << left_rm << "\n"
            << "left_rr: " << left_rr << "\n"
            << "right_rf: " << right_rf << "\n"
            << "right_rm: " << right_rm << "\n"
            << "right_rr: " << right_rr << "\n"
            << "\n"
            << "deltaLF: " << deltaLF * 180 / M_PI << "\n"
            << "deltaLM: " << deltaLM * 180 / M_PI << "\n"
            << "deltaLR: " << deltaLR * 180 / M_PI << "\n"
            << "deltaRF: " << deltaRF * 180 / M_PI << "\n"
            << "deltaRM: " << deltaRM * 180 / M_PI << "\n"
            << "deltaRR: " << deltaRR * 180 / M_PI );
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HolonomicRoverKinematics>());
  rclcpp::shutdown();
  return 0;
}
