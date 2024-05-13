#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class StampTwist : public rclcpp::Node {
public:
  StampTwist() : Node("stamp_twist") {
    input_topic_name_ =
        this->declare_parameter<std::string>("input_topic_name", "/twist");
    output_topic_name_ = this->declare_parameter<std::string>(
        "output_topic_name", "/twist_stamped");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "map");

    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        output_topic_name_, 10);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        input_topic_name_, 10,
        std::bind(&StampTwist::twist_callback, this, std::placeholders::_1));
  }

private:
  void twist_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg) {
    geometry_msgs::msg::TwistStamped msg_stamped;

    msg_stamped.header.stamp = this->get_clock()->now();
    msg_stamped.header.frame_id = frame_id_;
    msg_stamped.twist = *msg;

    publisher_->publish(msg_stamped);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  std::string input_topic_name_, output_topic_name_, frame_id_;
  bool use_odom_frames_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StampTwist>());
  rclcpp::shutdown();
  return 0;
}
