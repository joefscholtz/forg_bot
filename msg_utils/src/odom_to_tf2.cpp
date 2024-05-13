#include <functional>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class OdomToTf2 : public rclcpp::Node {
public:
  OdomToTf2() : Node("odom_to_tf2") {
    topic_name_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    use_odom_frames_ = this->declare_parameter<bool>("use_odom_frames", true);
    frame_id_ = this->declare_parameter<std::string>("frame_id", "odom");
    child_frame_id_ =
        this->declare_parameter<std::string>("child_frame_id", "base_link");

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic_name_, 10,
        std::bind(&OdomToTf2::odom_callback, this, std::placeholders::_1));
  }

private:
  void odom_callback(const std::shared_ptr<nav_msgs::msg::Odometry> msg) {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = msg->header.stamp;

    if (use_odom_frames_) {
      t.header.frame_id = msg->header.frame_id;
      t.child_frame_id = msg->child_frame_id;
    } else {
      t.header.frame_id = frame_id_.c_str();
      t.child_frame_id = child_frame_id_.c_str();
    }

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;

    t.transform.rotation = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string topic_name_, frame_id_, child_frame_id_;
  bool use_odom_frames_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToTf2>());
  rclcpp::shutdown();
  return 0;
}
