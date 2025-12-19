#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gazebo_msgs/msg/link_states.hpp"

class GazeboOdomBridge : public rclcpp::Node
{
public:
  GazeboOdomBridge()
  : Node("gazebo_odom_bridge")
  {
    // Parameter: base link name (default = base_footprint)
    this->declare_parameter<std::string>("base_link_name", "base_footprint");
    base_link_name_ = this->get_parameter("base_link_name").as_string();

    // Subscription to /link_states
    sub_link_states_ = this->create_subscription<gazebo_msgs::msg::LinkStates>(
      "/link_states",
      rclcpp::SystemDefaultsQoS(),
      std::bind(&GazeboOdomBridge::callbackLinkStates, this, std::placeholders::_1));

    // Publisher for /odom
    pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // TF broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "gazebo_odom_bridge started.");
  }

private:

  // Remove "xxx::" prefix from Gazebo link names
  std::string stripPrefix(const std::string &name)
  {
    auto pos = name.find("::");
    if (pos != std::string::npos)
      return name.substr(pos + 2);
    return name;
  }

  void callbackLinkStates(const gazebo_msgs::msg::LinkStates::SharedPtr msg)
  {
    int index = -1;

    // Find matching link index
    for (size_t i = 0; i < msg->name.size(); i++) {
      if (stripPrefix(msg->name[i]) == base_link_name_) {
        index = i;
        break;
      }
    }

    if (index < 0) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "base_link not found in /link_states");
      return;
    }

    // Odometry message
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";

    odom.pose.pose.position = msg->pose[index].position;
    odom.pose.pose.orientation = msg->pose[index].orientation;

    odom.twist.twist.linear = msg->twist[index].linear;
    odom.twist.twist.angular = msg->twist[index].angular;

    pub_odom_->publish(odom);

    // TF transform
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = odom.header.stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_footprint";
    tf.transform.translation.x = msg->pose[index].position.x;
    tf.transform.translation.y = msg->pose[index].position.y;
    tf.transform.translation.z = msg->pose[index].position.z;
    tf.transform.rotation = msg->pose[index].orientation;

    tf_broadcaster_->sendTransform(tf);
  }

  std::string base_link_name_;
  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr sub_link_states_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GazeboOdomBridge>());
  rclcpp::shutdown();
  return 0;
}

