/**
 * メカナムローバーのオドメトリ（位置や姿勢の推定値）情報とTF情報をパブリッシュするためのノードです。
 * 詳細は：http://wiki.ros.org/ja/navigation/Tutorials/RobotSetup/Odom
 * 
*/

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

class PubOdomNode : public rclcpp::Node
{
public:
  PubOdomNode()
  : Node("odometry_publisher")
  {
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(1));

    // publish odometry data and tf transform every 10ms (=100hz)
    timer_ = this->create_wall_timer(
      10ms, std::bind(&PubOdomNode::timer_callback, this));

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "rover_odo", rclcpp::SensorDataQoS(), std::bind(&PubOdomNode::rover_odom_callback, this, _1));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  }

private:
  void timer_callback()
  {
    auto msg = nav_msgs::msg::Odometry();
    
    // Convert tf2::Quaternion to geometry_msgs::msg::Quaternion
    geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg(q);

    //next, we'll publish the odometry message over ROS
    msg.header.stamp = current_time;
    msg.header.frame_id = "odom";
 
    //set the position
    msg.pose.pose.position.x = x; 
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = odom_quat;

    //set the velocity
    msg.child_frame_id = "base_footprint";
    msg.twist.twist.linear.x = vx;
    msg.twist.twist.linear.y = vy;
    msg.twist.twist.angular.z = vth;

    publisher_->publish(msg);
    tf_broadcaster_->sendTransform(t);
  }

  void rover_odom_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
  {  
    vx = odom_kvx * msg->linear.x;
    vy = odom_kvy * msg->linear.y;
    vth = odom_kth * msg->angular.z;

    current_time = this->get_clock()->now();
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).seconds();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = current_time;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_footprint";

    t.transform.translation.x = x;
    t.transform.translation.y = y;
    t.transform.translation.z = 0.0;

    q.setRPY(0, 0, th);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    last_time = current_time;
  }

  void calculate_time()
  {
    last_time = current_time;

    current_time = this->get_clock()->now();
  }

  double vx =  0.0;
  double vy =  0.0;
  double vth = 0.0;
  double odom_kvx = 1.0;
  double odom_kvy = 1.0;
  double odom_kth = 1.0;

  rclcpp::Time current_time = this->get_clock()->now();
  rclcpp::Time last_time = this->get_clock()->now();
  geometry_msgs::msg::TransformStamped t;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  tf2::Quaternion q;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PubOdomNode>());
  rclcpp::shutdown();
  return 0;
}
