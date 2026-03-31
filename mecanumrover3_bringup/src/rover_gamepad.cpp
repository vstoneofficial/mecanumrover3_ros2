#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class RoverGamepad : public rclcpp::Node
{
public:
  RoverGamepad() : Node("rover_gamepad"), last_time_(this->now())
  {
    // QoS settings
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", qos, std::bind(&RoverGamepad::joy_callback, this, _1));

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("rover_twist", qos);

    // 100Hz timer
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&RoverGamepad::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Rover Gamepad node started (Mecanum)");
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_time_ = this->now();

    current_cmd_ = geometry_msgs::msg::Twist();

    // -------------------------------
    // Axis definitions
    // -------------------------------
    const int AXIS_LEFT_X  = 0;   // left stick horizontal
    const int AXIS_LEFT_Y  = 1;   // left stick vertical
    const int AXIS_RIGHT_X = 3;   // right stick horizontal
    const int AXIS_RIGHT_Y = 4;   // right stick vertical

    const int AXIS_DPAD_X = 6;
    const int AXIS_DPAD_Y = 7;

    const int BTN_SQUARE   = 3;
    const int BTN_CROSS    = 0;
    const int BTN_CIRCLE   = 1;
    const int BTN_TRIANGLE = 2;

    const int BTN_L1 = 4;
    const int BTN_R1 = 5;
    const int BTN_L2 = 6;
    const int BTN_R2 = 7;

    // -------------------------------
    // Base speeds (from ESP32)
    // -------------------------------
    double XS_MICRO = 0.1;  // 100
    double XS_SLOW  = 0.3;  // 300
    double ZS_MICRO = 0.5;  // 500

    // Stick scale (speed mode)
    double stick_lin_scale  = 1.0;
    double stick_rot_scale  = 1.0;

    // Shoulder-button speed modes
    if (msg->buttons[BTN_R1]) { stick_lin_scale = 0.5; stick_rot_scale = 1.04; }
    if (msg->buttons[BTN_L1]) { stick_lin_scale = 0.8; stick_rot_scale = 1.57; }
    if (msg->buttons[BTN_R2]) { stick_lin_scale = 1.0; stick_rot_scale = 2.10; }
    if (msg->buttons[BTN_L2]) { stick_lin_scale = 1.5; stick_rot_scale = 2.50; }

    // -------------------------------
    // Symbol buttons
    // -------------------------------
    if (msg->buttons[BTN_TRIANGLE]) current_cmd_.linear.x += XS_MICRO;
    if (msg->buttons[BTN_CROSS])    current_cmd_.linear.x -= XS_MICRO;
    if (msg->buttons[BTN_SQUARE])   current_cmd_.angular.z += ZS_MICRO;
    if (msg->buttons[BTN_CIRCLE])   current_cmd_.angular.z -= ZS_MICRO;

    // -------------------------------
    // D-pad movement
    // -------------------------------
    if (msg->axes[AXIS_DPAD_Y] > 0.5)  current_cmd_.linear.x += XS_SLOW;
    if (msg->axes[AXIS_DPAD_Y] < -0.5) current_cmd_.linear.x -= XS_SLOW;

    // left/right d-pad does NOT strafe (linear.y=0)
    if (msg->axes[AXIS_DPAD_X] < -0.5) current_cmd_.linear.y -= ZS_MICRO;
    if (msg->axes[AXIS_DPAD_X] > 0.5)  current_cmd_.linear.y += ZS_MICRO;

    // -------------------------------
    // Analog sticks (FACT: both sticks behave identically)
    //
    //   Up/Down  → forward/backward (linear.x)
    //   Left/Right → rotation (angular.z)
    //
    //   active ONLY if L1/L2/R1/R2 pressed
    // -------------------------------

    bool stick_enabled =
      msg->buttons[BTN_L1] || msg->buttons[BTN_L2] ||
      msg->buttons[BTN_R1] || msg->buttons[BTN_R2];

    if (stick_enabled)
    {
      double left_forward  =  msg->axes[AXIS_LEFT_Y]  * stick_lin_scale;
      double left_strafe   =  msg->axes[AXIS_LEFT_X]  * stick_rot_scale;

      double right_forward =  msg->axes[AXIS_RIGHT_Y] * stick_lin_scale;
      double right_rotate  =  msg->axes[AXIS_RIGHT_X] * stick_rot_scale;

      // Combine
      current_cmd_.linear.x  += left_forward + right_forward;
      current_cmd_.linear.y  += left_strafe;
      current_cmd_.angular.z += right_rotate;
    }
  }

  void timer_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Safety stop after 1 sec timeout
    rclcpp::Time now = this->now();
    if ((now - last_time_).seconds() > 1.0) {
      current_cmd_.linear.x = 0.0;
      current_cmd_.angular.z = 0.0;
    }

    twist_pub_->publish(current_cmd_);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist current_cmd_;
  rclcpp::Time last_time_;
  std::mutex mutex_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RoverGamepad>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

