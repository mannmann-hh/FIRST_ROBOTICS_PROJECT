cat > ~/colcon_ws/src/first_project/src/odometer.cpp << 'ODOMETER_EOF'
// odometer.cpp - First Robotics Project - Politecnico di Milano
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "bunker_msgs/msg/bunker_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using std::placeholders::_1;
using std::placeholders::_2;

class Odometer : public rclcpp::Node
{
public:
  Odometer()
  : Node("odometer"), x_(0.0), y_(0.0), yaw_(0.0),
    initialized_pose_(false), initialized_time_(false)
  {
    wheel_radius_ = this->declare_parameter<double>("wheel_radius", 0.085);
    track_width_  = this->declare_parameter<double>("track_width",  0.555);
    rpm_scale_    = this->declare_parameter<double>("rpm_scale",    1.0);
    invert_left_     = this->declare_parameter<bool>("invert_left",     false);
    invert_right_    = this->declare_parameter<bool>("invert_right",    false);
    swap_left_right_ = this->declare_parameter<bool>("swap_left_right", false);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/project_odom", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // KEY FIX: Subscribe to /odom to align initial pose with GT
    gt_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&Odometer::gt_odom_callback, this, _1));

    status_sub_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
      "/bunker_status", 10, std::bind(&Odometer::status_callback, this, _1));

    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset", std::bind(&Odometer::reset_callback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(),
      "odometer started.  wheel_radius=%.4f m, track_width=%.4f m",
      wheel_radius_, track_width_);
  }

private:
  static double normalize_angle(double a) {
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
  }

  void gt_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (initialized_pose_) return;
    x_   = msg->pose.pose.position.x;
    y_   = msg->pose.pose.position.y;
    yaw_ = normalize_angle(tf2::getYaw(msg->pose.pose.orientation));
    initialized_pose_ = true;
    RCLCPP_INFO(this->get_logger(),
      "Initial pose aligned from /odom: x=%.3f, y=%.3f, yaw=%.3f rad",
      x_, y_, yaw_);
  }

  rclcpp::Time get_message_time(const bunker_msgs::msg::BunkerStatus::SharedPtr msg) {
    rclcpp::Time s(msg->header.stamp);
    if (s.nanoseconds() == 0) s = this->now();
    return s;
  }

  bool extract_left_right_rpm(const bunker_msgs::msg::BunkerStatus::SharedPtr msg,
                              double & left_rpm, double & right_rpm) {
    double rpm_id0 = 0.0, rpm_id1 = 0.0;
    bool have0 = false;
    for (const auto & act : msg->actuator_states) {
      const double rpm = static_cast<double>(act.rpm);
      if (act.motor_id == 0) {
        if (!have0 || rpm != 0.0) { rpm_id0 = rpm; if (rpm != 0.0) have0 = true; }
      } else if (act.motor_id == 1) {
        rpm_id1 = rpm;
      }
    }
    if (msg->actuator_states.empty()) return false;
    right_rpm = rpm_id0;
    left_rpm  = rpm_id1;
    if (swap_left_right_) std::swap(left_rpm, right_rpm);
    left_rpm  *= rpm_scale_;
    right_rpm *= rpm_scale_;
    if (invert_left_)  left_rpm  = -left_rpm;
    if (invert_right_) right_rpm = -right_rpm;
    return true;
  }

  void status_callback(const bunker_msgs::msg::BunkerStatus::SharedPtr msg) {
    if (!initialized_pose_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Waiting for first /odom message to align the initial pose ...");
      return;
    }
    const rclcpp::Time current_time = get_message_time(msg);
    if (!initialized_time_) {
      last_time_ = current_time;
      initialized_time_ = true;
      publish_odometry_and_tf(current_time, 0.0, 0.0);
      return;
    }
    const double dt = (current_time - last_time_).seconds();
    if (dt <= 0.0 || dt > 1.0) { last_time_ = current_time; return; }

    double left_rpm = 0.0, right_rpm = 0.0;
    if (!extract_left_right_rpm(msg, left_rpm, right_rpm)) {
      last_time_ = current_time; return;
    }

    const double left_speed  = left_rpm  * 2.0 * M_PI * wheel_radius_ / 60.0;
    const double right_speed = right_rpm * 2.0 * M_PI * wheel_radius_ / 60.0;
    const double v = (right_speed + left_speed) / 2.0;
    const double w = (right_speed - left_speed) / track_width_;

    const double yaw_mid = yaw_ + w * dt / 2.0;
    x_   += v * std::cos(yaw_mid) * dt;
    y_   += v * std::sin(yaw_mid) * dt;
    yaw_  = normalize_angle(yaw_ + w * dt);

    publish_odometry_and_tf(current_time, v, w);
    last_time_ = current_time;
  }

  void publish_odometry_and_tf(const rclcpp::Time & stamp, double v, double w) {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    q.normalize();

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp     = stamp;
    odom_msg.header.frame_id  = "odom";
    odom_msg.child_frame_id   = "base_link2";
    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = tf2::toMsg(q);
    odom_msg.twist.twist.linear.x  = v;
    odom_msg.twist.twist.angular.z = w;
    odom_pub_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp     = stamp;
    t.header.frame_id  = "odom";
    t.child_frame_id   = "base_link2";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);
  }

  void reset_callback(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                      std::shared_ptr<std_srvs::srv::Empty::Response>) {
    x_ = 0.0; y_ = 0.0; yaw_ = 0.0;
    initialized_time_ = false;
    initialized_pose_ = true;
    RCLCPP_INFO(this->get_logger(), "Odometry reset to (0, 0, 0)");
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gt_odom_sub_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Time last_time_;
  double x_, y_, yaw_;
  double wheel_radius_, track_width_, rpm_scale_;
  bool invert_left_, invert_right_, swap_left_right_;
  bool initialized_pose_, initialized_time_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometer>());
  rclcpp::shutdown();
  return 0;
}
ODOMETER_EOF
echo "✓ odometer.cpp updated"