// odometer.cpp - FINAL FIXED VERSION
// 修复策略：
// 1. 用 tf2_ros::Buffer + TransformListener 来获取 odom->base_link 初始位姿
//    （它内部使用正确的 QoS 配置，能与 ros2 bag 发布的 /tf 兼容）
// 2. 在每次 statusCallback 中尝试初始化（直到成功），避免 ros2 bag play
//    启动时机和节点启动时机不同步导致错过 /tf 消息

#include <algorithm>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "bunker_msgs/msg/bunker_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


class OdometerNode : public rclcpp::Node
{
public:
  OdometerNode()
  : Node("odometer"),
    x_(0.0),
    y_(0.0),
    yaw_(0.0),
    linear_vel_(0.0),
    angular_vel_(0.0),
    initialized_time_(false),
    got_initial_pose_(false)
  {
    wheel_radius_    = this->declare_parameter<double>("wheel_radius",    0.085);
    track_width_     = this->declare_parameter<double>("track_width",     0.635);
    rpm_scale_       = this->declare_parameter<double>("rpm_scale",       1.0);
    invert_left_     = this->declare_parameter<bool>("invert_left",       false);
    invert_right_    = this->declare_parameter<bool>("invert_right",      false);
    swap_left_right_ = this->declare_parameter<bool>("swap_left_right",   false);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/project_odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ========== 关键修改 1：使用 tf2_ros::Buffer + TransformListener ==========
    // 这会用正确的 QoS 配置自动订阅 /tf 和 /tf_static
    // 兼容 bag 中 RELIABLE + TRANSIENT_LOCAL 的 QoS
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    status_sub_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
      "/bunker_status", 10,
      std::bind(&OdometerNode::statusCallback, this, std::placeholders::_1));

    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&OdometerNode::resetCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
      "===== ODOMETER STARTED (TF-INIT VERSION) =====");
    RCLCPP_INFO(this->get_logger(),
      "wheel_radius=%.4f m, track_width=%.4f m",
      wheel_radius_, track_width_);
    RCLCPP_INFO(this->get_logger(),
      "will look up odom->base_link from /tf to initialize pose");
  }

private:
  static double normalize_angle(double angle)
  {
    while (angle >  M_PI) { angle -= 2.0 * M_PI; }
    while (angle < -M_PI) { angle += 2.0 * M_PI; }
    return angle;
  }

  // ========== 关键修改 2：在每次 status 回调时尝试初始化 ==========
  // 用 tf_buffer_->lookupTransform 主动查询，避免订阅时序问题
  bool tryInitializePoseFromTF()
  {
    if (got_initial_pose_) {
      return true;
    }

    try {
      // 查询最新的 odom -> base_link 变换
      auto tf = tf_buffer_->lookupTransform(
        "odom", "base_link", tf2::TimePointZero);

      x_ = tf.transform.translation.x;
      y_ = tf.transform.translation.y;

      tf2::Quaternion q(
        tf.transform.rotation.x,
        tf.transform.rotation.y,
        tf.transform.rotation.z,
        tf.transform.rotation.w);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      yaw_ = normalize_angle(yaw);

      got_initial_pose_ = true;

      RCLCPP_INFO(this->get_logger(),
        ">>> INIT POSE FROM /tf (odom->base_link): "
        "x=%.3f y=%.3f yaw=%.3f rad (%.1f deg)",
        x_, y_, yaw_, yaw_ * 180.0 / M_PI);
      return true;
    } catch (const tf2::TransformException & ex) {
      // /tf 还没收到，等下次再试
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "waiting for odom->base_link from /tf: %s", ex.what());
      return false;
    }
  }

  bool extractLeftRightRpm(
    const bunker_msgs::msg::BunkerStatus::SharedPtr msg,
    double & left_rpm,
    double & right_rpm)
  {
    if (msg->actuator_states.empty()) {
      return false;
    }

    double rpm_id0 = 0.0;
    double rpm_id1 = 0.0;
    bool   have_real_id0 = false;

    for (const auto & act : msg->actuator_states) {
      const double rpm = static_cast<double>(act.rpm);
      if (act.motor_id == 0) {
        if (!have_real_id0 || rpm != 0.0) {
          rpm_id0 = rpm;
          if (rpm != 0.0) {
            have_real_id0 = true;
          }
        }
      } else if (act.motor_id == 1) {
        rpm_id1 = rpm;
      }
    }

    right_rpm = rpm_id0;
    left_rpm  = rpm_id1;

    if (swap_left_right_) {
      std::swap(left_rpm, right_rpm);
    }

    left_rpm  *= rpm_scale_;
    right_rpm *= rpm_scale_;

    if (invert_left_)  { left_rpm  = -left_rpm;  }
    if (invert_right_) { right_rpm = -right_rpm; }

    return true;
  }

  void statusCallback(const bunker_msgs::msg::BunkerStatus::SharedPtr msg)
  {
    // ========== 关键修改 3：每次回调都尝试初始化，直到成功 ==========
    if (!tryInitializePoseFromTF()) {
      return;
    }

    const rclcpp::Time current_time = this->now();

    if (!initialized_time_) {
      last_time_ = current_time;
      initialized_time_ = true;
      // 立即发布初始位姿（这样起点就显示在GT位置）
      publishOdometry(current_time);
      publishTF(current_time);
      return;
    }

    const double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    double left_rpm  = 0.0;
    double right_rpm = 0.0;
    if (!extractLeftRightRpm(msg, left_rpm, right_rpm)) {
      return;
    }

    const double left_speed  = left_rpm  * 2.0 * M_PI * wheel_radius_ / 60.0;
    const double right_speed = right_rpm * 2.0 * M_PI * wheel_radius_ / 60.0;

    linear_vel_  = (right_speed + left_speed) / 2.0;
    angular_vel_ = (right_speed - left_speed) / track_width_;

    const double yaw_mid = yaw_ + angular_vel_ * dt / 2.0;
    x_   += linear_vel_ * std::cos(yaw_mid) * dt;
    y_   += linear_vel_ * std::sin(yaw_mid) * dt;
    yaw_  = normalize_angle(yaw_ + angular_vel_ * dt);

    publishOdometry(current_time);
    publishTF(current_time);
  }

  void publishOdometry(const rclcpp::Time & current_time)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp    = current_time;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id  = "base_link2";

    odom_msg.pose.pose.position.x = x_;
    odom_msg.pose.pose.position.y = y_;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x  = linear_vel_;
    odom_msg.twist.twist.angular.z = angular_vel_;

    odom_pub_->publish(odom_msg);
  }

  void publishTF(const rclcpp::Time & current_time)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = current_time;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id  = "base_link2";

    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    x_   = 0.0;
    y_   = 0.0;
    yaw_ = 0.0;
    linear_vel_  = 0.0;
    angular_vel_ = 0.0;
    initialized_time_ = false;
    RCLCPP_INFO(this->get_logger(), "odometry reset to (0, 0, 0)");
  }

  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           odom_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                reset_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                  tf_broadcaster_;

  // 用于查询 GT /tf 中的 odom->base_link
  std::unique_ptr<tf2_ros::Buffer>             tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener>  tf_listener_;

  rclcpp::Time last_time_;

  double x_, y_, yaw_;
  double linear_vel_, angular_vel_;
  double wheel_radius_, track_width_, rpm_scale_;
  bool invert_left_, invert_right_, swap_left_right_;
  bool initialized_time_;
  bool got_initial_pose_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometerNode>());
  rclcpp::shutdown();
  return 0;
}
