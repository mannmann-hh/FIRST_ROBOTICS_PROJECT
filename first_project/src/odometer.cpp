cat > ~/colcon_ws/src/first_project/src/odometer.cpp << 'ODOMETER_EOF'
// odometer.cpp - RPM-based odometry, initial pose synced with GT
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <std_srvs/srv/empty.hpp>
#include <bunker_msgs/msg/bunker_status.hpp>
#include <cmath>
#include <memory>

class Odometer : public rclcpp::Node
{
public:
  Odometer()
  : Node("odometer"),
    x_(0.0), y_(0.0), theta_(0.0),
    first_msg_(true),
    initial_pose_set_(false),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    rpm_to_mps_  = this->declare_parameter<double>("rpm_to_mps",  0.001172);
    track_width_ = this->declare_parameter<double>("track_width", 0.6876);

    sub_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
      "/bunker_status", 10,
      std::bind(&Odometer::statusCallback, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/project_odom", 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&Odometer::resetCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
      "Odometer started. rpm_to_mps=%.6f, track_width=%.4f m",
      rpm_to_mps_, track_width_);
  }

private:
  bool tryGetInitialPose()
  {
    try {
      auto t = tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
      x_ = t.transform.translation.x;
      y_ = t.transform.translation.y;
      theta_ = tf2::getYaw(t.transform.rotation);
      RCLCPP_INFO(this->get_logger(),
        "Initial pose synced with GT: x=%.3f, y=%.3f, theta=%.3f rad",
        x_, y_, theta_);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  void extractTrackRpms(const bunker_msgs::msg::BunkerStatus & msg,
                        int16_t & rpm_id0, int16_t & rpm_id1) const
  {
    rpm_id0 = 0;
    rpm_id1 = 0;
    bool found0 = false;
    for (const auto & act : msg.actuator_states) {
      if (act.motor_id == 0) {
        if (!found0 || act.rpm != 0) {
          rpm_id0 = act.rpm;
          if (act.rpm != 0) found0 = true;
        }
      } else if (act.motor_id == 1) {
        rpm_id1 = act.rpm;
      }
    }
  }

  void statusCallback(const bunker_msgs::msg::BunkerStatus::SharedPtr msg)
  {
    rclcpp::Time current_time(msg->header.stamp);

    // Try to align our initial pose with the GT's initial pose
    if (!initial_pose_set_) {
      if (tryGetInitialPose()) {
        initial_pose_set_ = true;
      }
    }

    int16_t rpm_right_raw, rpm_left_raw;
    extractTrackRpms(*msg, rpm_right_raw, rpm_left_raw);

    const double v_right = rpm_to_mps_ * static_cast<double>(rpm_right_raw);
    const double v_left  = rpm_to_mps_ * static_cast<double>(rpm_left_raw);
    const double v = (v_left + v_right) / 2.0;
    const double w = (v_right - v_left) / track_width_;

    if (first_msg_) {
      last_time_ = current_time;
      first_msg_ = false;
      publishOdom(current_time, v, w);
      publishTF(current_time);
      return;
    }

    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0.0 || dt > 1.0) {
      RCLCPP_WARN(this->get_logger(), "Skipping update: dt = %.3f s", dt);
      publishOdom(current_time, v, w);
      publishTF(current_time);
      return;
    }

    const double theta_mid = theta_ + w * dt / 2.0;
    x_     += v * dt * std::cos(theta_mid);
    y_     += v * dt * std::sin(theta_mid);
    theta_ += w * dt;

    while (theta_ >  M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    publishOdom(current_time, v, w);
    publishTF(current_time);
  }

  void publishOdom(const rclcpp::Time & stamp, double v, double w)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link2";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x  = v;
    odom.twist.twist.angular.z = w;
    odom_pub_->publish(odom);
  }

  void publishTF(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id  = "base_link2";
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }

  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    x_ = 0.0; y_ = 0.0; theta_ = 0.0;
    initial_pose_set_ = false;  // re-sync with GT next time
    RCLCPP_INFO(this->get_logger(), "Odometry has been reset to (0, 0, 0)");
  }

  double x_, y_, theta_;
  bool first_msg_;
  bool initial_pose_set_;
  rclcpp::Time last_time_;
  double rpm_to_mps_;
  double track_width_;

  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometer>());
  rclcpp::shutdown();
  return 0;
}
ODOMETER_EOF
echo "odometer updated! Now using track_width=0.6876, GT initial pose sync"