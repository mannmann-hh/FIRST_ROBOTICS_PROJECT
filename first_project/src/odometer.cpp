// odometer.cpp
// First Robotics Project - Node that computes wheel odometry from /bunker_status
// using linear and angular velocity (partial encoder data) and publishes it
// as /project_odom and as the TF odom->base_link2.
//
// Integration: Runge-Kutta 2nd order (midpoint method) for higher accuracy
// than simple Euler integration when the robot turns.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
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
    first_msg_(true)
  {
    // Subscriber: robot status (custom msg containing v and w)
    sub_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
      "/bunker_status", 10,
      std::bind(&Odometer::statusCallback, this, std::placeholders::_1));

    // Publisher: computed odometry
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/project_odom", 10);

    // TF broadcaster: odom -> base_link2
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // Service: reset the odometry to zero
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&Odometer::resetCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
                "Odometer node started. Publishing /project_odom and tf odom->base_link2");
  }

private:
  void statusCallback(const bunker_msgs::msg::BunkerStatus::SharedPtr msg)
  {
    // Use the timestamp from the message header (bag was fixed to populate this)
    rclcpp::Time current_time(msg->header.stamp);

    if (first_msg_) {
      last_time_ = current_time;
      first_msg_ = false;
      // Publish an initial zero odometry so consumers have something
      publishOdom(current_time, 0.0, 0.0);
      publishTF(current_time);
      return;
    }

    // Compute time delta
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Guard against bad dt (e.g. after reset, bag loop, or clock jump)
    if (dt <= 0.0 || dt > 1.0) {
      RCLCPP_WARN(this->get_logger(), "Skipping update: dt = %.3f s", dt);
      publishOdom(current_time, msg->linear_velocity, msg->angular_velocity);
      publishTF(current_time);
      return;
    }

    // Extract v and w (partial encoder data: the robot's low-level
    // controller already fuses left/right track speeds into these)
    const double v = msg->linear_velocity;
    const double w = msg->angular_velocity;

    // Runge-Kutta 2nd order (midpoint) integration of the differential-drive
    // kinematic model:
    //   theta_mid = theta + w * dt / 2
    //   x_new = x + v * dt * cos(theta_mid)
    //   y_new = y + v * dt * sin(theta_mid)
    //   theta_new = theta + w * dt
    const double theta_mid = theta_ + w * dt / 2.0;
    x_     += v * dt * std::cos(theta_mid);
    y_     += v * dt * std::sin(theta_mid);
    theta_ += w * dt;

    // Keep theta in [-pi, pi] (optional, not required)
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
    const std::shared_ptr<std_srvs::srv::Empty::Request>  /*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response>       /*res*/)
  {
    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    RCLCPP_INFO(this->get_logger(), "Odometry has been reset to (0, 0, 0)");
  }

  // State
  double x_, y_, theta_;
  bool first_msg_;
  rclcpp::Time last_time_;

  // ROS interface
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometer>());
  rclcpp::shutdown();
  return 0;
}