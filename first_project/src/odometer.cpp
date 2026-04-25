// odometer.cpp
// First Robotics Project - Politecnico di Milano - Robotics
//
// Computes wheel odometry of the AgileX Bunker Pro tracked robot.
//
// Strategy:
//   1. The first /odom message from the bag is used to copy the initial
//      pose (x, y, yaw) of the ground-truth base_link into our state.
//      After that we IGNORE /odom completely - we don't trace it.
//   2. From every /bunker_status message we read the per-track RPM
//      (the duplicated zero entry of motor[0] is filtered out as noted
//      in the project assignment, slide "Additions").
//   3. Each track speed is converted to a linear speed:
//          v_track = RPM * 2*pi*R / 60
//   4. The body twist (v, w) is obtained with the differential / skid-
//      steer model:
//          v = (v_right + v_left) / 2
//          w = (v_right - v_left) / track_width
//   5. The pose is integrated with Runge-Kutta 2nd order (midpoint):
//          theta_mid = yaw + w*dt/2
//          x_new   = x + v*dt*cos(theta_mid)
//          y_new   = y + v*dt*sin(theta_mid)
//          yaw_new = yaw + w*dt
//   6. The pose is published on /project_odom and as the TF
//      odom -> base_link2.
//   7. The /reset service (std_srvs/Empty) zeroes (x, y, yaw).

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
    // ----- ROS parameters (overridable from the launch file) ---------------
    //
    //   wheel_radius : effective rolling radius of the track sprocket [m]
    //   track_width  : distance between the two track centres        [m]
    //                  Spec sheet: 0.785 (overall width) - 0.230
    //                  (track width) = 0.555 m. Tracked robots also have
    //                  some slip so a slightly larger value can give a
    //                  better match if turns look too tight; tweak in
    //                  the launch file if needed.
    //   rpm_scale    : multiplicative factor in case the driver scales
    //                  the reported RPM (default 1.0).
    //
    // The three boolean flags help fix sign conventions if the trajectory
    // looks mirrored or rotates the wrong way.
    wheel_radius_    = this->declare_parameter<double>("wheel_radius",    0.085);
    track_width_     = this->declare_parameter<double>("track_width",     0.635);
    rpm_scale_       = this->declare_parameter<double>("rpm_scale",       1.0);
    invert_left_     = this->declare_parameter<bool>("invert_left",       false);
    invert_right_    = this->declare_parameter<bool>("invert_right",      false);
    swap_left_right_ = this->declare_parameter<bool>("swap_left_right",   false);

    // ----- Publishers / broadcaster ---------------------------------------
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/project_odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ----- Subscribers ----------------------------------------------------
    //
    // Subscribe to /odom only to copy the FIRST message as the initial
    // pose. After that, the callback returns immediately.
    odom_init_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&OdometerNode::odomInitCallback, this, std::placeholders::_1));

    status_sub_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
      "/bunker_status", 10,
      std::bind(&OdometerNode::statusCallback, this, std::placeholders::_1));

    // ----- Service --------------------------------------------------------
    reset_srv_ = this->create_service<std_srvs::srv::Empty>(
      "reset",
      std::bind(&OdometerNode::resetCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(),
      "odometer started.  wheel_radius=%.4f m, track_width=%.4f m",
      wheel_radius_, track_width_);
  }

private:
  // -------------------------------------------------------------- helpers
  static double normalize_angle(double angle)
  {
    while (angle >  M_PI) { angle -= 2.0 * M_PI; }
    while (angle < -M_PI) { angle += 2.0 * M_PI; }
    return angle;
  }

  // ------------------------------------------- initial-pose alignment cb
  void odomInitCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (got_initial_pose_) {
      return;
    }

    x_ = msg->pose.pose.position.x;
    y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    yaw_ = normalize_angle(yaw);

    got_initial_pose_ = true;

    RCLCPP_INFO(this->get_logger(),
      "initial pose from /odom: x=%.3f y=%.3f yaw=%.3f rad",
      x_, y_, yaw_);
  }

  // -------------------------------------- pick the two real per-track RPMs
  //
  // Project notes ("Additions" slide):
  //   "the agile-x driver duplicates motor[0] - one entry has the
  //    correct value, one is always zero. You can filter the zero one."
  //
  // We iterate over actuator_states (BunkerActuatorState[N]) and:
  //   - keep the first non-zero RPM with motor_id == 0  (RIGHT track)
  //   - keep the entry with motor_id == 1               (LEFT  track)
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

    // Default convention (verified empirically on the bag samples):
    //   motor_id 0 = RIGHT track,  motor_id 1 = LEFT track.
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

  // ----------------------------------------------------- main odom callback
  void statusCallback(const bunker_msgs::msg::BunkerStatus::SharedPtr msg)
  {
    // Wait for /odom to align our starting pose with base_link.
    if (!got_initial_pose_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "waiting for first /odom message to initialize pose...");
      return;
    }

    // We use this->now(): with use_sim_time=True and bag --clock, this
    // returns the bag's simulated time, which is consistent with the
    // timestamps inside the messages.
    const rclcpp::Time current_time = this->now();

    if (!initialized_time_) {
      last_time_ = current_time;
      initialized_time_ = true;
      // Publish the initial (aligned) pose so RViz immediately shows it
      publishOdometry(current_time);
      publishTF(current_time);
      return;
    }

    const double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Guard against bad dt (clock jump, bag restart, etc.)
    if (dt <= 0.0 || dt > 1.0) {
      return;
    }

    // ---- get the two track RPMs (filter the zero duplicate) ----
    double left_rpm  = 0.0;
    double right_rpm = 0.0;
    if (!extractLeftRightRpm(msg, left_rpm, right_rpm)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "no actuator_states in /bunker_status");
      return;
    }

    // ---- RPM -> track linear speed ----
    //   v_track = RPM * 2*pi*R / 60
    const double left_speed  = left_rpm  * 2.0 * M_PI * wheel_radius_ / 60.0;
    const double right_speed = right_rpm * 2.0 * M_PI * wheel_radius_ / 60.0;

    // ---- Differential / skid-steer kinematics ----
    //   v = (v_r + v_l) / 2
    //   w = (v_r - v_l) / track_width
    linear_vel_  = (right_speed + left_speed) / 2.0;
    angular_vel_ = (right_speed - left_speed) / track_width_;

    // ---- Runge-Kutta 2nd order (midpoint) integration ----
    //   yaw_mid = yaw + w*dt/2
    //   x   <- x + v*dt*cos(yaw_mid)
    //   y   <- y + v*dt*sin(yaw_mid)
    //   yaw <- yaw + w*dt
    const double yaw_mid = yaw_ + angular_vel_ * dt / 2.0;
    x_   += linear_vel_ * std::cos(yaw_mid) * dt;
    y_   += linear_vel_ * std::sin(yaw_mid) * dt;
    yaw_  = normalize_angle(yaw_ + angular_vel_ * dt);

    publishOdometry(current_time);
    publishTF(current_time);
  }

  // ------------------------------------------------ publish nav_msgs/Odom
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

  // -------------------------------------------- publish TF odom->base_link2
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

  // ----------------------------------------------------- /reset service cb
  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    // Required behaviour: reset position and orientation to zero.
    x_   = 0.0;
    y_   = 0.0;
    yaw_ = 0.0;
    linear_vel_  = 0.0;
    angular_vel_ = 0.0;
    initialized_time_ = false;

    // Note: we DO NOT reset got_initial_pose_ to false here, otherwise
    // the next /odom message would re-align us back to the GT pose
    // and the reset would not stay at zero.
    RCLCPP_INFO(this->get_logger(), "odometry reset to (0, 0, 0)");
  }

  // -------------------------------------------------------------- members
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr        odom_init_sub_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr status_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr           odom_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr                reset_srv_;
  std::unique_ptr<tf2_ros::TransformBroadcaster>                  tf_broadcaster_;

  rclcpp::Time last_time_;

  // pose state
  double x_;
  double y_;
  double yaw_;

  // last computed body twist (cached for /project_odom publication)
  double linear_vel_;
  double angular_vel_;

  // robot kinematic parameters
  double wheel_radius_;
  double track_width_;
  double rpm_scale_;
  bool   invert_left_;
  bool   invert_right_;
  bool   swap_left_right_;

  // bookkeeping
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
