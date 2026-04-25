// tf_error.cpp
// First Robotics Project - Node that compares the ground-truth TF from the bag
// (odom -> base_link) with our computed odometry TF (odom -> base_link2),
// and publishes the Euclidean error on /tf_error_msg.

#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "first_project/msg/tf_error_msg.hpp"

class TfError : public rclcpp::Node
{
public:
  TfError()
  : Node("tf_error"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    travelled_distance_(0.0),
    prev_gx_(0.0),
    prev_gy_(0.0),
    has_prev_gt_(false),
    got_start_time_(false)
  {
    pub_ = this->create_publisher<first_project::msg::TfErrorMsg>(
      "/tf_error_msg",
      10
    );

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TfError::timerCallback, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "tf_error node started. Publishing /tf_error_msg"
    );
  }

private:
  void timerCallback()
  {
    geometry_msgs::msg::TransformStamped gt_tf;
    geometry_msgs::msg::TransformStamped our_tf;

    try {
      /*
        Required comparison:
        bag TF: odom -> base_link
        our TF: odom -> base_link2
      */
      gt_tf = tf_buffer_.lookupTransform(
        "odom",
        "base_link",
        tf2::TimePointZero
      );

      our_tf = tf_buffer_.lookupTransform(
        "odom",
        "base_link2",
        tf2::TimePointZero
      );
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Could not look up transform: %s",
        ex.what()
      );
      return;
    }

    /*
      Since the fixed bag is played with --clock and the launch file uses
      use_sim_time=True, this->now() is the bag/ROS time.
    */
    const rclcpp::Time now = this->now();

    if (!got_start_time_) {
      start_time_ = now;
      got_start_time_ = true;
    }

    const double gt_x = gt_tf.transform.translation.x;
    const double gt_y = gt_tf.transform.translation.y;

    const double our_x = our_tf.transform.translation.x;
    const double our_y = our_tf.transform.translation.y;

    /*
      Euclidean 2D distance between base_link and base_link2.
    */
    const double dx = gt_x - our_x;
    const double dy = gt_y - our_y;
    const double error = std::sqrt(dx * dx + dy * dy);

    /*
      Travelled distance is accumulated along the GT path.
      The small jump filter avoids adding unreasonable jumps caused by
      TF discontinuities or bag restarts.
    */
    if (has_prev_gt_) {
      const double step_x = gt_x - prev_gx_;
      const double step_y = gt_y - prev_gy_;
      const double step = std::sqrt(step_x * step_x + step_y * step_y);

      if (step < 1.0) {
        travelled_distance_ += step;
      }
    }

    prev_gx_ = gt_x;
    prev_gy_ = gt_y;
    has_prev_gt_ = true;

    const double elapsed = (now - start_time_).seconds();

    first_project::msg::TfErrorMsg msg;
    msg.header.stamp = now;
    msg.header.frame_id = "odom";
    msg.tf_error = static_cast<float>(error);
    msg.time_from_start = static_cast<int32_t>(elapsed);
    msg.travelled_distance = static_cast<float>(travelled_distance_);

    pub_->publish(msg);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Publisher<first_project::msg::TfErrorMsg>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double travelled_distance_;
  double prev_gx_;
  double prev_gy_;
  bool has_prev_gt_;

  rclcpp::Time start_time_;
  bool got_start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfError>());
  rclcpp::shutdown();

  return 0;
}