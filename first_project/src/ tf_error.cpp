// tf_error.cpp
// First Robotics Project - Node that compares our computed odometry TF
// (odom -> base_link2) with the ground-truth TF from the bag
// (odom -> base_link), and publishes the Euclidean error plus some
// auxiliary information on /tf_error_msg.

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <first_project/msg/tf_error_msg.hpp>

#include <cmath>
#include <memory>

class TfError : public rclcpp::Node
{
public:
  TfError()
  : Node("tf_error"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    travelled_distance_(0.0),
    has_prev_gt_(false),
    got_start_time_(false)
  {
    pub_ = this->create_publisher<first_project::msg::TfErrorMsg>(
      "/tf_error_msg", 10);

    // Poll the TF tree at 10 Hz; GT and our TF are both published at 50 Hz
    // so 10 Hz is enough to produce a clean error signal without spamming.
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TfError::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "tf_error node started. Publishing /tf_error_msg");
  }

private:
  void timerCallback()
  {
    geometry_msgs::msg::TransformStamped gt_tf, our_tf;

    // Look up both transforms at the latest available time.
    // We use TimePointZero so tf2 returns the most recent valid transform.
    try {
      gt_tf  = tf_buffer_.lookupTransform(
        "odom", "base_link",  tf2::TimePointZero);
      our_tf = tf_buffer_.lookupTransform(
        "odom", "base_link2", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Could not look up transform: %s", ex.what());
      return;
    }

    // Record the first GT timestamp as our "start time"
    rclcpp::Time gt_stamp(gt_tf.header.stamp);
    if (!got_start_time_) {
      start_time_ = gt_stamp;
      got_start_time_ = true;
    }

    // Euclidean distance between the two origins (2D)
    const double dx = gt_tf.transform.translation.x - our_tf.transform.translation.x;
    const double dy = gt_tf.transform.translation.y - our_tf.transform.translation.y;
    const double err = std::sqrt(dx * dx + dy * dy);

    // Accumulate the distance travelled along the GT path
    const double gx = gt_tf.transform.translation.x;
    const double gy = gt_tf.transform.translation.y;
    if (has_prev_gt_) {
      const double ddx = gx - prev_gx_;
      const double ddy = gy - prev_gy_;
      travelled_distance_ += std::sqrt(ddx * ddx + ddy * ddy);
    }
    prev_gx_ = gx;
    prev_gy_ = gy;
    has_prev_gt_ = true;

    // Seconds elapsed since the first GT sample
    const double elapsed = (gt_stamp - start_time_).seconds();

    // Fill and publish the custom message
    first_project::msg::TfErrorMsg m;
    m.header.stamp = gt_stamp;
    m.header.frame_id = "odom";
    m.tf_error = static_cast<float>(err);
    m.time_from_start = static_cast<int32_t>(elapsed);
    m.travelled_distance = static_cast<float>(travelled_distance_);
    pub_->publish(m);
  }

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // Publisher and timer
  rclcpp::Publisher<first_project::msg::TfErrorMsg>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // State for accumulating travelled distance
  double travelled_distance_;
  double prev_gx_, prev_gy_;
  bool has_prev_gt_;

  // Start time (first valid GT timestamp)
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