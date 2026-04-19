#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/empty.hpp>
#include "bunker_msgs/msg/bunker_status.hpp"

class OdometerNode : public rclcpp::Node {
public:
    OdometerNode() : Node("odometer"), x_(0.0), y_(0.0), theta_(0.0) {
        // 1. 订阅机器人状态
        sub_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
            "/bunker_status", 10, std::bind(&OdometerNode::on_status_received, this, std::placeholders::_1));

        // 2. 发布里程计话题
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/project_odom", 10);

        // 3. 初始化 TF 广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 4. 实现 reset 服务
        reset_service_ = this->create_service<std_srvs::srv::Empty>(
            "reset", std::bind(&OdometerNode::handle_reset, this, std::placeholders::_1, std::placeholders::_2));

        last_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "Odometer Node has started.");
    }

private:
    void on_status_received(const bunker_msgs::msg::BunkerStatus::SharedPtr msg) {
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_time_).seconds();

        // 避免第一帧或时间戳错误
        if (dt <= 0) {
            last_time_ = current_time;
            return;
        }

        // FIX: 直接使用消息中已合成好的线速度和角速度
        // BunkerStatus 驱动层已完成运动学合成，无需手动用左右轮速计算
        double v = msg->linear_velocity;
        double w = msg->angular_velocity;

        // 航位推算 —— 欧拉积分
        x_     += v * std::cos(theta_) * dt;
        y_     += v * std::sin(theta_) * dt;
        theta_ += w * dt;

        // --- 1. 发布 Odometry 消息 ---
        nav_msgs::msg::Odometry odom;
        odom.header.stamp    = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_link2";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x  = v;
        odom.twist.twist.angular.z = w;
        odom_pub_->publish(odom);

        // --- 2. 发布 TF 变换 (odom -> base_link2) ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp    = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id  = "base_link2";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster_->sendTransform(t);

        last_time_ = current_time;
    }

    void handle_reset(const std::shared_ptr<std_srvs::srv::Empty::Request>,
                      std::shared_ptr<std_srvs::srv::Empty::Response>) {
        x_ = 0.0; y_ = 0.0; theta_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Odometry reset success.");
    }

    double x_, y_, theta_;
    rclcpp::Time last_time_;
    rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdometerNode>());
    rclcpp::shutdown();
    return 0;
}
