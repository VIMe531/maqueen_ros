#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"               // ← 追加
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class MaqueenLiteTF : public rclcpp::Node {
public:
  MaqueenLiteTF() : Node("maqueen_lite_tf"), x_(0.0), y_(0.0), theta_(0.0), last_time_(this->get_clock()->now()) {
    // cmd_vel subscribe
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&MaqueenLiteTF::cmdVelCallback, this, std::placeholders::_1));

    // odom publisher  ← 追加
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // tf broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // タイマー: 10Hzで更新
    timer_ = this->create_wall_timer(
        100ms, std::bind(&MaqueenLiteTF::updatePoseAndPublishTF, this));
  }

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    last_cmd_vel_ = *msg;
  }
  
  double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)   {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }

  void updatePoseAndPublishTF() {
    auto now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // MaqueenのmoveRobot()が与えるPWMを仮想的に再現する
    double dx = last_cmd_vel_.linear.x;
    double daz = last_cmd_vel_.angular.z;

    // Maqueenの物理パラメータと一致させる
    const double gear_radius      = 0.043;    // [m]
    const double gear_width_span  = 0.070;    // [m]
    const double speed_min        = -0.2;     // [rad/s]
    const double speed_max        =  0.2;     // [rad/s]

    // 左右ホイールの理論的角速度（rad/s）
    double omega_left  = (dx - daz * gear_width_span / 2.0) / gear_radius;
    double omega_right = (dx + daz * gear_width_span / 2.0) / gear_radius;
    
    // 並進・角速度として仮定
    double v = gear_radius * (omega_right + omega_left) / 2.0;               // [m/s]
    double w = gear_radius * (omega_right - omega_left) / gear_width_span;  // [rad/s]

    // 自己位置を積分
    double delta_x     = v * std::cos(theta_) * dt;
    double delta_y     = v * std::sin(theta_) * dt;
    double delta_theta = w * dt;

    x_     += delta_x;
    y_     += delta_y;
    theta_ += delta_theta;
    theta_  = std::atan2(std::sin(theta_), std::cos(theta_));

    // --- TF の送信 ---
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = now;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id  = "base_link";
    tf_msg.transform.translation.x = x_;
    tf_msg.transform.translation.y = y_;
    tf_msg.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(tf_msg);

    // --- /odom への Odometry メッセージ発行 ↓ここから↓ ---
    nav_msgs::msg::Odometry odom;
    odom.header.stamp    = now;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";

    // 位置
    odom.pose.pose.position.x    = x_;
    odom.pose.pose.position.y    = y_;
    odom.pose.pose.position.z    = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // 速度
    odom.twist.twist.linear.x  = v;
    odom.twist.twist.linear.y  = 0.0;
    odom.twist.twist.angular.z = w;

    odom_pub_->publish(odom);
    // --- ↑ここまで Odometry発行処理↑ ---
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;      // ← 追加
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  rclcpp::Time last_time_;

  double x_, y_, theta_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MaqueenLiteTF>());
  rclcpp::shutdown();
  return 0;
}

