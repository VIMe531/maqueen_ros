#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"


using namespace std::chrono_literals;

class MaqueenProTF : public rclcpp::Node {
public:
  MaqueenProTF() : Node("maqueen_pro_tf"), x_(0.0), y_(0.0), theta_(0.0) {
    // cmd_vel subscribe
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&MaqueenProTF::cmdVelCallback, this, std::placeholders::_1));

    // tf broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // タイマー: 10Hzで更新
    timer_ = this->create_wall_timer(
        100ms, std::bind(&MaqueenProTF::updatePoseAndPublishTF, this));
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
    const double gear_radius = 0.043;         // [m]
    const double gear_width_span = 0.090;     // [m]
    const double speed_min = -10.465;
    const double speed_max = 10.465;				// [rad/s]

    // 左右ホイールの理論的角速度（rad/s）
    double omega_left = (dx - daz * gear_width_span / 2.0) / gear_radius;
    double omega_right = (dx + daz * gear_width_span / 2.0) / gear_radius;
    
//    double rotate_speed_left = 

    // 実際の速度として仮定（PWM指令で動作したとみなす）
    double v = gear_radius * (omega_right + omega_left) / 2.0;     // 並進速度[m/s]
    double w = gear_radius * (omega_right - omega_left) / gear_width_span; // 角速度[rad/s]

    // 並進・角速度から自己位置を更新
    double delta_x = v * std::cos(theta_) * dt;
    double delta_y = v * std::sin(theta_) * dt;
    double delta_theta = w * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // 角度を -π〜π に正規化
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    // TFメッセージ作成・送信
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = now;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = x_;
    transformStamped.transform.translation.y = y_;
    transformStamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transformStamped);
  }


  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist last_cmd_vel_;
  rclcpp::Time last_time_ = this->now();

  double x_, y_, theta_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MaqueenProTF>());
  rclcpp::shutdown();
  return 0;
}

