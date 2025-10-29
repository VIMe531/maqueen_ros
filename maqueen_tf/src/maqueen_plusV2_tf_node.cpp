// maqueen_plusV2_tf_node.cpp

#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class MaqueenPlusV2TF : public rclcpp::Node {
public:
  MaqueenPlusV2TF()
  : Node("maqueen_plusV2_tf"),
    got_imu_(false),
    got_first_imu_(false),
    theta_(0.0),
    vel_forward_(0.0),
    x_(0.0),
    y_(0.0)
  {
    // IMU トピック購読 (センサ向け QoS)
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/uros_imu_node_publisher",
      rclcpp::SensorDataQoS(),
      std::bind(&MaqueenPlusV2TF::imuCallback, this, std::placeholders::_1));

    // TF ブロードキャスター
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // タイマー初回時間
    last_time_ = this->now();

    // 100ms ごとに自己位置更新＆TF送信
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MaqueenPlusV2TF::updatePoseAndPublishTF, this));
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    last_imu_ = *msg;
    got_imu_ = true;
  }

  void updatePoseAndPublishTF() {
    if (!got_imu_) {
      RCLCPP_INFO(get_logger(), "No IMU data received yet");
      return;
    }

    auto now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();

    // 初回は dt を無効化して時間合わせのみ
    if (!got_first_imu_) {
      got_first_imu_ = true;
      last_time_ = now;
      return;
    }
    last_time_ = now;

    // 1) Yaw のみ角速度から積分
    double wx = last_imu_.angular_velocity.x;  // [rad/s]
    theta_ += wx * dt;
    // -π～π に正規化
    theta_ = std::atan2(std::sin(theta_), std::cos(theta_));

    // 2) 前後方向の加速度成分を積分 → 速度 → 位置に反映
    double a_forward = last_imu_.linear_acceleration.z;  // [m/s²]
    vel_forward_ += a_forward * dt;                      // [m/s]
    double dx = vel_forward_ * std::cos(theta_) * dt;    // [m]
    double dy = vel_forward_ * std::sin(theta_) * dt;    // [m]
    x_ += dx;
    y_ += dy;

    // 3) TF メッセージ作成・送信
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp    = now;
    t.header.frame_id = "odom";
    t.child_frame_id  = "base_link";

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

  // メンバ変数
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  sensor_msgs::msg::Imu         last_imu_;
  bool                          got_imu_;
  bool                          got_first_imu_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr                  timer_;
  rclcpp::Time                                  last_time_;

  double theta_;         // yaw [rad]
  double vel_forward_;   // 前後方向速度 [m/s]
  double x_, y_;         // 推定位置 [m]
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MaqueenPlusV2TF>());
  rclcpp::shutdown();
  return 0;
}

