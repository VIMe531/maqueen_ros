#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class ImuToOdom : public rclcpp::Node {
private:
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	rclcpp::TimerBase::SharedPtr timer_;
	sensor_msgs::msg::Imu last_imu_;
	rclcpp::Time last_time_;
	nav_msgs::msg::Odometry odom_;
	double x_ = 0.0;
	double y_ = 0.0;
	double d_x_ = 0.0;
	double d_y_ = 0.0;
	double theta_ = 0.0;
	double velocity_ = 0.0;
	double acc_z_ = 0.0;
	double ang_x_ = 0.0;
public:
  ImuToOdom()
  : Node("imu_to_odom_node")
  , last_time_(this->get_clock()->now())
  {
    // cmd_vel subscribe
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu"
        , rclcpp::SensorDataQoS()
        ,std::bind(&ImuToOdom::ImuCallback, this, std::placeholders::_1)
        );

    // odom publisher  ← 追加
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // tf broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // タイマー: 10Hzで更新
    timer_ = this->create_wall_timer(
        100ms, std::bind(&ImuToOdom::updatePoseAndPublishTF, this));
  }

private:
  void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    last_imu_ = *msg;
  }
  
  void debug_disp(void)
  {
  	RCLCPP_INFO(this->get_logger(), "odom_.pose.pose.position.x = %f \n", x_);
  	RCLCPP_INFO(this->get_logger(), "odom_.pose.pose.position.y = %f \n", y_);
  	RCLCPP_INFO(this->get_logger(), "odom_.twist.twist.linear.x = %f \n", odom_.twist.twist.linear.x);
  	RCLCPP_INFO(this->get_logger(), "odom_.twist.twist.angular.z = %f \n", odom_.twist.twist.angular.z);
  }

  void updatePoseAndPublishTF()
  {
    auto now = this->get_clock()->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    // Imuデータから前後速度と角速度を取り出す
    acc_z_ = last_imu_.linear_acceleration.z * 9.8F;			// Atom S3の取り付け向きにより、Z軸が前後に向く
    ang_x_ = last_imu_.angular_velocity.x * 0.0174533F;		// pi/180.0の近似値をかけ、radに変換する

    // 自己位置を積分計算
    velocity_ = acc_z_ * dt;
    theta_	= ang_x_ * dt;
    d_x_	= velocity_ * std::cos(theta_) * dt;
    d_y_	= velocity_ * std::sin(theta_) * dt;

    x_     += d_x_;
    y_     += d_y_;
    theta_ += theta_;

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
    odom_.header.stamp    = now;
    odom_.header.frame_id = "odom";
    odom_.child_frame_id  = "base_link";

    // 位置
    odom_.pose.pose.position.x    = x_;
    odom_.pose.pose.position.y    = y_;
    odom_.pose.pose.position.z    = 0.0;
    odom_.pose.pose.orientation.x = q.x();
    odom_.pose.pose.orientation.y = q.y();
    odom_.pose.pose.orientation.z = q.z();
    odom_.pose.pose.orientation.w = q.w();

    // 速度
    odom_.twist.twist.linear.x  = velocity_;
    odom_.twist.twist.linear.y  = 0.0;
    odom_.twist.twist.angular.z = ang_x_;
    
	debug_disp();

    odom_pub_->publish(odom_);
    // --- ↑ここまで Odometry発行処理↑ ---
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuToOdom>());
  rclcpp::shutdown();
  return 0;
}

