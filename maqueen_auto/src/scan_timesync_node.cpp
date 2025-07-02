#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanTimesyncNode : public rclcpp::Node
{
public:
  ScanTimesyncNode()
  : Node("scan_timesync")
  {
    // Subscriber: 元の /scan を受信
    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10,
      std::bind(&ScanTimesyncNode::scanCallback, this, std::placeholders::_1));
    // Publisher: 書き換え後を /scan_resynced へ
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "scan_timesynced", 10);
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    auto out = *msg;  // コピー
    // stamp を現在時刻に書き換え
    out.header.stamp = this->get_clock()->now();
    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanTimesyncNode>());
  rclcpp::shutdown();
  return 0;
}

