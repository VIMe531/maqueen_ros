#include <Wire.h>
#include <M5Unified.h>
#include <Adafruit_NeoPixel.h>
// #include "maqueen_plus.h"
#include "maqueen_lite.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include <geometry_msgs/msg/twist.h>


#define LED_PIN 27
#define NUM_PIX 1
#define PIX_R 0
#define PIX_G 1
#define PIX_B 2
#define PIX_Y 3
#define PIX_M 4
#define PIX_C 5
#define PIX_OFF 6

// ROS entities
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_subscription_t cmd_vel_subscriber;

// messages
geometry_msgs__msg__Twist cmd_vel_msg;

Adafruit_NeoPixel pixel(NUM_PIX, LED_PIN, NEO_GRB + NEO_KHZ800);
// MaqueenPlus maqueen;
MaqueenLite maqueen;

char ssid[] = "SSID";
char password[] = "PASSWORD";
char host_ip[] = "HOST_IP";

uint32_t pix_colors[] = {
  pixel.Color(255, 0, 0),    // 赤
  pixel.Color(0, 255, 0),    // 緑
  pixel.Color(0, 0, 255),    // 青
  pixel.Color(255, 255, 0),  // 黄
  pixel.Color(255, 0, 255),  // マゼンタ
  pixel.Color(0, 255, 255),  // シアン
  pixel.Color(0, 0, 0)       // 消灯
};

void setLEDColor(uint8_t color_index) {
  if (color_index >= sizeof(pix_colors) / sizeof(pix_colors[0])) {
    return;
  }
  pixel.setPixelColor(0, pix_colors[color_index]);
  pixel.show();
}

// Error macros
#define RCCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop(); \
    } \
  }
#define RCSOFTCHECK(fn) \
  { \
    rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) {} \
  }

void error_loop() {
  setLEDColor(PIX_R);
  while (1) delay(100);
}

// cmd_vel callback
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *twist = (const geometry_msgs__msg__Twist *)msgin;

  maqueen.moveRobot(twist->linear.x, twist->angular.z);
  
  setLEDColor(PIX_B);
}

void setup() {
  M5.begin();
  pixel.begin();
  maqueen.begin();
  Serial.begin(115200);
  pixel.setBrightness(20);
  setLEDColor(PIX_OFF);

  set_microros_wifi_transports(ssid, password, host_ip, 8888);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "uros_combined_node", "", &support));

  // Subscriber /cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  setLEDColor(PIX_G);
}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
