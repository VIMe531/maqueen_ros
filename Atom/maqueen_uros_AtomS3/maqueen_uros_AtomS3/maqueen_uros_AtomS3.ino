#include <Wire.h>
#include <M5Unified.h>
#include <Adafruit_NeoPixel.h>
#include "maqueen_plus.h"
// #include "maqueen_lite.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include <geometry_msgs/msg/twist.h>

// ROS entities
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_subscription_t cmd_vel_subscriber;

// messages
geometry_msgs__msg__Twist cmd_vel_msg;

MaqueenPlus maqueen;
// MaqueenPlusV2 maqueen;
// MaqueenLite maqueen;

char ssid[] = "SSID";
char password[] = "PASSWORD";
char host_ip[] = "HOST_IP";

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
  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(RED, BLACK);
  M5.Lcd.print("ERROR!");
  while (1) delay(100);
}

// cmd_vel callback
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *twist = (const geometry_msgs__msg__Twist *)msgin;

  maqueen.moveRobot(twist->linear.x, twist->angular.z);
  
  M5.Lcd.setCursor(0,20);
  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(YELLOW, BLACK);
  M5.Lcd.print("linear.x = ");
  M5.Lcd.println(twist->linear.x);
  M5.Lcd.print("angular.z = ");
  M5.Lcd.println(twist->angular.z);
}

void setup() {
  M5.begin();
  M5.Lcd.begin();
  maqueen.begin();
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0,10);
  
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

  M5.Lcd.setTextSize(1);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.print("Initialized!");
}

void loop() {
  delay(10);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
