#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <M5Unified.h>

rcl_node_t node;
rcl_publisher_t scan_pub;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;

sensor_msgs__msg__LaserScan scan_msg;

#define LIDAR_BAUDRATE 115200
#define LIDAR_RX 22
#define LIDAR_TX -1
#define LIDAR_PWM 25
#define LIDAR_PWM_CHANNEL 0
#define LIDAR_PWM_FREQ 10
#define LIDAR_PWM_RESOLUTION 8

#define MAX_POINTS 500
#define M_PI 3.141592

char ssid[] = "SSID";
char password[] = "PASSWORD";
char host_ip[] = "HOST_IP";

uint8_t packet[512];
size_t packet_idx = 0;

TaskHandle_t serial_read_task_handle = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void send_start_scan_command() {
  Serial1.write(0xA5);
  Serial1.write(0x60);
}

uint16_t calculate_checksum(const uint8_t *data, int size) {
  uint16_t checksum = 0;
  for (int i = 0; i < size; i += 2) {
    uint16_t word = data[i] | (data[i + 1] << 8);
    checksum ^= word;
  }
  return checksum;
}

// クリティカル外で実行するパース関数
bool parse_packet_from(const uint8_t *buf, size_t len) {
  // リセット
  for (int i = 0; i < MAX_POINTS; i++) {
    scan_msg.ranges.data[i] = std::numeric_limits<float>::infinity();
    scan_msg.intensities.data[i] = 0.0f;
  }
  if (len < 10) return false;
  if (!(buf[0] == 0xAA && buf[1] == 0x55)) return false;

  uint8_t LSN = buf[3];
  if (LSN < 2) return false;

  uint16_t FSA = buf[4] | (buf[5] << 8);
  uint16_t LSA = buf[6] | (buf[7] << 8);

  float start_angle = (FSA >> 1) / 64.0f * (M_PI / 180.0f);
  float end_angle = (LSA >> 1) / 64.0f * (M_PI / 180.0f);
  start_angle = fmod(start_angle, 2.0f * M_PI);
  end_angle = fmod(end_angle, 2.0f * M_PI);

  int expected_size = 10 + LSN * 2;
  if (len < expected_size) return false;

  uint16_t recv_checksum = buf[8] | (buf[9] << 8);
  uint16_t checksum = 0;
  checksum ^= calculate_checksum(buf, 8);
  checksum ^= calculate_checksum(&buf[10], expected_size - 10);
  if (checksum != recv_checksum) {
    Serial.printf("Checksum mismatch: calc=0x%04X rec=0x%04X\n", checksum, recv_checksum);
    return false;
  }

  float delta = end_angle - start_angle;
  if (delta < 0) delta += 2.0f * M_PI;
  float d_angle = delta / float(LSN - 1);

  for (int i = 0; i < LSN; i++) {
    uint16_t raw = buf[10 + i * 2] | (buf[10 + i * 2 + 1] << 8);
    float raw_angle = start_angle + i * d_angle;
    float angle = fmod(raw_angle, 2.0f * M_PI);
    if (angle < 0) angle += 2.0 * M_PI;
    float distance = raw / 4.0f / 1000.0f;
    float ang_corr = 0;
    if(distance == 0) ang_corr = 0;
    else{
      ang_corr = (distance == 0) ? 0.0f : atan(21.8f * (155.3f - distance) / (155.3 * distance));
    }
    angle += ang_corr;
    angle = fmod(angle, 2.0f*M_PI);
    if(angle < 0) angle += 2.0f*M_PI;

    int idx = int(angle * MAX_POINTS / (2.0f * M_PI));
    if (idx < 0 || idx >= MAX_POINTS) {
      Serial.printf("DEBUG: angle=%.1f° idx=%d raw=%d\n",
                    angle * 180.0 / M_PI, idx, raw);
      continue;
    };
    scan_msg.ranges.data[idx] = distance;
    scan_msg.intensities.data[idx] = 100.0f;
  }
  scan_msg.ranges.size = MAX_POINTS;
  scan_msg.intensities.size = MAX_POINTS;
  return true;
}

void Serial1_receive_task(void *param) {
  while (1) {
    while (Serial1.available()) {
      uint8_t b = Serial1.read();
      portENTER_CRITICAL(&mux);
      if (packet_idx < sizeof(packet)) {
        if (packet_idx >= 1 && packet[packet_idx - 1] == 0xAA && b == 0x55) {
          packet[0] = 0xAA;
          packet[1] = 0x55;
          packet_idx = 2;
        } else {
          packet[packet_idx++] = b;
        }
      } else {
        packet_idx = 0;
      }
      portEXIT_CRITICAL(&mux);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void publish_scan_callback(rcl_timer_t *timer, int64_t) {
  if (!timer) return;
  // クリティカルセクション：バッファコピーのみ
  size_t len;
  static uint8_t local_buf[512];
  portENTER_CRITICAL(&mux);
  len = packet_idx;
  if (len > sizeof(local_buf)) len = sizeof(local_buf);
  memcpy(local_buf, packet, len);
  packet_idx = 0;
  portEXIT_CRITICAL(&mux);

  if (parse_packet_from(local_buf, len)) {
    uint64_t now = rmw_uros_epoch_nanos();
    scan_msg.header.stamp.sec = now / 1000000000ULL;
    scan_msg.header.stamp.nanosec = now % 1000000000ULL;
    rcl_publish(&scan_pub, &scan_msg, NULL);
  }
}

void setup() {
  M5.begin();
  Serial.begin(115200);
  Serial1.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX, LIDAR_TX);

  delay(2000);
  set_microros_wifi_transports(ssid, password, host_ip, 8888);

  ledcSetup(LIDAR_PWM_CHANNEL, LIDAR_PWM_FREQ, LIDAR_PWM_RESOLUTION);
  ledcAttachPin(LIDAR_PWM, LIDAR_PWM_CHANNEL);
  ledcWrite(LIDAR_PWM_CHANNEL, 128);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "m5_lidar_node", "", &support);

  rclc_publisher_init_default(
    &scan_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan),
    "/scan");

  scan_msg.ranges.capacity = MAX_POINTS;
  scan_msg.intensities.capacity = MAX_POINTS;
  scan_msg.ranges.data = (float *)malloc(sizeof(float) * MAX_POINTS);
  scan_msg.intensities.data = (float *)malloc(sizeof(float) * MAX_POINTS);
  scan_msg.ranges.size = MAX_POINTS;
  scan_msg.intensities.size = MAX_POINTS;
  scan_msg.angle_min = 0.0f;
  scan_msg.angle_max = 2.0f * M_PI;
  scan_msg.angle_increment = 0.01256637f;
  scan_msg.range_min = 0.12f;
  scan_msg.range_max = 8.0f;
  scan_msg.scan_time = 1.0f / 6.0f;
  scan_msg.time_increment = scan_msg.scan_time / MAX_POINTS;

  rosidl_runtime_c__String__init(&scan_msg.header.frame_id);
  rosidl_runtime_c__String__assign(&scan_msg.header.frame_id, "laser_frame");

  rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(100),
    publish_scan_callback);

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_timer(&executor, &timer);

  xTaskCreatePinnedToCore(
    Serial1_receive_task,
    "Serial1ReadTask",
    4096,
    NULL,
    1,
    &serial_read_task_handle,
    1);

  send_start_scan_command();
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
