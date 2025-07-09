// Arduino-compatible MaqueenIR class using ReceiverIR (function pointer version)
#ifndef MAQUEEN_IR_ARDUINO_H
#define MAQUEEN_IR_ARDUINO_H

#include <Wire.h>
#include <Arduino.h>

#define M1 1
#define M2 2
#define ALL 3

#define PIN_SDA 38
#define PIN_SCL 39

class MaqueenLite {
private:
  int CW = 0;   // forward
  int CCW = 2;  // backward
  float gear_radius = 0.043;
  float gear_width_span = 0.070;
  float motor_speed[2] = { 0.0, 0.0 };
  uint8_t m_rotate_speed[2] = { 0, 0 };
  uint8_t dir[2] = { CW, CW };
  uint8_t i2c_address = 0x10;

public:
  MaqueenLite() {}

  void begin() {
    Wire.begin(PIN_SDA, PIN_SCL);
    Serial.begin(115200);
  }

  void moveRobot(float dx, float daz) {
    motor_speed[0] = (dx / gear_radius) - ((daz * gear_width_span) / (2 * gear_radius));
    motor_speed[1] = (dx / gear_radius) + ((daz * gear_width_span) / (2 * gear_radius));

    for (int i = 0; i < 2; i++) {
      if (motor_speed[i] < 0) {
        dir[i] = CCW;
      } else {
        dir[i] = CW;
      }
      m_rotate_speed[i] = mapFloat(abs(motor_speed[i]), 0.0, 1.0, 0, 150);
    }

    this->motorRun(M1, dir[0], m_rotate_speed[0]);
    this->motorRun(M2, dir[1], m_rotate_speed[1]);
  }

  void motorRun(int motor, uint8_t direction, uint8_t speed) {
    if (speed > 240) speed = 240;

    uint8_t buf[4];
    int res = -1;

    switch (motor) {
      case M1:  // LEFT
        buf[0] = direction;
        buf[1] = speed;
        Wire.beginTransmission(i2c_address);
        Wire.write(0x00);  // Register for left motor
        Wire.write(buf, 2);
        res = Wire.endTransmission();
        break;

      case M2:  // RIGHT
        buf[0] = direction;
        buf[1] = speed;
        Wire.beginTransmission(i2c_address);
        Wire.write(0x02);  // Register for right motor
        Wire.write(buf, 2);
        res = Wire.endTransmission();
        break;

      case ALL:
        buf[0] = direction;
        buf[1] = speed;
        buf[2] = direction;
        buf[3] = speed;
        Wire.beginTransmission(i2c_address);
        Wire.write(0x00);  // Write to both left and right starting from 0x00
        Wire.write(buf, 4);
        Wire.endTransmission();
        break;

      default:
        Serial.println("Invalid motor ID");
        return;
    }

    Serial.print("direction: ");
    Serial.print(direction);
    Serial.println("");
  }

  float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  }
};

#endif