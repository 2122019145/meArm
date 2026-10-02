//
// mock Servo 实现 —— 仅用于 PC 端语法自检
//
#include "Servo.h"

namespace {
int g_pin[MOCK_SERVO_MAX];      /* 按构造顺序登记的槽位，与 servos[] 下标一致 */
int g_angle[MOCK_SERVO_MAX];
int g_writes = 0;
int g_next   = 0;
}

Servo::Servo() {
  slot_ = g_next++;
  if (slot_ < MOCK_SERVO_MAX) { g_pin[slot_] = -1; g_angle[slot_] = -1; }
}

void Servo::attach(uint8_t pin) {
  pin_ = (int) pin;
  if (slot_ >= 0 && slot_ < MOCK_SERVO_MAX) g_pin[slot_] = pin_;
}

void Servo::attach(uint8_t pin, int, int) {
  attach(pin);
}

void Servo::write(int angle) {
  angle_ = angle;
  g_writes++;
  if (slot_ >= 0 && slot_ < MOCK_SERVO_MAX) g_angle[slot_] = angle;
}

int Servo::read() { return angle_; }

void mockServoReset(void) {
  g_writes = 0;
  for (int i = 0; i < MOCK_SERVO_MAX; i++) { g_pin[i] = -1; g_angle[i] = -1; }
}

int mockServoPin(int index) {
  return (index >= 0 && index < MOCK_SERVO_MAX) ? g_pin[index] : -1;
}

int mockServoAngle(int index) {
  return (index >= 0 && index < MOCK_SERVO_MAX) ? g_angle[index] : -1;
}

int mockServoWriteCount(void) { return g_writes; }
