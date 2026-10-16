/*
 * servo_drive.cpp —— 4 路舵机驱动实现（Arduino 官方 Servo 库的薄封装）
 *
 * 结构：
 *   servoDriveBegin()          初始化钩子（空：Servo::attach 自带时序初始化）
 *   servoDriveAttach()         绑定引脚（Servo::attach）
 *   servoDriveWrite()          写目标角度（(int) 截断 + 夹取 0~180 + Servo::write）
 *
 * 脉宽映射与时序全部由官方库负责（20ms 帧、544~2400us、TRIM 2us），
 * 本文件不再持有任何时序状态 —— 旧的 servoDriveTicksForDeg() / servoDriveStep()
 * 随自研驱动一起删除，probe_servo_drive 已改测本封装的行为。
 */
#include "servo_drive.h"
#include <Servo.h>

/* 官方库的 Servo 对象每通道一个。Uno 上共用 Timer1（20ms 帧）。 */
static Servo s_servo[SERVO_CH_COUNT];

void servoDriveBegin(void) {
  /* Servo 库在 attach() 里完成定时器与引脚初始化，无需预处理。
   * 保留本函数是为了维持 meArm.ino 里"Begin 必须在 Attach 之前"的调用序。 */
}

void servoDriveAttach(uint8_t ch, uint8_t pin) {
  if (ch >= SERVO_CH_COUNT) { return; }
  s_servo[ch].attach(pin);
}

void servoDriveWrite(uint8_t ch, double deg) {
  if (ch >= SERVO_CH_COUNT) { return; }
  int a = (int)deg;                     /* 与原代码 (int) angle 的截断一致 */
  if (a < 0) { a = 0; } else if (a > 180) { a = 180; }
  s_servo[ch].write(a);
}
