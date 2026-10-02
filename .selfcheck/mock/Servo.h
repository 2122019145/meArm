//
// mock Servo.h —— 仅用于 PC 端语法自检
//
// 与真实的 Servo 库接口保持兼容（attach/write/read），另外多记一个引脚号与最近
// 写入的角度，供自检脚本核对 wearm.ino 的真实行为。
//
#ifndef MOCK_SERVO_H
#define MOCK_SERVO_H

#include <stdint.h>

#define MOCK_SERVO_MAX 20

class Servo {
public:
  Servo();
  void attach(uint8_t pin);
  void attach(uint8_t pin, int minUs, int maxUs);
  void write(int angle);
  int  read();
private:
  int angle_ = 0;
  int pin_   = -1;
  int slot_  = -1;
};

/* 仿真记录：让自检脚本能核对"哪个舵机挂在哪根引脚上""最后写进舵机的角度是多少"。
 * 有了它，wearm_ino_test 才能验证 wearm.ino 的真实 setup() 行为，
 * 而不只是"能编译"（旧的手抄复刻版把引脚写成 9/10/11/6，与 sketch 的 9/7/8/6 不一致，
 * 却照样编译通过 —— 这类漂移只有直接编译真 sketch 并核对行为才抓得到）。 */
void mockServoReset(void);
int  mockServoPin(int index);      /* 下标 = servos[] 数组下标；未 attach 返回 -1 */
int  mockServoAngle(int index);    /* 下标 = servos[] 数组下标；未写过返回 -1 */
int  mockServoWriteCount(void);    /* 累计 write() 次数 */

#endif
