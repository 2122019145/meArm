//
// mock Servo.h —— Arduino 官方 Servo 库的 PC 替身（仅供自检，不参与烧录）
//
// 【为什么需要它】
//   v1.6.6 把自研 Timer1 顺序脉冲驱动换成了官方 Servo 库的薄封装
//   （servo_drive.cpp 里 #include <Servo.h>）。PC 上没有这个库，
//   自检就会以 "fatal error: Servo.h: No such file or directory" 直接挂掉。
//
// 【复刻到什么程度】
//   只实现固件真正用到的那部分语义：attach(pin) / write(angle)。
//   脉宽与时序**刻意不在 PC 端复刻** —— 那是官方库在真机上负责的事，
//   旧的自研驱动才需要逐位复刻映射公式（那份自证式用例已随驱动一起删除）。
//   这里记录"谁绑到了哪个引脚、最后被写了多少度"，供 probe_servo_drive 断言：
//     - 通道→引脚顺序（历史 bug：上臂/下臂 D8/D7 曾经对调）
//     - 角度截断与夹取（(int) 截断 + 0~180）
//     - 越界通道号被静默忽略
//
#ifndef MOCK_SERVO_H
#define MOCK_SERVO_H

#include "Arduino.h"

#define MOCK_SERVO_SLOT_MAX 8
#define MOCK_SERVO_LOG_MAX  16

struct MockServoState {
  int attachCount;                          /* attach() 被调用的次数 */
  int attachPin[MOCK_SERVO_LOG_MAX];         /* 第 k 次 attach 的目标引脚 */
  int attachSlot[MOCK_SERVO_LOG_MAX];        /* 第 k 次 attach 落在哪个对象槽位 */
  int slotPin[MOCK_SERVO_SLOT_MAX];          /* 每个槽位当前绑定的引脚（-1 = 未绑） */
  int slotAngle[MOCK_SERVO_SLOT_MAX];        /* 每个槽位最后一次写到的角度（-1 = 没写过） */
  int slotWrites[MOCK_SERVO_SLOT_MAX];       /* 每个槽位收到多少次 write */
  int nextSlot;                              /* 下一个待分配槽位（只增不减） */
};

extern MockServoState g_mockServo;
void mockServoReset(void);

/* 真库里 Servo 对象在 attach() 时才占用定时器时隙；这里改成构造时分配槽位，
 * 因为本工程的四路是静态数组里的四个对象（s_servo[0..3]），槽位与通道号一一对应。 */
class Servo {
public:
  Servo() : m_slot(-1) {}

  int  attach(int pin);
  void detach(void);
  void write(int angle);
  void writeMicroseconds(int us);
  int  read(void) const;

private:
  int m_slot;
};

inline int Servo::attach(int pin) {
  if (m_slot < 0) { m_slot = g_mockServo.nextSlot++; }
  if (m_slot >= MOCK_SERVO_SLOT_MAX) { m_slot = MOCK_SERVO_SLOT_MAX - 1; }
  /* 真库的 attach() 里就是这一句把引脚设成输出（波形由定时器中断产生），
   * 这里照做，好让"setup() 到底配好了哪几根脚"仍然可在 PC 上断言。 */
  pinMode((uint8_t)pin, OUTPUT);
  g_mockServo.slotPin[m_slot] = pin;
  if (g_mockServo.attachCount < MOCK_SERVO_LOG_MAX) {
    g_mockServo.attachPin[g_mockServo.attachCount]  = pin;
    g_mockServo.attachSlot[g_mockServo.attachCount] = m_slot;
    g_mockServo.attachCount++;
  }
  return m_slot;
}

inline void Servo::detach(void) {
  if (m_slot >= 0) { g_mockServo.slotPin[m_slot] = -1; }
}

/* 官方库的 write(int) 语义：0~180 之外会被夹住（库里还有 544~2400us 的夹取，
 * 对 0~180 是恒等的），写过的角度留在内部定时状态里直到下次改写。 */
inline void Servo::write(int angle) {
  if (m_slot < 0) { return; }                 /* 未 attach：真库同样不动作 */
  if (angle < 0) { angle = 0; } else if (angle > 180) { angle = 180; }
  g_mockServo.slotAngle[m_slot] = angle;
  g_mockServo.slotWrites[m_slot]++;
}

inline void Servo::writeMicroseconds(int us) {
  (void) us;                                  /* 固件没用到；真库按 us 直接排脉冲 */
}

inline int Servo::read(void) const {
  return (m_slot >= 0) ? g_mockServo.slotAngle[m_slot] : -1;
}

#endif /* MOCK_SERVO_H */
