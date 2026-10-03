//
// wearm_ino_test.cpp
// 直接编译 wearm.ino 本体（不手抄复刻），核对 sketch 的接线与上电行为。
//
// 【为什么直接 #include "../wearm.ino"】
// 早期版本是把 setup()/loop() 手工抄一份到本文件里编译，只能证明"抄件能编译"。
// 抄件已经漂移过一次：抄件写 servos[2].attach(10)、servos[3].attach(11)，
// 而真 sketch 是 attach(7)/attach(8)，抄件照样编译通过、零警告，
// 完全没发现问题。改成本文件直接 include 真 sketch 之后，接线错误才会暴露。
//
// 注意：.ino 不是合法的 C++ 翻译单元，但它是纯 C++ 文本，直接 include 即可；
// PC 端只做语法/类型/行为自检，不参与烧录。
//
// 【v1.3.0】sketch 不再用 Arduino Servo 库，改用 servo_drive.h 的 4 路驱动。
//   所以"上电把哪个角度写进了哪一路"不能再用 mockServoAngle() 看，
//   改成直接步进 servo_drive.cpp 的脉冲状态机、测量每路脉宽（tick），
//   再和 servoDriveTicksForDeg((int)Pos.ser.angleN) 比对 —— 比读回角度更接近真实：
//   它验证的是"真正会被输出到引脚上的脉宽"。
//
#include "Arduino.h"
#include "servo_drive.h"
#include "../wearm.ino"

#include <stdio.h>
#include <math.h>

static int g_fail = 0;

static void check(const char *name, bool ok, const char *detail) {
  printf("  %-46s %s  %s\n", name, ok ? "PASS" : "FAIL", detail ? detail : "");
  if (!ok) g_fail++;
}

/* 从 mock 的 digitalWrite 日志里取出"拉高事件"的引脚顺序 */
static int highOrder(int out[8]) {
  int n = 0;
  for (int i = 0; i < g_mockWriteLogCount && n < 8; i++) {
    if (g_mockWriteLogLevel[i] == HIGH) out[n++] = g_mockWriteLogPin[i];
  }
  return n;
}

/* 步进一个完整的 20ms 帧。
 * 前置条件：脉冲状态机停在帧起点（setup() 后、或上一次 captureFrame 之后即是）。
 * 出参：pins[k] = 第 k 路的引脚；width[k] = 第 k 路脉宽（0.5us tick）；frameTicks = 帧长。
 * 这里完全按 ISR 的方式驱动：每个事件的返回值 1 表示"先把计数器清零"。 */
static void captureFrame(int pins[4], unsigned width[4], uint16_t *frameTicks) {
  mockDigitalWriteReset();
  uint16_t now = 0, next = 0;
  for (int k = 0; k < 4; k++) {
    uint8_t isFrameStart = servoDriveStep(now, &next);
    if (isFrameStart) now = 0;          /* 帧起点：TCNT1 清零 */
    width[k] = (unsigned)(next - now);
    now = next;
  }
  (void) servoDriveStep(now, &next);    /* 收尾事件：拉低最后一路 + 定帧尾 */
  *frameTicks = next;

  int order[8];
  int n = highOrder(order);
  for (int k = 0; k < 4; k++) pins[k] = (k < n) ? order[k] : -1;
}

int main(void) {
  printf("=== wearm.ino 真实 sketch 自检（直接 include 真文件）===\n");

  char buf[160];

  /* 0) setup() 的接线动作：servoDriveAttach(0..3) 必须先 pinMode(OUTPUT) 再拉低引脚，
   *    顺序 9,7,8,6。用 digitalWrite 日志看 —— 它是"真的动了哪根脚"的证据。 */
  mockDigitalWriteReset();
  setup();
  const int wantPin[4] = { 9, 7, 8, 6 };
  for (int i = 0; i < 4; i++) {
    int got = -1;
    if (i < g_mockWriteLogCount && g_mockWriteLogLevel[i] == LOW) got = g_mockWriteLogPin[i];
    snprintf(buf, sizeof buf, "attach 顺序[%d] 引脚 %d (期望 %d)", i, got, wantPin[i]);
    check(buf, got == wantPin[i], NULL);
  }
  for (int i = 0; i < 4; i++) {
    snprintf(buf, sizeof buf, "引脚 %d 被设成 OUTPUT (实际 %d)", wantPin[i], g_mockPinMode[wantPin[i]]);
    check(buf, g_mockPinMode[wantPin[i]] == OUTPUT, NULL);
  }

  /* 1) 上电姿态：setup() 末尾的 writeServo() 必须把 POS_HOME 的角度写进舵机。
   *    脉宽必须等于 servoDriveTicksForDeg((int)Pos.ser.angleN) —— 一路都不能漏。 */
  int pins[4];
  unsigned width[4];
  uint16_t frameTicks = 0;
  captureFrame(pins, width, &frameTicks);
  for (int i = 0; i < 4; i++) {
    double want = (i == 0) ? Pos.ser.angle1 : (i == 1) ? Pos.ser.angle2
                : (i == 2) ? Pos.ser.angle3 : Pos.ser.angle4;
    unsigned wantTicks = servoDriveTicksForDeg(want);
    snprintf(buf, sizeof buf, "上电脉宽 通道%d=%u tick (Pos=%.6f -> %u)", i, width[i], want, wantTicks);
    check(buf, width[i] == wantTicks, NULL);
  }
  snprintf(buf, sizeof buf, "帧长 %u tick = %.2f ms (期望 40000 tick = 20ms)", frameTicks, frameTicks * 0.5);
  check(buf, frameTicks == 40000u, NULL);

  /* 2) 上电角度必须是 POS_HOME 对应的 (90,90,90, 末端行程中位)，不能是 0 —— 角度 0 会让
   *    机械臂在上电瞬间甩出去，这是老版本踩过的坑。
   *    注意用容差比较：posInit() 走的是"设坐标 -> 反解 -> 落角度"的往返，
   *    b/r/c 会带 ~1e-13 的反解舍入误差，用 == 90.0 会把"正确"判成失败
   *    （曾经就是 89.99999999999999 != 90.0 报的假失败，而 %.1f 打印出来完全正常）。 */
  snprintf(buf, sizeof buf, "上电姿态 = (%.6f,%.6f,%.6f,%.6f)", Pos.ser.angle1, Pos.ser.angle2,
           Pos.ser.angle3, Pos.ser.angle4);
  check(buf, fabs(Pos.ser.angle1 - 90.0) < 1e-6 && fabs(Pos.ser.angle2 - 90.0) < 1e-6 &&
             fabs(Pos.ser.angle3 - 90.0) < 1e-6, NULL);
  /* 末端 f 的行程是 [60,150]，posInit() 取中位 105（不是 90）—— 这一条曾经把
   * "assert f == 90" 写错，是探针自己的错，固件行为才是对的。 */
  const double wantF = (servoLimit.minF + servoLimit.maxF) / 2.0;
  snprintf(buf, sizeof buf, "末端上电取行程中位 f=%.1f (期望 %.1f)", Pos.ser.angle4, wantF);
  check(buf, fabs(Pos.ser.angle4 - wantF) < 1e-9, NULL);
  check("上电脉宽非 0° 脉宽（不会甩向原点）",
        width[0] != servoDriveTicksForDeg(0.0) && width[1] != servoDriveTicksForDeg(0.0) &&
        width[2] != servoDriveTicksForDeg(0.0), NULL);
  check("上电脉冲顺序 = 引脚 9,7,8,6",
        pins[0] == 9 && pins[1] == 7 && pins[2] == 8 && pins[3] == 6, NULL);

  /* 3) loop(): 手柄不动 -> 角度不变、坐标自洽；writeServo 仍然照写 */
  double b0 = Pos.ser.angle1, r0 = Pos.ser.angle2, c0 = Pos.ser.angle3;
  for (int i = 0; i < 20; i++) loop();
  snprintf(buf, sizeof buf, "静置 20 轮 loop(): 角度不变 (%.1f,%.1f,%.1f)", Pos.ser.angle1,
           Pos.ser.angle2, Pos.ser.angle3);
  check(buf, Pos.ser.angle1 == b0 && Pos.ser.angle2 == r0 && Pos.ser.angle3 == c0, NULL);

  REC fk;
  bool okFk = recFromServo(&fk, &Pos.ser);
  double dx = fabs(fk.x - Pos.rec.x), dy = fabs(fk.y - Pos.rec.y), dz = fabs(fk.z - Pos.rec.z);
  snprintf(buf, sizeof buf, "Pos.rec 与 recFromServo(Pos.ser) 自洽 (Δ=%.1e)", dx > dy ? dx : dy);
  check(buf, okFk && dx < 1e-9 && dy < 1e-9 && dz < 1e-9, NULL);

  /* 4) 推杆 -> loop() 真的把新角度写进舵机（验证 sketch 的 writeServo 被调用） */
  g_mockAnalog[MOCK_AX] = 1023;             /* A0 推到底：基座角增大 */
  g_mockMillis += 5000;                     /* 跨过最大步进间隔 */
  loop();
  captureFrame(pins, width, &frameTicks);
  unsigned wantNew = servoDriveTicksForDeg(Pos.ser.angle1);
  snprintf(buf, sizeof buf, "推 A0 后 通道0 脉宽 %u tick (Pos.angle1=%.3f -> %u)", width[0],
           Pos.ser.angle1, wantNew);
  check(buf, width[0] == wantNew && Pos.ser.angle1 > 90.0, NULL);
  g_mockAnalog[MOCK_AX] = 512;

  printf(">>> %s (失败 %d 项)\n", g_fail == 0 ? "ALL PASS" : "HAS FAILURES", g_fail);
  return g_fail == 0 ? 0 : 1;
}
