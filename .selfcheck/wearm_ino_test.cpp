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
#include "Arduino.h"
#include "Servo.h"
#include "../wearm.ino"

#include <stdio.h>
#include <math.h>

static int g_fail = 0;

static void check(const char *name, bool ok, const char *detail) {
  printf("  %-46s %s  %s\n", name, ok ? "PASS" : "FAIL", detail ? detail : "");
  if (!ok) g_fail++;
}

int main(void) {
  printf("=== wearm.ino 真实 sketch 自检（直接 include 真文件）===\n");

  mockServoReset();
  setup();

  char buf[160];

  /* 1) 接线：sketch 里 servos[1..4] 的 attach 引脚必须与注释/硬件一致 */
  const int wantPin[5] = { -1, 9, 7, 8, 6 };
  for (int i = 1; i <= 4; i++) {
    int got = mockServoPin(i);
    snprintf(buf, sizeof buf, "servos[%d] attach 引脚 %d (期望 %d)", i, got, wantPin[i]);
    check(buf, got == wantPin[i], NULL);
  }

  /* 2) 上电姿态：setup() 末尾的 writeServo() 必须把 POS_HOME 的角度写进舵机。
   *    这里直接和 Pos.ser 比 —— 只要 Pos 与舵机一致就说明 writeServo 没漏项。 */
  for (int i = 1; i <= 4; i++) {
    double want = (i == 1) ? Pos.ser.angle1 : (i == 2) ? Pos.ser.angle2
                : (i == 3) ? Pos.ser.angle3 : Pos.ser.angle4;
    int got = mockServoAngle(i);
    snprintf(buf, sizeof buf, "上电写舵机 servos[%d] = %d (Pos=%.1f)", i, got, want);
    check(buf, got == (int) want, NULL);
  }

  /* 3) 上电角度必须是 POS_HOME 对应的 (90,90,90, 末端行程中位)，不能是 0 —— 角度 0 会让
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
  check("上电姿态非 0（不会甩向原点）",
        mockServoAngle(1) != 0 && mockServoAngle(2) != 0 && mockServoAngle(3) != 0, NULL);

  /* 4) loop(): 手柄不动 -> 角度不变、坐标自洽；writeServo 仍然照写 */
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

  /* 5) 推杆 -> loop() 真的把新角度写进舵机（验证 sketch 的 writeServo 被调用） */
  mockServoReset();   /* 只清记录，不改 Pos */
  g_mockAnalog[MOCK_AX] = 1023;             /* A0 推到底：基座角增大 */
  g_mockMillis += 5000;                     /* 跨过最大步进间隔 */
  loop();
  int wroteBase = mockServoAngle(1);
  snprintf(buf, sizeof buf, "推 A0 后 loop() 把新角度写进 servos[1] = %d", wroteBase);
  check(buf, wroteBase > 90, NULL);
  g_mockAnalog[MOCK_AX] = 512;

  printf(">>> %s (失败 %d 项)\n", g_fail == 0 ? "ALL PASS" : "HAS FAILURES", g_fail);
  return g_fail == 0 ? 0 : 1;
}
