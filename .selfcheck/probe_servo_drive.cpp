//
// probe_servo_drive.cpp
// servo_drive.cpp（替代 Arduino Servo 库的 4 路 Timer1 舵机驱动）PC 端自检。
//
// 【为什么需要这个探针】
//   v1.3.0 为了塞进 Uno 的 flash，把 Arduino Servo 库换成了自研驱动。
//   换驱动最大的风险不是"能不能编译"，而是"脉宽还对不对、引脚顺序还对不对"——
//   这两个只要错一点，机械臂就会在上电后乱动或抖动，而 PC 端又看不到。
//   所以这里做两件事：
//     1) 用**测试侧独立复刻**的 Servo 库映射公式（不复用固件实现，否则是自证）
//        逐位比对 servoDriveTicksForDeg()；
//     2) 直接按 ISR 的方式步进伺服状态机 servoDriveStep()，用 mock 的 digitalWrite
//        日志验证"引脚顺序 + 极性 + 每路脉宽 + 20ms 帧长"，让时序逻辑在 PC 上可测。
//
// 【注意】本探针会调用 servoDriveBegin()，它会重置驱动的通道表 ——
//   这只会影响舵机状态，不影响 Pos / 运动学，故与其它探针无耦合。
//
#include "Arduino.h"
#include "servo_drive.h"

#include <stdio.h>

static int g_fail = 0;

static void check(const char *name, bool ok, const char *detail) {
  printf("  %-52s %s  %s\n", name, ok ? "PASS" : "FAIL", detail ? detail : "");
  if (!ok) g_fail++;
}

/* 测试侧独立复刻 Arduino Servo 库的映射（源码 %LOCALAPPDATA%\Arduino15\libraries\Servo\src\avr\Servo.cpp）：
 *   Servo::write(int angle): angle<544 视为角度 -> 夹 0..180 -> map(angle,0,180,544,2400)
 *   Servo::writeMicroseconds(): 夹 [544,2400] -> value -= TRIM_DURATION(2) -> usToTicks = (16*us)/8 */
static unsigned libTicks(double deg) {
  int a = (int)deg;
  if (a < 0) a = 0; else if (a > 180) a = 180;
  int us = 544 + (int)(((long)a * 1856L) / 180L);
  if (us < 544) us = 544; else if (us > 2400) us = 2400;
  us -= 2;
  return (unsigned)(us * 2);
}

#define FRAME_TICKS_EXPECT 40000u   /* 20ms * 2 tick/us */

struct Frame {
  int      logPin[MOCK_WRITE_LOG_MAX];
  int      logLevel[MOCK_WRITE_LOG_MAX];
  int      logCount;
  unsigned width[4];     /* 每路脉宽（tick） */
  int      pins[4];      /* 每路实际被拉高的引脚（-1 = 没出现） */
  unsigned frameTicks;   /* 帧长（tick） */
  int      startEvents;  /* 状态机返回 1（帧起点）的次数 */
};

/* 步进一个完整帧。前置条件：状态机停在帧起点（servoDriveBegin() 后即是；
 * 每次 captureFrame 结束后也会回到帧起点，所以可以连续调用）。 */
static void captureFrame(Frame *f) {
  mockDigitalWriteReset();
  uint16_t now = 0, next = 0;
  f->startEvents = 0;
  for (int k = 0; k < 4; k++) {
    uint8_t start = servoDriveStep(now, &next);
    if (start != 0u) { f->startEvents++; now = 0; }   /* 帧起点：ISR 在这里把 TCNT1 清零 */
    f->width[k] = (unsigned)(next - now);
    now = next;
  }
  (void) servoDriveStep(now, &next);                  /* 收尾事件：拉低最后一路 + 定帧尾 */
  f->frameTicks = (unsigned) next;

  f->logCount = g_mockWriteLogCount;
  if (f->logCount > MOCK_WRITE_LOG_MAX) f->logCount = MOCK_WRITE_LOG_MAX;
  int n = 0;
  for (int i = 0; i < f->logCount; i++) {
    f->logPin[i]   = g_mockWriteLogPin[i];
    f->logLevel[i] = g_mockWriteLogLevel[i];
    if (f->logLevel[i] == HIGH && n < 4) f->pins[n++] = f->logPin[i];
  }
  for (int k = n; k < 4; k++) f->pins[k] = -1;
}

static void attach1234(void) {
  servoDriveBegin();
  servoDriveAttach(0, 9);   /* b 水平回转 */
  servoDriveAttach(1, 7);   /* r 上臂俯仰 */
  servoDriveAttach(2, 8);   /* c 下臂俯仰 */
  servoDriveAttach(3, 6);   /* f 末端 */
}

int main(void) {
  printf("=== servo_drive 自检（脉宽映射 / 脉冲时序）===\n");
  Frame f;
  char buf[160];

  /* 1) 脉宽映射与 Servo 库逐位一致（含截断与越界夹取） */
  const double angles[11] = { -10.0, 0.0, 1.0, 45.0, 89.999, 90.0, 90.9, 105.0, 150.0, 180.0, 200.0 };
  for (int i = 0; i < 11; i++) {
    unsigned got  = servoDriveTicksForDeg(angles[i]);
    unsigned want = libTicks(angles[i]);
    snprintf(buf, sizeof buf, "映射 %7.3f° -> %4u tick (Servo 库 %4u = %.1fus)", angles[i], got, want,
             (want / 2 + 2) * 0.5);
    check(buf, got == want, NULL);
  }
  /* 两个端点值单独点名：0° 与 180° 就是舵机行程两端，错 1 个 tick 都会被看到 */
  check("0°   -> 544us-2us = 1084 tick", servoDriveTicksForDeg(0.0) == 1084u, NULL);
  check("180° -> 2400us-2us = 4796 tick", servoDriveTicksForDeg(180.0) == 4796u, NULL);
  check("90°  -> 1470us = 2940 tick（Servo::write(90) 同值）", servoDriveTicksForDeg(90.0) == 2940u, NULL);

  /* 2) 通道绑定顺序与极性：一帧内必须是 9H,9L,7H,7L,8H,8L,6H,6L */
  attach1234();
  servoDriveWrite(0, 10.0);
  servoDriveWrite(1, 60.0);
  servoDriveWrite(2, 120.0);
  servoDriveWrite(3, 170.0);
  captureFrame(&f);
  const int wantPin[4] = { 9, 7, 8, 6 };
  const int wantLogPin[8]   = { 9, 9, 7, 7, 8, 8, 6, 6 };
  const int wantLogLevel[8] = { HIGH, LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW };
  bool logOk = (f.logCount == 8);
  for (int i = 0; i < 8 && i < f.logCount; i++) {
    if (f.logPin[i] != wantLogPin[i] || f.logLevel[i] != wantLogLevel[i]) logOk = false;
  }
  snprintf(buf, sizeof buf, "一帧写引脚序列 9H,9L,7H,7L,8H,8L,6H,6L (实际 %d 条)", f.logCount);
  check(buf, logOk, NULL);
  check("四路脉宽顺序 = 引脚 9,7,8,6",
        f.pins[0] == 9 && f.pins[1] == 7 && f.pins[2] == 8 && f.pins[3] == 6, NULL);

  const double set4[4] = { 10.0, 60.0, 120.0, 170.0 };
  for (int k = 0; k < 4; k++) {
    snprintf(buf, sizeof buf, "通道%d 脉宽 %4u tick (%.1f° -> %4u)", k, f.width[k], set4[k], libTicks(set4[k]));
    check(buf, f.width[k] == libTicks(set4[k]), NULL);
  }
  snprintf(buf, sizeof buf, "帧长 %u tick (期望 %u = 20ms)", f.frameTicks, FRAME_TICKS_EXPECT);
  check(buf, f.frameTicks == FRAME_TICKS_EXPECT, NULL);
  check("一帧只有一次帧起点（TCNT1 清零）", f.startEvents == 1, NULL);
  check("每路引脚号与 attach 一致（9/7/8/6）",
        f.pins[0] == wantPin[0] && f.pins[1] == wantPin[1] && f.pins[2] == wantPin[2] && f.pins[3] == wantPin[3], NULL);

  /* 3) 连续三个帧：脉宽稳定、帧长稳定、每帧一次清零 */
  bool stable = true;
  for (int rep = 0; rep < 3; rep++) {
    captureFrame(&f);
    for (int k = 0; k < 4; k++) if (f.width[k] != libTicks(set4[k])) stable = false;
    if (f.frameTicks != FRAME_TICKS_EXPECT || f.startEvents != 1 || f.logCount != 8) stable = false;
  }
  check("连续 3 帧脉宽/帧长/时序事件都稳定", stable, NULL);

  /* 4) 未 attach 的通道不驱动任何引脚（但照样占用自己的时间片） */
  servoDriveBegin();
  servoDriveAttach(0, 9);
  servoDriveAttach(2, 8);
  servoDriveWrite(0, 30.0);
  servoDriveWrite(2, 150.0);
  captureFrame(&f);
  bool onlyBound = (f.logCount == 4);
  for (int i = 0; i < f.logCount; i++) {
    if (f.logPin[i] != 9 && f.logPin[i] != 8) onlyBound = false;
  }
  snprintf(buf, sizeof buf, "只绑 0/2 号通道：一帧只写 9 与 8（实际 %d 条）", f.logCount);
  check(buf, onlyBound, NULL);
  check("未绑通道不影响帧长（仍 20ms）", f.frameTicks == FRAME_TICKS_EXPECT, NULL);
  check("已绑通道脉宽仍然正确",
        f.width[0] == libTicks(30.0) && f.width[2] == libTicks(150.0), NULL);

  /* 5) 未写过角度时保持 Servo 库 attach 的默认中位 1500us（3000 tick） */
  servoDriveBegin();
  servoDriveAttach(0, 9);
  captureFrame(&f);
  snprintf(buf, sizeof buf, "未写角度默认脉宽 %u tick (期望 3000 = 1500us)", f.width[0]);
  check(buf, f.width[0] == 3000u, NULL);

  /* 6) 越界与小数：写角度也要夹到 0~180 并按整数截断（与 write((int)angle) 一致） */
  servoDriveBegin();
  servoDriveAttach(0, 9);
  servoDriveWrite(0, -5.0);
  captureFrame(&f);
  check("写 -5° 夹到 0°", f.width[0] == libTicks(0.0), NULL);
  servoDriveWrite(0, 300.0);
  captureFrame(&f);
  check("写 300° 夹到 180°", f.width[0] == libTicks(180.0), NULL);
  servoDriveWrite(0, 90.6);
  captureFrame(&f);
  check("写 90.6° 按整数截断成 90°", f.width[0] == libTicks(90.0), NULL);

  /* 7) 通道号越界不写坏任何东西（防未来误用） */
  servoDriveBegin();
  servoDriveAttach(0, 9);
  servoDriveAttach(9, 5);          /* 越界：必须被忽略 */
  servoDriveWrite(9, 10.0);        /* 越界：必须被忽略 */
  captureFrame(&f);
  check("越界通道 attach/write 被忽略（引脚 5 没被驱动）",
        f.logCount == 2 && f.logPin[0] == 9 && f.logLevel[0] == HIGH, NULL);

  printf(">>> %s (失败 %d 项)\n", g_fail == 0 ? "ALL PASS" : "HAS FAILURES", g_fail);
  return g_fail == 0 ? 0 : 1;
}
