//
// probe_servo_drive.cpp
// servo_drive.cpp（Arduino 官方 Servo 库的薄封装）PC 端自检。
//
// 【这个探针为什么长这样】
//   v1.3.0~v1.6.5 这一层是自研的 Timer1 顺序脉冲驱动，当年探针做的是
//   "逐位复刻 Servo 库的脉宽映射 + 按 ISR 步进状态机验引脚顺序和 30ms 帧长"。
//   v1.6.6 换回官方 Servo 库后：
//     - 脉宽映射与 20ms 帧时序**已经不是本工程的代码**，在 PC 上再复刻一遍
//       只是拿"我抄的公式"验证"我抄的公式"，没有意义，故整段删除；
//     - 剩下真正属于本工程的契约有三条，仍然必须在 PC 上拦住：
//         1) 通道 → 引脚的绑定顺序（历史 bug：上臂/下臂 D8/D7 曾对调，
//            接线上电后机械臂直接反向，PC 端只有这里能提前发现）；
//         2) 角度的整数截断与 0~180 夹取（与官方 write((int)angle) 一致）；
//         3) 越界通道号必须被静默忽略，绝不能写坏别的通道。
//
// 【注意】本探针不推进时钟、不读引脚电平，只读 mock/Servo.h 记录的
//   "谁绑到哪个引脚、最后写了多少度"，因此与其它探针无耦合。
//
#include "Arduino.h"
#include "servo_drive.h"
#include "Servo.h"

#include <stdio.h>

static int g_fail = 0;

static void check(const char *name, bool ok, const char *detail) {
  printf("  %-52s %s  %s\n", name, ok ? "PASS" : "FAIL", detail ? detail : "");
  if (!ok) g_fail++;
}

/* 按 meArm.ino setup() 的接线依次绑定四路：b / r / c / f。
 * 顺序即通道号，所以"第 ch 次 attach"就是"通道 ch"。 */
static void attachFour(void) {
  servoDriveBegin();
  servoDriveAttach(0, 9);   /* b 水平回转 */
  servoDriveAttach(1, 8);   /* r 上臂俯仰 */
  servoDriveAttach(2, 7);   /* c 下臂俯仰 */
  servoDriveAttach(3, 6);   /* f 末端夹爪 */
}

/* 通道 ch 落在哪个对象槽位（依据 §1 断言过的"attach 顺序 = 通道号"） */
static int chSlot(int ch) { return g_mockServo.attachSlot[ch]; }

static int attachPinOr(int k, int fallback) {
  return (k < g_mockServo.attachCount) ? g_mockServo.attachPin[k] : fallback;
}

static int totalWrites(void) {
  int n = 0;
  for (int k = 0; k < MOCK_SERVO_SLOT_MAX; k++) { n += g_mockServo.slotWrites[k]; }
  return n;
}

int main(void) {
  printf("=== servo_drive 自检（官方 Servo 库薄封装：绑定顺序 / 角度夹取）===\n");
  char buf[176];

  check("SERVO_CH_COUNT == 4（探针与固件的通道数约定）", SERVO_CH_COUNT == 4u, NULL);

  /* 1) 通道 → 引脚绑定顺序。
   *    这条是整份固件里唯一"错了就上电乱动"的硬约束，逐路点名。 */
  mockServoReset();
  attachFour();

  snprintf(buf, sizeof buf, "attach 调用 4 次（实际 %d 次：D%d,D%d,D%d,D%d）", g_mockServo.attachCount,
           attachPinOr(0, -1), attachPinOr(1, -1), attachPinOr(2, -1), attachPinOr(3, -1));
  check(buf, g_mockServo.attachCount == 4, NULL);

  const int wantPin[4] = { 9, 8, 7, 6 };
  const char *wantName[4] = { "b 基座", "r 上臂", "c 下臂", "f 夹爪" };
  for (int ch = 0; ch < 4; ch++) {
    snprintf(buf, sizeof buf, "通道%d(%s) 绑定引脚 D%d（期望 D%d）", ch, wantName[ch],
             attachPinOr(ch, -1), wantPin[ch]);
    check(buf, attachPinOr(ch, -2) == wantPin[ch], NULL);
  }
  check("上臂 D8 / 下臂 D7 没有对调（历史 bug 回归点）",
        attachPinOr(1, -1) == 8 && attachPinOr(2, -1) == 7, NULL);

  /* 每路必须是**独立对象**：四路撞到同一个槽位就会出现"写夹爪把基座也改了" */
  bool distinct = true;
  for (int a = 0; a < 4; a++) {
    for (int b = a + 1; b < 4; b++) {
      if (g_mockServo.attachSlot[a] == g_mockServo.attachSlot[b]) distinct = false;
    }
  }
  snprintf(buf, sizeof buf, "四路各占独立对象槽位（%d,%d,%d,%d）", g_mockServo.attachSlot[0],
           g_mockServo.attachSlot[1], g_mockServo.attachSlot[2], g_mockServo.attachSlot[3]);
  check(buf, distinct && g_mockServo.attachSlot[0] >= 0, NULL);

  /* 2) servoDriveBegin() 是预留的空钩子：重复调用不得动已绑定的通道
   *    （旧自研版会重置通道表，改回官方库后语义变了，这里钉住新语义） */
  int attachBefore = g_mockServo.attachCount;
  int pinBefore[4];
  for (int ch = 0; ch < 4; ch++) { pinBefore[ch] = g_mockServo.slotPin[chSlot(ch)]; }
  servoDriveBegin();
  bool beginNoop = (g_mockServo.attachCount == attachBefore);
  for (int ch = 0; ch < 4; ch++) {
    if (g_mockServo.slotPin[chSlot(ch)] != pinBefore[ch]) beginNoop = false;
  }
  check("servoDriveBegin() 不改动已绑定通道（空钩子语义）", beginNoop, NULL);

  /* 3) 角度截断与夹取：必须与官方 Servo::write((int)angle) 完全一致 */
  struct AngleCase { double in; int want; };
  const AngleCase cases[] = {
    { -5.0,   0 },
    { -0.5,   0 },
    {  0.0,   0 },
    { 45.9,  45 },
    { 90.0,  90 },
    { 90.6,  90 },
    { 179.9, 179 },
    { 180.0, 180 },
    { 300.0, 180 },
  };
  for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    servoDriveWrite(0, cases[i].in);
    int got = g_mockServo.slotAngle[chSlot(0)];
    snprintf(buf, sizeof buf, "写 %7.1f° -> 库收到 %3d°（期望 %3d°：截断 + 夹取 0~180）",
             cases[i].in, got, cases[i].want);
    check(buf, got == cases[i].want, NULL);
  }

  /* 4) 四路互不干扰：一次写好四路，各自的槽位里必须是各自的值 */
  const double set4[4] = { 10.0, 60.0, 120.0, 170.0 };
  for (int ch = 0; ch < 4; ch++) { servoDriveWrite((uint8_t)ch, set4[ch]); }
  for (int ch = 0; ch < 4; ch++) {
    int got = g_mockServo.slotAngle[chSlot(ch)];
    snprintf(buf, sizeof buf, "通道%d 写入 %.0f° 后库收到 %d°", ch, set4[ch], got);
    check(buf, got == (int)set4[ch], NULL);
  }
  bool onceEach = true;
  for (int ch = 0; ch < 4; ch++) {
    if (g_mockServo.slotWrites[chSlot(ch)] == 0) onceEach = false;
  }
  check("四路都确实收到了 write（没有一路被漏掉）", onceEach, NULL);

  /* 5) 越界通道号（>= SERVO_CH_COUNT）：attach/write 都必须被静默忽略，
   *    既不能多绑一个引脚，也不能写坏任何已绑定通道 —— 防未来误用 */
  int acBefore = g_mockServo.attachCount;
  int wrBefore = totalWrites();
  servoDriveAttach(4, 5);
  servoDriveAttach(9, 5);
  servoDriveWrite(4, 10.0);
  servoDriveWrite(9, 10.0);
  servoDriveWrite(255, 10.0);
  int wrAfter = totalWrites();
  snprintf(buf, sizeof buf, "越界通道 attach/write 被忽略（attach %d->%d，write %d->%d）", acBefore,
           g_mockServo.attachCount, wrBefore, wrAfter);
  check(buf, g_mockServo.attachCount == acBefore && wrAfter == wrBefore, NULL);

  bool pin5Seen = false;
  for (int k = 0; k < g_mockServo.attachCount; k++) {
    if (g_mockServo.attachPin[k] == 5) pin5Seen = true;
  }
  check("越界 attach 没把 D5 绑上去（引脚表未被污染）", !pin5Seen, NULL);

  /* 6) 越界 write 之后，已绑定通道的值仍是上一步写的值（没有被顺手改掉） */
  bool untouched = true;
  for (int ch = 0; ch < 4; ch++) {
    if (g_mockServo.slotAngle[chSlot(ch)] != (int)set4[ch]) untouched = false;
  }
  check("越界 write 不影响已绑定通道的当前角度", untouched, NULL);

  printf(">>> %s (失败 %d 项)\n", g_fail == 0 ? "ALL PASS" : "HAS FAILURES", g_fail);
  return g_fail == 0 ? 0 : 1;
}
