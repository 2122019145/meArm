//
// wearm_ino_test.cpp
// 直接编译 meArm.ino 本体（不手抄复刻），核对 sketch 的接线与上电行为。
//
// （文件名保留 wearm_ino_test：它测的是"主 sketch 本体"，
//   而主 sketch 已随工作区一起改名为 meArm.ino。）
//
// 【为什么直接 #include "../meArm.ino"】
// 早期版本是把 setup()/loop() 手工抄一份到本文件里编译，只能证明"抄件能编译"。
// 抄件已经漂移过一次：抄件写 servos[2].attach(10)、servos[3].attach(11)，
// 而真 sketch 是 attach(7)/attach(8)，抄件照样编译通过、零警告，
// 完全没发现问题。改成本文件直接 include 真 sketch 之后，接线错误才会暴露。
//
// 注意：.ino 不是合法的 C++ 翻译单元，但它是纯 C++ 文本，直接 include 即可；
// PC 端只做语法/类型/行为自检，不参与烧录。
//
// 【v1.6.6】sketch 的舵机层换成"Arduino 官方 Servo 库的薄封装"
//   （servo_drive.cpp）。脉宽与帧时序归官方库管、PC 端不复刻，所以这里改成
//   读 mock/Servo.h 记录的"每路最后被写进库的角度"：
//     - 先由 writeServo() 的方向镜像算出**物理角**（physAngle），
//     - 再按官方 Servo::write((int)angle) 的语义取整/夹取（libAngle），
//     - 最后和库里实际收到的整数角度逐路比对。
//   这条链路正好覆盖用户实机报过的两个方向问题（左右反向、夹爪 O/S 反向）：
//   镜像一旦丢，比对立刻失败。
//
#include "Arduino.h"
#include "../weArm_config.h"
#include "servo_drive.h"
#include "Servo.h"
#include "../meArm.ino"

#include <stdio.h>
#include <math.h>

static int g_fail = 0;

static void check(const char *name, bool ok, const char *detail) {
  printf("  %-46s %s  %s\n", name, ok ? "PASS" : "FAIL", detail ? detail : "");
  if (!ok) g_fail++;
}

/* 逻辑角 -> 真正会写进 Servo 库的物理角：把 writeServo() 的方向镜像算进去。
 * 基座（通道 0 / b）与末端夹爪（通道 3 / f）在这台机器上是反向装配的，
 * writeServo() 把这两个通道的物理角算成 (min + max) - logical；
 * 限位、反解、录制、绘图用的全都是逻辑角（见 weArm_config.h 第 3 节）。 */
static double physAngle(int ch, double logical) {
#if WEARM_MIRROR_BASE
  if (ch == 0) { logical = 180.0 - logical; }   /* 硬编码 (minB+maxB)=180 */
#endif
  if (ch == 1 && g_mirrorShoulder) { logical = 180.0 - logical; }
  if (ch == 2 && g_mirrorElbow)    { logical = 180.0 - logical; }
#if WEARM_MIRROR_TOOL
  if (ch == 3) { logical = 210.0 - logical; }   /* 硬编码 (minF+maxF)=210 */
#endif
  return logical;
}

/* 官方 Servo::write(int angle) 之前，servo_drive.cpp 做的处理：
 * 整数截断 + 夹到 0~180。这里独立算一遍，作为期望值。 */
static int libAngle(double deg) {
  int a = (int)deg;
  if (a < 0) { a = 0; } else if (a > 180) { a = 180; }
  return a;
}

/* 通道 ch 当前真正被写进 Servo 库的整数角度（-1 = 这一路从没写过）。
 * attach 顺序即通道号，所以"第 ch 次 attach 的槽位"就是通道 ch 的槽位。 */
static int gotAngle(int ch) {
  int slot = g_mockServo.attachSlot[ch];
  return (slot < 0) ? -1 : g_mockServo.slotAngle[slot];
}

static int gotPin(int ch) {
  int slot = g_mockServo.attachSlot[ch];
  return (slot < 0) ? -1 : g_mockServo.slotPin[slot];
}

int main(void) {
  printf("=== meArm.ino 真实 sketch 自检（直接 include 真文件）===\n");

  char buf[176];

  /* 0) setup() 的接线动作：servoDriveAttach(0..3) 必须按 9,8,7,6 绑好，
   *    并且每一路都被设成 OUTPUT（官方库的 attach() 里就是这一句）。
   *    用 mock/Servo.h 的绑定记录看 —— 它就是"真的动了哪根脚"的证据。 */
  mockServoReset();
  setup();
  const int wantPin[4] = { 9, 8, 7, 6 };
  const char *wantName[4] = { "b 基座", "r 上臂", "c 下臂", "f 夹爪" };
  for (int i = 0; i < 4; i++) {
    snprintf(buf, sizeof buf, "attach 顺序[%d] %s -> 引脚 %d (期望 %d)", i, wantName[i], gotPin(i),
             wantPin[i]);
    check(buf, gotPin(i) == wantPin[i], NULL);
  }
  for (int i = 0; i < 4; i++) {
    snprintf(buf, sizeof buf, "引脚 %d 被设成 OUTPUT (实际 %d)", wantPin[i], g_mockPinMode[wantPin[i]]);
    check(buf, g_mockPinMode[wantPin[i]] == OUTPUT, NULL);
  }
  check("上臂 D8 / 下臂 D7 没有对调（历史 bug 回归点）",
        gotPin(1) == 8 && gotPin(2) == 7, NULL);

  /* 1) 上电姿态：setup() 末尾的 writeServo() 必须把 POS_HOME 的角度写进库，
   *    且每一路都经过了方向镜像。一路都不能漏。 */
  for (int i = 0; i < 4; i++) {
    double logical = (i == 0) ? Pos.ser.angle1 : (i == 1) ? Pos.ser.angle2
                   : (i == 2) ? Pos.ser.angle3 : Pos.ser.angle4;
    int want = libAngle(physAngle(i, logical));
    snprintf(buf, sizeof buf, "上电写入 通道%d(%s)=%d° (逻辑=%.6f -> 物理=%d)", i, wantName[i],
             gotAngle(i), logical, want);
    check(buf, gotAngle(i) == want, NULL);
  }

  /* 2) 上电角度必须是 POS_HOME 的反解结果（末端行程中位），不能是 0 —— 角度 0 会让
   *    机械臂在上电瞬间甩出去，这是老版本踩过的坑。
   *    用 posGetHomeAngles() 自洽比较：POS_HOME 改了以后无需再硬编码角度。 */
  {
    SER homeSer = Pos.ser;
    bool haveHome = posGetHomeAngles(&homeSer);
    snprintf(buf, sizeof buf, "上电姿态 = (%.6f,%.6f,%.6f,%.6f)", Pos.ser.angle1, Pos.ser.angle2,
             Pos.ser.angle3, Pos.ser.angle4);
    check(buf, haveHome && fabs(Pos.ser.angle1 - homeSer.angle1) < 1e-6 &&
               fabs(Pos.ser.angle2 - homeSer.angle2) < 1e-6 &&
               fabs(Pos.ser.angle3 - homeSer.angle3) < 1e-6, NULL);
  }
  /* 末端 f 的电气行程是 [0,180]，posInit() 取夹爪"开/合"两个名义角的中点 105
   * （不是 90）—— 这一条曾经把 "assert f == 90" 写错，是探针自己的错，
   * 固件行为才是对的。 */
  const double wantF = (TOOL_ANGLE_OPEN + TOOL_ANGLE_CLOSE) * 0.5;
  snprintf(buf, sizeof buf, "末端上电取行程中位 f=%.1f (期望 %.1f)", Pos.ser.angle4, wantF);
  check(buf, fabs(Pos.ser.angle4 - wantF) < 1e-9, NULL);
  check("上电写入的不是逻辑 0° 那一端（不会甩向原点）",
        gotAngle(0) != libAngle(physAngle(0, 0.0)) &&
        gotAngle(1) != libAngle(physAngle(1, 0.0)) &&
        gotAngle(2) != libAngle(physAngle(2, 0.0)), NULL);

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
  int wantNew = libAngle(physAngle(0, Pos.ser.angle1));
  snprintf(buf, sizeof buf, "推 A0 后 通道0 写入 %d° (angle1=%.3f -> 期望 %d)", gotAngle(0),
           Pos.ser.angle1, wantNew);
  check(buf, gotAngle(0) == wantNew && Pos.ser.angle1 > 90.0, NULL);
  g_mockAnalog[MOCK_AX] = 512;

  /* 5) 方向镜像：把基座与夹爪的逻辑角放到偏离中位的地方，检查真正写进库的是
   *    "镜像后"的那个角，而不是逻辑角本身。这条用例正面锁住用户报的两个
   *    现象：左右旋转的舵机转向反了、抓取时 O/S 反了。
   *    夹爪的镜像尤其要看：逻辑角范围放宽到 0~180 之后，镜像和仍是 210，
   *    所以逻辑 0 会算出物理 210（超出电气行程，由 servo_drive 夹到 180），
   *    方向反不反仍然看得见。 */
#if WEARM_MIRROR_BASE || WEARM_MIRROR_TOOL
  Pos.ser.angle1 = 30.0;
  Pos.ser.angle4 = 60.0;
  writeServo();
  int mir0 = libAngle(physAngle(0, 30.0));
  int mir3 = libAngle(physAngle(3, 60.0));
  snprintf(buf, sizeof buf, "镜像 通道0: 逻辑30° -> 写入 %d° (未镜像会写 %d°)", gotAngle(0),
           libAngle(30.0));
  check(buf, gotAngle(0) == mir0, NULL);
  snprintf(buf, sizeof buf, "镜像 通道3: 逻辑60° -> 写入 %d° (未镜像会写 %d°)", gotAngle(3),
           libAngle(60.0));
  check(buf, gotAngle(3) == mir3, NULL);
  check("镜像确实改变了写入角度（b 30° 与 f 60° 都不是原值）",
        mir0 != libAngle(30.0) && mir3 != libAngle(60.0), NULL);
  /* 镜像和是这台机器的装配常数：夹爪 210。逻辑角可能镜像到超出 0~180，
   * 那由 servo_drive 夹到电气行程端点，不再是本层的契约。 */
  check("夹爪镜像和 = TOOL_MIRROR_SUM(210)", TOOL_MIRROR_SUM == 210.0, NULL);
  posInit();
  writeServo();
#endif

  printf(">>> %s (失败 %d 项)\n", g_fail == 0 ? "ALL PASS" : "HAS FAILURES", g_fail);
  return g_fail == 0 ? 0 : 1;
}
