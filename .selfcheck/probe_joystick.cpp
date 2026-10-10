/* probe_joystick.cpp —— 双摇杆手柄层端到端回归（【角度模式】：摇杆直接控关节角）
 * 直接链接固件：joystick_control.cpp + constant_and_positions.cpp + move.cpp
 *
 * 仿真方式：改写 g_mockAnalog（A0 起算下标）模拟推杆，推进 g_mockMillis 走时间门控。
 *           （串口命令已搬到 serial_protocol 模块，由 probe_protocol.cpp 负责测试。）
 *
 * 角度模式与旧的坐标模式的区别：
 *   被控量是 Pos.ser.angle1..angle4，Pos.rec.x/y/z 是正运动学算出的派生量。
 *   所以断言主要看关节角，坐标只用来验证"角度变了坐标跟着变"，以及位置边界的作用。 */
#include <cmath>
#include <cstdio>
#include <cstring>
#include "Arduino.h"
#include "constant_and_positions.h"
#include "move.h"
#include "joystick_control.h"

static int failures = 0;

static void check(const char *name, bool ok, const char *detail) {
  printf("  %-48s %s  %s\n", name, ok ? "PASS" : "FAIL", detail);
  if (!ok) failures++;
}

/* 把四路轴都回到中位，并清掉串口输入 */
static void resetInputs(void) {
  for (int i = 0; i < 8; i++) g_mockAnalog[i] = 0;
  g_mockAnalog[MOCK_AX] = 512;
  g_mockAnalog[MOCK_AY] = 512;
  g_mockAnalog[MOCK_TX] = 512;
  g_mockAnalog[MOCK_TY] = 512;
  mockSerialClear();
}

/* 保持当前输入不变，推进若干轮 loop（每轮推进 ms 毫秒） */
static void runLoop(int rounds, unsigned long ms) {
  for (int i = 0; i < rounds; i++) {
    g_mockMillis += ms;
    joystickLoop();
  }
}

/* 四个关节角组成的快照，便于比较 */
struct Joints { double b, r, c, f; };
static struct Joints snap(void) {
  struct Joints j;
  j.b = Pos.ser.angle1; j.r = Pos.ser.angle2;
  j.c = Pos.ser.angle3; j.f = Pos.ser.angle4;
  return j;
}
static bool onlyChanged(const struct Joints &a, const struct Joints &b, int which) {
  /* which: 0=b 1=r 2=c 3=f；其它三个关节必须一模一样 */
  double d[4] = { b.b - a.b, b.r - a.r, b.c - a.c, b.f - a.f };
  for (int i = 0; i < 4; i++) {
    if (i == which) continue;
    if (fabs(d[i]) > 1e-9) return false;
  }
  return true;
}

int main(void) {
  resetInputs();
  g_mockMillis = 1000;
  joystickSetup();
  posInit();

  char buf[260];

  printf("=== 1) 四路轴各控一个关节角，互不干扰 ===\n");
  {
    /* A0 右推 -> 基座 b 增大 */
    resetInputs(); posInit();
    struct Joints j0 = snap();
    g_mockAnalog[MOCK_AX] = 900;
    runLoop(6, 20);
    snprintf(buf, sizeof(buf), "b: %.1f -> %.1f  (r=%.1f c=%.1f f=%.1f)", j0.b, Pos.ser.angle1,
             Pos.ser.angle2, Pos.ser.angle3, Pos.ser.angle4);
    check("A0 右推 -> 只有 b(angle1) 增大",
          Pos.ser.angle1 > j0.b && onlyChanged(j0, snap(), 0), buf);

    /* A0 左推 -> 基座 b 减小 */
    resetInputs(); posInit();
    j0 = snap();
    g_mockAnalog[MOCK_AX] = 200;
    runLoop(6, 20);
    snprintf(buf, sizeof(buf), "b: %.1f -> %.1f", j0.b, Pos.ser.angle1);
    check("A0 左推 -> 只有 b(angle1) 减小",
          Pos.ser.angle1 < j0.b && onlyChanged(j0, snap(), 0), buf);

    /* A1 前推 -> 上臂 r 减小 */
    resetInputs(); posInit();
    j0 = snap();
    g_mockAnalog[MOCK_AY] = 900;
    runLoop(6, 20);
    snprintf(buf, sizeof(buf), "r: %.1f -> %.1f  (b=%.1f c=%.1f f=%.1f)", j0.r, Pos.ser.angle2,
             Pos.ser.angle1, Pos.ser.angle3, Pos.ser.angle4);
    check("A1 前推 -> 只有 r(angle2) 减小",
          Pos.ser.angle2 < j0.r && onlyChanged(j0, snap(), 1), buf);

    /* A1 后拉 -> 上臂 r 增大 */
    resetInputs(); posInit();
    j0 = snap();
    g_mockAnalog[MOCK_AY] = 200;
    runLoop(6, 20);
    snprintf(buf, sizeof(buf), "r: %.1f -> %.1f", j0.r, Pos.ser.angle2);
    check("A1 后拉 -> 只有 r(angle2) 增大",
          Pos.ser.angle2 > j0.r && onlyChanged(j0, snap(), 1), buf);

    /* A3 前推 -> 下臂 c 减小 */
    resetInputs(); posInit();
    j0 = snap();
    g_mockAnalog[MOCK_TY] = 900;
    runLoop(6, 20);
    snprintf(buf, sizeof(buf), "c: %.1f -> %.1f  (b=%.1f r=%.1f f=%.1f)", j0.c, Pos.ser.angle3,
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle4);
    check("A3 前推 -> 只有 c(angle3) 减小",
          Pos.ser.angle3 < j0.c && onlyChanged(j0, snap(), 2), buf);

    /* A3 后拉 -> 下臂 c 增大 */
    resetInputs(); posInit();
    j0 = snap();
    g_mockAnalog[MOCK_TY] = 200;
    runLoop(6, 20);
    snprintf(buf, sizeof(buf), "c: %.1f -> %.1f", j0.c, Pos.ser.angle3);
    check("A3 后拉 -> 只有 c(angle3) 增大",
          Pos.ser.angle3 > j0.c && onlyChanged(j0, snap(), 2), buf);
  }

  printf("=== 2) 坐标是角度的派生量：角度变了坐标必须跟着变且自洽 ===\n");
  {
    resetInputs(); posInit();
    double x0 = Pos.rec.x, z0 = Pos.rec.z;
    g_mockAnalog[MOCK_AY] = 900;            /* 前推 -> 上臂 r 减小（放下） */
    runLoop(8, 20);
    snprintf(buf, sizeof(buf), "末端 (%.1f,%.1f,%.1f)，起 x=%.1f z=%.1f",
             Pos.rec.x, Pos.rec.y, Pos.rec.z, x0, z0);
    /* 这里只要求坐标跟着角度变，不判增/减方向（方向由第 1 段专测） */
    check("上臂角度变化后末端坐标随之改变", fabs(Pos.rec.z - z0) > 1e-9, buf);

    /* 用固件自己的正运动学独立复算一遍，验证 Pos.rec 确实与 Pos.ser 严格对应 */
    REC chk;
    bool ok = recFromServo(&chk, &Pos.ser);
    double err = ok ? sqrt(pow(chk.x - Pos.rec.x, 2) + pow(chk.y - Pos.rec.y, 2) +
                           pow(chk.z - Pos.rec.z, 2)) : 1e9;
    snprintf(buf, sizeof(buf), "复算 (%.3f,%.3f,%.3f) 误差=%.3g", chk.x, chk.y, chk.z, err);
    check("Pos.rec == recFromServo(Pos.ser)", ok && err < 1e-9, buf);
  }

  printf("=== 3) 末端舵机 angle4：右手柄左右推，停在机械限位内 ===\n");
  {
    /* 固件约定（沿用老手柄的手感）：A2 右推 = angle4 减小(收回)，左推 = 增大(张开)。
     * 上机若觉得反了，把 joystick_control.cpp 里 toolStep 的 sign 取反即可。 */
    resetInputs(); posInit();
    double f0 = Pos.ser.angle4;
    g_mockAnalog[MOCK_TX] = 900;              /* 右推 -> 角度减小 */
    runLoop(400, 30);
    snprintf(buf, sizeof(buf), "f: %.1f -> %.1f (下限 60)", f0, Pos.ser.angle4);
    check("A2 右推 -> angle4 减到 60 停住",
          Pos.ser.angle4 < f0 && Pos.ser.angle4 >= 60.0 - 1e-9, buf);

    resetInputs(); posInit();
    f0 = Pos.ser.angle4;
    g_mockAnalog[MOCK_TX] = 200;              /* 左推 -> 角度增大 */
    runLoop(400, 30);
        snprintf(buf, sizeof(buf), "f: %.1f -> %.1f (上限 150)", f0, Pos.ser.angle4);
    check("A2 左推 -> angle4 增到 150 停住",
          Pos.ser.angle4 > f0 && Pos.ser.angle4 <= 150.0 + 1e-9, buf);
  }

  printf("=== 4) 斜推可以同时动多个关节（角度模式下逐轴独立） ===\n");
  {
    resetInputs(); posInit();
    struct Joints j0 = snap();
    g_mockAnalog[MOCK_AX] = 900;    /* A0 大幅 */
    g_mockAnalog[MOCK_AY] = 600;    /* A1 小幅 */
    runLoop(6, 20);
    snprintf(buf, sizeof(buf), "Δb=%.1f Δr=%.1f", Pos.ser.angle1 - j0.b, Pos.ser.angle2 - j0.r);
    check("A0 与 A1 同时推 -> b 与 r 都动",
          fabs(Pos.ser.angle1 - j0.b) > 1e-9 && fabs(Pos.ser.angle2 - j0.r) > 1e-9, buf);
  }

  printf("=== 5) 死区抖动不产生任何动作 ===\n");
  {
    resetInputs(); posInit();
    struct Joints j0 = snap();
    /* JOY_DEADZONE 是 joystick_control.cpp 内部宏(=40，v1.6.4 起；旧版 15)，探针用等价值 */
    const int DZ = 40;
    g_mockAnalog[MOCK_AX] = 512 + DZ;                /* 刚好在死区边界 */
    g_mockAnalog[MOCK_AY] = 512 - DZ;
    g_mockAnalog[MOCK_TX] = 512 + DZ;
    g_mockAnalog[MOCK_TY] = 512 - DZ;
    runLoop(20, 20);
    struct Joints j1 = snap();
    snprintf(buf, sizeof(buf), "Δb=%.4f Δr=%.4f Δc=%.4f Δf=%.4f",
             j1.b - j0.b, j1.r - j0.r, j1.c - j0.c, j1.f - j0.f);
    check("死区内的偏转不驱动任何关节",
          fabs(j1.b - j0.b) < 1e-9 && fabs(j1.r - j0.r) < 1e-9 &&
          fabs(j1.c - j0.c) < 1e-9 && fabs(j1.f - j0.f) < 1e-9, buf);

    /* 新增：施密特迟滞起控门槛测试（41..64 计数段既不被中位跟踪吸收，也不会步进）
     * 【必须先清掉上一段留下的状态】迟滞是"锁存"而不是"瞬时判据"：
     * 上一段把摇杆推过 65 后 s_axisHot[] 已经是 true，门槛降到 40，此时再给 +50
     * 就会被当成推杆；另外运行期中位跟踪还可能留下 ±20 计数级别的偏移，
     * 让 +50 实际变成 +70。所以先回中、重新标定、再跑两轮把锁存清干净。 */
    resetInputs(); posInit();
    joystickSetup();   /* 四路都在 512：s_centerOff ≈ 0，抵消上一段的跟踪偏移 */
    runLoop(2, 20);    /* |d| = 0 < 40 ⇒ 四路 s_axisHot 全部落回 false */
    j0 = snap();
    /* JOY_HYST = 25 */
    /* 起控门槛 = 65 */
    g_mockAnalog[MOCK_AX] = 512 + 50;  /* 落在死区 40 与起控门槛 65 之间 */
    g_mockAnalog[MOCK_AY] = 512 + 50;
    g_mockAnalog[MOCK_TX] = 512 + 50;
    g_mockAnalog[MOCK_TY] = 512 + 50;
    runLoop(200, 20);  /* 连续 200 轮，四个关节必须一动不动 */
    struct Joints j2 = snap();
    snprintf(buf, sizeof(buf), "死区与起控门槛之间(50计数) Δb=%.4f Δr=%.4f Δc=%.4f Δf=%.4f",
             j2.b - j0.b, j2.r - j0.r, j2.c - j0.c, j2.f - j0.f);
    check("施密特迟滞：死区与起控门槛之间的偏转不驱动任何关节",
          fabs(j2.b - j0.b) < 1e-9 && fabs(j2.r - j0.r) < 1e-9 &&
          fabs(j2.c - j0.c) < 1e-9 && fabs(j2.f - j0.f) < 1e-9, buf);

    /* 接下来把该路设回 512 并多跑几十轮让中位回到中心 */
    resetInputs();
    g_mockMillis += 200 * 20;  /* 跳过中位跟踪时间 */
    joystickLoop();  /* 让中位跟踪学掉这 50 个计数 */
    runLoop(80, 20);  /* 再跑 80 轮让中位完全回到中心 */
  }

  printf("=== 6) 调速档位影响转角速度（同起点比较）===\n");
  {
    /* 慢速 */
    adjustSpeed(SPEED_SLOW);
    resetInputs(); posInit();
    double b0 = Pos.ser.angle1;
    g_mockAnalog[MOCK_AX] = 900;
    runLoop(10, 30);
    double slowDb = Pos.ser.angle1 - b0;

    /* 快速：必须回到同一起点再测，否则剩余行程不同会得出错误结论 */
    adjustSpeed(SPEED_FAST);
    resetInputs(); posInit();
    b0 = Pos.ser.angle1;
    g_mockAnalog[MOCK_AX] = 900;
    runLoop(10, 30);
    double fastDb = Pos.ser.angle1 - b0;

    snprintf(buf, sizeof(buf), "慢速 Δb=%.1f 度  快速 Δb=%.1f 度", slowDb, fastDb);
    check("同起点同轮数下 快速转角 > 慢速转角", fastDb > slowDb, buf);
  }

  printf("=== 7) 持续推杆：每个关节最终都停在 servoLimit 内 ===\n");
  {
    adjustSpeed(SPEED_NORMAL);
    resetInputs(); posInit();
    g_mockAnalog[MOCK_AX] = 900;   /* b 一直加 */
    g_mockAnalog[MOCK_AY] = 900;   /* r 一直减 */
    g_mockAnalog[MOCK_TY] = 900;   /* c 一直减 */
    g_mockAnalog[MOCK_TX] = 900;   /* f 一直减 */
    runLoop(600, 30);
    struct Joints j1 = snap();
    bool inRange = j1.b <= 180.0 + 1e-6 && j1.r >= 0.0 - 1e-6 &&
                   j1.c >= 0.0 - 1e-6 && j1.f >= 60.0 - 1e-6;
    bool valid = (j1.b == j1.b) && (j1.r == j1.r) && (j1.c == j1.c) && (j1.f == j1.f);
        snprintf(buf, sizeof(buf), "b=%.1f(max 180) r=%.1f(min 0) c=%.1f(min 0) f=%.1f",
             j1.b, j1.r, j1.c, j1.f);
    check("四路都推到头：角度全部落在 servoLimit 内且非 NaN", inRange && valid, buf);
  }

  printf("=== 8) 4000 轮随机推杆压力测试 ===\n");
  {
    resetInputs(); posInit();
    adjustSpeed(SPEED_FAST);
    unsigned long seed = 987654321UL;
    int bad = 0;
    for (int i = 0; i < 4000; i++) {
      seed = seed * 1103515245UL + 12345UL;
      int which = (int)((seed >> 16) % 4UL);
      int v = (int)((seed >> 8) % 1024UL);      /* 0..1023 */
      resetInputs();
      g_mockAnalog[which] = v;                  /* 轮流扰动四路轴 */
      runLoop(1, 25);

      /* 角度必须全部在 servoLimit 内 */
      if (Pos.ser.angle1 < 0.0 - 1e-6 || Pos.ser.angle1 > 180.0 + 1e-6) bad++;
      if (Pos.ser.angle2 < 0.0 - 1e-6 || Pos.ser.angle2 > 180.0 + 1e-6) bad++;
      if (Pos.ser.angle3 < 0.0 - 1e-6 || Pos.ser.angle3 > 180.0 + 1e-6) bad++;
      if (Pos.ser.angle4 < 60.0 - 1e-6 || Pos.ser.angle4 > 150.0 + 1e-6) bad++;
      /* 不能出现 NaN */
      if (!(Pos.ser.angle1 == Pos.ser.angle1) || !(Pos.ser.angle2 == Pos.ser.angle2) ||
          !(Pos.ser.angle3 == Pos.ser.angle3) || !(Pos.ser.angle4 == Pos.ser.angle4)) bad++;
      /* 关键不变量：坐标必须与角度严格自洽 */
      REC chk;
      if (!recFromServo(&chk, &Pos.ser)) { bad++; continue; }
      if (fabs(chk.x - Pos.rec.x) > 1e-9 || fabs(chk.y - Pos.rec.y) > 1e-9 ||
          fabs(chk.z - Pos.rec.z) > 1e-9) bad++;
    }
    snprintf(buf, sizeof(buf), "violations=%d 末态 b=%.1f r=%.1f c=%.1f f=%.1f (%.1f,%.1f,%.1f)",
             bad, Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3, Pos.ser.angle4,
             Pos.rec.x, Pos.rec.y, Pos.rec.z);
    check("4000 轮随机推杆：零越界/零 NaN/坐标与角度始终自洽", bad == 0, buf);
  }

  printf("=== 9) 摇杆中位自标定（修「无故乱动」）===\n");
  {
    /* 9a) 机械中位偏 +70 计数（超过死区 40、但在标定上限 80 内）：标定后静止不动 */
    adjustSpeed(SPEED_NORMAL);
    resetInputs();
    g_mockAnalog[MOCK_AX] = 582;   /* 四路都偏 +70，模拟摇杆机械中位不在 512 */
    g_mockAnalog[MOCK_AY] = 582;
    g_mockAnalog[MOCK_TX] = 582;
    g_mockAnalog[MOCK_TY] = 582;
    joystickSetup();               /* 自标定在这里采样 */
    posInit();
    struct Joints jc = snap();
    runLoop(30, 40);               /* 静止握杆 1200ms：新版固定 40ms 间隔 */
    snprintf(buf, sizeof(buf), "偏 +70 静止 30 轮: b %.2f->%.2f r %.2f->%.2f c %.2f->%.2f f %.2f->%.2f",
             jc.b, Pos.ser.angle1, jc.r, Pos.ser.angle2, jc.c, Pos.ser.angle3, jc.f, Pos.ser.angle4);
    check("中位偏 +70：标定后静止不动（旧版会一直乱走）",
          fabs(Pos.ser.angle1 - jc.b) < 1e-9 && fabs(Pos.ser.angle2 - jc.r) < 1e-9 &&
          fabs(Pos.ser.angle3 - jc.c) < 1e-9 && fabs(Pos.ser.angle4 - jc.f) < 1e-9, buf);

    /* 9b) 同一个偏置下真正推杆：仍然按"越过 512"判方向 */
    resetInputs();
    g_mockAnalog[MOCK_AX] = 582;   /* 保持偏置，让扣偏差后为 0 */
    g_mockAnalog[MOCK_AY] = 582;
    g_mockAnalog[MOCK_TX] = 582;
    g_mockAnalog[MOCK_TY] = 582;
    joystickSetup();
    posInit();
    jc = snap();
    g_mockAnalog[MOCK_AX] = 582 + 300;   /* 在偏置之上右推 300 */
    runLoop(6, 30);
    snprintf(buf, sizeof(buf), "偏置 +70 之上右推 300: b %.2f->%.2f", jc.b, Pos.ser.angle1);
    check("标定只扣偏差、不改方向判定（右推仍使 b 增大）",
          Pos.ser.angle1 > jc.b && onlyChanged(jc, snap(), 0), buf);

    /* 9c) 偏置超过 JOY_CAL_MAX_OFF（80）视为"开机手压着摇杆"：偏差按 0 处理 */
    resetInputs();
    g_mockAnalog[MOCK_AX] = 512 + 100;   /* 偏 +100 > 80 */
    joystickSetup();
    posInit();
    jc = snap();
    runLoop(30, 40);               /* 不推杆，但偏置没被采纳 => 仍被当成推杆 */
    snprintf(buf, sizeof(buf), "偏 +100 静止: b %.2f->%.2f（期望被当成推杆而离开起点）",
             jc.b, Pos.ser.angle1);
    check("偏置 >80 不采纳（防开机手压摇杆时把中位学歪）",
          fabs(Pos.ser.angle1 - jc.b) > 1e-9, buf);

    /* 9d) 回到标准 512 中位：标定结果必须是 0，行为与历史版本一致 */
    resetInputs();
    joystickSetup();
    posInit();
    jc = snap();
    runLoop(30, 40);
    snprintf(buf, sizeof(buf), "标准 512 中位静止: b %.2f->%.2f", jc.b, Pos.ser.angle1);
    check("标准 512 中位：标定后静止不动", fabs(Pos.ser.angle1 - jc.b) < 1e-9, buf);
  }

  printf("=== 10) 摇杆手感与漂移回归（v1.6.4：步长仍按偏转缩放 + 温漂自吸收，固定 40ms 间隔）===\n");
  {
    /* 10a) 同起点、同轮数、同毫秒：大幅偏转的位移必须大于小幅偏转。
     *      步长仍按偏转缩放，所以大幅偏转产生更大位移。偏转必须 > 起控门槛 65 */
    adjustSpeed(SPEED_NORMAL);
    resetInputs(); posInit();
    double b0 = Pos.ser.angle1;
    g_mockAnalog[MOCK_AX] = 512 + 70;      /* 小幅偏转（必须 > 起控门槛 65） */
    runLoop(24, 40);
    double smallDb = Pos.ser.angle1 - b0;

    resetInputs(); posInit();
    b0 = Pos.ser.angle1;
    g_mockAnalog[MOCK_AX] = 512 + 420;      /* 大幅偏转 */
    runLoop(24, 40);
    double bigDb = Pos.ser.angle1 - b0;

    snprintf(buf, sizeof(buf), "小幅(+70) Δb=%.2f 度  大幅(+420) Δb=%.2f 度", smallDb, bigDb);
    check("同样时间：大幅偏转位移 > 小幅偏转位移（步长按偏转缩放，起控门槛65）",
          bigDb > smallDb && smallDb > 0, buf);

    /* 10b) 中位缓慢漂移：输入每轮只挪 1 个计数，运行期中位跟踪应当把它全部吸收，
     *      全程关节一动不动（老版本没有跟踪，一旦漂过死区就开始一格格挪）。 */
    resetInputs();
    joystickSetup();                 /* 在 512 处标定：偏差 0 */
    posInit();
    struct Joints jd = snap();
    for (int i = 1; i <= 110; i++) {
      g_mockAnalog[MOCK_AX] = 512 + i;   /* 累计漂移 +110 计数（远超死区 40） */
      g_mockMillis += 40;
      joystickLoop();
    }
    snprintf(buf, sizeof(buf), "每轮漂移 +1 共 +110 计数：Δb=%.4f Δr=%.4f Δc=%.4f Δf=%.4f",
             Pos.ser.angle1 - jd.b, Pos.ser.angle2 - jd.r,
             Pos.ser.angle3 - jd.c, Pos.ser.angle4 - jd.f);
    check("中位缓慢漂移被跟踪吸收：关节一动不动（固定40ms间隔，JOY_TRACK_MS=20）",
          fabs(Pos.ser.angle1 - jd.b) < 1e-9 && fabs(Pos.ser.angle2 - jd.r) < 1e-9 &&
          fabs(Pos.ser.angle3 - jd.c) < 1e-9 && fabs(Pos.ser.angle4 - jd.f) < 1e-9, buf);
  }

  printf("\n>>> %s (失败 %d 项)\n", failures == 0 ? "ALL PASS" : "HAS FAILURES", failures);
  return failures == 0 ? 0 : 1;
}
