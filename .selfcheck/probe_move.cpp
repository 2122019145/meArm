/* probe_move.cpp —— 关节角步进 / 行程限位 / 整步回退 / 调速档位 回归验证
 * 【角度模式】被控量是 Pos.ser.angle1..angle4，坐标是正运动学派生量。
 * 直接链接固件，不重写任何公式。
 */
#include <cmath>
#include <cstdio>
#include "constant_and_positions.h"
#include "move.h"

static int failures = 0;

static void check(const char *name, bool ok, const char *detail) {
  printf("  %-46s %s  %s\n", name, ok ? "PASS" : "FAIL", detail);
  if (!ok) failures++;
}

static double dist(const REC *a, double x, double y, double z) {
  return sqrt((a->x - x) * (a->x - x) + (a->y - y) * (a->y - y) + (a->z - z) * (a->z - z));
}

/* 角度快照 */
struct Joints { double b, r, c, f; };
static struct Joints snap(void) {
  struct Joints j;
  j.b = Pos.ser.angle1; j.r = Pos.ser.angle2;
  j.c = Pos.ser.angle3; j.f = Pos.ser.angle4;
  return j;
}
static bool unchangedExcept(const struct Joints &a, const struct Joints &b, int which) {
  double d[4] = { b.b - a.b, b.r - a.r, b.c - a.c, b.f - a.f };
  for (int i = 0; i < 4; i++) {
    if (i == which) continue;
    if (fabs(d[i]) > 1e-12) return false;
  }
  return true;
}

/* 坐标与角度是否严格自洽（角度模式的核心不变量） */
static bool selfConsistent(void) {
  REC chk;
  if (!recFromServo(&chk, &Pos.ser)) return false;
  return fabs(chk.x - Pos.rec.x) < 1e-9 && fabs(chk.y - Pos.rec.y) < 1e-9 &&
         fabs(chk.z - Pos.rec.z) < 1e-9;
}
static bool inLimit(void) {
  return Pos.rec.x >= limit.minX - 1e-9 && Pos.rec.x <= limit.maxX + 1e-9 &&
         Pos.rec.y >= limit.minY - 1e-9 && Pos.rec.y <= limit.maxY + 1e-9 &&
         Pos.rec.z >= limit.minZ - 1e-9 && Pos.rec.z <= limit.maxZ + 1e-9;
}

int main(void) {
  (void) servoSelfCheck();
  (void) rangeClampConfig();
  posInit();

  char buf[220];

  printf("=== 0) 起点核对 ===\n");
  {
    double d = dist(&Pos.rec, 20.0, 0.0, 20.0);   /* POS_HOME = (20,0,20) */
    snprintf(buf, sizeof(buf), "Pos=(%.2f,%.2f,%.2f) b=%.2f r=%.2f c=%.2f f=%.2f 自洽=%d",
             Pos.rec.x, Pos.rec.y, Pos.rec.z,
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3, Pos.ser.angle4,
             selfConsistent() ? 1 : 0);
    check("home=(20,0,20)、在 limit 内、坐标与角度自洽",
          d < 1e-6 && inLimit() && selfConsistent() &&
          Pos.ser.angle4 >= servoLimit.minF && Pos.ser.angle4 <= servoLimit.maxF, buf);
  }

  printf("=== 1) 六个定长方向各走一步：目标关节正好 ±1 度，其它关节不动 ===\n");
  {
    struct { const char *name; void (*fn)(void); int which; double want; } cases[6] = {
      { "moveup        (c+1)", moveup,       2, +1.0 },
      { "movedown      (c-1)", movedown,     2, -1.0 },
      { "moveright     (b+1)", moveright,    0, +1.0 },
      { "moveleft      (b-1)", moveleft,     0, -1.0 },
      { "moveforward   (r+1)", moveforward,  1, +1.0 },
      { "movebackward  (r-1)", movebackward, 1, -1.0 }
    };
    for (int i = 0; i < 6; i++) {
      posInit();
      struct Joints j0 = snap();
      cases[i].fn();
      struct Joints j1 = snap();
      double got[4] = { j1.b - j0.b, j1.r - j0.r, j1.c - j0.c, j1.f - j0.f };
      snprintf(buf, sizeof(buf), "Δ=(%.3f,%.3f,%.3f,%.3f) 期望 %.0f 在[%d] 自洽=%d",
               got[0], got[1], got[2], got[3], cases[i].want, cases[i].which,
               selfConsistent() ? 1 : 0);
      check(cases[i].name,
            fabs(got[cases[i].which] - cases[i].want) < 1e-9 &&
            unchangedExcept(j0, j1, cases[i].which) && selfConsistent(), buf);
    }
  }

  printf("=== 2) 连续推进到头：停在同一位置，且返回 MOVE_AT_LIMIT / 零位移 ===\n");
  {
    struct { const char *name; void (*fn)(void); int dir; int which; double stop; } cases[6] = {
      { "连续 moveup 停在 c=maxC",       moveup,       JOINT_C_UP,    2, servoLimit.maxC },
      { "连续 movedown 停在 c=minC",     movedown,     JOINT_C_DOWN,  2, servoLimit.minC },
      { "连续 moveright 停在 b=maxB",    moveright,    JOINT_B_RIGHT, 0, servoLimit.maxB },
      { "连续 moveleft 停在 b=minB",     moveleft,     JOINT_B_LEFT,  0, servoLimit.minB },
      { "连续 moveforward 停在 r=maxR",  moveforward,  JOINT_R_FWD,   1, servoLimit.maxR },
      { "连续 movebackward 停在 r=minR", movebackward, JOINT_R_BWD,   1, servoLimit.minR }
    };
    for (int i = 0; i < 6; i++) {
      posInit();
      for (int k = 0; k < 400; k++) cases[i].fn();     /* 一路推到头 */
      struct Joints j1 = snap();
      double got[4] = { j1.b, j1.r, j1.c, j1.f };
      double x0 = Pos.rec.x, y0 = Pos.rec.y, z0 = Pos.rec.z;
      int res = moveJointStep(cases[i].dir, 1.0);      /* 再推一步应当被挡住 */
      snprintf(buf, sizeof(buf), "角=%.2f 期望=%.0f res=%d 位移=%.6f 自洽=%d",
               got[cases[i].which], cases[i].stop, res, dist(&Pos.rec, x0, y0, z0),
               selfConsistent() ? 1 : 0);
      check(cases[i].name,
            fabs(got[cases[i].which] - cases[i].stop) < 1e-6 && res == MOVE_AT_LIMIT &&
            dist(&Pos.rec, x0, y0, z0) < 1e-9 && selfConsistent(), buf);
    }
  }

  printf("=== 3) 非法方向 / 非正步长 ===\n");
  {
    posInit();
    struct Joints j0 = snap();
    int res = moveJointStep(0, 1.0);                    /* 0 = 非法方向 */
    struct Joints j1 = snap();
    snprintf(buf, sizeof(buf), "res=%d Δb=%.6f", res, j1.b - j0.b);
    check("方向 0 -> MOVE_NONE 且不动", res == MOVE_NONE && j1.b == j0.b, buf);

    /* stepSize <= 0 表示跟随全局调速，而不是"不动" */
    adjustSpeed(SPEED_NORMAL);
    posInit();
    j0 = snap();
    res = moveJointStep(JOINT_B_RIGHT, 0.0);
    j1 = snap();
    snprintf(buf, sizeof(buf), "res=%d Δb=%.3f（应等于全局 stepSize %.1f）",
             res, j1.b - j0.b, speed.stepSize);
    check("stepSize=0 -> 跟随全局调速", res == MOVE_OK && fabs((j1.b - j0.b) - speed.stepSize) < 1e-9, buf);
  }

  printf("=== 4) 坐标是派生量：每步都与 recFromServo 严格一致、且不出 limit ===\n");
  {
    posInit();
    int res = moveJointStep(JOINT_R_FWD, 1.0);
    snprintf(buf, sizeof(buf), "res=%d (%.4f,%.4f,%.4f) 自洽=%d 在界内=%d",
             res, Pos.rec.x, Pos.rec.y, Pos.rec.z, selfConsistent() ? 1 : 0, inLimit() ? 1 : 0);
    check("单步后坐标自洽且在 limit 内", res == MOVE_OK && selfConsistent() && inLimit(), buf);

    /* 超臂展点仍然要被 isReachable 拒绝（这条是坐标层的兜底，角度模式也用得到） */
    REC far;
    far.x = 0; far.y = 0; far.z = 100;
    check("超臂展点 isReachable() == false", !isReachable(&far), "z=100 远超 L1+L2=40");
  }

  printf("=== 5) 调速档位 ===\n");
  {
    /* 注意: adjustSpeed 返回"生效档位"本身 (0/1/2)，非法档位返回 -1 */
    int r0 = adjustSpeed(SPEED_SLOW);
    snprintf(buf, sizeof(buf), "ret=%d step=%.1f min=%d full=%d", r0, speed.stepSize, speed.minDelayMs, speed.fullDelayMs);
    check("adjustSpeed(SLOW) -> 0.5/20/80",
          r0 == SPEED_SLOW && fabs(speed.stepSize - 0.5) < 1e-9 && speed.minDelayMs == 20 && speed.fullDelayMs == 80, buf);

    int r1 = adjustSpeed(SPEED_NORMAL);
    snprintf(buf, sizeof(buf), "ret=%d step=%.1f min=%d full=%d", r1, speed.stepSize, speed.minDelayMs, speed.fullDelayMs);
    check("adjustSpeed(NORMAL) -> 1.0/10/40",
          r1 == SPEED_NORMAL && fabs(speed.stepSize - 1.0) < 1e-9 && speed.minDelayMs == 10 && speed.fullDelayMs == 40, buf);

    int r2 = adjustSpeed(SPEED_FAST);
    snprintf(buf, sizeof(buf), "ret=%d step=%.1f min=%d full=%d", r2, speed.stepSize, speed.minDelayMs, speed.fullDelayMs);
    check("adjustSpeed(FAST) -> 2.0/5/20",
          r2 == SPEED_FAST && fabs(speed.stepSize - 2.0) < 1e-9 && speed.minDelayMs == 5 && speed.fullDelayMs == 20, buf);

    check("adjustSpeed(3) 非法 -> -1", adjustSpeed(3) == -1, "越界档位被拒绝");
    /* setSpeed 返回 void：非法参数应被内部忽略，配置保持不变 */
    setSpeed(-1.0, 0, -5);
    snprintf(buf, sizeof(buf), "step=%.1f min=%d full=%d（应仍为 2.0/5/20）",
             speed.stepSize, speed.minDelayMs, speed.fullDelayMs);
    check("setSpeed(-1,0,-5) 被忽略、配置不变",
          fabs(speed.stepSize - 2.0) < 1e-9 && speed.minDelayMs == 5 && speed.fullDelayMs == 20, buf);
  }

  printf("=== 6) 大行程压力: 随机方向 4000 步，零越界/零 NaN/零失配 ===\n");
  {
    adjustSpeed(SPEED_NORMAL);
    posInit();
    unsigned long seed = 12345UL;
    int bad = 0;
    for (int i = 0; i < 4000; i++) {
      seed = seed * 1103515245UL + 12345UL;
      int dir = (int)((seed >> 16) % 6UL) + 1;    /* 1..6 -> DIR_UP..DIR_BWD */
      (void) moveJointStep(dir, 0.5);
      if (!inLimit()) bad++;
      if (!(Pos.ser.angle1 == Pos.ser.angle1) || !(Pos.ser.angle2 == Pos.ser.angle2) ||
          !(Pos.ser.angle3 == Pos.ser.angle3) || !(Pos.ser.angle4 == Pos.ser.angle4)) bad++;
      if (Pos.ser.angle1 < servoLimit.minB - 1e-6 || Pos.ser.angle1 > servoLimit.maxB + 1e-6) bad++;
      if (Pos.ser.angle2 < servoLimit.minR - 1e-6 || Pos.ser.angle2 > servoLimit.maxR + 1e-6) bad++;
      if (Pos.ser.angle3 < servoLimit.minC - 1e-6 || Pos.ser.angle3 > servoLimit.maxC + 1e-6) bad++;
      if (!selfConsistent()) bad++;
    }
    snprintf(buf, sizeof(buf), "violations=%d 末态=(%.1f,%.1f,%.1f) b=%.1f r=%.1f c=%.1f",
             bad, Pos.rec.x, Pos.rec.y, Pos.rec.z, Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);
    check("4000 步随机压力：零越界/零 NaN/零关节越限/坐标始终自洽", bad == 0, buf);
  }

  printf("\n>>> %s (失败 %d 项)\n", failures == 0 ? "ALL PASS" : "HAS FAILURES", failures);
  return failures == 0 ? 0 : 1;
}
