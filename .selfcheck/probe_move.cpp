/* probe_move.cpp —— 关节角步进 / 行程限位 / 整步回退 / 调速档位 回归验证
 * 【去配置化版本】servoLimit / rangeLimit 已从固件删除，b/r/c 硬编码 0~180。
 * 直接链接固件，不重写任何公式。
 */
#include <cmath>
#include <cstdio>
#include <cstring>
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

static bool selfConsistent(void) {
  REC chk;
  if (!recFromServo(&chk, &Pos.ser)) return false;
  return fabs(chk.x - Pos.rec.x) < 1e-9 && fabs(chk.y - Pos.rec.y) < 1e-9 &&
         fabs(chk.z - Pos.rec.z) < 1e-9;
}

 

static bool solveRef(double x, double y, double z, double *b, double *r, double *c) {
  pos p = Pos;
  p.rec.x = x; p.rec.y = y; p.rec.z = z;
  bool clamped = false;
  if (!getAngleEx(&p, &clamped)) return false;
  if (clamped) return false;
  *b = p.ser.angle1; *r = p.ser.angle2; *c = p.ser.angle3;
  return true;
}

int main(void) {
  posInit();

  char buf[220];

  printf("=== 0) 起点核对 ===\n");
  {
    double d = dist(&Pos.rec, 20.0, 0.0, 20.0);
    snprintf(buf, sizeof(buf), "Pos=(%.2f,%.2f,%.2f) b=%.2f r=%.2f c=%.2f f=%.2f 自洽=%d",
             Pos.rec.x, Pos.rec.y, Pos.rec.z,
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3, Pos.ser.angle4,
             selfConsistent() ? 1 : 0);
    check("home=(20,0,20)、在 limit 内、坐标与角度自洽",
          d < 1e-6 && selfConsistent() &&
          Pos.ser.angle4 >= 60.0 && Pos.ser.angle4 <= 150.0, buf);
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

  printf("=== 2) 连续推进到头：停在 0/180，且返回 MOVE_AT_LIMIT / 零位移 ===\n");
  {
    struct { const char *name; void (*fn)(void); int dir; int which; double stop; } cases[6] = {
      { "连续 moveup 停在 c=180",       moveup,       JOINT_C_UP,    2, 180.0 },
      { "连续 movedown 停在 c=0",       movedown,     JOINT_C_DOWN,  2,   0.0 },
      { "连续 moveright 停在 b=180",    moveright,    JOINT_B_RIGHT, 0, 180.0 },
      { "连续 moveleft 停在 b=0",       moveleft,     JOINT_B_LEFT,  0,   0.0 },
      { "连续 moveforward 停在 r=180",  moveforward,  JOINT_R_FWD,   1, 180.0 },
      { "连续 movebackward 停在 r=0",   movebackward, JOINT_R_BWD,   1,   0.0 }
    };
    for (int i = 0; i < 6; i++) {
      posInit();
      for (int k = 0; k < 400; k++) cases[i].fn();
      struct Joints j1 = snap();
      double got[4] = { j1.b, j1.r, j1.c, j1.f };
      double x0 = Pos.rec.x, y0 = Pos.rec.y, z0 = Pos.rec.z;
      int res = moveJointStep(cases[i].dir, 1.0);
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
    int res = moveJointStep(0, 1.0);
    struct Joints j1 = snap();
    snprintf(buf, sizeof(buf), "res=%d Δb=%.6f", res, j1.b - j0.b);
    check("方向 0 -> MOVE_NONE 且不动", res == MOVE_NONE && j1.b == j0.b, buf);

    adjustSpeed(SPEED_NORMAL);
    posInit();
    j0 = snap();
    res = moveJointStep(JOINT_B_RIGHT, 0.0);
    j1 = snap();
    snprintf(buf, sizeof(buf), "res=%d Δb=%.3f（应等于全局 stepSize %.1f）",
             res, j1.b - j0.b, speed.stepSize);
    check("stepSize=0 -> 跟随全局调速", res == MOVE_OK && fabs((j1.b - j0.b) - speed.stepSize) < 1e-9, buf);
  }

  printf("=== 4) 坐标是派生量：每步都与 recFromServo 严格一致 ===\n");
  {
    posInit();
    int res = moveJointStep(JOINT_R_FWD, 1.0);
    snprintf(buf, sizeof(buf), "res=%d (%.4f,%.4f,%.4f) 自洽=%d",
             res, Pos.rec.x, Pos.rec.y, Pos.rec.z, selfConsistent() ? 1 : 0);
    check("单步后坐标自洽", res == MOVE_OK && selfConsistent(), buf);

    REC far;
    far.x = 0; far.y = 0; far.z = 100;
    check("超臂展点 isReachable() == false", !isReachable(&far), "z=100 远超 L1+L2=40");
  }

  printf("=== 5) 调速档位 ===\n");
  {
    int r0 = adjustSpeed(SPEED_SLOW);
    snprintf(buf, sizeof(buf), "ret=%d step=%.1f delay=%d", r0, speed.stepSize, speed.stepDelayMs);
    check("adjustSpeed(SLOW) -> 0.5/80",
          r0 == SPEED_SLOW && fabs(speed.stepSize - 0.5) < 1e-9 && speed.stepDelayMs == 80, buf);

    int r1 = adjustSpeed(SPEED_NORMAL);
    snprintf(buf, sizeof(buf), "ret=%d step=%.1f delay=%d", r1, speed.stepSize, speed.stepDelayMs);
    check("adjustSpeed(NORMAL) -> 1.0/40",
          r1 == SPEED_NORMAL && fabs(speed.stepSize - 1.0) < 1e-9 && speed.stepDelayMs == 40, buf);

    int r2 = adjustSpeed(SPEED_FAST);
    snprintf(buf, sizeof(buf), "ret=%d step=%.1f delay=%d", r2, speed.stepSize, speed.stepDelayMs);
    check("adjustSpeed(FAST) -> 2.0/20",
          r2 == SPEED_FAST && fabs(speed.stepSize - 2.0) < 1e-9 && speed.stepDelayMs == 20, buf);

    check("adjustSpeed(3) 非法 -> -1", adjustSpeed(3) == -1, "越界档位被拒绝");

    setSpeed(-1.0, 0);
    snprintf(buf, sizeof(buf), "step=%.1f delay=%d（应仍为 2.0/20）",
             speed.stepSize, speed.stepDelayMs);
    check("setSpeed(-1.0, 0) 两个参数都非法 -> 配置完全不变",
          fabs(speed.stepSize - 2.0) < 1e-9 && speed.stepDelayMs == 20, buf);

    setSpeed(1.0, 0);
    snprintf(buf, sizeof(buf), "step=%.1f delay=%d（应为 1.0/20）",
             speed.stepSize, speed.stepDelayMs);
    check("setSpeed(1.0, 0) 只采纳合法项 -> 步长 1.0、间隔仍为 20",
          fabs(speed.stepSize - 1.0) < 1e-9 && speed.stepDelayMs == 20, buf);

    adjustSpeed(SPEED_FAST);
  }

  printf("=== 6) 大行程压力: 随机方向 4000 步，零越界/零 NaN/零失配 ===\n");
  {
    adjustSpeed(SPEED_NORMAL);
    posInit();
    unsigned long seed = 12345UL;
    int bad = 0;
    for (int i = 0; i < 4000; i++) {
      seed = seed * 1103515245UL + 12345UL;
      int dir = (int)((seed >> 16) % 6UL) + 1;
      (void) moveJointStep(dir, 0.5);
      if (!(Pos.ser.angle1 == Pos.ser.angle1) || !(Pos.ser.angle2 == Pos.ser.angle2) ||
          !(Pos.ser.angle3 == Pos.ser.angle3) || !(Pos.ser.angle4 == Pos.ser.angle4)) bad++;
      if (Pos.ser.angle1 < 0.0 - 1e-6 || Pos.ser.angle1 > 180.0 + 1e-6) bad++;
      if (Pos.ser.angle2 < 0.0 - 1e-6 || Pos.ser.angle2 > 180.0 + 1e-6) bad++;
      if (Pos.ser.angle3 < 0.0 - 1e-6 || Pos.ser.angle3 > 180.0 + 1e-6) bad++;
      if (!selfConsistent()) bad++;
    }
    snprintf(buf, sizeof(buf), "violations=%d 末态=(%.1f,%.1f,%.1f) b=%.1f r=%.1f c=%.1f",
             bad, Pos.rec.x, Pos.rec.y, Pos.rec.z, Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);
    check("4000 步随机压力：零越界/零 NaN/零关节越限/坐标始终自洽", bad == 0, buf);
  }

  printf("=== 7) moveToPoint(): 绘图 / 串口共用的「移动到指定 x,y,z」 ===\n");
  {
    unsigned char before[sizeof(pos)];

    posInit();
    memcpy(before, &Pos, sizeof(pos));
    {
      double g[3] = { 0.0, 0.0, 100.0 };
      int res = moveToPoint(g, 0x07u, 110.0, 0.05, MOVE_XYZ_TRACK);
      snprintf(buf, sizeof(buf), "res=%d（期望 %d） 逐字节不变=%d",
               res, MOVE_XYZ_REJECTED, memcmp(before, &Pos, sizeof(pos)) == 0 ? 1 : 0);
      check("7a 不可达点 -> REJECTED 且 Pos 逐字节不变",
            res == MOVE_XYZ_REJECTED && memcmp(before, &Pos, sizeof(pos)) == 0, buf);
    }

    posInit();
    memcpy(before, &Pos, sizeof(pos));
    {
      double g[3] = { Pos.rec.x + 5.0, Pos.rec.y, Pos.rec.z };
      int res = moveToPoint(g, 0x07u, 1.0, 0.005, MOVE_XYZ_TRACK);
      snprintf(buf, sizeof(buf), "res=%d（期望 %d） 逐字节不变=%d",
               res, MOVE_XYZ_REJECTED, memcmp(before, &Pos, sizeof(pos)) == 0 ? 1 : 0);
      check("7b TRACK 超速 -> REJECTED 且不动",
            res == MOVE_XYZ_REJECTED && memcmp(before, &Pos, sizeof(pos)) == 0, buf);
    }

    {
      double g[3] = { Pos.rec.x + 5.0, Pos.rec.y, Pos.rec.z };
      double eb = 0.0, er = 0.0, ec = 0.0;
      bool ref = solveRef(g[0], g[1], g[2], &eb, &er, &ec);
      double need = fabs(eb - Pos.ser.angle1);
      if (fabs(er - Pos.ser.angle2) > need) need = fabs(er - Pos.ser.angle2);
      if (fabs(ec - Pos.ser.angle3) > need) need = fabs(ec - Pos.ser.angle3);
      int res = moveToPoint(g, 0x07u, need * 2.0 / 0.1, 0.1, MOVE_XYZ_TRACK);
      snprintf(buf, sizeof(buf), "res=%d 参照解=%d 需动=%.3f 上限=%.3f b=%.3f/%.3f r=%.3f/%.3f c=%.3f/%.3f 自洽=%d",
               res, ref ? 1 : 0, need, need * 2.0, Pos.ser.angle1, eb, Pos.ser.angle2, er,
               Pos.ser.angle3, ec, selfConsistent() ? 1 : 0);
      check("7c TRACK 限内 -> OK 且角度 == 反解结果（逐位）",
            ref && res == MOVE_XYZ_OK && Pos.ser.angle1 == eb && Pos.ser.angle2 == er &&
            Pos.ser.angle3 == ec && selfConsistent(), buf);
    }

    posInit();
    {
      double g[3] = { Pos.rec.x + 3.0, Pos.rec.y, Pos.rec.z };
      double eb = 0.0, er = 0.0, ec = 0.0;
      bool ref = solveRef(g[0], g[1], g[2], &eb, &er, &ec);
      double maxStep = 10.0 * 0.02;
      int steps = 0;
      double worst = 0.0;
      for (int i = 0; i < 200; i++) {
        struct Joints a = snap();
        int res = moveToPoint(g, 0x07u, 10.0, 0.02, MOVE_XYZ_JOG);
        struct Joints b = snap();
        if (res != MOVE_XYZ_OK) { steps = -1; break; }
        double d = fabs(b.b - a.b);
        if (fabs(b.r - a.r) > d) d = fabs(b.r - a.r);
        if (fabs(b.c - a.c) > d) d = fabs(b.c - a.c);
        if (d > worst) worst = d;
        steps++;
        if (b.b == a.b && b.r == a.r && b.c == a.c) break;
      }
      snprintf(buf, sizeof(buf), "参照解=%d 步数=%d 每步最大变化=%.6f（上限 %.2f） 末态=%.4f/%.4f/%.4f",
               ref ? 1 : 0, steps, worst, maxStep, Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);
      check("7d JOG 每步不超上限、反复调用收敛到反解结果",
            ref && steps > 0 && worst <= maxStep + 1e-12 &&
            fabs(Pos.ser.angle1 - eb) < 1e-9 && fabs(Pos.ser.angle2 - er) < 1e-9 &&
            fabs(Pos.ser.angle3 - ec) < 1e-9, buf);
    }

    posInit();
    {
      double g[3] = { Pos.rec.x + 5.0, Pos.rec.y + 2.0, Pos.rec.z - 1.0 };
      double eb = 0.0, er = 0.0, ec = 0.0;
      bool ref = solveRef(g[0], g[1], g[2], &eb, &er, &ec);
      double b0 = Pos.ser.angle1;
      int res = moveToPoint(g, 0x07u, 0.0, 0.0, MOVE_XYZ_NOW);
      double jump = fabs(Pos.ser.angle1 - b0);
      snprintf(buf, sizeof(buf), "参照解=%d res=%d 一次跳变=%.2f 度 自洽=%d",
               ref ? 1 : 0, res, jump, selfConsistent() ? 1 : 0);
      check("7e NOW 一次到位且坐标自洽",
            ref && res == MOVE_XYZ_OK && Pos.ser.angle1 == eb && Pos.ser.angle2 == er &&
            Pos.ser.angle3 == ec && jump > 1.0 && selfConsistent(), buf);
    }

    posInit();
    {
      double y0 = Pos.rec.y, z0 = Pos.rec.z;
      double g[3] = { Pos.rec.x + 1.0, -99.0, -99.0 };
      int res = moveToPoint(g, (uint8_t)(1u << 0), 0.0, 0.0, MOVE_XYZ_NOW);
      snprintf(buf, sizeof(buf), "res=%d x=%.4f（请求 %.4f） Δy=%.2e Δz=%.2e 自洽=%d",
               res, Pos.rec.x, g[0], fabs(Pos.rec.y - y0), fabs(Pos.rec.z - z0),
               selfConsistent() ? 1 : 0);
      check("7f 只提及 x -> 只有 x 变、y/z 沿用当前值",
            res == MOVE_XYZ_OK && fabs(Pos.rec.x - g[0]) < 1e-9 &&
            fabs(Pos.rec.y - y0) < 1e-9 && fabs(Pos.rec.z - z0) < 1e-9 && selfConsistent(), buf);
    }

    posInit();
    memcpy(before, &Pos, sizeof(pos));
    {
      double g[3] = { Pos.rec.x, Pos.rec.y, Pos.rec.z };
      int r1 = moveToPoint(NULL, 0x07u, 110.0, 0.05, MOVE_XYZ_TRACK);
      int r2 = moveToPoint(g, 0x07u, 110.0, 0.05, 99);
      snprintf(buf, sizeof(buf), "NULL=%d 未知模式=%d 逐字节不变=%d",
               r1, r2, memcmp(before, &Pos, sizeof(pos)) == 0 ? 1 : 0);
      check("7g NULL / 未知模式 -> REJECTED 且不动",
            r1 == MOVE_XYZ_REJECTED && r2 == MOVE_XYZ_REJECTED &&
            memcmp(before, &Pos, sizeof(pos)) == 0, buf);
    }
  }

  printf("\n>>> %s (失败 %d 项)\n", failures == 0 ? "ALL PASS" : "HAS FAILURES", failures);
  return failures == 0 ? 0 : 1;
}