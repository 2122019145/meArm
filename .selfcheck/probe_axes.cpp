/* probe_axes.cpp —— 扫描真实可达空间，为 rangeLimit 定边界。
 *
 * 【重要】本探针直接 #include "constant_and_positions.h" 并调用固件自己的
 * recFromServo() 与 servoLimit，绝不重写运动学公式。
 * 历史教训：早期版本在探针里自己重写正解，把基座角写成 theta=(b-90)
 * （固件是 (90-b)），整张可达表镜像，把"可达"读成"不可达"，浪费了好几轮。
 * 需要自己算的时候，必须用已知标定点手算核对一次。
 *
 * 关节限位直接取自固件全局 servoLimit（b[0,180] r[45,105] c[0,180] f[60,150]）。
 */
#include <cmath>
#include <cstdio>
#include "constant_and_positions.h"

static int excluded = 0;   /* 落在当前 limit 盒外的采样点数 */

/* 用固件正解扫一个 (b, r, c) 组合，返回坐标是否落在当前 limit 内 */
static void sample(double b, double r, double c, double *x, double *y, double *z) {
  SER ser;
  ser.angle1 = b;
  ser.angle2 = r;
  ser.angle3 = c;
  ser.angle4 = 105.0;
  REC rec;
  if (!recFromServo(&rec, &ser)) {
    *x = *y = *z = NAN;
    return;
  }
  *x = rec.x; *y = rec.y; *z = rec.z;
}

static bool inLimit(double x, double y, double z) {
  return x >= limit.minX && x <= limit.maxX &&
         y >= limit.minY && y <= limit.maxY &&
         z >= limit.minZ && z <= limit.maxZ;
}

int main(void) {
  (void) servoSelfCheck();       /* 建好关节限位映射表 */
  (void) rangeClampConfig();

  int fails = 0;
  char buf[128];
  snprintf(buf, sizeof(buf), "关节限位 b[%.0f,%.0f] r[%.0f,%.0f] c[%.0f,%.0f]",
           servoLimit.minB, servoLimit.maxB, servoLimit.minR, servoLimit.maxR,
           servoLimit.minC, servoLimit.maxC);
  printf("=== 固件 servoSelfCheck 后的 %s ===\n", buf);

  const double BMIN = servoLimit.minB, BMAX = servoLimit.maxB;
  const double RMIN = servoLimit.minR, RMAX = servoLimit.maxR;
  const double CMIN = servoLimit.minC, CMAX = servoLimit.maxC;
  const double STEP = 1.0;

  double xmin = 1e9, xmax = -1e9, ymin = 1e9, ymax = -1e9, zmin = 1e9, zmax = -1e9;
  long total = 0;

  /* 1) 全域包络 */
  for (double b = BMIN; b <= BMAX + 1e-9; b += STEP) {
    for (double r = RMIN; r <= RMAX + 1e-9; r += STEP) {
      for (double c = CMIN; c <= CMAX + 1e-9; c += STEP) {
        double x, y, z;
        sample(b, r, c, &x, &y, &z);
        if (x != x) continue;
        total++;
        if (x < xmin) xmin = x;
        if (x > xmax) xmax = x;
        if (y < ymin) ymin = y;
        if (y > ymax) ymax = y;
        if (z < zmin) zmin = z;
        if (z > zmax) zmax = z;
        if (!inLimit(x, y, z)) excluded++;
      }
    }
  }
  printf("\n=== 1) 全域实测包络（%ld 个采样点）===\n", total);
  printf("  x[%.2f, %.2f]  y[%.2f, %.2f]  z[%.2f, %.2f]\n", xmin, xmax, ymin, ymax, zmin, zmax);
  printf("  当前 limit: x[%.1f,%.1f] y[%.1f,%.1f] z[%.1f,%.1f]\n",
         limit.minX, limit.maxX, limit.minY, limit.maxY, limit.minZ, limit.maxZ);
  printf("  落在 limit 外的采样点: %d / %ld = %.1f%%\n",
         excluded, total, 100.0 * (double)excluded / (double)total);
  /* limit 是按实测包络定出来的（外扩 0.5），因此"没有任何可达姿态落在 limit 外"
   * 是这套配置的核心不变量。若这里非 0，说明 limit 比真实包络小，
   * 角度模式下会表现为"某个关节走不到行程端点就被软护栏挡住"。
   * 外扩量也打出来，方便确认 limit 没有反过来放得太宽（软护栏失去意义）。 */
  {
    /* 余量 = 包络离 limit 边界还有多远。两侧都必须是正数：
     *   lowX  = xmin - limit.minX  （limit 下界比包络最低还低多少）
     *   highX = limit.maxX - xmax  （limit 上界比包络最高还高多少）
     * 注意别写反成 limit.minX - xmin —— 那样"limit 覆盖包络"会被算成负数。 */
    double lowX = xmin - limit.minX, highX = limit.maxX - xmax;
    double lowY = ymin - limit.minY, highY = limit.maxY - ymax;
    double lowZ = zmin - limit.minZ, highZ = limit.maxZ - zmax;
    printf("  limit 相对包络的余量: x[%.2f,%.2f] y[%.2f,%.2f] z[%.2f,%.2f]"
           "  (负数 = limit 比包络窄，会提前挡住关节)\n", lowX, highX, lowY, highY, lowZ, highZ);
    if (excluded > 0 || lowX < 0 || highX < 0 || lowY < 0 || highY < 0 || lowZ < 0 || highZ < 0) fails++;
  }

  /* 2) 逐高度 z 的 x/y 可达区间（每 2 度一层，避免刷屏） */
  printf("\n=== 2) 逐高度可达区间（x 取全域，y 取该高度观察到的范围）===\n");
  for (double zt = zmin; zt <= zmax + 1e-9; zt += 2.0) {
    double lo = 1e9, hi = -1e9, ylo = 1e9, yhi = -1e9;
    for (double b = BMIN; b <= BMAX + 1e-9; b += 2.0 * STEP) {
      for (double r = RMIN; r <= RMAX + 1e-9; r += STEP) {
        for (double c = CMIN; c <= CMAX + 1e-9; c += STEP) {
          double x, y, z;
          sample(b, r, c, &x, &y, &z);
          if (x != x || fabs(z - zt) > 1.0) continue;
          if (x < lo) lo = x;
          if (x > hi) hi = x;
          if (y < ylo) ylo = y;
          if (y > yhi) yhi = y;
        }
      }
    }
    if (lo > hi) {
      printf("  z=%6.1f : 本高度无采样点\n", zt);
    } else {
      printf("  z=%6.1f : x[%6.2f, %6.2f]  y[%6.2f, %6.2f]\n", zt, lo, hi, ylo, yhi);
    }
  }

  /* 3) 关节行程能否全部走到（有没有被 limit 提前挡住） */
  printf("\n=== 3) 单轴满行程：从初始位姿把某个角从最小推到最大 ===\n");
  {
    struct { const char *name; int which; double lo, hi; } ax[3] = {
      { "b 基座", 1, BMIN, BMAX }, { "r 上臂", 2, RMIN, RMAX }, { "c 下臂", 3, CMIN, CMAX }
    };
    for (int i = 0; i < 3; i++) {
      SER ser;
      ser.angle1 = 90; ser.angle2 = 90; ser.angle3 = 90; ser.angle4 = 105;
      int blockedLo = 0, blockedHi = 0;
      for (double v = ax[i].lo; v <= ax[i].hi + 1e-9; v += 1.0) {
        SER t = ser;
        if (ax[i].which == 1) t.angle1 = v;
        if (ax[i].which == 2) t.angle2 = v;
        if (ax[i].which == 3) t.angle3 = v;
        REC rec;
        if (!recFromServo(&rec, &t)) continue;
        if (!inLimit(rec.x, rec.y, rec.z)) {
          if (v < 90.0) blockedLo = 1;
          else blockedHi = 1;
        }
      }
      const bool okLo = !blockedLo, okHi = !blockedHi;
      printf("  %s: 向小端%s  向大端%s\n", ax[i].name,
             blockedLo ? "被 limit 挡（走不到行程下限）" : "可走满行程",
             blockedHi ? "被 limit 挡（走不到行程上限）" : "可走满行程");
      if (!okLo || !okHi) fails++;
    }
  }

  /* 4) 标定点核对（用固件正解，与探针自己算的对照） */
  printf("\n=== 4) 标定点核对（固件 recFromServo）===\n");
  struct { double b, r, c; double ex, ey, ez; } pts[5] = {
    { 90, 90,  90, 20.00, 0.00, 20.00 },
    { 90, 90, 105, 19.32, 0.00, 14.82 },
    { 90, 45,  90, 28.28, 0.00,  0.00 },
    { 90, 105,  0,-10.35, 0.00, 38.64 },
    { 90, 45, 105, 24.14, 0.00, -3.18 }
  };
  for (int i = 0; i < 5; i++) {
    double x, y, z;
    sample(pts[i].b, pts[i].r, pts[i].c, &x, &y, &z);
    double err = fabs(x - pts[i].ex) + fabs(y - pts[i].ey) + fabs(z - pts[i].ez);
    printf("  (b,r,c)=(%.0f,%.0f,%.0f) -> (%.2f,%.2f,%.2f)  期望 (%.2f,%.2f,%.2f)  %s\n",
           pts[i].b, pts[i].r, pts[i].c, x, y, z, pts[i].ex, pts[i].ey, pts[i].ez,
           err < 0.01 ? "OK" : "MISMATCH");
    if (!(err < 0.01)) fails++;
  }

  /* 结论行：run_all.cmd 靠 "ALL PASS" 判定本探针是否通过。
   * 本探针是"报告型"的（会打印包络表），但上面的每条断言都要计数，
   * 否则脚本没法区分"打印了很多字"和"检查都过了"。 */
  printf("\n>>> %s (失败 %d 项)\n", fails == 0 ? "ALL PASS" : "HAS FAILURES", fails);
  return fails == 0 ? 0 : 1;
}
