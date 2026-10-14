/* probe_axes.cpp —— 扫描真实可达空间（去配置化版本）
 *
 * 【本版本】servoLimit / rangeLimit 已从固件删除，b/r/c 硬编码 0~180。
 * 本探针只扫描这个固定行程并报告包络。
 * 直接 #include 固件，用 recFromServo() 算坐标，绝不重写公式。
 */
#include <cmath>
#include <cstdio>
#include "constant_and_positions.h"

static const double BMIN = 0.0, BMAX = 180.0;
static const double RMIN = 0.0, RMAX = 180.0;
static const double CMIN = 0.0, CMAX = 180.0;
static const double STEP = 1.0;

static void sample(double b, double r, double c, double *x, double *y, double *z) {
  SER ser;
  ser.angle1 = b; ser.angle2 = r; ser.angle3 = c; ser.angle4 = 105.0;
  REC rec;
  if (!recFromServo(&rec, &ser)) { *x = *y = *z = NAN; return; }
  *x = rec.x; *y = rec.y; *z = rec.z;
}

int main(void) {
  int fails = 0;

  printf("=== 可达空间扫描（b/r/c 硬编码 0~180）===\n");

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
      }
    }
  }
  printf("\n=== 1) 全域实测包络（%ld 个采样点）===\n", total);
  printf("  x[%.2f, %.2f]  y[%.2f, %.2f]  z[%.2f, %.2f]\n",
         xmin, xmax, ymin, ymax, zmin, zmax);
  printf("  (预期: x[-40,40] y[-40,40] z[-20,40])\n");

  /* 2) 逐高度 z 的 x/y 可达区间 */
  printf("\n=== 2) 逐高度可达区间（每 2 单位一层）===\n");
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

  /* 3) 单轴满行程：三个关节 0→180 正解都要成功 */
  printf("\n=== 3) 单轴满行程（三个关节 0~180 正解都要成功）===\n");
  {
    struct { const char *name; int which; } ax[3] = {
      { "b 基座", 1 }, { "r 上臂", 2 }, { "c 下臂", 3 }
    };
    for (int i = 0; i < 3; i++) {
      SER ser;
      ser.angle1 = 90; ser.angle2 = 90; ser.angle3 = 90; ser.angle4 = 105;
      int okCount = 0, total2 = 0;
      for (double v = 0.0; v <= 180.0 + 1e-9; v += 1.0) {
        SER t = ser;
        if (ax[i].which == 1) t.angle1 = v;
        if (ax[i].which == 2) t.angle2 = v;
        if (ax[i].which == 3) t.angle3 = v;
        REC rec;
        total2++;
        if (recFromServo(&rec, &t)) okCount++;
      }
      printf("  %s: %d/%d 个采样点正解成功\n", ax[i].name, okCount, total2);
      if (okCount != total2) fails++;
    }
  }

  /* 4) 标定点核对（需要 armheight=0） */
  printf("\n=== 4) 标定点核对（固件 recFromServo，需要 armheight=0）===\n");
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

  printf("\n>>> %s (失败 %d 项)\n", fails == 0 ? "ALL PASS" : "HAS FAILURES", fails);
  return fails == 0 ? 0 : 1;
}