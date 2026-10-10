/* probe_rt.cpp —— 正解/反解往返验证（去配置化版本）
 *
 * 固件约定（已用两处实测标定核对，误差 0.00）：
 *   平面: alpha = r, beta = r - c
 *         x_planar = L1 cos(alpha) + L2 cos(beta)
 *         z        = L1 sin(alpha) + L2 sin(beta)      (armheight = 0)
 *   基座: theta = 90 - b   (b=90 朝 +x, b<90 转向 +y)
 *
 * 【本版本】servoLimit / rangeLimit 已从固件删除：
 *   关节行程硬编码 0~180，坐标范围不设限制。
 */
#include <cstdio>
#include <cmath>
#include "constant_and_positions.h"

static void fk(double b, double r, double c, double *x, double *y, double *z) {
  SER ser;
  ser.angle1 = b; ser.angle2 = r; ser.angle3 = c; ser.angle4 = 105.0;
  REC rec;
  if (!recFromServo(&rec, &ser)) { *x = *y = *z = 0.0 / 0.0; return; }
  *x = rec.x; *y = rec.y; *z = rec.z;
}

static int fails = 0;
static void check(const char *label, bool ok, const char *detail) {
  printf("  %-46s %s  %s\n", label, ok ? "PASS" : "FAIL", detail);
  if (!ok) fails++;
}

static const int BMIN = 0, BMAX = 180;
static const int RMIN = 0, RMAX = 180;
static const int CMIN = 0, CMAX = 180;

int main(void) {
  printf("=== 0) 标定点核对（固件模型正解）===\n");
  struct { double b, r, c, ex, ez; } anchors[] = {
    { 90,  90,  90, 20.00, 20.00 },
    { 90,  90, 105, 19.32, 14.82 },
    { 90,  45,   0, 28.28, 28.28 },
    {  0,  90,  90,  0.00, 20.00 },
  };
  for (unsigned i = 0; i < sizeof(anchors)/sizeof(anchors[0]); i++) {
    double x, y, z;
    fk(anchors[i].b, anchors[i].r, anchors[i].c, &x, &y, &z);
    char buf[160];
    snprintf(buf, sizeof(buf), "-> (%.2f, %.2f, %.2f)", x, y, z);
    check("anchor", fabs(x - anchors[i].ex) < 0.05 && fabs(z - anchors[i].ez) < 0.05, buf);
  }

  printf("\n=== 1) 反解: 给笛卡尔点，检查 getAngle 是否给出预期关节角 ===\n");
  {
    struct { double x, y, z, eb, er, ec; } cases[] = {
      { 20.0,   0.0, 20.0, 90,  90,  90 },
      { 19.32,  0.0, 14.82, 90, 90, 105 },
      { 28.28,  0.0,  0.00, 90, 46.00,  1.99 },
      { 28.28,  0.0, 28.28, 90, 45.01, 90.02 },
      {  0.0,  20.0, 20.0,  0, 90,  90 },
    };
    for (unsigned i = 0; i < sizeof(cases)/sizeof(cases[0]); i++) {
      pos p;
      p.rec.x = cases[i].x; p.rec.y = cases[i].y; p.rec.z = cases[i].z;
      p.ser.angle1 = p.ser.angle2 = p.ser.angle3 = 0; p.ser.angle4 = 105.0;
      bool ok = getAngle(&p);
      double fx, fy, fz;
      fk(p.ser.angle1, p.ser.angle2, p.ser.angle3, &fx, &fy, &fz);
      double err = sqrt(pow(fx - cases[i].x, 2) + pow(fy - cases[i].y, 2) + pow(fz - cases[i].z, 2));
      char buf[220];
      snprintf(buf, sizeof(buf), "-> b=%.2f r=%.2f c=%.2f (正解残差 %.4f)",
               p.ser.angle1, p.ser.angle2, p.ser.angle3, err);
      bool good = ok && err < 0.05;
      check("ik", good, buf);
    }
  }

  printf("\n=== 2) 往返: 关节角 -> 正解 -> 反解 -> 关节角 ===\n");
  int n = 0, bad = 0, clampedN = 0, otherBranch = 0, branchBad = 0;
  double worst = 0;
  for (int b = BMIN; b <= BMAX; b += 15) {
    for (int r = RMIN; r <= RMAX; r += 15) {
      for (int c = CMIN; c <= CMAX; c += 15) {
        double x, y, z;
        fk(b, r, c, &x, &y, &z);
        {
          double rho = sqrt(x*x + y*y);
          if (rho < 0.5) continue;
        }
        pos p;
        p.rec.x = x; p.rec.y = y; p.rec.z = z;
        p.ser.angle1 = p.ser.angle2 = p.ser.angle3 = 0; p.ser.angle4 = 105.0;
        bool limClamped = false;
        n++;
        if (!getAngleEx(&p, &limClamped)) {
          bad++;
          if (bad <= 5) printf("    往返失败: 起点(b=%.0f,r=%.0f,c=%.0f) -> (%.2f,%.2f,%.2f) 反解 false\n",
                               (double)b, (double)r, (double)c, x, y, z);
          continue;
        }
        if (limClamped) { clampedN++; continue; }
        double e = fabs(p.ser.angle1 - b);
        if (e > 180) e = 360 - e;
        if (e > 0.2) {
          if (fabs(e - 180.0) < 1.0) {
            double bx, by, bz;
            fk(p.ser.angle1, p.ser.angle2, p.ser.angle3, &bx, &by, &bz);
            double back = sqrt(pow(bx - x, 2) + pow(by - y, 2) + pow(bz - z, 2));
            if (back < 0.05) { otherBranch++; continue; }
          }
          branchBad++;
        }
        if (e > worst) worst = e;
      }
    }
  }
  {
    char buf[200];
    snprintf(buf, sizeof(buf),
             "共 %d 组, 反解 false %d 组, 被吸附 %d 组, 镜像分支 %d 组, "
             "非镜像回转角最大误差 %.3f° (无法解释的 %d 组)",
             n, bad, clampedN, otherBranch, worst, branchBad);
    check("round-trip", bad == 0 && worst < 0.2 && branchBad == 0, buf);
  }

  printf("\n=== 3) 正解回代残差（能解时必须 <= 0.05）===\n");
  {
    int nn = 0, badn = 0, shown = 0, clampedN3 = 0, solved = 0;
    double maxErr = 0;
    for (int b = BMIN; b <= BMAX; b += 10) {
      for (int r = RMIN; r <= RMAX; r += 10) {
        for (int c = CMIN; c <= CMAX; c += 10) {
          double x, y, z;
          fk(b, r, c, &x, &y, &z);
          {
            double rho = sqrt(x*x + y*y);
            if (rho < 0.5) continue;
          }
          pos p;
          p.rec.x = x; p.rec.y = y; p.rec.z = z;
          p.ser.angle1 = p.ser.angle2 = p.ser.angle3 = 0; p.ser.angle4 = 105.0;
          bool limClamped = false;
          nn++;
          bool solved1 = getAngleEx(&p, &limClamped);
          double fx, fy, fz;
          fk(p.ser.angle1, p.ser.angle2, p.ser.angle3, &fx, &fy, &fz);
          double err = sqrt((fx-x)*(fx-x) + (fy-y)*(fy-y) + (fz-z)*(fz-z));
          if (!solved1) {
            badn++;
            if (badn <= 8) {
              printf("    反解 false: 起点(b=%.0f,r=%.0f,c=%.0f) -> (%.3f,%.3f,%.3f)\n",
                     (double)b, (double)r, (double)c, x, y, z);
            }
            continue;
          }
          if (limClamped) { clampedN3++; continue; }
          solved++;
          if (err > maxErr) maxErr = err;
          if (err > 0.05 && shown < 8) {
            shown++;
            printf("    残差 %.4f: 期望(b=%.0f,r=%.0f,c=%.0f)->(%.2f,%.2f,%.2f)  "
                   "反解(b=%.2f,r=%.2f,c=%.2f)\n",
                   err, (double)b, (double)r, (double)c, x, y, z,
                   p.ser.angle1, p.ser.angle2, p.ser.angle3);
          }
        }
      }
    }
    char buf[220];
    snprintf(buf, sizeof(buf),
             "共 %d 组, 成功解 %d, 反解 false %d, 被吸附 %d, 成功解最大残差 %.6f",
             nn, solved, badn, clampedN3, maxErr);
    check("residual", maxErr < 0.05, buf);
  }

  printf("\n%s (失败 %d 项)\n", fails == 0 ? ">>> ALL PASS" : ">>> HAS FAILURES", fails);
  return fails == 0 ? 0 : 1;
}