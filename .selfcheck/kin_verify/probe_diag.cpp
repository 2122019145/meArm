/* probe_diag.cpp -- diagnostic companion to kin_verify.cpp.
 *
 * NOTE ON CONSTRUCTION: the firmware header in this repo is UTF-8, and g++
 * decodes it as UTF-8 here. The first build of kin_verify.cpp appeared to fail
 * inside the header only because an apostrophe in my own block comment opened a
 * character literal that swallowed the following text; it is fixed, the file is
 * pure ASCII now, and the header compiles cleanly (-fsyntax-only: 0 lines).
 *
 * Purpose: root-cause the (iv) status mismatches (ref accepts / port rejects)
 * and the (ii) 70 false round-trip results, by printing the port's raw interior
 * values and replaying every rejection gate with and without the armheight term
 * in the residual z comparison.
 */
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include "constant_and_positions.h"

#define RA 57.295779513082323

static double clampd(double v, double lo, double hi) {
  double a = lo < hi ? lo : hi, b = lo < hi ? hi : lo;
  return v < a ? a : (v > b ? b : v);
}

/* exact replay of the port gates, with switches to attribute a rejection */
struct Gate {
  bool reach, dmin, dmax, sing, resid3, resid2, resid3nc;
  double rhChk, bRaw, rRaw, cRaw, nb, nr, nc, res3, res2, res3nc, dz, zCorr;
};
static Gate replay(double x, double y, double z, double L1, double L2, double L3) {
  Gate g; memset(&g, 0, sizeof(g));
  double rho = sqrt(x * x + y * y), zv = z - L1, d = sqrt(rho * rho + zv * zv);
  g.reach = std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  double dMin = fabs(L2 - L3) + 1.0, dMax = (L2 + L3) - 1.0;
  g.dmin = (d > dMin); g.dmax = (d < dMax);
  if (!g.dmin || !g.dmax) return g;
  double cosB = (L2 * L2 + L3 * L3 - d * d) / (2.0 * L2 * L3);
  if (cosB > 1.0) cosB = 1.0; if (cosB < -1.0) cosB = -1.0;
  double beta = acos(cosB);
  double phi = atan2(zv, rho);
  double alpha = phi + atan2(L3 * sin(beta), L2 - L3 * cos(beta));
  double rh = L2 * cos(alpha) - L3 * cos(alpha + beta);
  double zz = L1 + L2 * sin(alpha) - L3 * sin(alpha + beta);
  g.rhChk = rh; (void)zz;
  g.sing = (rh >= 5.0);
  if (!g.sing) return g;
  double theta = -atan2(x, y);
  g.bRaw = RA * theta + 90.0;
  g.rRaw = 180.0 - RA * alpha;
  g.cRaw = RA * beta;
  g.nb = clampd(g.bRaw, KIN_BASE_MIN, KIN_BASE_MAX);
  g.nr = clampd(g.rRaw, KIN_RARM_MIN, KIN_RARM_MAX);
  g.nc = clampd(g.cRaw, KIN_FARM_MIN, KIN_FARM_MAX);
  double vT = (g.nb - 90.0) / RA, vA = (180.0 - g.nr) / RA, vB = g.nc / RA;
  double vRh = L2 * cos(vA) - L3 * cos(vA + vB);
  double vZ = L1 + L2 * sin(vA) - L3 * sin(vA + vB);
  double vx = -vRh * sin(vT), vy = vRh * cos(vT);
  double dx = vx - x, dy = vy - y, dz = vZ - z;
  g.res3 = sqrt(dx * dx + dy * dy + dz * dz);
  g.res2 = sqrt(dx * dx + dy * dy);
  /* what the residual would be if z were compared in the same frame as the FK */
  g.dz = dz; g.zCorr = dz - 0.0;
  return g;
}
/* exact replay of the reference solveIK (double variant) */
static bool refIK(double x, double y, double z, double L1, double L2, double L3,
                  double &ob, double &orr, double &oc, int &reason) {
  double R = sqrt(x * x + y * y), Z = z - L1, d = sqrt(R * R + Z * Z);
  if (d > L2 + L3 - 1.0 || d < fabs(L2 - L3) + 1.0) { reason = 1; return false; }
  double cosB = (L2 * L2 + L3 * L3 - d * d) / (2.0 * L2 * L3);
  if (cosB > 1.0) cosB = 1.0; if (cosB < -1.0) cosB = -1.0;
  double beta = acos(cosB), phi = atan2(Z, R);
  double alpha = phi + atan2(L3 * sin(beta), L2 - L3 * cos(beta));
  double rC = L2 * cos(alpha) - L3 * cos(alpha + beta);
  if (rC < 5.0) { reason = 2; return false; }
  double t1 = -atan2(x, y);
  double b = clampd(RA * t1 + 90.0, 0, 180);
  double rr = clampd(180.0 - RA * alpha, 45, 160);
  double c = clampd(RA * beta, 30, 110);
  double vT = (b - 90.0) / RA, vA = (180.0 - rr) / RA, vB = c / RA;
  double vr = L2 * cos(vA) - L3 * cos(vA + vB);
  double vmx = -vr * sin(vT), vmy = vr * cos(vT);
  if (sqrt((vmx - x) * (vmx - x) + (vmy - y) * (vmy - y)) > 5.0) { reason = 3; return false; }
  ob = b; orr = rr; oc = c; reason = 0; return true;
}

int main(void) {
  const double L1 = arm1.armheight, L2 = arm1.armLength1, L3 = arm1.armLength2;
  printf("=== diagnostic A: first mismatching grid points, port interior vs reference ===\n");
  printf("gate order: reach, dmin, dmax, singular(R_h>=5), residual(3-D as coded)\n");
  int shown = 0;
  long nRes3 = 0, nRes3WouldPassIfZFixed = 0, nOther = 0, nSing = 0, nTot = 0;
  double worstDz = 0;
  for (int xi = -160; xi <= 160 && shown < 12; xi += 8) {
    for (int yi = 0; yi <= 160 && shown < 12; yi += 8) {
      for (int zi = -122; zi <= 198 && shown < 12; zi += 8) {
        double x = xi, y = yi, z = zi;
        pos p; p.rec.x = x; p.rec.y = y; p.rec.z = z;
        p.ser.angle1 = 12345.0; p.ser.angle2 = 12345.0; p.ser.angle3 = 12345.0;
        bool cl = false;
        bool our = getAngleEx(&p, &cl);
        double rb, rr, rc; int reason = 0;
        bool ref = refIK(x, y, z, L1, L2, L3, rb, rr, rc, reason);
        if (ref == our || !ref) continue;
        /* ref accepted, port rejected: attribute */
        Gate g = replay(x, y, z, L1, L2, L3);
        double zErrIfFixed = g.res3; /* 3-D with the coded dz */
        double dzCoded = g.dz;
        /* residual with z compared correctly: fkPlanar's z already includes L1, and
         * rec.z is the same frame, so the coded dz is already correct-frame.
         * The candidate defect: re-run the reference residual rule (x,y only). */
        double dx = 0, dy = 0; /* recompute explicitly for clarity */
        {
          double vT = (g.nb - 90.0) / RA, vA = (180.0 - g.nr) / RA, vB = g.nc / RA;
          double vRh = L2 * cos(vA) - L3 * cos(vA + vB);
          double vZ = L1 + L2 * sin(vA) - L3 * sin(vA + vB);
          double vx = -vRh * sin(vT), vy = vRh * cos(vT);
          dx = vx - x; dy = vy - y;
          (void)vZ;
        }
        printf("  (%7.1f,%7.1f,%7.1f) refOk=1 b=%.6f r=%.6f c=%.6f | port bRaw=%.6f rRaw=%.6f cRaw=%.6f -> nb=%.6f nr=%.6f nc=%.6f\n",
               x, y, z, rb, rr, rc, g.bRaw, g.rRaw, g.cRaw, g.nb, g.nr, g.nc);
        printf("        gates: reach=%d dmin=%d dmax=%d sing(R_h=%.4f)=%d | residual 3D=%.6f (dx=%.6f dy=%.6f dz=%.6f) 2D=%.6f\n",
               (int)g.reach, (int)g.dmin, (int)g.dmax, g.rhChk, (int)g.sing,
               g.res3, dx, dy, dzCoded, sqrt(dx * dx + dy * dy));
        shown++;
      }
    }
  }
  printf("\n=== diagnostic B: classify every ref-accepts/port-rejects point on the full grid ===\n");
  for (int xi = -160; xi <= 160; xi += 8) {
    for (int yi = 0; yi <= 160; yi += 8) {
      for (int zi = -122; zi <= 198; zi += 8) {
        double x = xi, y = yi, z = zi;
        pos p; p.rec.x = x; p.rec.y = y; p.rec.z = z;
        p.ser.angle1 = 12345.0; p.ser.angle2 = 12345.0; p.ser.angle3 = 12345.0;
        bool cl = false;
        bool our = getAngleEx(&p, &cl);
        double rb, rr, rc; int reason = 0;
        bool ref = refIK(x, y, z, L1, L2, L3, rb, rr, rc, reason);
        if (!ref || our) continue;
        nTot++;
        Gate g = replay(x, y, z, L1, L2, L3);
        if (!g.sing) { nSing++; continue; }
        bool res3fail = (g.res3 > 5.0);
        bool res2fail = (g.res2 > 5.0);
        if (res3fail && !res2fail) {
          nRes3++;
          if (fabs(g.dz) > worstDz) worstDz = fabs(g.dz);
          /* would it pass with the reference rule (x,y only)? */
          if (!res2fail) nRes3WouldPassIfZFixed++;
        } else nOther++;
      }
    }
  }
  printf("  total ref-accepts/port-rejects = %ld\n", nTot);
  printf("  rejected by the singular gate (R_h<5)      = %ld\n", nSing);
  printf("  rejected by the residual with 3-D distance = %ld   <-- passes the reference 2-D rule\n", nRes3);
  printf("  of those, 2-D-only residual would pass     = %ld\n", nRes3WouldPassIfZFixed);
  printf("  rejected for any other reason              = %ld\n", nOther);
  printf("  max |dz| among the 3-D-only rejections     = %.6f mm\n", worstDz);

  printf("\n=== diagnostic C: the (ii) false round-trip results, attributed ===\n");
  const double bs[] = { 0, 30, 45, 60, 90, 120, 135, 150, 180 };
  long nFalse = 0, fBoundary = 0, fResid3 = 0, fOther = 0;
  int shown2 = 0;
  for (int bi = 0; bi < 9; bi++) {
    for (int r = 45; r <= 160; r += 5) {
      for (int c = 30; c <= 110; c += 5) {
        SER s; s.angle1 = bs[bi]; s.angle2 = r; s.angle3 = c; s.angle4 = 105.0;
        REC o;
        if (!recFromServo(&o, &s)) continue;
        pos p; p.rec = o;
        p.ser.angle1 = 12345.0; p.ser.angle2 = 12345.0; p.ser.angle3 = 12345.0;
        bool cl = false;
        if (getAngleEx(&p, &cl)) continue;
        nFalse++;
        Gate g = replay(o.x, o.y, o.z, L1, L2, L3);
        bool isBoundary = (bs[bi] == 0 || bs[bi] == 180 || r == 45 || r == 160 || c == 30 || c == 110);
        if (isBoundary) fBoundary++;
        else if (g.sing && g.res3 > 5.0 && g.res2 <= 5.0) fResid3++;
        else fOther++;
        if (shown2 < 12) {
          shown2++;
          printf("  src b=%.0f r=%d c=%d -> (%.6f,%.6f,%.6f) | bRaw=%.9f rRaw=%.9f cRaw=%.9f nb=%.6f nr=%.6f nc=%.6f R_h=%.6f res3=%.6f res2=%.6f\n",
                 bs[bi], r, c, o.x, o.y, o.z, g.bRaw, g.rRaw, g.cRaw, g.nb, g.nr, g.nc, g.rhChk, g.res3, g.res2);
        }
      }
    }
  }
  printf("  false=%ld  of which on a window boundary (b 0/180, r 45/160, c 30/110)=%ld  residual-3D-only=%ld  other=%ld\n",
         nFalse, fBoundary, fResid3, fOther);

  printf("\n=== diagnostic D: does the port reproduce the reference if it used the reference 2-D rule? ===\n");
  long agree = 0, total = 0, refOnly = 0;
  for (int xi = -160; xi <= 160; xi += 8) {
    for (int yi = 0; yi <= 160; yi += 8) {
      for (int zi = -122; zi <= 198; zi += 8) {
        double x = xi, y = yi, z = zi;
        total++;
        double rb, rr, rc; int reason = 0;
        bool ref = refIK(x, y, z, L1, L2, L3, rb, rr, rc, reason);
        Gate g = replay(x, y, z, L1, L2, L3);
        bool portStrict = g.reach && g.dmin && g.dmax && g.sing && (g.res3 <= 5.0);
        bool portRefRule = g.reach && g.dmin && g.dmax && g.sing && (g.res2 <= 5.0);
        if (portRefRule && ref) agree++;
        if (ref && !portStrict) refOnly++;
        (void)portStrict;
      }
    }
  }
  printf("  grid=%ld  ref-accepts=%ld our-port-rejects=%ld  (2-D-rule replay agrees with ref on %ld of %ld)\n",
         total, agree, refOnly, agree, total);
  printf("\n=== diagnostic E: POS_HOME exact bit pattern ===\n");
  {
    pos p; p.rec.x = 0.0; p.rec.y = 80.0; p.rec.z = 118.0;
    p.ser.angle1 = 12345.0; p.ser.angle2 = 12345.0; p.ser.angle3 = 12345.0; p.ser.angle4 = 105.0;
    bool cl = true;
    bool ok = getAngleEx(&p, &cl);
    printf("  ok=%d clamped=%d b=(%.20g) r=(%.20g) c=(%.20g)\n", (int)ok, (int)cl,
           p.ser.angle1, p.ser.angle2, p.ser.angle3);
    printf("  b==90.0:%d r==90.0:%d c==90.0:%d  |c-90|=%.3e  |b-90|=%.3e |r-90|=%.3e\n",
           p.ser.angle1 == 90.0, p.ser.angle2 == 90.0, p.ser.angle3 == 90.0,
           fabs(p.ser.angle3 - 90.0), fabs(p.ser.angle1 - 90.0), fabs(p.ser.angle2 - 90.0));
    printf("  c in KIN window [%.1f,%.1f]: %d (so no clamping is lost by the 1.4e-14 offset)\n",
           KIN_FARM_MIN, KIN_FARM_MAX, p.ser.angle3 >= KIN_FARM_MIN && p.ser.angle3 <= KIN_FARM_MAX);
    SER s; s.angle1 = 1; s.angle2 = 2; s.angle3 = 3; s.angle4 = 105.0;
    bool h = posGetHomeAngles(&s);
    printf("  posGetHomeAngles ok=%d b=%.20g r=%.20g c=%.20g a4=%.1f\n", (int)h, s.angle1, s.angle2, s.angle3, s.angle4);
  }
  printf("\n=== diagnostic F: singular-gate contact: how many grid points sit on R_h in [5,6)? ===\n");
  {
    long near = 0, rej = 0, total2 = 0;
    double minRh = 1e9;
    for (int xi = -160; xi <= 160; xi += 8) {
      for (int yi = 0; yi <= 160; yi += 8) {
        for (int zi = -122; zi <= 198; zi += 8) {
          double x = xi, y = yi, z = zi;
          Gate g = replay(x, y, z, L1, L2, L3);
          if (!g.dmin || !g.dmax) continue;
          total2++;
          if (g.rhChk < minRh) minRh = g.rhChk;
          if (g.rhChk >= 5.0 && g.rhChk < 6.0) near++;
          if (!g.sing) rej++;
        }
      }
    }
    printf("  in-workspace grid points=%ld  rejected by singular gate=%ld  with R_h in [5,6)=%ld  min R_h=%.6f\n",
           total2, rej, near, minRh);
  }
  return 0;
}
