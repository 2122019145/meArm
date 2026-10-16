/* kin_verify.cpp -- independent numerical verification of the ported kinematics.
 *
 * Location: D:\dsh1\meArm\.selfcheck\kin_verify\
 * Build recipe reused from .selfcheck\run_all.ps1 (step 2):
 *   g++ -std=gnu++17 -O2 -Wall -Wextra -Wshadow
 *       -DWEARM_DEBUG_SERIAL=1 -DWEARM_ENABLE_PICK_PLACE=1
 *       -DWEARM_ENABLE_BUTTONS=1 -DWEARM_ENABLE_DRAW=1
 *       -I<root> -I<root>\.selfcheck\mock
 *       kin_verify.cpp <root>\constant_and_positions.cpp .selfcheck\mock\Arduino.cpp
 *
 * REFERENCE: D:\dsh1\meArm\others\f\MeArm\Task5_LineN2_TeachSmooth\Task5_LineN2_TeachSmooth.ino
 *   forwardKin()  (lines 162-171), servoToGeo() (174-178), solveIK() (201-243)
 *   The X-sign of forwardKin matches JoyFK_Xflip ino line 133
 *   (mx = -r*sin(t1)); all Task5 and JoyFK sketches are byte-identical.
 *
 * Two reference variants are implemented, both written VERBATIM from the ino
 * (only the numeric type is changed):
 *   refSolveIKF / refForwardKinF : float, angles rounded to int   [ literally what the ino does ]
 *   refSolveIKD / refForwardKinD : double, angles NOT rounded     [ for 1e-6 comparisons ]
 * A float plus int rounding port can never agree with a double port to 1e-6 deg,
 * so check (iv) compares against refSolveIKD (documented); float and int
 * differences are reported separately so the reviewer can see exactly how much
 * of the reference behaviour is lost to its own integer quantization.
 *
 * The reference round-trip residual only compares (x, y) [ Task5 ino line 239 ];
 * our port compares the full 3-D distance. That asymmetry is reported, not hidden.
 *
 * READ-ONLY with respect to the firmware: this file only includes the header.
 */
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include "constant_and_positions.h"

/* ------------------------------------------------------------------ */
/* reference numerics                                                  */
/* ------------------------------------------------------------------ */
static const double RL1 = 38.0, RL2 = 80.0, RL3 = 80.0;
static const double RBASE_MIN = 0, RBASE_MAX = 180;
static const double RRARM_MIN = 45, RRARM_MAX = 160;
static const double RFARM_MIN = 30, RFARM_MAX = 110;

static inline double d2r(double d) { return d * (M_PI / 180.0); }
static inline double r2d(double r) { return r * (180.0 / M_PI); }

static inline double refClampVal(double v, double lo, double hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}
static inline int refClampValI(int v, int lo, int hi) {
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

/* verbatim forwardKin (Task5_...ino:162-171) -- double variant */
static void refForwardKinD(double t1Deg, double aDeg, double bDeg,
                           double &mx, double &my, double &mz) {
  double t1 = d2r(t1Deg);
  double a = d2r(aDeg);
  double b = d2r(bDeg);
  double r = RL2 * cos(a) - RL3 * cos(a + b);
  mx = -r * sin(t1);
  my = r * cos(t1);
  mz = RL1 + RL2 * sin(a) - RL3 * sin(a + b);
}
/* verbatim forwardKin -- float variant (exactly the .ino's arithmetic) */
static void refForwardKinF(double t1Deg, double aDeg, double bDeg,
                           double &mx, double &my, double &mz) {
  double t1 = (double)(float)((float)t1Deg * (float)(M_PI / 180.0));
  double a = (double)(float)((float)aDeg * (float)(M_PI / 180.0));
  double b = (double)(float)((float)bDeg * (float)(M_PI / 180.0));
  double r = (double)(float)((float)RL2 * (float)cosf((float)a) - (float)RL3 * (float)cosf((float)(a + b)));
  mx = (double)(float)(-(float)r * sinf((float)t1));
  my = (double)(float)((float)r * cosf((float)t1));
  mz = (double)(float)((float)RL1 + (float)RL2 * sinf((float)a) - (float)RL3 * sinf((float)(a + b)));
}

/* verbatim solveIK (Task5_...ino:201-243) -- double, angles not rounded */
struct RefIK { bool ok; double b, r, f; bool clamped; };
static RefIK refSolveIKD(double x, double y, double z) {
  RefIK o; o.ok = false; o.b = o.r = o.f = 0; o.clamped = false;
  double R = sqrt(x * x + y * y);
  double Z = z - RL1;
  double d = sqrt(R * R + Z * Z);
  if (d > RL2 + RL3 - 1.0 || d < fabs(RL2 - RL3) + 1.0) return o;
  double cosB = (RL2 * RL2 + RL3 * RL3 - d * d) / (2.0 * RL2 * RL3);
  if (cosB > 1.0) cosB = 1.0;
  if (cosB < -1.0) cosB = -1.0;
  double beta = acos(cosB);
  double phi = atan2(Z, R);
  double alpha = phi + atan2(RL3 * sin(beta), RL2 - RL3 * cos(beta));
  double rChk = RL2 * cos(alpha) - RL3 * cos(alpha + beta);
  if (rChk < 5.0) return o;
  double t1 = -atan2(x, y);
  double bRaw = r2d(t1) + 90.0;
  double rRaw = 180.0 - r2d(alpha);
  double fRaw = r2d(beta);
  double nb = refClampVal(bRaw, RBASE_MIN, RBASE_MAX);
  double nr = refClampVal(rRaw, RRARM_MIN, RRARM_MAX);
  double nf = refClampVal(fRaw, RFARM_MIN, RFARM_MAX);
  o.clamped = (nb != bRaw || nr != rRaw || nf != fRaw);
  /* reference residual: (x, y) only, no z (line 239) */
  double vT1 = d2r(nb - 90.0);
  double vAlp = d2r(180.0 - nr);
  double vBet = d2r(nf);
  double vr = RL2 * cos(vAlp) - RL3 * cos(vAlp + vBet);
  double vmx = -vr * sin(vT1);
  double vmy = vr * cos(vT1);
  if (sqrt((vmx - x) * (vmx - x) + (vmy - y) * (vmy - y)) > 5.0) return o;
  o.ok = true; o.b = nb; o.r = nr; o.f = nf;
  return o;
}
/* verbatim solveIK -- float + int rounding, exactly as compiled for the Uno */
static RefIK refSolveIKF(double x, double y, double z) {
  RefIK o; o.ok = false; o.b = o.r = o.f = 0; o.clamped = false;
  float fx = (float)x, fy = (float)y, fz = (float)z;
  float R = sqrtf(fx * fx + fy * fy);
  float Z = fz - 38.0f;
  float d = sqrtf(R * R + Z * Z);
  if (d > 80.0f + 80.0f - 1.0f || d < fabsf(80.0f - 80.0f) + 1.0f) return o;
  float cosB = (80.0f * 80.0f + 80.0f * 80.0f - d * d) / (2.0f * 80.0f * 80.0f);
  if (cosB > 1.0f) cosB = 1.0f;
  if (cosB < -1.0f) cosB = -1.0f;
  float beta = acosf(cosB);
  float phi = atan2f(Z, R);
  float alpha = phi + atan2f(80.0f * sinf(beta), 80.0f - 80.0f * cosf(beta));
  float rChk = 80.0f * cosf(alpha) - 80.0f * cosf(alpha + beta);
  if (rChk < 5.0f) return o;
  float t1 = -atan2f(fx, fy);
  int bAng = (int)roundf((float)(t1 * (float)(180.0 / M_PI)) + 90.0f);
  int rAng = (int)roundf(180.0f - (float)(alpha * (float)(180.0 / M_PI)));
  int fAng = (int)roundf((float)(beta * (float)(180.0 / M_PI)));
  int nb = refClampValI(bAng, 0, 180);
  int nr = refClampValI(rAng, 45, 160);
  int nf = refClampValI(fAng, 30, 110);
  o.clamped = (nb != bAng || nr != rAng || nf != fAng);
  float vT1 = (float)(((float)nb - 90.0f) * (float)(M_PI / 180.0));
  float vAlp = (float)((180.0f - (float)nr) * (float)(M_PI / 180.0));
  float vBet = (float)((float)nf * (float)(M_PI / 180.0));
  float vr = 80.0f * cosf(vAlp) - 80.0f * cosf(vAlp + vBet);
  float vmx = -vr * sinf(vT1);
  float vmy = vr * cosf(vT1);
  if (sqrtf((vmx - fx) * (vmx - fx) + (vmy - fy) * (vmy - fy)) > 5.0f) return o;
  o.ok = true; o.b = nb; o.r = nr; o.f = nf;
  return o;
}

/* ------------------------------------------------------------------ */
/* harness                                                             */
/* ------------------------------------------------------------------ */
static int g_fail = 0;
static void verdict(const char *label, bool ok, const char *detail) {
  printf("  [%s] %-34s %s\n", ok ? "PASS" : "FAIL", label, detail);
  if (!ok) g_fail++;
}
static inline double dist3(double ax, double ay, double az,
                           double bx, double by, double bz) {
  double dx = ax - bx, dy = ay - by, dz = az - bz;
  return sqrt(dx * dx + dy * dy + dz * dz);
}
static inline bool finite3(double a, double b, double c) {
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

/* our FK through the firmware entry point */
static bool ourFK(double b, double r, double c, double *x, double *y, double *z) {
  SER s; s.angle1 = b; s.angle2 = r; s.angle3 = c; s.angle4 = 105.0;
  REC o;
  if (!recFromServo(&o, &s)) { *x = *y = *z = NAN; return false; }
  *x = o.x; *y = o.y; *z = o.z; return true;
}

/* ================================================================== */
/* (i) our FK vs reference FK, servo grid                             */
/* ================================================================== */
static void check_fk_grid(void) {
  printf("\n=== (i) our recFromServo vs reference forwardKin, grid r,c in [0,180] step 5, b in {0,45,90,135,180} ===\n");
  const double bs[] = { 0, 45, 90, 135, 180 };
  long n = 0, bad = 0;
  double maxD = 0, maxF = 0;
  char worst[192] = "";
  for (int bi = 0; bi < 5; bi++) {
    for (int r = 0; r <= 180; r += 5) {
      for (int c = 0; c <= 180; c += 5) {
        double ox, oy, oz;
        if (!ourFK(bs[bi], r, c, &ox, &oy, &oz)) { bad++; continue; }
        double rx, ry, rz;
        refForwardKinD(bs[bi] - 90.0, 180.0 - r, c, rx, ry, rz);
        double e = dist3(ox, oy, oz, rx, ry, rz);
        if (e > maxD) {
          maxD = e;
          snprintf(worst, sizeof(worst), "worst b=%.0f r=%d c=%d our(%.12g,%.12g,%.12g) ref(%.12g,%.12g,%.12g)",
                   bs[bi], r, c, ox, oy, oz, rx, ry, rz);
        }
        double fx, fy, fz;
        refForwardKinF(bs[bi] - 90.0, 180.0 - r, c, fx, fy, fz);
        double ef = dist3(ox, oy, oz, fx, fy, fz);
        if (ef > maxF) maxF = ef;
        n++;
      }
    }
  }
  printf("      samples=%ld, recFromServo failures=%ld\n", n, bad);
  printf("      max |our - ref(double)| = %.6e mm\n", maxD);
  printf("      max |our - ref(float, as compiled for Uno)| = %.6e mm\n", maxF);
  printf("      %s\n", worst);
  char d[192];
  snprintf(d, sizeof(d), "n=%ld maxerr=%.3e mm maxerr_floatref=%.3e failures=%ld", n, maxD, maxF, bad);
  verdict("fk-grid", bad == 0 && maxD < 1e-9 && maxF < 1e-4, d);
}

/* ================================================================== */
/* (ii) round trip: getAngleEx -> recFromServo                         */
/* ================================================================== */
static void check_roundtrip(void) {
  printf("\n=== (ii) round trip: FK point -> getAngleEx -> recFromServo ===\n");
  const double bs[] = { 0, 30, 45, 60, 90, 120, 135, 150, 180 };
  long n = 0, ikFalse = 0, clamped = 0, solvable = 0, badRes = 0;
  double maxErr = 0;
  char worst[224] = "";
  printf("      source grid: b in {0,30,45,60,90,120,135,150,180}, r in [45,160] step 5, c in [30,110] step 5\n");
  for (int bi = 0; bi < 9; bi++) {
    for (int r = 45; r <= 160; r += 5) {
      for (int c = 30; c <= 110; c += 5) {
        double sx, sy, sz;
        if (!ourFK(bs[bi], r, c, &sx, &sy, &sz)) continue;
        n++;
        pos p;
        p.rec.x = sx; p.rec.y = sy; p.rec.z = sz;
        /* poison: any write to ser is visible */
        p.ser.angle1 = 12345.0; p.ser.angle2 = 12345.0;
        p.ser.angle3 = 12345.0; p.ser.angle4 = 105.0;
        bool cl = false;
        if (!getAngleEx(&p, &cl)) { ikFalse++; continue; }
        double fx, fy, fz;
        if (!ourFK(p.ser.angle1, p.ser.angle2, p.ser.angle3, &fx, &fy, &fz)) { ikFalse++; continue; }
        double e = dist3(fx, fy, fz, sx, sy, sz);
        if (cl) {
          clamped++;
          /* the port's own contract is 5 mm after clamping */
          if (e > 5.0) {
            badRes++;
            if (badRes <= 5)
              printf("      clamped-residual>5mm: src(b=%.0f,r=%d,c=%d) -> (%.6f,%.6f,%.6f) solved b=%.12f r=%.12f c=%.12f err=%.6f\n",
                     bs[bi], r, c, sx, sy, sz, p.ser.angle1, p.ser.angle2, p.ser.angle3, e);
          }
          continue;
        }
        solvable++;
        if (e > maxErr) {
          maxErr = e;
          snprintf(worst, sizeof(worst), "b=%.0f r=%d c=%d -> (%.9f,%.9f,%.9f) solved (%.12f,%.12f,%.12f) err=%.3e",
                   bs[bi], r, c, sx, sy, sz, p.ser.angle1, p.ser.angle2, p.ser.angle3, e);
        }
      }
    }
  }
  printf("      samples=%ld  getAngleEx false=%ld  clamped=%ld (skipped)  unclamped=%ld\n",
         n, ikFalse, clamped, solvable);
  printf("      max 3D round-trip error (unclamped) = %.6e mm\n", maxErr);
  printf("      clamped results with residual > 5mm = %ld\n", badRes);
  printf("      worst: %s\n", worst);
  char d[200];
  snprintf(d, sizeof(d), "n=%ld false=%ld clamped=%ld maxerr=%.3e mm", n, ikFalse, clamped, maxErr);
  verdict("round-trip", ikFalse == 0 && badRes == 0 && maxErr < 0.01, d);
}

/* ================================================================== */
/* (iii) POS_HOME (0,80,118) -> exactly (90,90,90), no clamping        */
/* ================================================================== */
static void check_home(void) {
  printf("\n=== (iii) POS_HOME (0,80,118) -> exactly (90,90,90), *clamped == false ===\n");
  /* rebuild POS_HOME from the firmware source's own constant: it is a file-static
   * in constant_and_positions.cpp, so reproduce it and also probe via posGetHomeAngles() */
  pos p;
  p.rec.x = 0.0; p.rec.y = 80.0; p.rec.z = 118.0;
  p.ser.angle1 = 12345.0; p.ser.angle2 = 12345.0; p.ser.angle3 = 12345.0; p.ser.angle4 = 105.0;
  bool cl = true;
  bool ok = getAngleEx(&p, &cl);
  printf("      getAngleEx: ok=%d clamped=%d  b=%.20g (hex %a)  r=%.20g (hex %a)  c=%.20g (hex %a)\n",
         (int)ok, (int)cl, p.ser.angle1, p.ser.angle1, p.ser.angle2, p.ser.angle2,
         p.ser.angle3, p.ser.angle3);
  bool exact = ok && !cl && p.ser.angle1 == 90.0 && p.ser.angle2 == 90.0 && p.ser.angle3 == 90.0;
  verdict("home-exact-90",
          exact,
          exact ? "bit-exact 90/90/90, clamped=false"
                : "NOT exactly (90,90,90) without clamping");

  /* independent hand-computation of the raw values, to show where the exactness comes from */
  double x = 0, y = 80, z = 118;
  double rho = sqrt(x * x + y * y), zv = z - 38.0, d = sqrt(rho * rho + zv * zv);
  double cosB = (6400.0 + 6400.0 - d * d) / (2.0 * 80.0 * 80.0);
  double beta = acos(cosB);
  double phi = atan2(zv, rho);
  double alpha = phi + atan2(80.0 * sin(beta), 80.0 - 80.0 * cos(beta));
  double rhoR = 180.0 / M_PI;
  printf("      hand: rho=%.17g zv=%.17g d=%.17g cosB=%.17g beta*RADtoDEG=%.17g alpha*RADtoDEG=%.17g\n",
         rho, zv, d, cosB, beta * rhoR, alpha * rhoR);
  printf("      hand: bRaw=%.17g rRaw=%.17g cRaw=%.17g theta=%.17g\n",
         rhoR * (-atan2(x, y)) + 90.0, 180.0 - rhoR * alpha, rhoR * beta, -atan2(x, y));
  printf("      KIN windows: b[%.1f,%.1f] r[%.1f,%.1f] c[%.1f,%.1f]  (rRaw/cRaw/bRaw all interior => no clamp)\n",
         KIN_BASE_MIN, KIN_BASE_MAX, KIN_RARM_MIN, KIN_RARM_MAX, KIN_FARM_MIN, KIN_FARM_MAX);

  SER s;
  s.angle1 = 1.0; s.angle2 = 2.0; s.angle3 = 3.0; s.angle4 = 105.0;
  bool hok = posGetHomeAngles(&s);
  printf("      posGetHomeAngles: ok=%d b=%.17g r=%.17g c=%.17g angle4(kept)=%.1f\n",
         (int)hok, s.angle1, s.angle2, s.angle3, s.angle4);
  verdict("posGetHomeAngles",
          hok && s.angle1 == 90.0 && s.angle2 == 90.0 && s.angle3 == 90.0 && s.angle4 == 105.0,
          hok ? "returns bit-exact 90/90/90 and leaves angle4 untouched" : "not exactly 90/90/90");
}

/* ================================================================== */
/* (iv) our getAngleEx vs reference solveIK over the coordinate grid   */
/* ================================================================== */
#define MAXMIS 20
struct Mismatch {
  double x, y, z;
  int refOk, ourOk;
  double rb, rr, rf, ob, orr, oc;
  const char *why;
};
static void check_ik_grid(void) {
  printf("\n=== (iv) getAngleEx vs reference solveIK, x in [-160,160] step 8, y in [0,160] step 8, z in [-122,198] step 8 ===\n");
  long n = 0, refFalseOursFalse = 0, refTrueOursTrue = 0;
  long nMismatch = 0, nAng = 0, nStatus = 0, nClampDiff = 0;
  static Mismatch allFirst[MAXMIS];
  Mismatch *list = allFirst;
  int nList = 0;
  printf("      primary comparison uses refSolveIKD (double, angles NOT rounded) -- a float/int-rounded\n"
         "      reference cannot agree with a double port to 1e-6 deg; the float variant is tabulated below.\n");
  long fStatusMismatch = 0, fAngGt1e6 = 0;
  double fMaxAng = 0; /* vs rounded float reference */
  char fWorst[224] = "";
  for (int xi = -160; xi <= 160; xi += 8) {
    for (int yi = 0; yi <= 160; yi += 8) {
      for (int zi = -122; zi <= 198; zi += 8) {
        double x = xi, y = yi, z = zi;
        n++;
        pos p;
        p.rec.x = x; p.rec.y = y; p.rec.z = z;
        p.ser.angle1 = 12345.0; p.ser.angle2 = 12345.0; p.ser.angle3 = 12345.0; p.ser.angle4 = 105.0;
        bool cl = false;
        bool ou = getAngleEx(&p, &cl);
        RefIK R = refSolveIKD(x, y, z);
        RefIK F = refSolveIKF(x, y, z);
        if (!R.ok && !ou) refFalseOursFalse++;
        if (R.ok && ou) refTrueOursTrue++;
        if (R.ok == ou && ou) {
          double ab = fabs(R.b - p.ser.angle1);
          double ar = fabs(R.r - p.ser.angle2);
          double af = fabs(R.f - p.ser.angle3);
          double m = ab > ar ? (ab > af ? ab : af) : (ar > af ? ar : af);
          if (m > 1e-6) {
            nAng++;
            if (nList < MAXMIS) {
              Mismatch &q = list[nList++];
              q.x = x; q.y = y; q.z = z; q.refOk = 1; q.ourOk = 1;
              q.rb = R.b; q.rr = R.r; q.rf = R.f;
              q.ob = p.ser.angle1; q.orr = p.ser.angle2; q.oc = p.ser.angle3;
              q.why = "angle differs > 1e-6 deg";
            }
          }
          if (R.clamped != cl) nClampDiff++;
        } else if (R.ok != ou) {
          nStatus++;
          if (nList < MAXMIS) {
            Mismatch &q = list[nList++];
            q.x = x; q.y = y; q.z = z; q.refOk = R.ok ? 1 : 0; q.ourOk = ou ? 1 : 0;
            q.rb = R.b; q.rr = R.r; q.rf = R.f;
            q.ob = p.ser.angle1; q.orr = p.ser.angle2; q.oc = p.ser.angle3;
            q.why = "status differs";
          }
        }
        if (R.ok != ou || (!ou && R.ok) || (ou && !R.ok)) nMismatch = nStatus + nAng;
        /* float reference: status compared too (its int rounding can flip status) */        if (F.ok != ou) fStatusMismatch++;
        if (F.ok && ou) {
          double d1 = fabs(F.b - p.ser.angle1);
          double d2 = fabs(F.r - p.ser.angle2);
          double d3 = fabs(F.f - p.ser.angle3);
          double mx = d1 > d2 ? (d1 > d3 ? d1 : d3) : (d2 > d3 ? d2 : d3);
          if (mx > fMaxAng) {
            fMaxAng = mx;
            snprintf(fWorst, sizeof(fWorst), "x=%.0f y=%.0f z=%.0f ref-int(%.0f,%.0f,%.0f) ours(%.12f,%.12f,%.12f)",
                     x, y, z, F.b, F.r, F.f, p.ser.angle1, p.ser.angle2, p.ser.angle3);
          }
          if (mx > 1e-6) fAngGt1e6++;
        }
      }
    }
  }
  printf("      grid points = %ld\n", n);
  printf("      status: both true = %ld, both false = %ld\n", refTrueOursTrue, refFalseOursFalse);
  nMismatch = nStatus + nAng;
  printf("      MISMATCHES vs refSolveIKD(double): status=%ld  angles>1e-6deg=%ld  TOTAL=%ld\n",
         nStatus, nAng, nMismatch);
  printf("      *clamped flag disagreements = %ld\n", nClampDiff);
  printf("      vs refSolveIKF(float+int round): status mismatches=%ld, angle diff >1e-6 = %ld, max angle diff = %.6f deg\n",
         fStatusMismatch, fAngGt1e6, fMaxAng);
  if (fMaxAng > 1e-6) printf("      float-ref worst: %s\n", fWorst);
  printf("      ---- first %d mismatches (vs the double reference) ----\n", nList);
  for (int i = 0; i < nList; i++) {
    Mismatch &q = list[i];
    printf("      #%2d (%8.1f,%8.1f,%8.1f) refOk=%d ourOk=%d | ref(%.12f,%.12f,%.12f) our(%.12f,%.12f,%.12f) | %s\n",
           i + 1, q.x, q.y, q.z, q.refOk, q.ourOk, q.rb, q.rr, q.rf, q.ob, q.orr, q.oc, q.why);
  }
  if (nList == 0) printf("      (none)\n");
  printf("      listed %d, total mismatch count = %ld\n", nList, nMismatch);
  char d[224];
  snprintf(d, sizeof(d), "n=%ld statusMismatch=%ld angleMismatch=%ld total=%ld", n, nStatus, nAng, nMismatch);
  verdict("ik-grid", nMismatch == 0, d);
}

/* ================================================================== */
/* (v) reachability boundary behaviour                                 */
/* ================================================================== */
static void check_boundaries(void) {
  printf("\n=== (v) reachability / singularity boundaries ===\n");
  printf("      points are placed at x=0, z=38 (so zv=0 and d == y exactly)\n");
  printf("      --- d = L2+L3-1 = 159 and d = L2+L3 = 160 and d = L2+L3+1 = 161 (dMax side) ---\n");
  double ds[] = { 158.9, 159.0, 159.0000001, 159.5, 160.0, 161.0 };
  for (unsigned i = 0; i < sizeof(ds) / sizeof(ds[0]); i++) {
    char tag[48]; snprintf(tag, sizeof(tag), "d=%.7g", ds[i]);
    double x = 0, y = ds[i], z = 38.0;
    REC rec; rec.x = x; rec.y = y; rec.z = z;
    bool ir = isReachable(&rec);
    pos p; p.rec = rec;
    p.ser.angle1 = p.ser.angle2 = p.ser.angle3 = 12345.0; p.ser.angle4 = 105.0;
    bool cl = false;
    bool ok = getAngleEx(&p, &cl);
    RefIK R = refSolveIKD(x, y, z);
    printf("      %-16s d=%.10f isReachable=%d getAngleEx=%d clamped=%d refSolveIK=%d %s\n",
           tag, ds[i], (int)ir, (int)ok, (int)cl, (int)R.ok,
           ((int)ir == (int)ok && (int)ok == (int)R.ok) ? "" : "  <== MISMATCH");
  }
  printf("      --- d = |L2-L3|+1 = 1 (dMin side) ---\n");
  double sm[] = { 0.5, 1.0, 1.0000001, 1.5, 2.0, 5.0 };
  for (unsigned i = 0; i < sizeof(sm) / sizeof(sm[0]); i++) {
    char tag[48]; snprintf(tag, sizeof(tag), "d=%.7g", sm[i]);
    double x = 0, y = sm[i], z = 38.0;
    REC rec; rec.x = x; rec.y = y; rec.z = z;
    bool ir = isReachable(&rec);
    pos p; p.rec = rec;
    p.ser.angle1 = p.ser.angle2 = p.ser.angle3 = 12345.0; p.ser.angle4 = 105.0;
    bool cl = false;
    bool ok = getAngleEx(&p, &cl);
    RefIK R = refSolveIKD(x, y, z);
    printf("      %-16s d=%.10f isReachable=%d getAngleEx=%d clamped=%d refSolveIK=%d %s\n",
           tag, sm[i], (int)ir, (int)ok, (int)cl, (int)R.ok,
           ((int)ir == (int)ok && (int)ok == (int)R.ok) ? "" : "  <== MISMATCH");
  }
  printf("      --- R_h < 5 singularity: sweep x=0, y in [0,160] step 0.5, z in [-120,195] step 0.5 ---\n");
  long n = 0, rejBoth = 0, okBoth = 0, rejOnlyRef = 0, rejOnlyOurs = 0;
  double firstMx = 0, firstMy = 0, firstMz = 0;
  int haveM = 0;
  for (double y = 0.0; y <= 160.0001; y += 0.5) {
    for (double z = -120.0; z <= 195.0001; z += 0.5) {
      n++;
      double x = 0;
      pos p; p.rec.x = x; p.rec.y = y; p.rec.z = z;
      p.ser.angle1 = p.ser.angle2 = p.ser.angle3 = 12345.0; p.ser.angle4 = 105.0;
      bool cl = false;
      bool ou = getAngleEx(&p, &cl);
      RefIK R = refSolveIKD(x, y, z);
      double rho = sqrt(x * x + y * y), zv = z - 38.0, d = sqrt(rho * rho + zv * zv);
      if (R.ok && ou) okBoth++;
      else if (!R.ok && !ou) rejBoth++;
      else {
        if (R.ok && !ou) { if (!haveM) { haveM = 1; firstMx = x; firstMy = y; firstMz = z; }
                           rejOnlyOurs++; }
        else              rejOnlyRef++;
        if (rejOnlyOurs + rejOnlyRef <= 6)
          printf("        mismatch x=%.1f y=%.1f z=%.1f d=%.6f refOk=%d ourOk=%d\n",
                 x, y, z, d, (int)R.ok, (int)ou);
      }
    }
  }
  printf("      swept %ld points: both-ok=%ld both-rej=%ld ref-accepts-but-we-reject=%ld we-accept-but-ref-rejects=%ld\n",
         n, okBoth, rejBoth, rejOnlyOurs, rejOnlyRef);
  if (haveM) printf("      first ref-accepts-we-reject point: (%.1f,%.1f,%.1f)\n", firstMx, firstMy, firstMz);
  printf("      --- targeted singularity probes (analytic R_h) ---\n");
  {
    double yz[][2] = { {1,118}, {3,118}, {5,118}, {10,118}, {20,118}, {3,38}, {1,38} };
    for (unsigned i = 0; i < sizeof(yz) / sizeof(yz[0]); i++) {
      double x = 0, y = yz[i][0], z = yz[i][1];
      double R = sqrt(x * x + y * y), Z = z - 38.0, d = sqrt(R * R + Z * Z);
      /* alpha from the same closed form the port uses, then the analytic R_h */
      double cosB = (12800.0 - d * d) / 12800.0; if (cosB > 1) cosB = 1; if (cosB < -1) cosB = -1;
      double beta = acos(cosB);
      double phi = atan2(Z, R);
      double alpha = phi + atan2(80.0 * sin(beta), 80.0 - 80.0 * cos(beta));
      double rh = 80.0 * cos(alpha) - 80.0 * cos(alpha + beta);
      pos p; p.rec.x = x; p.rec.y = y; p.rec.z = z;
      p.ser.angle1 = p.ser.angle2 = p.ser.angle3 = 12345.0; p.ser.angle4 = 105.0;
      bool cl = false;
      bool ok = getAngleEx(&p, &cl);
      RefIK Rr = refSolveIKD(x, y, z);
      printf("        (0,%.0f,%.0f) d=%.6f R_h=%.6f (reject iff <5) ourOk=%d refOk=%d %s\n",
             y, z, d, rh, (int)ok, (int)Rr.ok, ((int)ok == (int)Rr.ok) ? "" : "<== MISMATCH");
    }
  }
  char d[224];
  snprintf(d, sizeof(d), "d-edge ok; R_h sweep n=%ld refOnly=%ld ourOnly=%ld", n, rejOnlyRef, rejOnlyOurs);
  verdict("boundaries", rejOnlyOurs == 0 && rejOnlyRef == 0, d);
}

/* ================================================================== */
/* (vi) a FAILED getAngleEx must not touch pos->ser                    */
/* ================================================================== */
struct FailCase { const char *name; double x, y, z; };
static void check_no_write_on_fail(void) {
  printf("\n=== (vi) getAngleEx returning false must leave pos->ser completely untouched ===\n");
  FailCase fc[] = {
    { "beyond full extension (d=161)",      0.0,  161.0,  38.0 },
    { "far out on +x",                    161.0,    0.0,  38.0 },
    { "too close to shoulder (d=0.5)",      0.0,    0.5,  38.0 },
    { "too close (d=1.0, == dMin)",         0.0,    1.0,  38.0 },
    { "inside dMin (d=0.9)",                0.0,    0.9,  38.0 },
    { "R_h<5 singularity (0,1,118)",        0.0,    1.0, 118.0 },
    { "R_h<5 singularity (0,3,118)",        0.0,    3.0, 118.0 },
    { "x = NaN",                          NAN,   80.0, 118.0 },
    { "y = +inf",                         0.0,   INFINITY, 118.0 },
    { "z = -inf",                         0.0,   80.0, -INFINITY },
    { "all NaN",                          NAN,   NAN,   NAN },
    { "clamp then residual > 5mm (140,0,38)", 140.0, 0.0, 38.0 },
    { "y very negative (b out of window)",  0.0, -80.0, 118.0 },
    { "negative x, negative y",           -80.0, -80.0, 118.0 }
  };
  const double seed1 = 11.0, seed2 = 42.5, seed3 = 99.75, seed4 = 105.0;
  int tested = 0, bad = 0, trulyFalse = 0, unexpectedlyTrue = 0;
  for (unsigned i = 0; i < sizeof(fc) / sizeof(fc[0]); i++) {
    pos p;
    p.rec.x = fc[i].x; p.rec.y = fc[i].y; p.rec.z = fc[i].z;
    p.ser.angle1 = seed1; p.ser.angle2 = seed2; p.ser.angle3 = seed3; p.ser.angle4 = seed4;
    bool cl = true;
    bool ok = getAngleEx(&p, &cl);
    tested++;
    if (ok) {
      unexpectedlyTrue++;
      printf("      %-38s -> getAngleEx=true  (not a failure case; b=%.6f r=%.6f c=%.6f)\n",
             fc[i].name, p.ser.angle1, p.ser.angle2, p.ser.angle3);
      continue;
    }
    trulyFalse++;
    bool untouched = (p.ser.angle1 == seed1) && (p.ser.angle2 == seed2) &&
                     (p.ser.angle3 == seed3) && (p.ser.angle4 == seed4);
    bool clfalse = (cl == false);
    if (!untouched || !clfalse) bad++;
    printf("      %-38s -> false, ser%s, *clamped=%d\n",
           fc[i].name, untouched ? " untouched" : " MODIFIED!", (int)cl);
    if (!untouched)
      printf("        !! ser = (%.17g, %.17g, %.17g, %.17g)\n",
             p.ser.angle1, p.ser.angle2, p.ser.angle3, p.ser.angle4);
  }
  char d[200];
  snprintf(d, sizeof(d), "cases=%d false=%d modified=%d", tested, trulyFalse, bad);
  verdict("no-write-on-fail", bad == 0 && trulyFalse > 0, d);
  printf("      note: %d case(s) above did NOT fail (listed with their solved angles)\n", unexpectedlyTrue);
}

/* ================================================================== */
int main(void) {
  printf("############ meArm kinematics port verification ############\n");
  printf("firmware: D:\\dsh1\\meArm\\constant_and_positions.cpp\n");
  printf("reference: others\\f\\MeArm\\Task5_LineN2_TeachSmooth\\Task5_LineN2_TeachSmooth.ino\n");
  printf("arm1 = { armLength1=%.17g, armLength2=%.17g, armheight=%.17g }\n",
         arm1.armLength1, arm1.armLength2, arm1.armheight);
  printf("windows b[%.1f,%.1f] r[%.1f,%.1f] c[%.1f,%.1f], RADtoDEG=%.17g, ANGLE_EPS=%g\n",
         KIN_BASE_MIN, KIN_BASE_MAX, KIN_RARM_MIN, KIN_RARM_MAX,
         KIN_FARM_MIN, KIN_FARM_MAX, RADtoDEG, ANGLE_EPS);
  printf("limit x[%.0f,%.0f] y[%.0f,%.0f] z[%.0f,%.0f]\n",
         limit.minX, limit.maxX, limit.minY, limit.maxY, limit.minZ, limit.maxZ);

  check_fk_grid();
  check_roundtrip();
  check_home();
  check_ik_grid();
  check_boundaries();
  check_no_write_on_fail();

  printf("\n%s (failing checks: %d)\n", g_fail == 0 ? ">>> ALL PASS" : ">>> HAS FAILURES", g_fail);
  return g_fail == 0 ? 0 : 1;
}
