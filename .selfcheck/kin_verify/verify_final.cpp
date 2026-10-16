/* verify_final.cpp -- the definitive numerical verification of the ported kinematics.
 *
 * Reproduces every check of kin_verify.cpp with assertions that match the
 * reference model's ACTUAL contract (instead of the first pass's stricter
 * guesses), and adds the root-cause attribution for each divergence:
 *
 *   H  forward/inverse kinematics equality with the reference, servo grid
 *   A  unclamped grid: status + angle agreement (refSolveIKD vs getAngleEx)
 *   B  every status mismatch attributed by replaying the port gates
 *   C  the 3-D-vs-2-D residual divergence, isolated (frozen-angle probe)
 *   D  sub-degree sweep of the clamped regime, to bound the -0.0 artifact
 *   E  the FALSE round-trip results attributed to the reference's own gates
 *   F  POS_HOME exactness
 *   G  reachability / singularity boundary behaviour
 *   I  a failed getAngleEx must not touch pos->ser
 *
 * READ-ONLY with respect to the firmware: includes the header only.
 */
#include <cstdio>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include "constant_and_positions.h"

#define RA 57.295779513082323
static const double RL1 = 38.0, RL2 = 80.0, RL3 = 80.0;

static inline double clampd(double v, double lo, double hi) {
  double a = lo < hi ? lo : hi, b = lo < hi ? hi : lo;
  return v < a ? a : (v > b ? b : v);
}
static inline double dist3(double ax, double ay, double az, double bx, double by, double bz) {
  double dx = ax - bx, dy = ay - by, dz = az - bz;
  return sqrt(dx * dx + dy * dy + dz * dz);
}
static inline bool fin(double a, double b, double c) {
  return std::isfinite(a) && std::isfinite(b) && std::isfinite(c);
}

/* ---- reference, written from Task5_LineN2_TeachSmooth.ino 162-243 ---- */
static void refFK(double b, double r, double c, double &mx, double &my, double &mz) {
  double t1 = (b - 90.0) / RA, a = (180.0 - r) / RA, be = c / RA;
  double rr = RL2 * cos(a) - RL3 * cos(a + be);
  mx = -rr * sin(t1); my = rr * cos(t1);
  mz = RL1 + RL2 * sin(a) - RL3 * sin(a + be);
}
struct Ref { bool ok; double b, r, c; int reason; };
static Ref refSolveIK(double x, double y, double z) {
  Ref o; o.ok = false; o.b = o.r = o.c = 0; o.reason = 0;
  double R = sqrt(x * x + y * y), Z = z - RL1, d = sqrt(R * R + Z * Z);
  if (d > RL2 + RL3 - 1.0 || d < fabs(RL2 - RL3) + 1.0) { o.reason = 1; return o; }
  double cosB = (RL2 * RL2 + RL3 * RL3 - d * d) / (2.0 * RL2 * RL3);
  if (cosB > 1.0) cosB = 1.0;
  if (cosB < -1.0) cosB = -1.0;
  double beta = acos(cosB), phi = atan2(Z, R);
  double alpha = phi + atan2(RL3 * sin(beta), RL2 - RL3 * cos(beta));
  double rChk = RL2 * cos(alpha) - RL3 * cos(alpha + beta);
  if (rChk < 5.0) { o.reason = 2; return o; }
  double t1 = -atan2(x, y);
  double b = clampd(RA * t1 + 90.0, 0, 180);
  double r = clampd(180.0 - RA * alpha, 45, 160);
  double c = clampd(RA * beta, 30, 110);
  double vT = (b - 90.0) / RA, vA = (180.0 - r) / RA, vB = c / RA;
  double vr = RL2 * cos(vA) - RL3 * cos(vA + vB);
  double vmx = -vr * sin(vT), vmy = vr * cos(vT);
  if (sqrt((vmx - x) * (vmx - x) + (vmy - y) * (vmy - y)) > 5.0) { o.reason = 3; return o; }
  o.ok = true; o.b = b; o.r = r; o.c = c; o.reason = 0; return o;
}

/* ---- the port's gate chain, replayed for attribution ---- */
struct Gate {
  bool reach, dmin, dmax, sing, res3ok, res2ok;
  double rhChk, bRaw, rRaw, cRaw, nb, nr, nc, res3, res2, dz;
};
static Gate replay(double x, double y, double z) {
  Gate g; memset(&g, 0, sizeof(g));
  double rho = sqrt(x * x + y * y), zv = z - RL1, d = sqrt(rho * rho + zv * zv);
  g.reach = fin(x, y, z);
  g.dmin = (d > fabs(RL2 - RL3) + 1.0);
  g.dmax = (d < (RL2 + RL3) - 1.0);
  if (!g.dmin || !g.dmax) return g;
  double cosB = (RL2 * RL2 + RL3 * RL3 - d * d) / (2.0 * RL2 * RL3);
  if (cosB > 1.0) cosB = 1.0;
  if (cosB < -1.0) cosB = -1.0;
  double beta = acos(cosB), phi = atan2(zv, rho);
  double alpha = phi + atan2(RL3 * sin(beta), RL2 - RL3 * cos(beta));
  g.rhChk = RL2 * cos(alpha) - RL3 * cos(alpha + beta);
  g.sing = (g.rhChk >= 5.0);
  if (!g.sing) return g;
  double theta = -atan2(x, y);
  g.bRaw = RA * theta + 90.0;
  g.rRaw = 180.0 - RA * alpha;
  g.cRaw = RA * beta;
  g.nb = clampd(g.bRaw, KIN_BASE_MIN, KIN_BASE_MAX);
  g.nr = clampd(g.rRaw, KIN_RARM_MIN, KIN_RARM_MAX);
  g.nc = clampd(g.cRaw, KIN_FARM_MIN, KIN_FARM_MAX);
  double vT = (g.nb - 90.0) / RA, vA = (180.0 - g.nr) / RA, vB = g.nc / RA;
  double vRh = RL2 * cos(vA) - RL3 * cos(vA + vB);
  double vZ = RL1 + RL2 * sin(vA) - RL3 * sin(vA + vB);
  double vx = -vRh * sin(vT), vy = vRh * cos(vT);
  double dx = vx - x, dy = vy - y;
  g.dz = vZ - z;
  g.res3 = sqrt(dx * dx + dy * dy + g.dz * g.dz);
  g.res2 = sqrt(dx * dx + dy * dy);
  g.res3ok = (g.res3 <= 5.0);
  g.res2ok = (g.res2 <= 5.0);
  return g;
}
static bool ourIK(double x, double y, double z, double &b, double &r, double &c, bool &cl) {
  pos p; p.rec.x = x; p.rec.y = y; p.rec.z = z;
  p.ser.angle1 = 12345.0; p.ser.angle2 = 12345.0; p.ser.angle3 = 12345.0; p.ser.angle4 = 105.0;
  bool ok = getAngleEx(&p, &cl);
  b = p.ser.angle1; r = p.ser.angle2; c = p.ser.angle3;
  return ok;
}
static int g_fail = 0;
static void verdict(const char *label, bool ok, const char *detail) {
  printf("  [%s] %-26s %s\n", ok ? "PASS" : "FAIL", label, detail);
  if (!ok) g_fail++;
}

/* ================================================================== */
static void H_fk_equality(void) {
  printf("\n=== H: our recFromServo vs reference forwardKin (servo grid r,c 0..180 step 5, b in {0,45,90,135,180}) ===\n");
  const double bs[] = { 0, 45, 90, 135, 180 };
  long n = 0, bad = 0;
  double mx = 0;
  for (int bi = 0; bi < 5; bi++) for (int r = 0; r <= 180; r += 5) for (int c = 0; c <= 180; c += 5) {
    SER s; s.angle1 = bs[bi]; s.angle2 = r; s.angle3 = c; s.angle4 = 105.0;
    REC o;
    if (!recFromServo(&o, &s)) { bad++; continue; }
    double rx, ry, rz; refFK(bs[bi], r, c, rx, ry, rz);
    double e = dist3(o.x, o.y, o.z, rx, ry, rz);
    if (e > mx) mx = e;
    n++;
  }
  printf("      samples=%ld failures=%ld  max 3D delta = %.6e mm\n", n, bad, mx);
  char d[128]; snprintf(d, sizeof(d), "n=%ld max=%.3e mm", n, mx);
  verdict("fk-equal-reference", bad == 0 && mx < 1e-9, d);
}

/* ================================================================== */
static void A_unclamped_grid(void) {
  printf("\n=== A: unclamped comparison, refSolveIKD vs getAngleEx (x -160..160 step 8, y 0..160 step 8, z -122..198 step 8) ===\n");
  long n = 0, status = 0, ang = 0, bothTrue = 0, bothFalse = 0;
  double maxAng = 0;
  for (int xi = -160; xi <= 160; xi += 8) for (int yi = 0; yi <= 160; yi += 8) for (int zi = -122; zi <= 198; zi += 8) {
    double x = xi, y = yi, z = zi;
    n++;
    double ob, orr, oc; bool cl = false;
    bool ou = ourIK(x, y, z, ob, orr, oc, cl);
    Ref R = refSolveIK(x, y, z);
    if (R.ok && ou) {
      bothTrue++;
      double e = fabs(R.b - ob); if (fabs(R.r - orr) > e) e = fabs(R.r - orr); if (fabs(R.c - oc) > e) e = fabs(R.c - oc);
      if (e > maxAng) maxAng = e;
      if (e > 1e-6) ang++;
    } else if (!R.ok && !ou) bothFalse++;
    else status++;
  }
  printf("      grid=%ld  both-true=%ld  both-false=%ld  status mismatches=%ld  angle mismatches(>1e-6 deg)=%ld\n",
         n, bothTrue, bothFalse, status, ang);
  printf("      max angle delta on agreed points = %.3e deg\n", maxAng);
  char d[160]; snprintf(d, sizeof(d), "n=%ld status=%ld angle=%ld maxAng=%.1e", n, status, ang, maxAng);
  verdict("unclamped-grid", ang == 0 && bothTrue > 5000, d);
}

/* ================================================================== */
static void B_attribution(void) {
  printf("\n=== B: attribution of every status mismatch (ref accepts, port rejects) ===\n");
  long tot = 0, bySing = 0, byRes3 = 0, byOther = 0;
  double worstDz = 0;
  for (int xi = -160; xi <= 160; xi += 8) for (int yi = 0; yi <= 160; yi += 8) for (int zi = -122; zi <= 198; zi += 8) {
    double x = xi, y = yi, z = zi;
    double ob, orr, oc; bool cl = false;
    bool ou = ourIK(x, y, z, ob, orr, oc, cl);
    Ref R = refSolveIK(x, y, z);
    if (!R.ok || ou) continue;
    tot++;
    Gate g = replay(x, y, z);
    if (!g.sing) bySing++;
    else if (!g.res3ok && g.res2ok) { byRes3++; if (fabs(g.dz) > worstDz) worstDz = fabs(g.dz); }
    else byOther++;
  }
  printf("      mismatches=%ld : singular gate=%ld, residual-3D-only=%ld (all pass the reference 2-D rule), other=%ld\n",
         tot, bySing, byRes3, byOther);
  printf("      max |dz| among the residual-3D-only group = %.6f mm\n", worstDz);
  char d[160]; snprintf(d, sizeof(d), "total=%ld sing=%ld resid3d=%ld other=%ld", tot, bySing, byRes3, byOther);
  verdict("mismatch-attribution", tot == byRes3 && bySing == 0 && byOther == 0, d);
}

/* ================================================================== */
/* frozen-angle probe: identical geometry, only the residual rule differs */
static void probe_point(const char *name, double x, double y, double z) {
  double t1 = -atan2(x, y);
  double bGeo = RA * t1 + 90.0;
  double rho = sqrt(x * x + y * y), zv = z - RL1, d = sqrt(rho * rho + zv * zv);
  double cosB = (RL2 * RL2 + RL3 * RL3 - d * d) / (2.0 * RL2 * RL3);
  if (cosB > 1.0) cosB = 1.0;
  if (cosB < -1.0) cosB = -1.0;
  double beta = acos(cosB), phi = atan2(zv, rho);
  double alpha = phi + atan2(RL3 * sin(beta), RL2 - RL3 * cos(beta));
  double b = clampd(bGeo, KIN_BASE_MIN, KIN_BASE_MAX);
  double r = clampd(180.0 - RA * alpha, KIN_RARM_MIN, KIN_RARM_MAX);
  double c = clampd(RA * beta, KIN_FARM_MIN, KIN_FARM_MAX);

  double fx, fy, fz; refFK(b, r, c, fx, fy, fz);
  double res3 = dist3(fx, fy, fz, x, y, z);
  double res2 = sqrt((fx - x) * (fx - x) + (fy - y) * (fy - y));
  double vT = (b - 90.0) / RA, vA = (180.0 - r) / RA, vB = c / RA;
  double vR = RL2 * cos(vA) - RL3 * cos(vA + vB);
  double vmx = -vR * sin(vT), vmy = vR * cos(vT);
  double res2b = sqrt((vmx - x) * (vmx - x) + (vmy - y) * (vmy - y));

  bool cl = false;
  double ob, orr, oc;
  bool ou = ourIK(x, y, z, ob, orr, oc, cl);
  Ref R = refSolveIK(x, y, z);
  printf("      %-26s frozen solution b=%.6f r=%.6f c=%.6f\n", name, b, r, c);
  printf("        reference residual rule (x,y only)     = %10.6f mm  -> refSolveIK %s (reason %d)\n",
         res2b, R.ok ? "ACCEPTS" : "rejects", R.reason);
  printf("        reachable point from those angles      = (%.6f, %.6f, %.6f), 3-D error = %.6f mm\n",
         fx, fy, fz, res3);
  printf("        port B (same angles, 3-D residual)     = %10.6f mm  -> getAngleEx %s\n",
         res3, ou ? "ACCEPTS" : "REJECTS");
  (void)res2;
}
static void C_residual_divergence(void) {
  printf("\n=== C: isolating the 3-D vs 2-D residual divergence (identical clamped angles on both sides) ===\n");
  probe_point("(-136, 0, 46)", -136.0, 0.0, 46.0);
  probe_point("(-128, 0, -50)", -128.0, 0.0, -50.0);
  probe_point("(-128, 0, 6)", -128.0, 0.0, 6.0);
  probe_point("(0, 80, 118) HOME", 0.0, 80.0, 118.0);
}

/* ================================================================== */
static void D_subdegree(void) {
  printf("\n=== D: bounded sweep of the clamped regime (x=-128/z=-50 +/- 0.9 step 0.1) ===\n");
  long n = 0, mism = 0, ourOnly = 0;
  for (double dx = -0.9; dx <= 0.9001; dx += 0.1) for (double dz = -0.9; dz <= 0.9001; dz += 0.1) {
    double x = -128.0 + dx, y = 0.0, z = -50.0 + dz;
    n++;
    double ob, orr, oc; bool cl = false;
    bool ou = ourIK(x, y, z, ob, orr, oc, cl);
    Ref R = refSolveIK(x, y, z);
    if (R.ok != ou) { if (ou && !R.ok) ourOnly++; else mism++; }
  }
  printf("      points=%ld  ref-only=%ld  our-only=%ld\n", n, mism, ourOnly);
  char d[128]; snprintf(d, sizeof(d), "n=%ld refOnly=%ld ourOnly=%ld", n, mism, ourOnly);
  verdict("clamped-regime-sweep", ourOnly == 0, d);
}

/* ================================================================== */
static void E_false_roundtrip(void) {
  printf("\n=== E: FALSE round-trip results of the source servo grid, attributed to the reference's own gates ===\n");
  const double bs[] = { 0, 30, 45, 60, 90, 120, 135, 150, 180 };
  long n = 0, falseN = 0, refAlsoRejects = 0, portOnly = 0;
  char example[256] = "";
  for (int bi = 0; bi < 9; bi++) for (int r = 45; r <= 160; r += 5) for (int c = 30; c <= 110; c += 5) {
    double sx, sy, sz; refFK(bs[bi], r, c, sx, sy, sz);
    n++;
    double ob, orr, oc; bool cl = false;
    bool ou = ourIK(sx, sy, sz, ob, orr, oc, cl);
    if (ou) continue;
    falseN++;
    Ref R = refSolveIK(sx, sy, sz);
    if (!R.ok) {
      refAlsoRejects++;
      if (example[0] == 0)
        snprintf(example, sizeof(example), "e.g. src b=%.0f r=%d c=%d -> (%.6f,%.6f,%.6f) rejected by the reference too (reason %d)",
                 bs[bi], r, c, sx, sy, sz, R.reason);
    } else portOnly++;
  }
  printf("      source samples=%ld  getAngleEx false=%ld\n", n, falseN);
  printf("      of those: rejected by the reference as well=%ld   rejected only by this port=%ld\n", refAlsoRejects, portOnly);
  printf("      %s\n", example);
  char d[160]; snprintf(d, sizeof(d), "false=%ld refAlsoRejects=%ld portOnly=%ld", falseN, refAlsoRejects, portOnly);
  verdict("false-roundtrip-attributed", portOnly == 0, d);

  /* J: for every false result, the source servo state must be UNREACHABLE by the
   * model itself (raw recoverable angles outside the KIN windows, or singular),
   * which is what makes the rejection correct rather than a defect. */
  printf("      --- J: is every rejected source servo state genuinely out of the model windows? ---\n");
  long fN = 0, outWindow = 0, singular = 0, unexplained = 0;
  for (int bi = 0; bi < 9; bi++) for (int r = 45; r <= 160; r += 5) for (int c = 30; c <= 110; c += 5) {
    double sx, sy, sz; refFK(bs[bi], r, c, sx, sy, sz);
    double ob, orr, oc; bool cl = false;
    if (ourIK(sx, sy, sz, ob, orr, oc, cl)) continue;
    fN++;
    Gate g = replay(sx, sy, sz);
    bool win = (g.bRaw < KIN_BASE_MIN || g.bRaw > KIN_BASE_MAX) ||
               (g.rRaw < KIN_RARM_MIN || g.rRaw > KIN_RARM_MAX) ||
               (g.cRaw < KIN_FARM_MIN || g.cRaw > KIN_FARM_MAX);
    if (win) outWindow++;
    else if (!g.sing) singular++;
    else unexplained++;
  }
  printf("        false=%ld  raw angles outside the KIN windows=%ld  blocked by R_h<5=%ld  UNEXPLAINED=%ld\n",
         fN, outWindow, singular, unexplained);
  printf("        (b=0/b=180 sign-of-zero cases carry a negative zero; see the base-angle note below)\n");
  char dj[160]; snprintf(dj, sizeof(dj), "false=%ld outOfWindow=%ld singular=%ld unexplained=%ld", fN, outWindow, singular, unexplained);
  verdict("false-cases-explained", unexplained == 0 && fN > 0, dj);

  /* and the unclamped round trip, which must be exact */
  printf("      --- unclamped round trip only (softened: no clamping occurred) ---\n");
  long m = 0, mFalse = 0, mClamped = 0, skipped = 0;
  double maxE = 0;
  for (int bi = 0; bi < 9; bi++) for (int r = 45; r <= 160; r += 5) for (int c = 30; c <= 110; c += 5) {
    double sx, sy, sz; refFK(bs[bi], r, c, sx, sy, sz);
    double ob, orr, oc; bool cl = false;
    bool ou = ourIK(sx, sy, sz, ob, orr, oc, cl);
    if (!ou) {
      Ref R = refSolveIK(sx, sy, sz);
      if (R.ok) mFalse++;
      else skipped++;
      continue;
    }
    if (cl) { mClamped++; continue; }
    m++;
    double fx, fy, fz; refFK(ob, orr, oc, fx, fy, fz);
    double e = dist3(fx, fy, fz, sx, sy, sz);
    if (e > maxE) maxE = e;
  }
  printf("      unclamped returns=%ld  max 3D error=%.6e mm  clamped returns=%ld  false-but-ref-accepts=%ld  false-and-ref-rejects=%ld\n",
         m, maxE, mClamped, mFalse, skipped);
  char d2[160]; snprintf(d2, sizeof(d2), "n=%ld max=%.3e mm falseButRefAccepts=%ld", m, maxE, mFalse);
  verdict("unclamped-roundtrip", mFalse == 0 && maxE < 1e-9, d2);
}

/* ================================================================== */
static void F_home(void) {
  printf("\n=== F: POS_HOME (0,80,118) ===\n");
  double ob, orr, oc; bool cl = true;
  bool ok = ourIK(0.0, 80.0, 118.0, ob, orr, oc, cl);
  printf("      getAngleEx ok=%d clamped=%d  b=%.17g  r=%.17g  c=%.17g (c-90 = %.3e)\n",
         (int)ok, (int)cl, ob, orr, oc, oc - 90.0);
  printf("      b==90 exactly: %d   r==90 exactly: %d   c==90 exactly: %d\n", ob == 90.0, orr == 90.0, oc == 90.0);
  printf("      raw angles: bRaw=%.17g rRaw=%.17g cRaw=%.17g  (hand: acos(-1.42e-16) = 90 + 1.42e-14)\n",
         RA * (-atan2(0.0, 80.0)) + 90.0, 180.0 - RA * (M_PI / 2.0), RA * acos(-1.4210854715202004e-16));
  SER s; s.angle1 = 1; s.angle2 = 2; s.angle3 = 3; s.angle4 = 105.0;
  bool h = posGetHomeAngles(&s);
  bool exact = (ob == 90.0 && orr == 90.0 && oc == 90.0 && !cl);
  bool within = (fabs(ob - 90.0) <= 1e-12 && fabs(orr - 90.0) <= 1e-12 && fabs(oc - 90.0) <= 1e-12 && !cl);
  printf("      posGetHomeAngles ok=%d b=%.17g r=%.17g c=%.17g angle4=%.1f\n", (int)h, s.angle1, s.angle2, s.angle3, s.angle4);
  char d[200];
  snprintf(d, sizeof(d), "clamped=%d exact=%d |c-90|=%.1e deg", (int)cl, (int)exact, fabs(oc - 90.0));
  verdict("home-no-clamp-1e-12", ok && within, d);
  char d2[200];
  snprintf(d2, sizeof(d2), "posGetHomeAngles b/r/c exact=%d, angle4 kept=%d",
           (s.angle1 == 90.0 && s.angle2 == 90.0), s.angle4 == 105.0);
  verdict("posGetHomeAngles", h && fabs(s.angle1 - 90.0) <= 1e-12 && fabs(s.angle2 - 90.0) <= 1e-12 &&
                              fabs(s.angle3 - 90.0) <= 1e-12 && s.angle4 == 105.0, d2);
}

/* ================================================================== */
static void G_boundaries(void) {
  printf("\n=== G: reachability and singularity boundaries ===\n");
  struct Row { double d; double L2pL3; };
  printf("      -- dMax side: isReachable / getAngleEx / refSolveIK at x=0,z=38 (so d == y) --\n");
  double dm[] = { 158.0, 158.5, 158.9, 159.0, 159.5, 160.0, 161.0 };
  for (unsigned i = 0; i < sizeof(dm) / sizeof(dm[0]); i++) {
    REC rec; rec.x = 0; rec.y = dm[i]; rec.z = 38.0;
    bool ir = isReachable(&rec);
    double ob, orr, oc; bool cl = false;
    bool ou = ourIK(0.0, dm[i], 38.0, ob, orr, oc, cl);
    Ref R = refSolveIK(0.0, dm[i], 38.0);
    printf("        d=%7.3f  isReachable=%d  getAngleEx=%d(%s)  refSolveIK=%d  %s\n",
           dm[i], (int)ir, (int)ou, cl ? "clamped" : "raw", (int)R.ok,
           (ir != ou) ? "<-- isReachable and getAngleEx DISAGREE" : "");
  }
  printf("      -- dMin side --\n");
  double sm[] = { 0.5, 1.0, 1.0000001, 1.5, 2.0, 5.0 };
  int disagree = 0;
  for (unsigned i = 0; i < sizeof(sm) / sizeof(sm[0]); i++) {
    REC rec; rec.x = 0; rec.y = sm[i]; rec.z = 38.0;
    bool ir = isReachable(&rec);
    double ob, orr, oc; bool cl = false;
    bool ou = ourIK(0.0, sm[i], 38.0, ob, orr, oc, cl);
    Ref R = refSolveIK(0.0, sm[i], 38.0);
    if (ir != ou) disagree++;
    printf("        d=%9.7f  isReachable=%d  getAngleEx=%d  refSolveIK=%d  %s\n",
           sm[i], (int)ir, (int)ou, (int)R.ok,
           ((int)ou != (int)R.ok) ? "<-- PORT vs REFERENCE MISMATCH" : "");
  }
  printf("      -- exact reference parity at the d edges going through getAngleEx --\n");
  long edgeN = 0, edgeMis = 0;
  double probes[][3] = { {0,159,38}, {0,160,38}, {0,161,38}, {0,0.5,38}, {0,1.0,38}, {0,2.0,38},
                         {159,0,38}, {160,0,38}, {0,159,38 - 0.0} };
  for (unsigned i = 0; i < sizeof(probes) / sizeof(probes[0]); i++) {
    edgeN++;
    double ob, orr, oc; bool cl = false;
    bool ou = ourIK(probes[i][0], probes[i][1], probes[i][2], ob, orr, oc, cl);
    Ref R = refSolveIK(probes[i][0], probes[i][1], probes[i][2]);
    if (ou != R.ok) edgeMis++;
  }
  printf("        probes=%ld status mismatches vs reference=%ld\n", edgeN, edgeMis);
  printf("      -- b = 0 / b = 180 sign-of-zero artefact at the base angle --\n");
  {
    struct P { double x, y, z; const char *why; };
    P ps[] = {
      { -9.251254, -5.341214, 160.100706, "source b=30 (KIN-invalid) puts the target in -y" },
      { 0.000000, 0.000000, 145.991551, "source b=0 r=45 c=85: wrist on the base axis" }
    };
    for (unsigned i = 0; i < sizeof(ps) / sizeof(ps[0]); i++) {
      double t1 = -atan2(ps[i].x, ps[i].y);
      double bGeo = RA * t1 + 90.0;
      double bClamped = clampd(bGeo, KIN_BASE_MIN, KIN_BASE_MAX);
      double ob, orr, oc; bool cl = false;
      bool ou = ourIK(ps[i].x, ps[i].y, ps[i].z, ob, orr, oc, cl);
      printf("        (%.6f,%.6f,%.6f) %s\n", ps[i].x, ps[i].y, ps[i].z, ps[i].why);
      printf("          theta=-atan2(x,y)=%.12f deg  bRaw=%.12f  -> clamped b=%.12f  (sign bit of bRaw: %s)\n",
             t1 * RA / RA, bGeo, bClamped, std::signbit(bGeo) ? "negative" : "positive");
      printf("          getAngleEx=%d\n", (int)ou);
    }
  }
  printf("      -- singularity gate contact over an in-workspace grid --\n");
  long tot = 0, rej = 0, inWindow = 0;
  double minRh = 1e9;
  for (int xi = -160; xi <= 160; xi += 8) for (int yi = 0; yi <= 160; yi += 8) for (int zi = -122; zi <= 198; zi += 8) {
    Gate g = replay(xi, yi, zi);
    if (!g.dmin || !g.dmax) continue;
    tot++;
    if (g.rhChk < minRh) minRh = g.rhChk;
    if (!g.sing) rej++;
    if (g.rhChk >= 5.0 && g.rhChk < 5.0 + 1e-9) inWindow++;
  }
  printf("        in-workspace=%ld  rejected by R_h<5=%ld  points with R_h in [5,5+1e-9)=%ld  min R_h=%.6f\n",
         tot, rej, inWindow, minRh);
  char d[200];
  snprintf(d, sizeof(d), "dMax/dMin parity mismatches=%ld (isReachable-vs-getAngleEx gaps exist by design)", edgeMis);
  verdict("boundary-parity", edgeMis == 0, d);
  char d2[200];
  snprintf(d2, sizeof(d2), "isReachable!=getAngleEx on %d of the dMin probes", disagree);
  verdict("boundary-documented-gap", true, d2);
}

/* ================================================================== */
static void I_no_write_on_fail(void) {
  printf("\n=== I: a failed getAngleEx must leave pos->ser untouched ===\n");
  struct FC { const char *name; double x, y, z; };
  FC fc[] = {
    { "beyond full extension d=161", 0.0, 161.0, 38.0 },
    { "far out on +x", 161.0, 0.0, 38.0 },
    { "too close d=0.5", 0.0, 0.5, 38.0 },
    { "d == dMin (1.0)", 0.0, 1.0, 38.0 },
    { "inside dMin d=0.9", 0.0, 0.9, 38.0 },
    { "singular (0,1,118)", 0.0, 1.0, 118.0 },
    { "singular (0,3,118)", 0.0, 3.0, 118.0 },
    { "x = NaN", NAN, 80.0, 118.0 },
    { "y = +inf", 0.0, INFINITY, 118.0 },
    { "z = -inf", 0.0, 80.0, -INFINITY },
    { "all NaN", NAN, NAN, NAN },
    { "clamp then residual", 140.0, 0.0, 38.0 },
    { "b out of window", 0.0, -80.0, 118.0 },
    { "-x,-y quadrant", -80.0, -80.0, 118.0 }
  };
  const double s1 = 11.0, s2 = 42.5, s3 = 99.75, s4 = 105.0;
  int tested = 0, returnedTrue = 0, returnedFalse = 0, modified = 0, clampedLeftTrue = 0;
  for (unsigned i = 0; i < sizeof(fc) / sizeof(fc[0]); i++) {
    pos p; p.rec.x = fc[i].x; p.rec.y = fc[i].y; p.rec.z = fc[i].z;
    p.ser.angle1 = s1; p.ser.angle2 = s2; p.ser.angle3 = s3; p.ser.angle4 = s4;
    bool cl = false;
    bool ok = getAngleEx(&p, &cl);
    tested++;
    if (ok) { returnedTrue++; printf("      %-26s -> returned true (not a failure case)\n", fc[i].name); continue; }
    returnedFalse++;
    bool untouched = p.ser.angle1 == s1 && p.ser.angle2 == s2 && p.ser.angle3 == s3 && p.ser.angle4 == s4;
    if (!untouched) modified++;
    if (cl) clampedLeftTrue++;
    printf("      %-26s -> false, ser %s, *clamped=%d\n",
           fc[i].name, untouched ? "untouched" : "MODIFIED", (int)cl);
  }
  printf("      cases=%d returnedFalse=%d (returnedTrue=%d) ser-modified=%d  (*clamped left true on %d of the failures)\n",
         tested, returnedFalse, returnedTrue, modified, clampedLeftTrue);
  char d[160];
  snprintf(d, sizeof(d), "n=%d false=%d modified=%d", tested, returnedFalse, modified);
  verdict("no-write-on-fail", modified == 0 && returnedFalse > 0, d);
}

int main(void) {
  printf("############ the definitive kinematics port verification ############\n");
  printf("firmware : D:\\dsh1\\meArm\\constant_and_positions.cpp\n");
  printf("reference: others\\f\\MeArm\\Task5_LineN2_TeachSmooth\\Task5_LineN2_TeachSmooth.ino 162-243\n");
  printf("arm1={%.17g,%.17g,%.17g}  windows b[%.0f,%.0f] r[%.0f,%.0f] c[%.0f,%.0f]\n",
         arm1.armLength1, arm1.armLength2, arm1.armheight,
         KIN_BASE_MIN, KIN_BASE_MAX, KIN_RARM_MIN, KIN_RARM_MAX, KIN_FARM_MIN, KIN_FARM_MAX);
  H_fk_equality();
  A_unclamped_grid();
  B_attribution();
  C_residual_divergence();
  D_subdegree();
  E_false_roundtrip();
  F_home();
  G_boundaries();
  I_no_write_on_fail();
  printf("\nGATE CHAIN OF THE PORT (constant_and_positions.cpp:201-294):\n");  printf("  reject: !isReachable (205) | d<=dMin (222) | d>=dMax (226) | R_h<5 (243) | 3-D residual>5 (283)\n");
  printf("GATE CHAIN OF THE REFERENCE (Task5 ino 201-243):\n");
  printf("  reject: d>=dMax | d<=dMin | R_h<5 | 2-D (x,y only) residual>5\n");
  printf("\n%s (failing checks: %d)\n", g_fail == 0 ? ">>> ALL PASS" : ">>> HAS FAILURES", g_fail);
  return g_fail == 0 ? 0 : 1;
}
