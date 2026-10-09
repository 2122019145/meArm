/*
 * probe_draw.cpp -- 绘图模块（draw_control.cpp）的自检探针
 *
 * 【为什么存在】"在纸面上画图"最怕的不是算不出来，而是悄悄错：
 *   - 抬笔高度不够：平移时笔尖在纸面上拖出一道多余的线
 *   - 落笔/绘制阶段 z 没盯住纸面：线条一段深一段浅、甚至离开纸面
 *   - 暂停后还在动，或继续后又从起点重画一遍
 *   - 取消时先平移再抬笔，把已经画好的部分拖花（更糟：取消后接着画）
 *   - 五点示教"记录的点"和"画出来的轨迹"不是一回事（过点判定必然失败）
 *   - 内置图形的归一化顶点到工作区坐标的换算搞错（画出来是个歪的字母）
 * 这些都能在 PC 上用 mock 时钟 + mock 摇杆跑完整流程验证，而且比上机试快得多。
 *
 * 【约定（与其它探针一致）】
 *   - 直接链接固件，绝不自己重写运动学/限位：反解正解都调固件函数，
 *     限位读固件全局 limit/servoLimit。
 *   - 必须先 posInit()：限位映射表与 Pos 都在它里面初始化。
 *   - 按键脚电平写在 g_mockDigital[2..5]（HIGH=松开、LOW=按下），
 *     摇杆写在 g_mockAnalog[0..3]（0=A0 … 3=A3）。
 *   - pump() 按主程序 loop() 的真实顺序推进：
 *     serial -> pickPlace -> button -> draw -> joystick，5ms 一步。
 *   - 采样只记状态、不做判断：判断集中在各小节的 check() 里，便于失败时定位。
 */

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "Arduino.h"
#include "constant_and_positions.h"
#include "move.h"
#include "joystick_control.h"
#include "serial_protocol.h"
#include "protocol_constants.h"
#include "pick_place.h"
#include "button_control.h"
#include "draw_control.h"

static int failures = 0;

static void check(const char *name, bool ok, const char *detail) {
  printf("  %-58s %s  %s\n", name, ok ? "PASS" : "FAIL", detail);
  if (!ok) failures++;
}

/* ==================== 仿真工具 ==================== */

/* 按键 1~4 接 D2~D5，与 button_control.cpp 的 BTN_PIN_* 一致 */
static const int KEY_PIN[4] = { 2, 3, 4, 5 };

static void centerSticks(void) {
  for (int i = 0; i < 4; i++) g_mockAnalog[i] = 512;
}

/* 按主程序 loop() 的顺序推进所有模块 */
static void pump(void) {
  serialProtocolLoop();
  pickPlaceLoop();
  buttonLoop();
  drawLoop();
  joystickLoop();
}

/* ==================== 采样 ==================== */

#define TRACE_MAX 9000

struct DrawSample {
  int    phase;
  double x, y, z;
};

static struct DrawSample g_tr[TRACE_MAX];
static int g_trN = 0;

static int g_seq[64];        /* 观察到的阶段顺序（去重） */
static int g_seqN = 0;

static const char *phaseName(int p) {
  static const char *const N[9] = {
    "空闲", "回待机", "抬笔移动", "落笔", "绘制中", "抬笔", "示教中", "准备绘制", "结束回待机"
  };
  if (p < 0 || p > 8) return "未知";
  return N[p];
}

/* 阶段顺序拼成一行，失败时直接看得到"实际是怎么走的" */
static const char *seqText(void) {
  static char buf[256];
  int n = 0;
  buf[0] = '\0';
  for (int i = 0; i < g_seqN; i++) {
    int w = snprintf(buf + n, sizeof(buf) - (size_t)n, "%s%s", (i > 0 ? ">" : ""), phaseName(g_seq[i]));
    if (w <= 0) break;
    n += w;
    if ((size_t)n >= sizeof(buf)) break;
  }
  return buf;
}

static void traceReset(void) { g_trN = 0; }
static void seqReset(void) { g_seqN = 0; }

/* 记一次采样（阶段变化也记进顺序表） */
static void note(void) {
  int p = drawGetPhase();
  if (g_seqN == 0 || g_seq[g_seqN - 1] != p) {
    if (g_seqN < 64) g_seq[g_seqN++] = p;
  }
  if (g_trN < TRACE_MAX) {
    g_tr[g_trN].phase = p;
    g_tr[g_trN].x = Pos.rec.x;
    g_tr[g_trN].y = Pos.rec.y;
    g_tr[g_trN].z = Pos.rec.z;
    g_trN++;
  }
}

static void step(void) {
  note();
  pump();
  g_mockMillis += 5UL;
}

static void advance(unsigned long ms) {
  unsigned long target = g_mockMillis + ms;
  while (g_mockMillis < target) step();
}

/* 推进到模式回到空闲（起始就是空闲也不算：要求至少走过一个非空闲阶段） */
static bool runToIdleTrace(unsigned long cap) {
  unsigned long target = g_mockMillis + cap;
  while (g_mockMillis < target) {
    if (drawGetPhase() == DRAW_PHASE_IDLE && g_seqN > 1) {
      note();                     /* 补记最后这一次"回到空闲"，否则阶段顺序里看不到它 */
      return true;
    }
    step();
  }
  return drawGetPhase() == DRAW_PHASE_IDLE;
}

static bool runUntilPhase(int phase, unsigned long cap) {
  unsigned long target = g_mockMillis + cap;
  while (g_mockMillis < target) {
    if (drawGetPhase() == phase) { note(); return true; }
    step();
  }
  return drawGetPhase() == phase;
}

/* 推进到"绘制中"并且笔尖 x 已经越过 xMin（内置直线/横线类轨迹 x 单调） */
static bool runUntilPathX(double xMin, unsigned long cap) {
  unsigned long target = g_mockMillis + cap;
  while (g_mockMillis < target) {
    if (drawGetPhase() == DRAW_PHASE_PATH && Pos.rec.x >= xMin) { note(); return true; }
    step();
  }
  return false;
}

/* 完整按一下某个按键：按下 -> 等过消抖 -> 松手 -> 等过消抖 */
static void clickKey(int key) {
  g_mockDigital[KEY_PIN[key]] = LOW;
  advance(60);
  g_mockDigital[KEY_PIN[key]] = HIGH;
  advance(60);
}

/* ==================== 统计 ==================== */

struct PhaseStats {
  int    n;
  double xmin, xmax, ymin, ymax, zmin, zmax;
  double maxXYStep;     /* 相邻采样点 |dx|+|dy| 的最大值（垂直运动时应为 0） */
  double maxStep;       /* 相邻采样点三维间距的最大值（连续性） */
  double maxTurnDeg;    /* 相邻两段方向的最大夹角（平滑度） */
};

static void statsFor(int phase, struct PhaseStats *st) {
  st->n = 0;
  st->xmin = st->ymin = st->zmin = 1e9;
  st->xmax = st->ymax = st->zmax = -1e9;
  st->maxXYStep = 0.0;
  st->maxStep = 0.0;
  st->maxTurnDeg = 0.0;

  int p1 = -1, p2 = -1;
  for (int i = 0; i < g_trN; i++) {
    if (g_tr[i].phase != phase) { p1 = -1; p2 = -1; continue; }
    st->n++;
    if (g_tr[i].x < st->xmin) st->xmin = g_tr[i].x;
    if (g_tr[i].x > st->xmax) st->xmax = g_tr[i].x;
    if (g_tr[i].y < st->ymin) st->ymin = g_tr[i].y;
    if (g_tr[i].y > st->ymax) st->ymax = g_tr[i].y;
    if (g_tr[i].z < st->zmin) st->zmin = g_tr[i].z;
    if (g_tr[i].z > st->zmax) st->zmax = g_tr[i].z;

    if (p1 >= 0) {
      double dx = g_tr[i].x - g_tr[p1].x;
      double dy = g_tr[i].y - g_tr[p1].y;
      double dz = g_tr[i].z - g_tr[p1].z;
      double dxy = fabs(dx) + fabs(dy);
      if (dxy > st->maxXYStep) st->maxXYStep = dxy;
      double d = sqrt(dx * dx + dy * dy + dz * dz);
      if (d > st->maxStep) st->maxStep = d;
      if (p2 >= 0 && d > 1e-12) {
        double ux = g_tr[p1].x - g_tr[p2].x;
        double uy = g_tr[p1].y - g_tr[p2].y;
        double uz = g_tr[p1].z - g_tr[p2].z;
        double lu = sqrt(ux * ux + uy * uy + uz * uz);
        double cu = (lu > 1e-12) ? ((ux * dx + uy * dy + uz * dz) / (lu * d)) : 1.0;
        if (cu > 1.0) cu = 1.0;
        if (cu < -1.0) cu = -1.0;
        double ang = acos(cu) * 180.0 / 3.14159265358979;
        if (ang > st->maxTurnDeg) st->maxTurnDeg = ang;
      }
    }
    p2 = p1;
    p1 = i;
  }
}

/* 点到线段的距离 */
static double ptSegDist(double px, double py, double pz, const double a[3], const double b[3]) {
  double vx = b[0] - a[0], vy = b[1] - a[1], vz = b[2] - a[2];
  double wx = px - a[0], wy = py - a[1], wz = pz - a[2];
  double vv = vx * vx + vy * vy + vz * vz;
  double t = (vv > 1e-12) ? ((wx * vx + wy * vy + wz * vz) / vv) : 0.0;
  if (t < 0.0) t = 0.0;
  if (t > 1.0) t = 1.0;
  double dx = wx - t * vx, dy = wy - t * vy, dz = wz - t * vz;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

/* 绘制阶段采样点里，离某个目标点最近的距离（"过点"） */
static double minDistToPoint(const double p[3]) {
  double best = 1e9;
  for (int i = 0; i < g_trN; i++) {
    if (g_tr[i].phase != DRAW_PHASE_PATH) continue;
    double dx = g_tr[i].x - p[0], dy = g_tr[i].y - p[1], dz = g_tr[i].z - p[2];
    double d = sqrt(dx * dx + dy * dy + dz * dz);
    if (d < best) best = d;
  }
  return best;
}

/* 绘制阶段采样点里，离折线（顶点串）最远的距离（直线度 / 曲线性） */
static double maxDistToPolyline(const double pts[][3], int n) {
  double worst = 0.0;
  for (int i = 0; i < g_trN; i++) {
    if (g_tr[i].phase != DRAW_PHASE_PATH) continue;
    double best = 1e9;
    for (int j = 0; j + 1 < n; j++) {
      double d = ptSegDist(g_tr[i].x, g_tr[i].y, g_tr[i].z, pts[j], pts[j + 1]);
      if (d < best) best = d;
    }
    if (best > worst) worst = best;
  }
  return worst;
}

/* 某阶段的第一个采样点 */
static bool firstSampleOf(int phase, double out[3]) {
  for (int i = 0; i < g_trN; i++) {
    if (g_tr[i].phase == phase) {
      out[0] = g_tr[i].x;
      out[1] = g_tr[i].y;
      out[2] = g_tr[i].z;
      return true;
    }
  }
  return false;
}

/* 阶段 a 是否出现在阶段 b 之前（两者都出现过） */
static bool seqBefore(int a, int b) {
  int ia = -1, ib = -1;
  for (int i = 0; i < g_seqN; i++) {
    if (ia < 0 && g_seq[i] == a) ia = i;
    if (ib < 0 && g_seq[i] == b) ib = i;
  }
  return (ia >= 0 && ib >= 0 && ia < ib);
}

/* ==================== 示教工具 ==================== */

/* 用摇杆把笔尖闭环点动到目标点（笛卡尔点动）。
 * 方向约定（draw_control.cpp 里 DRAW_JOG_INVERT_* 全 0）：
 *   A0 偏大 -> x 增大      A1 偏大 -> y 增大      A3 偏大 -> z 减小（前推=下压）
 *   A2 不动（末端开合，绘图用不到） */
static void jogTo(double tx, double ty, double tz) {
  for (int iter = 0; iter < 800; iter++) {
    double dx = tx - Pos.rec.x;
    double dy = ty - Pos.rec.y;
    double dz = tz - Pos.rec.z;
    if (fabs(dx) < 0.30 && fabs(dy) < 0.30 && fabs(dz) < 0.30) break;
    centerSticks();
    if (fabs(dx) >= 0.30) g_mockAnalog[0] = (dx > 0.0) ? 900 : 100;
    if (fabs(dy) >= 0.30) g_mockAnalog[1] = (dy > 0.0) ? 900 : 100;
    if (fabs(dz) >= 0.30) g_mockAnalog[3] = (dz > 0.0) ? 100 : 900;
    step();
  }
  centerSticks();
  advance(40);
}

/* 依次点动到 5 个目标点，每到一个就用按键1 记录；返回是否 5 个都记上 */
static bool teachFive(const double tgt[5][3], double out[5][3]) {
  for (int i = 0; i < 5; i++) {
    jogTo(tgt[i][0], tgt[i][1], tgt[i][2]);
    out[i][0] = Pos.rec.x;
    out[i][1] = Pos.rec.y;
    out[i][2] = Pos.rec.z;
    clickKey(DRAW_KEY_1);
    if (drawTeachCount() != i + 1) return false;
  }
  return true;
}

/* ==================== 用例 ==================== */

/* 内置图形的期望顶点（中心 + 归一化 u,v × 半宽，z = 纸面高度）
 * 题目要求从 字母N / 三角形 / 字母Z / 字母V 里选一种，本工程四种都保留（同组不得完全相同）。 */
static void shapeVerts(int task, double half, double z, double out[5][3], int *n) {
  static const double LINE[2][2]     = { { -1.0, 0.0 },  { 1.0, 0.0 } };
  static const double NN[4][2]       = { { -1.0, -1.0 }, { -1.0, 1.0 }, { 1.0, -1.0 }, { 1.0, 1.0 } };
  static const double TRIANGLE[4][2] = { { -1.0, -1.0 }, { 1.0, -1.0 }, { 0.0, 1.0 }, { -1.0, -1.0 } };
  static const double ZZ[4][2]       = { { -1.0, 1.0 },  { 1.0, 1.0 },  { -1.0, -1.0 }, { 1.0, -1.0 } };
  static const double VV[3][2]       = { { -1.0, 1.0 },  { 0.0, -1.0 }, { 1.0, 1.0 } };

  const double (*src)[2] = LINE;
  int cnt = 2;
  if (task == DRAW_TASK_N)        { src = NN;       cnt = 4; }
  if (task == DRAW_TASK_TRIANGLE) { src = TRIANGLE; cnt = 4; }
  if (task == DRAW_TASK_Z)        { src = ZZ;       cnt = 4; }
  if (task == DRAW_TASK_V)        { src = VV;       cnt = 3; }

  for (int i = 0; i < cnt; i++) {
    out[i][0] = drawGetCenterX() + src[i][0] * half;   /* 中心与半宽由标定接口给出 */
    out[i][1] = drawGetCenterY() + src[i][1] * half;
    out[i][2] = z;
  }
  *n = cnt;
}

int main(void) {
  char d[256];

  posInit();               /* 必须先做：限位映射表与 Pos 都在里面初始化 */
  joystickSetup();         /* 摇杆模块（示教点动要真的读 mock 摇杆） */
  buttonSetup();           /* 按键模块 */
  drawSetup();             /* 绘图模块 */
  serialProtocolBegin();
  centerSticks();

  /* ================= 1) 初始化与纸面标定 ================= */
  printf("\n[1] 初始化与纸面标定\n");
  check("drawSetup 后默认任务 = 直线", drawGetTask() == DRAW_TASK_LINE, drawTaskName(drawGetTask()));
  snprintf(d, sizeof d, "纸面 z=%.2f 半宽=%.2f 中心=(%.2f,%.2f)",
           drawGetPaperZ(), drawGetHalfSize(), drawGetCenterX(), drawGetCenterY());
  check("默认纸面 z=12 / 半宽 6 / 中心 (20,0)",
        fabs(drawGetPaperZ() - 12.0) < 1e-9 && fabs(drawGetHalfSize() - 6.0) < 1e-9 &&
        fabs(drawGetCenterX() - 20.0) < 1e-9 && fabs(drawGetCenterY()) < 1e-9, d);
  check("空闲时 drawControlBusy()/Locked() 都是 false",
        !drawControlBusy() && !drawControlLocked(), drawStateName());
  check("空闲时阶段名 = 空闲", drawGetPhase() == DRAW_PHASE_IDLE && strcmp(drawPhaseName(), "空闲") == 0,
        drawPhaseName());

  {
    bool names = true;
    for (int t = 0; t < DRAW_TASK_COUNT; t++) {
      const char *nm = drawTaskName(t);
      if (nm == NULL || nm[0] == '\0' || strcmp(nm, "未知") == 0) names = false;
    }
    snprintf(d, sizeof d, "%s/%s/%s/%s/%s/%s/%s", drawTaskName(0), drawTaskName(1),
             drawTaskName(2), drawTaskName(3), drawTaskName(4), drawTaskName(5), drawTaskName(6));
    check("7 个任务名都齐全（直线/字母N/三角形/字母Z/字母V/五点折线/五点曲线）",
          names && DRAW_TASK_COUNT == 7, d);
  }
  check("drawSelectTask 非法任务回 -1",
        drawSelectTask(-1) == -1 && drawSelectTask(DRAW_TASK_COUNT) == -1, "task<0 / task>=COUNT");

  {
    drawSelectTask(DRAW_TASK_LINE);
    int a = drawTaskCycle();
    int b = drawTaskCycle();
    int c = drawTaskCycle();
    int e = drawTaskCycle();
    int f = drawTaskCycle();
    int g = drawTaskCycle();
    int h = drawTaskCycle();
    snprintf(d, sizeof d, "直线->%s->%s->%s->%s->%s->%s->%s", drawTaskName(a), drawTaskName(b),
             drawTaskName(c), drawTaskName(e), drawTaskName(f), drawTaskName(g), drawTaskName(h));
    check("循环顺序 = 直线>字母N>三角形>字母Z>字母V>折线>曲线>直线",
          a == DRAW_TASK_N && b == DRAW_TASK_TRIANGLE && c == DRAW_TASK_Z &&
          e == DRAW_TASK_V && f == DRAW_TASK_POLYLINE && g == DRAW_TASK_CURVE &&
          h == DRAW_TASK_LINE, d);
  }

  check("标定纸面高度 p：合法通过", drawSetPaperZ(11.5) && fabs(drawGetPaperZ() - 11.5) < 1e-9, "11.5");
  check("标定纸面高度 p：超范围拒绝且不改动",
        !drawSetPaperZ(999.0) && fabs(drawGetPaperZ() - 11.5) < 1e-9, "p999 被拒");
  check("标定半宽 n：超范围拒绝", !drawSetHalfSize(1.0) && !drawSetHalfSize(13.0), "1.0 / 13.0 被拒");
  check("标定中心 o：超上限拒绝且不改动",
        !drawSetCenter(100.0, 0.0) && fabs(drawGetCenterX() - 20.0) < 1e-9, "o100,0 被拒");
  check("标定恢复默认（p12/n6/o20,0）",
        drawSetPaperZ(12.0) && drawSetHalfSize(6.0) && drawSetCenter(20.0, 0.0), "已恢复");
  check("空闲时暂停/继续/取消都回 BUSY",
        drawPause() == PROTO_RES_BUSY && drawResume() == PROTO_RES_BUSY &&
        drawCancel() == PROTO_RES_BUSY, "空闲没有可暂停/取消的任务");

  /* ================= 2) 直线绘制（全流程） ================= */
  printf("\n[2] 直线绘制：抬笔 -> 落笔 -> 画线 -> 抬笔 -> 回待机\n");
  {
    traceReset();
    seqReset();
    check("选中直线任务", drawSelectTask(DRAW_TASK_LINE) == DRAW_TASK_LINE, drawTaskName(drawGetTask()));
    check("drawStartTask 回 PROTO_RES_DRAW_STARTED", drawStartTask() == PROTO_RES_DRAW_STARTED, "内置图形直接开始");
    check("开始后立刻离开空闲（忙碌/独占）", drawControlBusy() && drawControlLocked(), drawPhaseName());

    bool done = runToIdleTrace(30000);
    snprintf(d, sizeof d, "阶段顺序 %s", seqText());
    check("阶段顺序 回待机>抬笔移动>落笔>绘制中>抬笔>结束回待机>空闲",
          seqBefore(DRAW_PHASE_HOME, DRAW_PHASE_TRAVEL) &&
          seqBefore(DRAW_PHASE_TRAVEL, DRAW_PHASE_PLUNGE) &&
          seqBefore(DRAW_PHASE_PLUNGE, DRAW_PHASE_PATH) &&
          seqBefore(DRAW_PHASE_PATH, DRAW_PHASE_LIFT) &&
          seqBefore(DRAW_PHASE_LIFT, DRAW_PHASE_RETURN) &&
          seqBefore(DRAW_PHASE_RETURN, DRAW_PHASE_IDLE), d);
    check("跑完回到空闲", done && drawGetPhase() == DRAW_PHASE_IDLE && !drawControlBusy(), drawStateName());

    struct PhaseStats mv, pl, pa, lf;
    statsFor(DRAW_PHASE_TRAVEL, &mv);
    statsFor(DRAW_PHASE_PLUNGE, &pl);
    statsFor(DRAW_PHASE_PATH, &pa);
    statsFor(DRAW_PHASE_LIFT, &lf);

    snprintf(d, sizeof d, "抬笔段 z 最低 %.2f（纸面 12）", mv.zmin);
    check("抬笔平移全程都在纸面之上", mv.n > 5 && mv.zmin > 12.5, d);

    snprintf(d, sizeof d, "落笔段 x/y 漂移 %.3f，z 到 %.2f", pl.maxXYStep, pl.zmin);
    check("落笔是垂直下降（x/y 不动，z 落到纸面）",
          pl.n > 5 && pl.maxXYStep < 0.05 && fabs(pl.zmin - 12.0) < 0.02, d);

    snprintf(d, sizeof d, "线段 z %.4f~%.4f，x %.2f~%.2f，y %.3f~%.3f，相邻步最大 %.3f",
             pa.zmin, pa.zmax, pa.xmin, pa.xmax, pa.ymin, pa.ymax, pa.maxStep);
    check("绘制中 z 恒定 = 纸面高度（不会深一段浅一段）",
          pa.n > 20 && fabs(pa.zmax - pa.zmin) < 0.01 && fabs(pa.zmin - 12.0) < 0.02, d);
    check("绘制中沿 y=0 走，x 从 14 到 26",
          fabs(pa.ymin) < 0.02 && fabs(pa.ymax) < 0.02 &&
          fabs(pa.xmin - 14.0) < 0.05 && fabs(pa.xmax - 26.0) < 0.05, d);
    check("直线真直（每步位移 ≤ 0.02 单位 —— 本分支的精细化插值）",
          pa.maxStep > 0.0 && pa.maxStep <= 0.025, d);

    {
      const double seg[2][3] = { { 14.0, 0.0, 12.0 }, { 26.0, 0.0, 12.0 } };
      double worst = maxDistToPolyline(seg, 2);
      snprintf(d, sizeof d, "离理想直线最远 %.4f 单位", worst);
      check("整条线都贴合理想直线（<= 0.02）", worst <= 0.02, d);
    }

    snprintf(d, sizeof d, "抬笔段 z 最高 %.2f，x/y 漂移 %.3f", lf.zmax, lf.maxXYStep);
    check("画完垂直抬笔（先离纸再走）", lf.n > 5 && lf.zmax > 12.0 + 3.5 && lf.maxXYStep < 0.05, d);

    snprintf(d, sizeof d, "用时 %lu ms（弧长 12 单位，笔尖 3 单位/秒）", drawLastRunMs());
    check("绘制用时合理（12 单位 / 3 单位每秒，加上抬笔落笔与回待机）",
          drawLastRunMs() > 1000UL && drawLastRunMs() < 20000UL, d);

    snprintf(d, sizeof d, "b=%.2f r=%.2f c=%.2f", Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);
    check("结束后回到待机位（b/r/c 都是 90 度附近）",
          fabs(Pos.ser.angle1 - 90.0) < 0.5 && fabs(Pos.ser.angle2 - 90.0) < 0.5 &&
          fabs(Pos.ser.angle3 - 90.0) < 0.5, d);
  }

  /* ================= 3) 内置图形几何（题目四种图形全测；直线在 [2] 段已覆盖） ================= */
  printf("\n[3] 内置图形几何：顶点都走到、轨迹只在这些顶点的连线上\n");
  {
    const int letterTasks[4] = { DRAW_TASK_N, DRAW_TASK_TRIANGLE, DRAW_TASK_Z, DRAW_TASK_V };
    for (int li = 0; li < 4; li++) {
      int task = letterTasks[li];
      double want[5][3];
      int n = 0;
      shapeVerts(task, 6.0, 12.0, want, &n);

      (void) drawSelectTask(task);
      traceReset();
      seqReset();
      if (drawStartTask() != PROTO_RES_DRAW_STARTED) {
        check("内置图形能启动", false, drawTaskName(task));
      } else {
        bool done = runToIdleTrace(40000);

        double worstVert = 0.0;
        for (int i = 0; i < n; i++) {
          double dd = minDistToPoint(want[i]);
          if (dd > worstVert) worstVert = dd;
        }
        double worstOff = maxDistToPolyline(want, n);
        snprintf(d, sizeof d, "%s：顶点最差 %.3f、离线最远 %.3f", drawTaskName(task), worstVert, worstOff);
        check("形状正确（顶点都经过 + 只走顶点连线）",
              done && worstVert <= 0.30 && worstOff <= 0.05, d);
      }

      /* 串口换到该模式时必须在回复里带上模式提示（题目要求的"输出当前图形模式的提示"） */
      snprintf(d, sizeof d, "task=%d tag=%s", task, drawTaskTag(task));
      check("每个内置图形都有非空 ASCII 模式标签", drawTaskTag(task) != NULL &&
            drawTaskTag(task)[0] != '\0' && strcmp(drawTaskTag(task), "?") != 0, d);
    }
  }

  /* ================= 4) 暂停与继续 ================= */
  printf("\n[4] 暂停 / 继续\n");
  {
    (void) drawSelectTask(DRAW_TASK_LINE);
    seqReset();
    traceReset();
    (void) drawStartTask();
    bool inPath = runUntilPathX(17.0, 30000);
    double px = Pos.rec.x, py = Pos.rec.y, pz = Pos.rec.z;
    const SER serAtPause = Pos.ser;

    check("先画到一半（进入绘制中）", inPath, drawPhaseName());
    check("drawPause 回 PROTO_RES_DRAW_PAUSED", drawPause() == PROTO_RES_DRAW_PAUSED, "按键1 等价");
    check("暂停后 drawIsPaused() = true", drawIsPaused(), drawStateName());
    check("暂停后状态名 = 已暂停", strcmp(drawStateName(), "已暂停") == 0, drawStateName());
    check("已暂停时 drawPause 幂等（仍回 PAUSED）", drawPause() == PROTO_RES_DRAW_PAUSED, "重复暂停无害");

    advance(800);
    snprintf(d, sizeof d, "暂停 800ms 后 x=%.3f（暂停时 %.3f）", Pos.rec.x, px);
    check("暂停期间笔尖一动不动（四个关节角都不变）",
          Pos.rec.x == px && Pos.rec.y == py && Pos.rec.z == pz &&
          Pos.ser.angle1 == serAtPause.angle1 && Pos.ser.angle2 == serAtPause.angle2 &&
          Pos.ser.angle3 == serAtPause.angle3 && Pos.ser.angle4 == serAtPause.angle4, d);
    check("暂停期间阶段仍是绘制中（任务状态保留）",
          drawGetPhase() == DRAW_PHASE_PATH && drawControlBusy(), drawPhaseName());

    check("drawResume 回 PROTO_RES_DRAW_RESUMED", drawResume() == PROTO_RES_DRAW_RESUMED, "按键2 等价");
    check("继续后 drawIsPaused() = false", !drawIsPaused(), drawStateName());
    check("没暂停时 drawResume 回 BUSY", drawResume() == PROTO_RES_BUSY, "不该重复继续");

    traceReset();                      /* 只看"继续之后"的采样，才能证明没有从头重画 */
    bool done = runToIdleTrace(30000);
    struct PhaseStats pa;
    statsFor(DRAW_PHASE_PATH, &pa);
    snprintf(d, sizeof d, "继续后 x 最小 %.3f（暂停点 %.3f），终点 %.3f", pa.xmin, px, pa.xmax);
    check("从暂停位置接续（x 不倒退、不从头重画）", done && pa.xmin >= px - 0.05, d);
    check("继续后仍然画到终点 x≈26", fabs(pa.xmax - 26.0) < 0.05, d);
  }

  /* ================= 5) 取消 ================= */
  printf("\n[5] 取消：先垂直抬笔，再回待机\n");
  {
    (void) drawSelectTask(DRAW_TASK_LINE);
    (void) drawStartTask();
    bool inPath = runUntilPathX(18.0, 30000);
    double cx = Pos.rec.x, cz = Pos.rec.z;

    traceReset();
    seqReset();
    check("先画到一半", inPath && drawGetPhase() == DRAW_PHASE_PATH, drawPhaseName());
    check("drawCancel 回 PROTO_RES_DRAW_CANCELED", drawCancel() == PROTO_RES_DRAW_CANCELED, "按键3 等价");
    check("取消后立刻进入抬笔阶段（不再沿轨迹画）",
          drawGetPhase() == DRAW_PHASE_LIFT, drawPhaseName());

    bool done = runToIdleTrace(30000);
    struct PhaseStats lf;
    statsFor(DRAW_PHASE_LIFT, &lf);
    snprintf(d, sizeof d, "抬笔段 x/y 漂移 %.3f，z %.2f -> %.2f（取消点 %.2f）",
             lf.maxXYStep, lf.zmin, lf.zmax, cz);
    check("抬笔是垂直的（x/y 不动，z 升到纸面之上）",
          lf.n > 3 && lf.maxXYStep < 0.05 && lf.zmax > cz + 3.5, d);
    check("取消后整段不再碰纸面（z 不低于纸面）", lf.zmin >= 12.0 - 0.05, d);
    snprintf(d, sizeof d, "取消点 x=%.2f，最终 b=%.2f r=%.2f c=%.2f", cx,
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);
    check("取消后回到待机位并空闲",
          done && drawGetPhase() == DRAW_PHASE_IDLE && !drawControlBusy() &&
          fabs(Pos.ser.angle1 - 90.0) < 0.5 && fabs(Pos.ser.angle2 - 90.0) < 0.5 &&
          fabs(Pos.ser.angle3 - 90.0) < 0.5, d);
    check("取消后能重新启动新任务", drawSelectTask(DRAW_TASK_LINE) == DRAW_TASK_LINE, "任务选择恢复可用");
  }

  /* ================= 6) 五点示教折线 ================= */
  printf("\n[6] 五点示教 + 折线轨迹（按键记录、自动开始、绘制中暂停/继续）\n");
  {
    static double tgt[5][3] = {
      { 15.0, -5.0, 12.0 }, { 25.0, -5.0, 12.0 }, { 15.0, 5.0, 12.0 },
      { 25.0,  5.0, 12.0 }, { 20.0,  0.0, 12.0 }
    };
    static double got[5][3];

    setSpeed(0.5, 60);            /* 示教点动用细步长，好对准目标点，固定间隔60ms */

    (void) drawSelectTask(DRAW_TASK_POLYLINE);
    check("选中五点折线", drawGetTask() == DRAW_TASK_POLYLINE, drawTaskName(drawGetTask()));
    traceReset();
    seqReset();
    check("启动示教任务回 PROTO_RES_DRAW_STARTED", drawStartTask() == PROTO_RES_DRAW_STARTED, "先回待机再示教");
    {
      bool inTeach = runUntilPhase(DRAW_PHASE_TEACH, 30000);
      check("回待机后进入示教阶段", inTeach, drawPhaseName());
    }
    check("示教开始时已记录 0 个点", drawTeachCount() == 0, "每个任务都从 0 开始");

    /* —— 示教点动的逐轴回归（修的是"示教状态下摇杆控制错乱"）——
     * 满偏一格只该让对应的那一个坐标动一点（≤0.26 工作区单位），而不是一格跳半个工作区；
     * 轻推要走比满偏更小的步；A2（夹爪）默认整路不响应，免得把夹着的笔带歪。 */
    {
      const double CAP  = 0.26;   /* draw_control.cpp 的 DRAW_JOG_STEP_MAX(0.25) + 余量 */
      const double ZERO = 0.02;   /* 判"这个坐标没动"的容差（此时步长 ~0.19） */
      double x0, y0, z0, dx, dy, dz, dFull;

      /* A0 满偏：x 增大，y/z 不动 */
      centerSticks(); advance(40);
      x0 = Pos.rec.x; y0 = Pos.rec.y; z0 = Pos.rec.z;
      g_mockAnalog[0] = 900; step(); centerSticks(); advance(40);
      dx = Pos.rec.x - x0; dy = Pos.rec.y - y0; dz = Pos.rec.z - z0;
      dFull = dx;
      snprintf(d, sizeof d, "dx=%+.3f dy=%+.3f dz=%+.3f", dx, dy, dz);
      check("示教点动 A0：只让 x 增大（方向对、无串扰）",
            dx > 0.01 && fabs(dy) < ZERO && fabs(dz) < ZERO, d);
      check("示教点动 A0：单格位移不超过 0.25 单位上限", fabs(dx) <= CAP, d);

      /* A1 满偏：y 增大，x/z 不动 */
      centerSticks(); advance(40);
      x0 = Pos.rec.x; y0 = Pos.rec.y; z0 = Pos.rec.z;
      g_mockAnalog[1] = 900; step(); centerSticks(); advance(40);
      dx = Pos.rec.x - x0; dy = Pos.rec.y - y0; dz = Pos.rec.z - z0;
      snprintf(d, sizeof d, "dx=%+.3f dy=%+.3f dz=%+.3f", dx, dy, dz);
      check("示教点动 A1：只让 y 增大（方向对、无串扰）",
            dy > 0.01 && fabs(dx) < ZERO && fabs(dz) < ZERO, d);
      check("示教点动 A1：单格位移不超过 0.25 单位上限", fabs(dy) <= CAP, d);

      /* A3 满偏：z 减小（前推 = 下压），x/y 不动 */
      centerSticks(); advance(40);
      x0 = Pos.rec.x; y0 = Pos.rec.y; z0 = Pos.rec.z;
      g_mockAnalog[3] = 900; step(); centerSticks(); advance(40);
      dx = Pos.rec.x - x0; dy = Pos.rec.y - y0; dz = Pos.rec.z - z0;
      snprintf(d, sizeof d, "dx=%+.3f dy=%+.3f dz=%+.3f", dx, dy, dz);
      check("示教点动 A3：只让 z 减小（前推=下压）",
            dz < -0.01 && fabs(dx) < ZERO && fabs(dy) < ZERO, d);
      check("示教点动 A3：单格位移不超过 0.25 单位上限", fabs(dz) <= CAP, d);

      /* 轻推：步长按偏转比例缩小（DRAW_JOG_STEP_MIN 0.02 下限），必须明显小于满偏。
       * 注意偏转量要真的越过死区：v1.6.4 起 JOY_DEADZONE = 40，旧探针用 550（偏 38）
       * 在新死区里等于"没推杆"；这里用偏 +100（有效偏转 60 计数）。 */
      centerSticks(); advance(40);
      x0 = Pos.rec.x;
      g_mockAnalog[0] = 512 + 100; step(); centerSticks(); advance(40);
      {
        const double dSmall = Pos.rec.x - x0;
        snprintf(d, sizeof d, "轻推 dx=%+.3f（满偏 %.3f）", dSmall, dFull);
        check("示教点动：轻推走小步（比满偏小很多），对点更细",
              dSmall > 0.005 && dSmall < dFull * 0.5, d);
      }

      /* A2（夹爪）：默认整路不响应 */
      {
        const SER before = Pos.ser;
        const double px = Pos.rec.x, py = Pos.rec.y, pz = Pos.rec.z;
        centerSticks(); advance(80);
        g_mockAnalog[2] = 900; step();
        g_mockAnalog[2] = 100; step();
        centerSticks(); advance(80);
        snprintf(d, sizeof d, "笔尖 (%.2f,%.2f,%.2f) angle4=%.3f", Pos.rec.x, Pos.rec.y, Pos.rec.z, Pos.ser.angle4);
        check("示教点动 A2（夹爪）：默认整路不响应，笔尖与 angle4 都不动",
              Pos.rec.x == px && Pos.rec.y == py && Pos.rec.z == pz &&
              Pos.ser.angle4 == before.angle4, d);
      }

      /* 节奏方向（v1.6.4）：同样时长下"推得越狠走得越多"。
       * 老版本 jogIntervalMs() 方向相反（轻推用最短间隔），这条会失败 ——
       * 它和摇杆模块的 speedIntervalMs() 必须是同一条约定。 */
      {
        double xr0;
        centerSticks(); advance(80);
        xr0 = Pos.rec.x;
        g_mockAnalog[0] = 512 + 100;      /* 轻推（有效偏转 60 计数） */
        advance(200);
        const double dLightRun = Pos.rec.x - xr0;

        centerSticks(); advance(80);
        xr0 = Pos.rec.x;
        g_mockAnalog[0] = 900;            /* 满偏 */
        advance(200);
        const double dHeavyRun = Pos.rec.x - xr0;

        snprintf(d, sizeof d, "200ms 内 轻推 dx=%+.3f  满偏 dx=%+.3f", dLightRun, dHeavyRun);
        check("示教点动：同样时长下大幅偏转位移 > 小幅偏转（推得越狠越快）",
              dHeavyRun > dLightRun && dLightRun > 0.001, d);
      }
    }

    {
      bool okAll = teachFive(tgt, got);
      snprintf(d, sizeof d, "记录 %d/5，第1点 (%.2f,%.2f,%.2f) 第5点 (%.2f,%.2f,%.2f)",
               drawTeachCount(), got[0][0], got[0][1], got[0][2], got[4][0], got[4][1], got[4][2]);
      check("按键1 依次记录 5 个示教点", okAll && drawTeachCount() == 5, d);
    }
    check("记满 5 点后进入准备绘制（自动开始）",
          drawGetPhase() == DRAW_PHASE_TEACH_WAIT, drawPhaseName());

    /* 从"准备绘制"重新开始采样：只观察自动开始这一段，阶段顺序才干净 */
    traceReset();
    seqReset();

    /* 子任务4 也要求折线绘制中支持暂停/继续/取消：这里用物理按键走一遍暂停 -> 继续 */
    {
      bool inPath = runUntilPhase(DRAW_PHASE_PATH, 60000);
      advance(1000);
      clickKey(DRAW_KEY_1);
      check("折线绘制中按键1 = 暂停",
            inPath && drawIsPaused() && drawGetPhase() == DRAW_PHASE_PATH, drawStateName());
      double px = Pos.rec.x;
      const SER freeze = Pos.ser;
      advance(600);
      snprintf(d, sizeof d, "暂停 600ms 后 x=%.3f（暂停时 %.3f）", Pos.rec.x, px);
      check("折线暂停期间笔尖停住（四个关节角都不变）",
            Pos.ser.angle1 == freeze.angle1 && Pos.ser.angle2 == freeze.angle2 &&
            Pos.ser.angle3 == freeze.angle3 && Pos.ser.angle4 == freeze.angle4, d);
      clickKey(DRAW_KEY_2);
      check("折线绘制中按键2 = 继续（就地接续）",
            !drawIsPaused() && drawGetPhase() == DRAW_PHASE_PATH, drawStateName());
    }

    bool done = runToIdleTrace(60000);
    snprintf(d, sizeof d, "阶段顺序 %s", seqText());
    check("自动开始：准备绘制>回待机>抬笔移动>落笔>绘制中",
          seqBefore(DRAW_PHASE_TEACH_WAIT, DRAW_PHASE_HOME) &&
          seqBefore(DRAW_PHASE_HOME, DRAW_PHASE_TRAVEL) &&
          seqBefore(DRAW_PHASE_TRAVEL, DRAW_PHASE_PLUNGE) &&
          seqBefore(DRAW_PHASE_PLUNGE, DRAW_PHASE_PATH), d);
    {
      double first[3];
      bool has = firstSampleOf(DRAW_PHASE_TRAVEL, first);
      snprintf(d, sizeof d, "抬笔起点 (%.2f,%.2f,%.2f)（待机位 20,0,20）",
               has ? first[0] : 0.0, has ? first[1] : 0.0, has ? first[2] : 0.0);
      check("先回待机位再走向第一个目标点",
            has && fabs(first[0] - 20.0) < 1.0 && fabs(first[1]) < 1.0 && fabs(first[2] - 20.0) < 1.0, d);
    }
    check("画完并回到待机", done && drawGetPhase() == DRAW_PHASE_IDLE, drawStateName());

    {
      double worst = 0.0;
      for (int i = 0; i < 5; i++) {
        double dd = minDistToPoint(got[i]);
        if (dd > worst) worst = dd;
      }
      snprintf(d, sizeof d, "5 个示教点的最近距离最大值 %.3f 单位", worst);
      check("轨迹逐一经过 5 个示教点（<= 0.30）", worst <= 0.30, d);
    }
    {
      double off = maxDistToPolyline(got, 5);
      snprintf(d, sizeof d, "采样点离线最远 %.3f 单位", off);
      check("折线段真直（采样点都在示教点连线上，<= 0.05）", off <= 0.05, d);
    }
    snprintf(d, sizeof d, "过点 %d/%d", drawLastPointsHit(), drawLastPointsTotal());
    check("模块自己判定的过点数 = 5/5",
          drawLastPointsHit() == 5 && drawLastPointsTotal() == 5, d);
    {
      struct PhaseStats pa;
      statsFor(DRAW_PHASE_PATH, &pa);
      snprintf(d, sizeof d, "绘制中 z %.3f~%.3f（示教点 z=%.3f），相邻步最大 %.3f",
               pa.zmin, pa.zmax, got[0][2], pa.maxStep);
      /* 高度要恒定，并且就是示教时记下的那个高度（点动收在 ±0.3 里，所以不比 12.0 死） */
      check("折线全程贴着纸面画（高度恒定 = 示教高度）",
            fabs(pa.zmax - pa.zmin) < 0.01 && fabs(pa.zmin - got[0][2]) < 0.05, d);
    }
  }

  /* ================= 7) 五点示教平滑曲线 ================= */
  printf("\n[7] 五点示教 + 平滑曲线轨迹\n");
  {
    static double tgt[5][3] = {
      { 15.0, -5.0, 12.0 }, { 22.0, -5.5, 12.0 }, { 25.0, 0.0, 12.0 },
      { 22.0,  5.5, 12.0 }, { 15.0,  5.0, 12.0 }
    };
    static double got[5][3];

    (void) drawSelectTask(DRAW_TASK_CURVE);
    check("选中五点曲线", drawGetTask() == DRAW_TASK_CURVE, drawTaskName(drawGetTask()));
    traceReset();
    seqReset();
    check("启动曲线示教", drawStartTask() == PROTO_RES_DRAW_STARTED, "先回待机再示教");
    {
      bool inTeach = runUntilPhase(DRAW_PHASE_TEACH, 30000);
      check("进入示教阶段", inTeach, drawPhaseName());
    }

    {
      bool okAll = teachFive(tgt, got);
      snprintf(d, sizeof d, "记录 %d/5", drawTeachCount());
      check("按键1 记录 5 个示教点", okAll && drawTeachCount() == 5, d);
    }

    bool done = runToIdleTrace(60000);
    check("曲线画完并回到待机", done && drawGetPhase() == DRAW_PHASE_IDLE, drawStateName());

    {
      double worst = 0.0;
      for (int i = 0; i < 5; i++) {
        double dd = minDistToPoint(got[i]);
        if (dd > worst) worst = dd;
      }
      snprintf(d, sizeof d, "5 个示教点的最近距离最大值 %.3f 单位", worst);
      check("曲线依次经过 5 个示教点（<= 0.30）", worst <= 0.30, d);
    }

    struct PhaseStats pa;
    statsFor(DRAW_PHASE_PATH, &pa);
    {
      double off = maxDistToPolyline(got, 5);
      snprintf(d, sizeof d, "采样点离线最远 %.3f 单位（折线的话应该 ~0）", off);
      check("确实是曲线不是折线（离折线明显偏离，>= 0.10）", off >= 0.10, d);
    }
    snprintf(d, sizeof d, "相邻采样点最大间距 %.3f，最大转角 %.2f 度", pa.maxStep, pa.maxTurnDeg);
    check("轨迹连续（相邻采样点不跳变）", pa.maxStep > 0.0 && pa.maxStep <= 0.10, d);
    check("转角平缓（每 5ms 采样点的方向变化 <= 30 度）", pa.maxTurnDeg <= 30.0, d);
    snprintf(d, sizeof d, "过点 %d/%d，曲线绘制用时 %lu ms",
             drawLastPointsHit(), drawLastPointsTotal(), drawLastRunMs());
    check("模块自己判定的过点数 = 5/5",
          drawLastPointsHit() == 5 && drawLastPointsTotal() == 5, d);

    setSpeed(1.0, 40);            /* 恢复默认调速 */
  }

  /* ================= 8) 串口入口与忙守卫 ================= */
  printf("\n[8] 串口入口 F/D/G/E/Q/U/W 与忙碌互斥\n");
  {
    check("drawIsCommandChar 认得全部绘图命令、不误吃别的",
          drawIsCommandChar('F') && drawIsCommandChar('D') && drawIsCommandChar('G') &&
          drawIsCommandChar('E') && drawIsCommandChar('Q') && drawIsCommandChar('U') &&
          drawIsCommandChar('W') && drawIsCommandChar('p') && drawIsCommandChar('n') &&
          drawIsCommandChar('o') && !drawIsCommandChar('A') && !drawIsCommandChar('x'), "F/D/G/E/Q/U/W/p/n/o");

    {
      int before = drawGetTask();
      int after = -1;
      Serial.clearOutput();
      int rc = protoHandleLine("F");
      after = drawGetTask();
      std::string out = Serial.getOutput();
      char wantTag[32];
      snprintf(wantTag, sizeof(wantTag), "OK F=%s\n", drawTaskTag(after));
      snprintf(d, sizeof d, "%s -> %s（回 %d，串口=%s）", drawTaskName(before), drawTaskName(after),
               rc, out.c_str());
      check("串口 F 换任务回 PROTO_RES_DRAW_TASK_SELECTED 且回复里带当前模式标签",
            rc == PROTO_RES_DRAW_TASK_SELECTED && after == (before + 1) % DRAW_TASK_COUNT &&
            out.find(wantTag) != std::string::npos, d);
    }

    check("串口 p11.5 标定纸面高度",
          protoHandleLine("p11.5") == PROTO_RES_DRAW_CALIBRATED && fabs(drawGetPaperZ() - 11.5) < 1e-9, "CALIBRATED");
    check("串口 p999 超范围被拒且不改动",
          protoHandleLine("p999") == PROTO_RES_DRAW_REJECTED && fabs(drawGetPaperZ() - 11.5) < 1e-9, "REJECTED");
    check("串口 pabc 非数字被拒", protoHandleLine("pabc") == PROTO_RES_DRAW_REJECTED, "REJECTED");
    check("串口 n7 标定半宽",
          protoHandleLine("n7") == PROTO_RES_DRAW_CALIBRATED && fabs(drawGetHalfSize() - 7.0) < 1e-9, "CALIBRATED");
    check("串口 o20,0 标定中心",
          protoHandleLine("o20,0") == PROTO_RES_DRAW_CALIBRATED &&
          fabs(drawGetCenterX() - 20.0) < 1e-9 && fabs(drawGetCenterY()) < 1e-9, "CALIBRATED");
    check("串口 o100,0 超上限被拒", protoHandleLine("o100,0") == PROTO_RES_DRAW_REJECTED, "REJECTED");
    check("标定恢复默认 p12 / n6",
          protoHandleLine("p12") == PROTO_RES_DRAW_CALIBRATED &&
          protoHandleLine("n6") == PROTO_RES_DRAW_CALIBRATED, "已恢复");

    check("选回直线任务", drawSelectTask(DRAW_TASK_LINE) == DRAW_TASK_LINE, drawTaskName(drawGetTask()));
    traceReset();
    seqReset();
    check("串口 D 启动绘制", protoHandleLine("D") == PROTO_RES_DRAW_STARTED, "STARTED");
    bool inPath = runUntilPathX(16.0, 30000);
    check("D 之后确实在绘制中", inPath && drawGetPhase() == DRAW_PHASE_PATH, drawPhaseName());
    check("绘制中发角度指令被忙守卫拒绝", protoHandleLine("x10") == PROTO_RES_BUSY, "x10 -> BUSY");
    check("绘制中发 Q 暂停（绘图命令豁免忙守卫）",
          protoHandleLine("Q") == PROTO_RES_DRAW_PAUSED && drawIsPaused(), "Q -> PAUSED");
    check("暂停中发 U 继续", protoHandleLine("U") == PROTO_RES_DRAW_RESUMED && !drawIsPaused(), "U -> RESUMED");
    check("绘制中发 W 取消", protoHandleLine("W") == PROTO_RES_DRAW_CANCELED, "W -> CANCELED");
    check("取消后回到空闲", runToIdleTrace(30000) && drawGetPhase() == DRAW_PHASE_IDLE, drawStateName());

    /* 示教任务的串口等价命令 */
    check("选中五点折线", drawSelectTask(DRAW_TASK_POLYLINE) == DRAW_TASK_POLYLINE, "示教任务");
    check("串口 D 进入示教", protoHandleLine("D") == PROTO_RES_DRAW_STARTED, "STARTED");
    check("回待机后进入示教阶段", runUntilPhase(DRAW_PHASE_TEACH, 30000), drawPhaseName());
    check("示教点从 0 开始", drawTeachCount() == 0, "0");
    check("串口 G 记录一个示教点",
          protoHandleLine("G") == PROTO_RES_DRAW_TEACH_POINT && drawTeachCount() == 1, "G -> TEACH_POINT");
    check("串口 E 撤销一个示教点",
          protoHandleLine("E") == PROTO_RES_DRAW_TEACH_UNDO && drawTeachCount() == 0, "E -> TEACH_UNDO");
    check("没有点可撤时 E 回 REJECTED", protoHandleLine("E") == PROTO_RES_DRAW_REJECTED, "REJECTED");
    check("串口 W 取消示教", protoHandleLine("W") == PROTO_RES_DRAW_CANCELED, "W -> CANCELED");
    check("取消示教后回空闲且点数清零",
          runToIdleTrace(30000) && drawTeachCount() == 0 && drawGetPhase() == DRAW_PHASE_IDLE, drawStateName());

    /* 与取放序列互斥 */
    {
      int pr = pickPlaceStart(0);
      snprintf(d, sizeof d, "pickPlaceStart(0) 回 %d", pr);
      check("取放序列可启动（或路径校验拒绝）", pr == 0 || pr == -3, d);
      if (pr == 0) {
        check("取放忙时 drawStartTask 回 BUSY", drawStartTask() == PROTO_RES_BUSY, "BUSY");
        long guard = 0;
        while (pickPlaceIsBusy() && guard < 400000) {
          pump();
          g_mockMillis += 20UL;
          guard++;
        }
        check("取放序列能在时限内跑完", !pickPlaceIsBusy(), "已结束");
      }
    }
  }

  /* ================= 汇总 ================= */
  if (failures == 0) {
    printf("\n>>> probe_draw ALL PASS (失败 0 项)\n");
    return 0;
  }
  printf("\n>>> probe_draw 失败 %d 项\n", failures);
  return 1;
}
