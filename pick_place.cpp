/*
 * pick_place.cpp -- 物体 A / B / C 的自动取放序列（实现）
 *
 * 上位机发单字母 A / B / C，本模块用一条非阻塞状态机走完整套动作。
 * 【本条路径是按用户口径重写的】以物体高度为基准，1 工作区单位 = 4 mm
 * （上臂/下臂各 20 单位 = 80 mm），所以 5 cm = 12.5 单位、2 cm = 5.0 单位：
 *
 *   RISE_SRC      先上移到「物体高度 + 5cm」（垂直上升，x/y 不动）
 *   TO_BEHIND     沿水平方向移到「物体后方 2cm」（与物体同半径方向后退）
 *   DESCEND_SRC   垂直下降到与物体同高
 *   REACH_IN      沿水平半径方向「向前 2cm」，把物体送进两爪之间
 *   CLOSE_TOOL    夹爪快速合拢到名义合拢角 TOOL_ANGLE_CLOSE(60)
 *   SQUEEZE_TOOL  再以很慢的速度压到夹紧角 TOOL_ANGLE_TIGHT(45)
 *   DWELL_CLOSE   停顿 PICK_DWELL_MS，等夹稳（力矩建立 + 物体落定）
 *   LIFT_SRC      垂直抬回「物体高度 + 5cm」
 *   TRAVERSE      沿水平方向移到「目标点正上方」
 *   DESCEND_DST   垂直下降到目标高度（名义上正好是下降 5cm）
 *   OPEN_TOOL     夹爪张到 TOOL_ANGLE_OPEN = 150（与串口 'O' 同一个角度）
 *   DWELL_OPEN    停顿，等物体落稳
 *   RETREAT       垂直撤离回飞行高度
 *   GO_HOME       关节空间平滑回到「开机初始位姿」POS_HOME（回到初始化的点）
 *
 * 【夹紧】合爪分两步是有意的：先用正常速度合到"刚碰到物体"的名义角，再用
 * 12°/s 慢慢压紧。一步冲到底（尤其是用 60°/s）时，两爪的斜面会像楔子一样把
 * 物体挤出去；慢压则让物体有时间落进两爪之间。夹持力最终等于舵机堵转力矩，
 * 和压进去的速度无关，所以慢不会更松。
 * SQUEEZE_TOOL 结束之后到 OPEN_TOOL 之前的每一帧都会重申一次夹紧角
 * （见 pickHoldingStage()），所以搬运途中夹爪不会因为别处的写入而松开。
 *
 * 【轴向】夹爪是沿基座的水平半径方向伸出去的，所以"后方 / 向前"都取半径方向
 * （见 pickBehindPoint），而不是固定 ±x：物体 y 分量较大时两者的方向差很多。
 *
 * 【物体高度范围】-10cm ~ +10cm（对称于 z = 0，即 PICK_HEIGHT_MIN/MAX = ±25 单位）。
 * 物体越高，可行的水平半径越窄（末端到肩的距离 sqrt(rho² + z²) ≤ 40，而且
 * alpha = 90-r、beta = c-90 都必须落在 [-90,90]）：z = ±25（±10cm）时 rho 只能取
 * 19.4~31.2，z = 0 时 rho 可以取 0~40。所以"物体高度 + 5cm"这个理想飞行高度，
 * 在物体又高又远时会整个落出可达包络（比如 +10cm 的物体自身要求 rho ≥ 19.4，
 * 而它上方 5cm 只允许 rho ≤ 13.9，两者没有交集）。planTravelZ() 于是按
 * PICK_TRAVEL_STEP 逐档降低飞行高度，直到整条路径都成立，最低不低于
 * "最高物体 + 1cm"。也就是说物体摆得很高时抬升量会自动变小，但序列仍然能正常
 * 取放（探针 [2] 会断言实际飞行高度落在 [最高+1cm, 最高+5cm] 内并打印出来）。
 *
 * 启动前把整条路径采样校验一遍（在 limit 内、isReachable()、反解没被吸附、
 * 相邻采样点的反解分支不跳变），任何一条不满足就拒绝启动、一个字节都不改。
 * 校验的采样步长是 PICK_PATH_SAMPLE_STEP，也就是"校验过的点"比实际插值点还密。
 *
 * 【位置表可以随便改】见下面的 PICK_SRC / PICK_DST：三个物体的初始位置互不相同，
 * 三个放置位置也互不相同；且每个物体放置点相对初始点在 x 与 y 上都有明显位移。
 * 改高度时只要落在 ±25 单位（±10cm）以内就不会被 -4 拒绝。
 */

#include <math.h>

#include <Arduino.h>
#include "constant_and_positions.h"
#include "path_core.h"
#include "pick_place.h"

#if WEARM_ENABLE_PICK_PLACE
/* ==================== 可自定义的位置与参数 ==================== */

/* 尺度换算：1 工作区单位 = 4 mm（arm1 = {20, 20, 0}，两节各 80 mm）。
 * 用户口径的 5cm / 2cm / 10cm 在这里换成工作区单位。 */
#define PICK_LIFT_DZ      12.5   /* 5 cm：物体上方的接近/撤离高度（理想飞行高度） */
#define PICK_CLEAR_DZ      2.5   /* 1 cm：飞行高度的下限，再低就会蹭到物体 */
#define PICK_BACK_DZ       5.0   /* 2 cm：物体"后方"的水平退让距离 */
#define PICK_TRAVEL_STEP   1.0   /* 0.25 cm：飞行高度逐档下降的步长 */
#define PICK_HEIGHT_MIN  (-25.0) /* -10 cm：物体放置高度下限 */
#define PICK_HEIGHT_MAX  ( 25.0) /* +10 cm：物体放置高度上限 */

struct pickPoint {
  double x;
  double y;
  double z;
};

/* Both tables below are read-only, so they live in flash (PROGMEM) to keep
 * 72 bytes out of RAM; every read goes through pgm_read_float(). */
/* 物体初始位置（= 夹取点）。三个位置互不相同，高度分别是 -5cm / 0 / +5cm。
 *
 * 【可达性】两节各 20 单位，末端到肩的距离 ≤ 40 单位，所以在高度 z 上可用的
 * 水平半径 rho 是一条带子：
 *     x_max(z) = sqrt(1600 - z*z)              （两节一起往外伸的极限）
 *     x_min(z) = 20*sqrt(1 - (|z|/20 - 1)^2)   （上臂竖直、小臂水平折回来的极限）
 * 反解要求 alpha = 90-r、beta = c-90 都落在 [-90,90]，这正是这两条边界。
 * 举例：z=0 时 rho 可以是 0~40；z=±20（±5cm）时 rho 必须在 20~34.6；
 * z=±25（±10cm）时必须落在 19.4~31.2。改表里的 z 时先过一遍这个判据，
 * 否则会被 pickPlaceStart() 以 -4 拒绝（探针 [2] 会把它指出来）。
 *
 * 【飞行高度】理想是"物体高度 + 5cm"，但末端到肩的距离要不超 40：
 * 物体越高、半径越大，这个高度就越做不到。planTravelZ() 会逐档降低，
 * 最低不低于"最高物体 + 1cm"。表里 C 在 +5cm / rho=26，飞行高度只能到 29.5
 * （抬升 2.4cm），就是这条规则在生效。 */
static const struct pickPoint PICK_SRC[PICK_OBJECT_COUNT] PROGMEM = {
  { 24.0,  16.0, -20.0 },   /* A：右侧偏前，低处（-5cm），水平半径 28.8 */
  { 24.0, -12.0,   0.0 },   /* B：右侧偏后，肩高（0），水平半径 26.8 */
  { 26.0,   0.0,  20.0 }    /* C：正前方，高处（+5cm），水平半径 26.0 */
};

/* 放置位置。逐条的位移（探针要求 x、y 都有明显位移，门限 5.0）：
 *   A: Δx = 15.0 - 24.0 =  -9.0    Δy = -18.0 - 16.0 = -34.0
 *   B: Δx = 18.0 - 24.0 =  -6.0    Δy =  16.0 - (-12.0) = +28.0
 *   C: Δx = 20.0 - 26.0 =  -6.0    Δy = -16.0 -   0.0  = -16.0
 * 三个放置点两两之间的距离：A-B 34.1、A-C 5.4、B-C 32.1。
 * 还有一条约束（探针 [1] 会断言，门限 5.0）：放置点离**别人**的初始位置不能太近，
 * 否则给某个物体放件时会蹭到还在地上的另一个物体。实测最近的几对：
 *   A 放 (15,-18) 对 C 初始 (26,0)  = √445 ≈ 21.10
 *   B 放 (18,16)  对 A 初始 (24,16) = 6.00          ← 全局最近
 *   B 放 (18,16)  对 C 初始 (26,0)  = √320 ≈ 17.89
 *   C 放 (20,-16) 对 B 初始 (24,-12) = √32 ≈ 5.66
 * 都大于门限 5.0。 */
static const struct pickPoint PICK_DST[PICK_OBJECT_COUNT] PROGMEM = {
  { 15.0, -18.0, -20.0 },   /* A 放这里，水平半径 23.4 */
  { 18.0,  16.0,   0.0 },   /* B 放这里，水平半径 24.1 */
  { 20.0, -16.0,  20.0 }    /* C 放这里，水平半径 25.6 */
};

static const char PICK_LETTER[PICK_OBJECT_COUNT] = { 'A', 'B', 'C' };

/* 关节角速度（度/秒）。按当前调速档位选一个，所以 H 提速 / L 降速在序列中途也生效。 */
#define PICK_RATE_SLOW_DPS   25.0
#define PICK_RATE_NORMAL_DPS 50.0
#define PICK_RATE_FAST_DPS   90.0

/* 夹爪开合速度（度/秒）：张开、以及"合到名义合拢角"这一段走这个速度。
 * 最后压紧的那一段（SQUEEZE_TOOL）另外用下面这个慢得多的速度：
 * 它是"取物不容易松开"的另一半——慢压让物体落进两爪之间，而不是被斜面顶飞。
 * 两者都只影响走完这段要多久，不影响力矩（力矩是舵机堵转力矩，由夹紧角决定）。 */
#define PICK_TOOL_RATE_DPS         60.0
#define PICK_TOOL_SQUEEZE_RATE_DPS 12.0

/* 开合后用来等物体夹稳 / 放稳的停顿。夹紧那一步压得慢，这里也要留足时间：
 * 舵机把角度走到位 ≠ 力矩已经建立，而且物体常常在这几百毫秒里才真正落定
 * （爪面压出一点形变、塑料件回弹），此时抬臂才最不容易掉。 */
#define PICK_DWELL_MS      600

/* 单个阶段的耗时下限 / 上限（毫秒）：太短会抖，太长会显得呆 */
#define PICK_MIN_SEG_MS 180
#define PICK_MAX_SEG_MS 4000

/* 路径校验：采样步长（工作区单位）、单段最多采样点数、
 * 以及"相邻采样点关节角跳这么多就认为是反解换了分支"的门限 */
#define PICK_PATH_SAMPLE_STEP 0.5
#define PICK_PATH_SAMPLE_MAX  64
#define PICK_BRANCH_JUMP_DEG  25.0

/* 启动时允许"手里的关节角"与"当前坐标的反解角"差多少度。
 * 摇杆推过之后 Pos.ser 未必等于 IK(Pos.rec)，差太多说明不在同一个反解分支上，
 * 直接用会让第一段开头猛跳一下，所以宁可拒绝启动。 */
#define PICK_START_TOL_DEG 10.0

/* 夹爪在取放序列里的三个目标角（末端 f 关节，逻辑角；定义见 constant_and_positions.h）。
 * 合爪分两步，这是"取物不容易松开"的关键：
 *   CLOSE_TOOL    快速合到名义合拢角 TOOL_ANGLE_CLOSE(60)——把两爪送到物体附近，
 *                 但不在这里使劲；
 *   SQUEEZE_TOOL  用 PICK_TOOL_SQUEEZE_RATE_DPS 很慢地压到夹紧角
 *                 TOOL_ANGLE_TIGHT(45)。慢是必须的：一步冲到底时，两爪斜面会像
 *                 楔子一样把物体顶出去；慢慢压，物体才有时间落进两爪之间。
 * 夹紧力最终等于舵机堵转力矩，与速度无关，所以慢不会夹得更松。
 * 60~150 的"夹爪范围限制"已删除，正是它以前让"比 60 更紧"写不进去；
 * 合爪之后到开爪之前的每一帧都会重申夹紧角（见 pickHoldingStage()）。 */
#define PICK_TOOL_OPEN_ANGLE    TOOL_ANGLE_OPEN
#define PICK_TOOL_CLOSE_ANGLE   TOOL_ANGLE_CLOSE
#define PICK_TOOL_SQUEEZE_ANGLE TOOL_ANGLE_TIGHT

/* ==================== 状态机 ==================== */

enum {
  PICK_ST_IDLE = 0,
  PICK_ST_RISE_SRC,        /* 上移到 物体高度+5cm */
  PICK_ST_TO_BEHIND,       /* 水平移到 物体后方 2cm */
  PICK_ST_DESCEND_SRC,     /* 下降到 与物体同高 */
  PICK_ST_REACH_IN,        /* 向前 2cm 到物体上 */
  PICK_ST_CLOSE_TOOL,
  PICK_ST_SQUEEZE_TOOL,
  PICK_ST_DWELL_CLOSE,
  PICK_ST_LIFT_SRC,        /* 抬回 物体高度+5cm */
  PICK_ST_TRAVERSE,        /* 水平移到 目标点正上方 */
  PICK_ST_DESCEND_DST,     /* 下降 5cm 到目标高度 */
  PICK_ST_OPEN_TOOL,
  PICK_ST_DWELL_OPEN,
  PICK_ST_RETREAT,         /* 抬回飞行高度 */
  PICK_ST_GO_HOME          /* 回到开机初始位姿 */
};

static const char *const PICK_STAGE_NAME[] = {
  "idle", "rise-src", "to-behind", "descend-src", "reach-in",
  "close-tool", "squeeze-tool", "dwell-close", "lift-src", "traverse",
  "descend-dst", "open-tool", "dwell-open", "retreat", "go-home"
};

/* 阶段类型：直线插值移动 / 夹爪角度渐变 / 原地停顿 / 关节空间插值（回初始位姿） */
enum { PICK_KIND_MOVE = 0, PICK_KIND_TOOL, PICK_KIND_DWELL, PICK_KIND_HOME };

/* The scalars below only ever hold -1..2 / 0..14 / 0..3, and the stage duration
 * is clamped to PICK_MAX_SEG_MS (4000), so narrow types are enough and save
 * 5 bytes of RAM.  All of them keep their previous values and comparisons. */
static signed char s_obj = -1;                  /* current object index, -1 = idle */
static signed char s_stage = PICK_ST_IDLE;
static signed char s_kind = PICK_KIND_DWELL;
static unsigned long s_stageStartMs = 0;        /* 本阶段开始时刻 */
static unsigned short s_stageMs = 0;            /* planned duration of this stage */

static double s_fromX = 0.0, s_fromY = 0.0, s_fromZ = 0.0;   /* 移动段起点 */
static double s_toX = 0.0, s_toY = 0.0, s_toZ = 0.0;         /* 移动段终点 */
/* The tool stage reuses s_fromX / s_toX for its start and end angle, and the
 * home stage reuses all six for the three joint angles; s_kind selects which
 * meaning applies, and two stages never overlap, so extra doubles would only
 * waste RAM. */

/* 规划出来的飞行高度（水平移动时保持的 z）。只在一次序列期间有效，
 * 由 pickPlaceStart() 写入；见 planTravelZ()。 */
static double s_travelZ = 0.0;

/* ==================== 小工具 ==================== */

/* Fetch one coordinate of a table point.
 * English note: on AVR a 'double' *is* the 4-byte float the table stores, and
 * PROGMEM puts PICK_SRC / PICK_DST in flash, so the fetch is pgm_read_float().
 * The PC self-check harness defines PROGMEM as nothing, has no pgmspace
 * helpers, and uses 8-byte doubles, so there the tables are plain RAM arrays
 * and are read directly.  Both builds read exactly the same numbers. */
#ifdef __AVR__
#define PICK_COORD(tab, field) ((double)pgm_read_float((const float *)&(tab)->field))
#else
#define PICK_COORD(tab, field) ((double)((tab)->field))
#endif

/* Copy one flash-resident table point into a RAM 3-element array.
 * The bits are exactly what the old RAM table held. */
static void loadPickPoint(const struct pickPoint *tab, double *out) {
  out[0] = PICK_COORD(tab, x);
  out[1] = PICK_COORD(tab, y);
  out[2] = PICK_COORD(tab, z);
}

/* 物体"后方 2cm"的那个点：从物体沿水平半径方向朝基座退 PICK_BACK_DZ。
 * 夹爪沿基座的半径方向伸出去，所以"后方 / 向前"都该取半径方向；物体 y 分量
 * 大时若固定退 -x，退让方向会和夹爪的开合方向差出几十度。
 * z 沿用物体自身的高度（调用方需要别的 z 时自己覆盖 out[2]）。 */
static void pickBehindPoint(const double *p, double *out) {
  double rho = sqrt(p[0] * p[0] + p[1] * p[1]);
  if (rho < 1e-6) {            /* 物体正好在基座正上方：没有半径方向可言 */
    out[0] = p[0];
    out[1] = p[1];
    out[2] = p[2];
    return;
  }
  double k = PICK_BACK_DZ / rho;
  out[0] = p[0] - p[0] * k;
  out[1] = p[1] - p[1] * k;
  out[2] = p[2];
}

/* 反解 / 点校验 / 直线段校验三件套搬到了 path_core.h，与 draw_control 共用同一份
 * 实现（原来两个模块各存了一份逐字相同的代码）。采样步长与分支跳变门限这些
 * 常量仍由本文件提供（PICK_PATH_SAMPLE_STEP / PICK_PATH_SAMPLE_MAX /
 * PICK_BRANCH_JUMP_DEG），调用点直接用 pathCore* 系列函数。 */

/* 当前调速档位对应的关节角速度 */
static double pickRateDps(void) {
  int level = speedGetLevel();
  if (level == SPEED_SLOW) return PICK_RATE_SLOW_DPS;
  if (level == SPEED_FAST) return PICK_RATE_FAST_DPS;
  return PICK_RATE_NORMAL_DPS;
}

/* How many milliseconds it takes to travel `degrees`, clamped to min/max.
 * Called from both beginMove and beginTool; the body (one divide, two clamps
 * and a rounding) is far bigger than the call convention, so keep a single
 * out-of-line copy and let both callers share it.  noclone stops the compiler
 * from cloning a specialised copy per call site: the rate is always a
 * compile-time constant (pickRateDps() for moves, PICK_TOOL_RATE_DPS or
 * PICK_TOOL_SQUEEZE_RATE_DPS for the gripper), so without it the compiler
 * would emit one inlined copy for each distinct constant. */
static unsigned long __attribute__((noinline, noclone))
durationMs(double degrees, double rateDps) {
  double ms = 0.0;
  if (degrees > 0.0 && rateDps > 0.0) {
    ms = degrees / rateDps * 1000.0;
  }
  if (ms < (double)PICK_MIN_SEG_MS) ms = (double)PICK_MIN_SEG_MS;
  if (ms > (double)PICK_MAX_SEG_MS) ms = (double)PICK_MAX_SEG_MS;
  return (unsigned long)(ms + 0.5);
}

/* 三个轴一次写完，只做一次正解刷新（与串口角度指令同一约定）——实现见 path_core.h */

/* ==================== 飞行高度规划 ==================== */

/* 找一个能让整条路径都成立的"飞行高度"（水平平移时保持的 z）。
 *
 * 理想值 = 最高物体 + PICK_LIFT_DZ(5cm)。但两节各 20 单位，末端到肩的距离
 * sqrt(rho² + z²) ≤ 40，物体越高可达的水平半径越小：+10cm（z=+25）的物体，
 * 5cm 的飞行高度要求 rho ≤ 13.9 单位，绝大多数取放点都做不到。这时按
 * PICK_TRAVEL_STEP 逐档降低飞行高度，直到下面七个关键点全部可解：
 *     起点正上方 / 物体后方(飞行高度) / 物体正上方(飞行高度) /
 *     目标点正上方 / 物体后方(物体同高) / 物体同高 / 目标同高
 * 因为每一段只在一个 z 上（或一根竖直线上）走，而判据对 rho、|z| 单调，
 * 端点可解就等价于整段可解，所以先只查端点，确认后再由调用方做整段采样校验。
 *
 * 最低不低于"最高物体 + PICK_CLEAR_DZ(1cm)"——再低就会蹭到物体。
 * 找到返回 true 并写 *outZ；整个区间都不可达时返回 false（调用方拒绝启动）。 */
static bool planTravelZ(const double *src, const double *dst, const double *behind,
                        double fromX, double fromY, double *outZ) {
  const double top = (src[2] > dst[2]) ? src[2] : dst[2];
  const double zMin = top + PICK_CLEAR_DZ;
  const int maxSteps = (int)((PICK_LIFT_DZ - PICK_CLEAR_DZ) / PICK_TRAVEL_STEP);

  for (int i = 0; i <= maxSteps; i++) {
    const double z = top + PICK_LIFT_DZ - (double)i * PICK_TRAVEL_STEP;
    if (z < zMin - 1e-9) break;
    double b = 0.0, r = 0.0, c = 0.0;
    if (!pathCorePointOk(fromX, fromY, z, &b, &r, &c)) continue;
    if (!pathCorePointOk(behind[0], behind[1], z, &b, &r, &c)) continue;
    if (!pathCorePointOk(src[0], src[1], z, &b, &r, &c)) continue;
    if (!pathCorePointOk(dst[0], dst[1], z, &b, &r, &c)) continue;
    if (!pathCorePointOk(behind[0], behind[1], src[2], &b, &r, &c)) continue;
    if (!pathCorePointOk(src[0], src[1], src[2], &b, &r, &c)) continue;
    if (!pathCorePointOk(dst[0], dst[1], dst[2], &b, &r, &c)) continue;
    *outZ = z;
    return true;
  }
  return false;
}

/* ==================== 阶段切换 ==================== */

/* 目标点 p（三个 double）及其 z：beginMove 用 p 自己的 z（下降/平移段），
 * beginMoveAt 用调用方给的 z（"回到飞行高度"这种抬高段）。
 * 主体只留一份，两个薄壳靠 LTO 内联，避免每个调用点各存一份三 double 的装配代码。 */
static void beginMoveAt(int stage, const double *p, double z) {
  s_stage = (signed char)stage;
  s_kind = PICK_KIND_MOVE;
  s_fromX = Pos.rec.x;
  s_fromY = Pos.rec.y;
  s_fromZ = Pos.rec.z;
  s_toX = p[0];
  s_toY = p[1];
  s_toZ = z;
  s_stageStartMs = millis();

  /* 这段要走多久：按"变化最大的那个关节"算**名义**耗时（dMax / 当前档位的角速度），
   * 让各轴同步到达。注意 durationMs() 还有 PICK_MAX_SEG_MS(4000ms) 上限，所以
   * 大幅度运动时实际角速度会高于档位标称值：180° 按 25°/s 需要 7200ms，被夹到
   * 4000ms 就变成 45°/s。这是"宁可快一点，也不要一段走 7 秒"的有意取舍，
   * 不是"任何关节都不超过设定角速度"（旧注释就是这么写的，不成立）。
   * 反解失败时 dMax 保持 0，落到耗时下限 PICK_MIN_SEG_MS；推进时还会再查一次。 */
  double dMax = 0.0;
  double b = 0.0, r = 0.0, c = 0.0;
  if (pathCoreSolveJoint(s_toX, s_toY, s_toZ, &b, &r, &c)) {
    double d;
    d = fabs(b - Pos.ser.angle1); if (d > dMax) dMax = d;
    d = fabs(r - Pos.ser.angle2); if (d > dMax) dMax = d;
    d = fabs(c - Pos.ser.angle3); if (d > dMax) dMax = d;
  }
  s_stageMs = (unsigned short)durationMs(dMax, pickRateDps());
}

static void beginMove(int stage, const double *p) {
  beginMoveAt(stage, p, p[2]);
}

/* English note: this body (four state stores, millis(), durationMs(), one
 * subtraction) is reached from three places in advanceStage().  Keeping a single
 * out-of-line copy costs less flash than the three inlined copies the compiler
 * would otherwise emit; noclone stops it from rebuilding a specialised copy
 * for the call sites whose rate is a compile-time constant (all of them are,
 * and they no longer share one value: the close/open steps use
 * PICK_TOOL_RATE_DPS while the squeeze step uses the much slower
 * PICK_TOOL_SQUEEZE_RATE_DPS).  No behaviour change beyond that rate. */
static void __attribute__((noinline, noclone))
beginTool(int stage, double targetAngle, double rateDps) {
  s_stage = (signed char)stage;
  s_kind = PICK_KIND_TOOL;
  s_fromX = Pos.ser.angle4;
  s_toX = targetAngle;
  s_stageStartMs = millis();
  s_stageMs = (unsigned short)durationMs(fabs(s_toX - s_fromX), rateDps);
}

static void beginDwell(int stage, unsigned long ms) {
  s_stage = (signed char)stage;
  s_kind = PICK_KIND_DWELL;
  s_stageStartMs = millis();
  /* s_stageMs 是 unsigned short：先夹到 60000ms 再窄化，免得调用方传进超长停顿时
   * 静默回绕成"立刻结束"。目前唯一调用点用的是 PICK_DWELL_MS(600)。 */
  s_stageMs = (ms > 60000UL) ? (unsigned short)60000 : (unsigned short)ms;
}

/* 最后一段：关节空间平滑回到开机初始位姿 POS_HOME。
 * 走关节插值而不是再反解一个笛卡尔点，是因为初始位姿本身就是一个关节姿态，
 * 直接插值不经过反解、不会在收拢姿态附近碰上分支跳变；Pos.rec 由 recFromServo()
 * 每帧刷新，坐标与关节角一样保持自洽。angle4 不动（此时已经张开）。 */
static void beginHome(void) {
  s_stage = PICK_ST_GO_HOME;
  s_kind = PICK_KIND_HOME;
  s_fromX = Pos.ser.angle1;
  s_fromY = Pos.ser.angle2;
  s_fromZ = Pos.ser.angle3;

  SER home;
  if (posGetHomeAngles(&home)) {
    s_toX = home.angle1;
    s_toY = home.angle2;
    s_toZ = home.angle3;
  } else {
    s_toX = s_fromX;
    s_toY = s_fromY;
    s_toZ = s_fromZ;
  }

  s_stageStartMs = millis();
  double dMax = fabs(s_toX - s_fromX);
  double d = fabs(s_toY - s_fromY); if (d > dMax) dMax = d;
  d = fabs(s_toZ - s_fromZ);        if (d > dMax) dMax = d;
  s_stageMs = (unsigned short)durationMs(dMax, pickRateDps());
}

/* 序列结束（正常跑完） */
static void finishSequence(void) {
#if WEARM_DEBUG_SERIAL
  Serial.print(F("[pick] "));
  Serial.print(PICK_LETTER[s_obj]);
  Serial.print(F(" done, final pos = ("));
  Serial.print(Pos.rec.x, 2);
  Serial.print(F(", "));
  Serial.print(Pos.rec.y, 2);
  Serial.print(F(", "));
  Serial.print(Pos.rec.z, 2);
  Serial.println(F(")"));
#endif
  s_obj = -1;
  s_stage = PICK_ST_IDLE;
  s_kind = PICK_KIND_DWELL;
  s_stageMs = 0;
  s_stageStartMs = millis();
}

/* 异常中止（正常路径上不该发生；发生了也只是原地停住，不会乱动） */
/* The reason string is only ever printed by the debug build, so the non-debug
 * build takes no message argument at all: that keeps the string literals out of
 * flash.  The state reset is the whole observable behaviour either way. */
#if WEARM_DEBUG_SERIAL
static void abortSequence(const __FlashStringHelper *why) {
  Serial.print(F("[pick] ERROR: "));
  Serial.print(why);
  Serial.println(F(", sequence aborted (arm holds position)"));
  s_obj = -1;
  s_stage = PICK_ST_IDLE;
  s_kind = PICK_KIND_DWELL;
  s_stageMs = 0;
  s_stageStartMs = millis();
}
#define PICK_ABORT(msg) abortSequence(F(msg))
#else
static void abortSequence(void) {
  s_obj = -1;
  s_stage = PICK_ST_IDLE;
  s_kind = PICK_KIND_DWELL;
  s_stageMs = 0;
  s_stageStartMs = millis();
}
#define PICK_ABORT(msg) abortSequence()
#endif

/* 【每一帧都夹紧】手里的东西从"压紧"到"松开"之间必须一直夹着：
 * 判据 = 当前阶段是否处于"已经夹住、还没放开"的区间。
 *   DWELL_CLOSE   压紧结束后的稳定停顿（此时已经是夹紧角）
 *   LIFT_SRC/TRAVERSE/DESCEND_DST  抬起、平移、下降到目标（全程不能松）
 * 不含开始阶段的 DESCEND_SRC / REACH_IN / CLOSE_TOOL / SQUEEZE_TOOL（还在往合拢角、
 * 夹紧角插值，s_kind==TOOL 自己会处理），也不含 DWELL_OPEN/OPEN_TOOL/RETREAT/GO_HOME
 *（已经松开）。调用点用 s_kind != PICK_KIND_TOOL 把 TOOL 阶段排除掉，避免与插值打架。 */
static bool pickHoldingStage(int st) {
  return st == PICK_ST_DWELL_CLOSE || st == PICK_ST_LIFT_SRC ||
         st == PICK_ST_TRAVERSE    || st == PICK_ST_DESCEND_DST;
}

/* 本阶段跑完，进入下一阶段 */
static void advanceStage(void) {
  double src[3], dst[3], p[3];
  loadPickPoint(&PICK_SRC[s_obj], src);
  loadPickPoint(&PICK_DST[s_obj], dst);
  pickBehindPoint(src, p);            /* p = 物体后方 2cm 的水平点 */

  switch (s_stage) {
    case PICK_ST_RISE_SRC:
      p[2] = s_travelZ;               /* 水平移到物体后方（还是在飞行高度上） */
      beginMove(PICK_ST_TO_BEHIND, p);
      break;

    case PICK_ST_TO_BEHIND:
      p[2] = src[2];                  /* 垂直下降到与物体同高 */
      beginMove(PICK_ST_DESCEND_SRC, p);
      break;

    case PICK_ST_DESCEND_SRC:
      beginMove(PICK_ST_REACH_IN, src);   /* 向前 2cm，把物体送进两爪之间 */
      break;

    case PICK_ST_REACH_IN:
      beginTool(PICK_ST_CLOSE_TOOL, PICK_TOOL_CLOSE_ANGLE, PICK_TOOL_RATE_DPS);
      break;

    case PICK_ST_CLOSE_TOOL:
      /* 只合到"刚碰到物体"的名义角，此时还不使劲；
       * 真正压紧交给下一段用很慢的速度做，免得把物体顶出去。 */
      beginTool(PICK_ST_SQUEEZE_TOOL, PICK_TOOL_SQUEEZE_ANGLE,
                PICK_TOOL_SQUEEZE_RATE_DPS);
      break;

    case PICK_ST_SQUEEZE_TOOL:
#if WEARM_DEBUG_SERIAL
      Serial.print(F("[pick] gripper squeezed at angle4 = "));
      Serial.println(Pos.ser.angle4);
#endif
      beginDwell(PICK_ST_DWELL_CLOSE, PICK_DWELL_MS);
      break;

    case PICK_ST_DWELL_CLOSE:
      p[0] = src[0]; p[1] = src[1]; p[2] = s_travelZ;
      beginMove(PICK_ST_LIFT_SRC, p);     /* 抬回物体高度 + 5cm */
      break;

    case PICK_ST_LIFT_SRC:
      p[0] = dst[0]; p[1] = dst[1]; p[2] = s_travelZ;
      beginMove(PICK_ST_TRAVERSE, p);     /* 水平移到目标点正上方 */
      break;

    case PICK_ST_TRAVERSE:
      beginMove(PICK_ST_DESCEND_DST, dst);   /* 下降到目标高度（名义 5cm） */
      break;

    case PICK_ST_DESCEND_DST:
      beginTool(PICK_ST_OPEN_TOOL, PICK_TOOL_OPEN_ANGLE, PICK_TOOL_RATE_DPS);
      break;

    case PICK_ST_OPEN_TOOL:
#if WEARM_DEBUG_SERIAL
      Serial.print(F("[pick] gripper opened at angle4 = "));
      Serial.println(Pos.ser.angle4);
#endif
      beginDwell(PICK_ST_DWELL_OPEN, PICK_DWELL_MS);
      break;

    case PICK_ST_DWELL_OPEN:
      p[0] = dst[0]; p[1] = dst[1]; p[2] = s_travelZ;
      beginMove(PICK_ST_RETREAT, p);      /* 垂直撤离回飞行高度 */
      break;

    case PICK_ST_RETREAT:
      beginHome();                        /* 回到初始化的点 */
      break;

    case PICK_ST_GO_HOME:
      finishSequence();
      break;

    default:
      PICK_ABORT("unexpected stage");
      break;
  }
}

/* ==================== 对外接口 ==================== */

int pickPlaceStart(int object) {
  if (object < 0 || object >= PICK_OBJECT_COUNT) return -1;

  if (s_obj >= 0) {
#if WEARM_DEBUG_SERIAL
    Serial.print(F("[pick] busy: "));
    Serial.print(PICK_LETTER[s_obj]);
    Serial.println(F(" still running, command ignored"));
#endif
    return -2;
  }

  double src[3], dst[3], behind[3];
  loadPickPoint(&PICK_SRC[object], src);
  loadPickPoint(&PICK_DST[object], dst);
  pickBehindPoint(src, behind);

  /* 0) 物体高度必须落在 -10cm ~ +10cm 里 */
  if (src[2] < PICK_HEIGHT_MIN || src[2] > PICK_HEIGHT_MAX ||
      dst[2] < PICK_HEIGHT_MIN || dst[2] > PICK_HEIGHT_MAX) {
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[pick] rejected: object height outside -10cm..+10cm"));
#endif
    return -4;
  }

  /* 1) 当前位姿可用，而且手里的关节角与它的反解是同一个分支 */
  double b = 0.0, r = 0.0, c = 0.0;
  if (!pathCorePointOk(Pos.rec.x, Pos.rec.y, Pos.rec.z, &b, &r, &c)) return -3;
  if (fabs(b - Pos.ser.angle1) > PICK_START_TOL_DEG ||
      fabs(r - Pos.ser.angle2) > PICK_START_TOL_DEG ||
      fabs(c - Pos.ser.angle3) > PICK_START_TOL_DEG) {
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[pick] rejected: current pose is not on the ik branch of its own"
                     " coordinates, run posInit()/回到初始位姿后再试"));
#endif
    return -3;
  }

  /* 2) 规划飞行高度（物体很高时会自动降低，见 planTravelZ） */
  double travelZ = 0.0;
  if (!planTravelZ(src, dst, behind, Pos.rec.x, Pos.rec.y, &travelZ)) {
#if WEARM_DEBUG_SERIAL
    Serial.print(F("[pick] rejected: no reachable travel height for "));
    Serial.println(PICK_LETTER[object]);
#endif
    return -4;
  }

  /* 3) 整条路径采样校验（7 段直线）：从当前位姿一路到目标高度。
   *    拐点顺序就是执行顺序，逐段调 pathCoreSegmentOk()：
   *      起点 -> 起点正上方(飞行高度) -> 物体后方(飞行高度) -> 物体后方(物体同高)
   *      -> 物体同高 -> 物体正上方(飞行高度) -> 目标正上方(飞行高度) -> 目标同高
   *    返回 HOME 那一段走关节插值，不需要也不该做笛卡尔校验。 */
  double path[8][3];
  path[0][0] = Pos.rec.x; path[0][1] = Pos.rec.y; path[0][2] = Pos.rec.z;
  path[1][0] = Pos.rec.x; path[1][1] = Pos.rec.y; path[1][2] = travelZ;
  path[2][0] = behind[0]; path[2][1] = behind[1]; path[2][2] = travelZ;
  path[3][0] = behind[0]; path[3][1] = behind[1]; path[3][2] = src[2];
  path[4][0] = src[0];    path[4][1] = src[1];    path[4][2] = src[2];
  path[5][0] = src[0];    path[5][1] = src[1];    path[5][2] = travelZ;
  path[6][0] = dst[0];    path[6][1] = dst[1];    path[6][2] = travelZ;
  path[7][0] = dst[0];    path[7][1] = dst[1];    path[7][2] = dst[2];

  for (int i = 0; i + 1 < 8; i++) {
    if (!pathCoreSegmentOk(path[i], path[i + 1], PICK_PATH_SAMPLE_STEP,
                           PICK_PATH_SAMPLE_MAX, PICK_BRANCH_JUMP_DEG)) {
#if WEARM_DEBUG_SERIAL
      Serial.print(F("[pick] rejected: path check failed for "));
      Serial.print(PICK_LETTER[object]);
      Serial.print(F(" at segment "));
      Serial.println(i);
#endif
      return -3;
    }
  }

  s_obj = (signed char)object;
  s_travelZ = travelZ;
#if WEARM_DEBUG_SERIAL
  Serial.print(F("[pick] start "));
  Serial.print(PICK_LETTER[object]);
  Serial.print(F(": src=("));
  Serial.print(src[0], 2);
  Serial.print(F(", "));
  Serial.print(src[1], 2);
  Serial.print(F(", "));
  Serial.print(src[2], 2);
  Serial.print(F(") dst=("));
  Serial.print(dst[0], 2);
  Serial.print(F(", "));
  Serial.print(dst[1], 2);
  Serial.print(F(", "));
  Serial.print(dst[2], 2);
  Serial.print(F(") travelZ="));
  Serial.print(travelZ, 2);
  Serial.print(F(" speed="));
  Serial.println(speedLevelName(speedGetLevel()));
#endif

  /* 【开始前先张开夹爪】上一次序列若在中途被中止（或操作者用 'S' 手动合过爪），
   * angle4 会停在合拢角 60 甚至夹紧角 45；那样下降段会"闭着爪子"压到物体上，
   * 紧接着合爪反而是把它张开（45→60），物体被推开或夹空。
   * 这里先归位到张开角，保证整个接近过程爪口都是开的；真正合爪在 REACH_IN
   * 之后（向前 2cm 把物体送进两爪之间）才开始。 */
  (void) posSetAngle4(PICK_TOOL_OPEN_ANGLE);

  /* 第一段：垂直上移到飞行高度（x/y 不动），也就是"先上移到物体高度+5cm" */
  double up[3];
  up[0] = Pos.rec.x;
  up[1] = Pos.rec.y;
  up[2] = travelZ;
  beginMove(PICK_ST_RISE_SRC, up);
  return 0;
}

void pickPlaceLoop(void) {
  if (s_obj < 0 || s_stage == PICK_ST_IDLE) return;

  unsigned long elapsed = millis() - s_stageStartMs;
  bool stageDone = (elapsed >= s_stageMs);
  double t = 1.0;
  if (!stageDone && s_stageMs > 0) {
    t = (double)elapsed / (double)s_stageMs;
  }

  if (s_kind == PICK_KIND_MOVE) {
    double x = s_fromX + (s_toX - s_fromX) * t;
    double y = s_fromY + (s_toY - s_fromY) * t;
    double z = s_fromZ + (s_toZ - s_fromZ) * t;
    double b = 0.0, r = 0.0, c = 0.0;
    if (!pathCoreSolveJoint(x, y, z, &b, &r, &c)) {
      PICK_ABORT("ik failed mid-path");
      return;
    }
    pathCoreSetJoints(b, r, c);
  } else if (s_kind == PICK_KIND_TOOL) {
    (void) posSetAngle4(s_fromX + (s_toX - s_fromX) * t);
  } else if (s_kind == PICK_KIND_HOME) {
    Pos.ser.angle1 = s_fromX + (s_toX - s_fromX) * t;
    Pos.ser.angle2 = s_fromY + (s_toY - s_fromY) * t;
    Pos.ser.angle3 = s_fromZ + (s_toZ - s_fromZ) * t;
    (void) recFromServo(&Pos.rec, &Pos.ser);
  } else {
    /* 停顿：什么都不动，等物体被夹稳 / 放稳 */
  }

  /* 【每一帧都夹紧】合爪之后、松开之前的每一帧都重申一次夹紧角（见 pickHoldingStage）。
   * posSetAngle4() 在值没变时返回 false 且不写状态，所以这里只是保证目标角始终是
   * 夹紧角（TOOL_ANGLE_TIGHT），不会和 CLOSE_TOOL / SQUEEZE_TOOL / OPEN_TOOL 的
   * 插值打架。注意这里必须是**夹紧角**而不是名义合拢角：否则一进入 LIFT_SRC 就会
   * 悄悄把夹爪松回 60，正是"搬起来就掉"的老毛病。 */
  if (s_kind != PICK_KIND_TOOL && pickHoldingStage(s_stage)) {
    (void) posSetAngle4(PICK_TOOL_SQUEEZE_ANGLE);
  }

  if (stageDone) advanceStage();
}

bool pickPlaceIsBusy(void) {
  return s_obj >= 0;
}

int pickPlaceCurrentObject(void) {
  return s_obj;
}

const char *pickPlaceStageName(void) {
  /* s_stage 目前恒为 0..14，但这是给探针用的对外接口：以后往枚举里插新阶段却忘了
   * 同步 PICK_STAGE_NAME[] 时，宁可返回 "?" 也不要越界读。（负值转 unsigned 会变成
   * 极大的数，同样被这一条判据拦住。） */
  const unsigned idx = (unsigned)s_stage;
  if (idx >= (unsigned)(sizeof(PICK_STAGE_NAME) / sizeof(PICK_STAGE_NAME[0]))) {
    return "?";
  }
  return PICK_STAGE_NAME[idx];
}

bool pickPlaceGetSource(int object, double *x, double *y, double *z) {
  if (object < 0 || object >= PICK_OBJECT_COUNT) return false;
  if (x) *x = PICK_COORD(&PICK_SRC[object], x);
  if (y) *y = PICK_COORD(&PICK_SRC[object], y);
  if (z) *z = PICK_COORD(&PICK_SRC[object], z);
  return true;
}

bool pickPlaceGetTarget(int object, double *x, double *y, double *z) {
  if (object < 0 || object >= PICK_OBJECT_COUNT) return false;
  if (x) *x = PICK_COORD(&PICK_DST[object], x);
  if (y) *y = PICK_COORD(&PICK_DST[object], y);
  if (z) *z = PICK_COORD(&PICK_DST[object], z);
  return true;
}

double pickPlaceApproachDz(void) {
  return PICK_LIFT_DZ;
}

double pickPlaceBackDz(void) {
  return PICK_BACK_DZ;
}

bool pickPlacePlanZ(int object, double *z) {
  if (object < 0 || object >= PICK_OBJECT_COUNT) return false;
  double src[3], dst[3], behind[3];
  loadPickPoint(&PICK_SRC[object], src);
  loadPickPoint(&PICK_DST[object], dst);
  pickBehindPoint(src, behind);
  double travelZ = 0.0;
  if (!planTravelZ(src, dst, behind, Pos.rec.x, Pos.rec.y, &travelZ)) return false;
  if (z) *z = travelZ;
  return true;
}
#endif
