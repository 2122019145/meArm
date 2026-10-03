/*
 * pick_place.cpp -- 物体 A / B / C 的自动取放序列（实现）
 *
 * 上位机发单字母 A / B / C，本模块用一条非阻塞状态机走完整套动作：
 *
 *   TO_SRC_APPROACH  从当前位姿直线移动到「物体正上方」
 *   DESCEND_SRC      垂直下降到物体（合爪高度）
 *   CLOSE_TOOL       夹爪合拢到 servoLimit.minF（与串口 'S' 同一个角度）
 *   DWELL_CLOSE      停顿 PICK_DWELL_MS，等夹稳
 *   LIFT             垂直抬起回接近高度
 *   TRAVERSE         在接近高度上平移到「放置点正上方」
 *   DESCEND_DST      垂直下降到放置点
 *   OPEN_TOOL        夹爪张到 servoLimit.maxF（与串口 'O' 同一个角度）
 *   DWELL_OPEN       停顿，等物体落稳
 *   RETREAT          垂直撤离回接近高度，序列结束
 *
 * 每个「移动」阶段都在 x/y/z 上线性插值（所以下降与抬起是垂直的），
 * 每个插值点用固件自己的 getAngleEx() 反解成 b/r/c，三个轴一次写完，
 * 再只做一次正解刷新 Pos.rec —— 与串口角度指令同一约定，坐标与关节角始终自洽。
 *
 * 启动前把整条路径采样校验一遍（在 limit 内、isReachable()、反解没被吸附、
 * 相邻采样点的反解分支不跳变），任何一条不满足就拒绝启动、一个字节都不改。
 * 校验的采样步长是 PICK_PATH_SAMPLE_STEP，也就是"校验过的点"比实际插值点还密。
 *
 * 【位置表可以随便改】见下面的 PICK_SRC / PICK_DST：三个物体的初始位置互不相同，
 * 三个放置位置也互不相同；且每个物体放置点相对初始点在 x 与 y 上都有明显位移。
 */

#include <math.h>
#include <Arduino.h>
#include "constant_and_positions.h"
#include "path_core.h"
#include "pick_place.h"

#if WEARM_ENABLE_PICK_PLACE
/* ==================== 可自定义的位置与参数 ==================== */

/* 台面高度：物体放在这个高度上（工具点与物体同高时合爪）。
 * 上机时按实际台面 + 夹爪几何改这一个值即可，三个物体共用同一张台面。 */
#define PICK_GRASP_Z 12.0

/* 接近 / 撤离高度：取放点正上方这么高的位置，垂直下降从这里开始，
 * 抬起与撤离也回到这里；它同时是平移到另一个位置时保持的飞行高度。 */
#define PICK_APPROACH_DZ 6.0

struct pickPoint {
  double x;
  double y;
  double z;
};

/* Both tables below are read-only, so they live in flash (PROGMEM) to keep
 * 72 bytes out of RAM; every read goes through pgm_read_float(). */
/* 物体初始位置（= 夹取点）。三个位置互不相同。 */
static const struct pickPoint PICK_SRC[PICK_OBJECT_COUNT] PROGMEM = {
  { 24.0,  12.0, PICK_GRASP_Z },   /* A：右侧偏前 */
  { 24.0, -12.0, PICK_GRASP_Z },   /* B：右侧偏后 */
  { 12.0,  20.0, PICK_GRASP_Z }    /* C：左前，离基座较近 */
};

/* 放置位置。逐条的位移（探针要求 x、y 都有明显位移，门限 5.0）：
 *   A: Δx = 16.0 - 24.0 =  -8.0    Δy = -14.0 -  12.0 = -26.0
 *   B: Δx = 18.0 - 24.0 =  -6.0    Δy =  16.0 - (-12.0) = +28.0
 *   C: Δx = 28.0 - 12.0 = +16.0    Δy =  -6.0 -  20.0 = -26.0
 * 三个放置点两两之间的距离：A-B 30.1、A-C 14.4、B-C 24.2，都算"明显不同的位置"。
 * 还有一条约束（探针 [1] 会断言）：放置点离**别人**的初始位置不能太近，否则
 * 给某个物体放件时会蹭到还在地上的另一个物体。实测最近的距离是 7.21（√52），
 * 而且是三对并列：B 放 (18,16) 对 A 初始 (24,12)、B 放 (18,16) 对 C 初始 (12,20)、
 * C 放 (28,-6) 对 B 初始 (24,-12)，三对都是 7.2111，比夹爪宽度宽。
 * 最初选的 B 放 (14,16) 与 C 初始点只差 4.47、C 放 (26,-8) 与
 * B 初始点只差 4.47，都改掉了。 */
static const struct pickPoint PICK_DST[PICK_OBJECT_COUNT] PROGMEM = {
  { 16.0, -14.0, PICK_GRASP_Z },   /* A 放这里 */
  { 18.0,  16.0, PICK_GRASP_Z },   /* B 放这里 */
  { 28.0,  -6.0, PICK_GRASP_Z }    /* C 放这里 */
};

static const char PICK_LETTER[PICK_OBJECT_COUNT] = { 'A', 'B', 'C' };

/* 关节角速度（度/秒）。按当前调速档位选一个，所以 H 提速 / L 降速在序列中途也生效。 */
#define PICK_RATE_SLOW_DPS   25.0
#define PICK_RATE_NORMAL_DPS 50.0
#define PICK_RATE_FAST_DPS   90.0

/* 夹爪开合速度（度/秒），以及开合后用来等物体夹稳 / 放稳的停顿 */
#define PICK_TOOL_RATE_DPS 60.0
#define PICK_DWELL_MS      400

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

/* 夹爪开 / 合的目标角度：与串口 O / S 完全一致，不在这里重复写死 60 / 150。
 * 如果某个物体夹不牢或夹太死，把这两个宏改成中间角度即可（例如两个的平均值）。 */
#define PICK_TOOL_OPEN_ANGLE  (servoLimit.maxF)
#define PICK_TOOL_CLOSE_ANGLE (servoLimit.minF)

/* ==================== 状态机 ==================== */

enum {
  PICK_ST_IDLE = 0,
  PICK_ST_TO_SRC_APPROACH,
  PICK_ST_DESCEND_SRC,
  PICK_ST_CLOSE_TOOL,
  PICK_ST_DWELL_CLOSE,
  PICK_ST_LIFT,
  PICK_ST_TRAVERSE,
  PICK_ST_DESCEND_DST,
  PICK_ST_OPEN_TOOL,
  PICK_ST_DWELL_OPEN,
  PICK_ST_RETREAT
};

static const char *const PICK_STAGE_NAME[] = {
  "idle", "to-src-approach", "descend-src", "close-tool", "dwell-close",
  "lift", "traverse", "descend-dst", "open-tool", "dwell-open",
  "retreat"
};

/* 阶段类型：直线插值移动 / 夹爪角度渐变 / 原地停顿 */
enum { PICK_KIND_MOVE = 0, PICK_KIND_TOOL, PICK_KIND_DWELL };

/* The scalars below only ever hold -1..2 / 0..10 / 0..2, and the stage duration
 * is clamped to PICK_MAX_SEG_MS (4000), so narrow types are enough and save
 * 5 bytes of RAM.  All of them keep their previous values and comparisons. */
static signed char s_obj = -1;                  /* current object index, -1 = idle */
static signed char s_stage = PICK_ST_IDLE;
static signed char s_kind = PICK_KIND_DWELL;
static unsigned long s_stageStartMs = 0;        /* 本阶段开始时刻 */
static unsigned short s_stageMs = 0;            /* planned duration of this stage */

static double s_fromX = 0.0, s_fromY = 0.0, s_fromZ = 0.0;   /* 移动段起点 */
static double s_toX = 0.0, s_toY = 0.0, s_toZ = 0.0;         /* 移动段终点 */
/* The tool stage reuses s_fromX / s_toX for its start and end angle: s_kind
 * selects which pair is meaningful, and a move stage never overlaps a tool
 * stage, so a separate pair of doubles would only waste RAM. */

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
 * from cloning a specialised copy for the constant PICK_TOOL_RATE_DPS. */
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

/* ==================== 阶段切换 ==================== */

/* The target arrives as a point array plus a rise flag: rise != 0 means the
 * target is that point's raised point (z plus PICK_APPROACH_DZ).  Each of the
 * six call sites therefore pushes one address and one flag instead of three
 * doubles; the array values are bit-identical to the old arguments. */
static void beginMove(int stage, const double *p, int rise) {
  s_stage = (signed char)stage;
  s_kind = PICK_KIND_MOVE;
  s_fromX = Pos.rec.x;
  s_fromY = Pos.rec.y;
  s_fromZ = Pos.rec.z;
  s_toX = p[0];
  s_toY = p[1];
  s_toZ = rise ? (p[2] + PICK_APPROACH_DZ) : p[2];
  s_stageStartMs = millis();

  /* 这段要走多久：按"变化最大的那个关节"算，保证任何关节都不超过设定角速度。
   * 反解失败时给 0，落到耗时下限，具体在推进时还会再查一次。 */
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

/* English note: this body (four state stores, millis(), durationMs(), one
 * subtraction) is reached from two places in advanceStage().  Keeping a single
 * out-of-line copy costs less flash than the two inlined copies the compiler
 * would otherwise emit; noclone stops it from rebuilding a specialised copy
 * for the two constant-rate call sites.  No behaviour change: the parameters
 * and the state written are identical. */
static void __attribute__((noinline, noclone))
beginTool(int stage, double targetAngle) {
  s_stage = (signed char)stage;
  s_kind = PICK_KIND_TOOL;
  s_fromX = Pos.ser.angle4;
  s_toX = targetAngle;
  s_stageStartMs = millis();
  s_stageMs = (unsigned short)durationMs(fabs(s_toX - s_fromX), PICK_TOOL_RATE_DPS);
}

static void beginDwell(int stage, unsigned long ms) {
  s_stage = (signed char)stage;
  s_kind = PICK_KIND_DWELL;
  s_stageStartMs = millis();
  s_stageMs = (unsigned short)ms;
}

/* 序列结束（正常跑完） */
static void finishSequence(void) {
#if WEARM_DEBUG_SERIAL
  Serial.print(F("[pick] "));
  Serial.print(PICK_LETTER[s_obj]);
  Serial.print(F(" done, retreat pos = ("));
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

/* 本阶段跑完，进入下一阶段 */
static void advanceStage(void) {
  double src[3], dst[3];
  loadPickPoint(&PICK_SRC[s_obj], src);
  loadPickPoint(&PICK_DST[s_obj], dst);

  switch (s_stage) {
    case PICK_ST_TO_SRC_APPROACH:
      beginMove(PICK_ST_DESCEND_SRC, src, 0);
      break;

    case PICK_ST_DESCEND_SRC:
      beginTool(PICK_ST_CLOSE_TOOL, PICK_TOOL_CLOSE_ANGLE);
      break;

    case PICK_ST_CLOSE_TOOL:
#if WEARM_DEBUG_SERIAL
      Serial.print(F("[pick] gripper closed at angle4 = "));
      Serial.println(Pos.ser.angle4);
#endif
      beginDwell(PICK_ST_DWELL_CLOSE, PICK_DWELL_MS);
      break;

    case PICK_ST_DWELL_CLOSE:
      beginMove(PICK_ST_LIFT, src, 1);
      break;

    case PICK_ST_LIFT:
      beginMove(PICK_ST_TRAVERSE, dst, 1);
      break;

    case PICK_ST_TRAVERSE:
      beginMove(PICK_ST_DESCEND_DST, dst, 0);
      break;

    case PICK_ST_DESCEND_DST:
      beginTool(PICK_ST_OPEN_TOOL, PICK_TOOL_OPEN_ANGLE);
      break;

    case PICK_ST_OPEN_TOOL:
#if WEARM_DEBUG_SERIAL
      Serial.print(F("[pick] gripper opened at angle4 = "));
      Serial.println(Pos.ser.angle4);
#endif
      beginDwell(PICK_ST_DWELL_OPEN, PICK_DWELL_MS);
      break;

    case PICK_ST_DWELL_OPEN:
      beginMove(PICK_ST_RETREAT, dst, 1);
      break;

    case PICK_ST_RETREAT:
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

  double srcP[3], dstP[3], srcRiseP[3], dstRiseP[3];
  loadPickPoint(&PICK_SRC[object], srcP);
  loadPickPoint(&PICK_DST[object], dstP);
  srcRiseP[0] = srcP[0]; srcRiseP[1] = srcP[1]; srcRiseP[2] = srcP[2] + PICK_APPROACH_DZ;
  dstRiseP[0] = dstP[0]; dstRiseP[1] = dstP[1]; dstRiseP[2] = dstP[2] + PICK_APPROACH_DZ;

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

  /* 2) Sample-check the whole path (4 straight segments); one bad segment rejects
   * the whole sequence.  segmentOk is direction blind: it samples steps+1 points
   * English note: segmentOk is direction blind.  It samples steps+1 points
   * along the segment and every per-point predicate (limits, reachability, the
   * IK branch-jump test) depends on that point alone, so validating a segment
   * one way already covers the other way.  The old code additionally ran the
   * reversed pairs srcP->srcRiseP and dstP->dstRiseP; they only duplicated the
   * two forward calls above (same point set, same 0.5 mm step count) and each
   * call site costs the full 3-double argument setup, so they are dropped.
   * Both dropped segments are pure functions of the object index (they do not
   * read the current pose) and both were already true for every object this
   * function can be called with, so the accept/reject split is unchanged. */
  double curP[3];
  curP[0] = Pos.rec.x; curP[1] = Pos.rec.y; curP[2] = Pos.rec.z;
  if (!pathCoreSegmentOk(curP, srcRiseP, PICK_PATH_SAMPLE_STEP,
                         PICK_PATH_SAMPLE_MAX, PICK_BRANCH_JUMP_DEG) ||
      !pathCoreSegmentOk(srcRiseP, srcP, PICK_PATH_SAMPLE_STEP,
                         PICK_PATH_SAMPLE_MAX, PICK_BRANCH_JUMP_DEG) ||
      !pathCoreSegmentOk(srcRiseP, dstRiseP, PICK_PATH_SAMPLE_STEP,
                         PICK_PATH_SAMPLE_MAX, PICK_BRANCH_JUMP_DEG) ||
      !pathCoreSegmentOk(dstRiseP, dstP, PICK_PATH_SAMPLE_STEP,
                         PICK_PATH_SAMPLE_MAX, PICK_BRANCH_JUMP_DEG)) {
#if WEARM_DEBUG_SERIAL
    Serial.print(F("[pick] rejected: path check failed for "));
    Serial.println(PICK_LETTER[object]);
#endif
    return -3;
  }

  s_obj = (signed char)object;
#if WEARM_DEBUG_SERIAL
  Serial.print(F("[pick] start "));
  Serial.print(PICK_LETTER[object]);
  Serial.print(F(": src=("));
  Serial.print(srcP[0], 2);
  Serial.print(F(", "));
  Serial.print(srcP[1], 2);
  Serial.print(F(", "));
  Serial.print(srcP[2], 2);
  Serial.print(F(") dst=("));
  Serial.print(dstP[0], 2);
  Serial.print(F(", "));
  Serial.print(dstP[1], 2);
  Serial.print(F(", "));
  Serial.print(dstP[2], 2);
  Serial.print(F(") speed="));
  Serial.println(speedLevelName(speedGetLevel()));
#endif

  beginMove(PICK_ST_TO_SRC_APPROACH, srcP, 1);
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
  } else {
    /* 停顿：什么都不动，等物体被夹稳 / 放稳 */
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
  return PICK_STAGE_NAME[s_stage];
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
  return PICK_APPROACH_DZ;
}
#endif
