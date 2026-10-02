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
#include "pick_place.h"

#define WEARM_DEBUG_SERIAL 1   /* 置 1: 打开取放序列的串口日志 */

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

/* 物体初始位置（= 夹取点）。三个位置互不相同。 */
static const struct pickPoint PICK_SRC[PICK_OBJECT_COUNT] = {
  { 24.0,  12.0, PICK_GRASP_Z },   /* A：右侧偏前 */
  { 24.0, -12.0, PICK_GRASP_Z },   /* B：右侧偏后 */
  { 12.0,  20.0, PICK_GRASP_Z }    /* C：左前，离基座较近 */
};

/* 放置位置。逐条的位移（探针要求 x、y 都有明显位移，门限 5.0）：
 *   A: Δx = 16.0 - 24.0 =  -8.0    Δy = -14.0 -  12.0 = -26.0
 *   B: Δx = 14.0 - 24.0 = -10.0    Δy =  16.0 - (-12.0) = +28.0
 *   C: Δx = 26.0 - 12.0 = +14.0    Δy =  -8.0 -  20.0 = -28.0
 * 三个放置点两两之间的距离：A-B 30.1、A-C 11.7、B-C 26.8，都算"明显不同的位置"。 */
static const struct pickPoint PICK_DST[PICK_OBJECT_COUNT] = {
  { 16.0, -14.0, PICK_GRASP_Z },   /* A 放这里 */
  { 14.0,  16.0, PICK_GRASP_Z },   /* B 放这里 */
  { 26.0,  -8.0, PICK_GRASP_Z }    /* C 放这里 */
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
  PICK_ST_RETREAT,
  PICK_ST_DONE
};

static const char *const PICK_STAGE_NAME[] = {
  "idle", "to-src-approach", "descend-src", "close-tool", "dwell-close",
  "lift", "traverse", "descend-dst", "open-tool", "dwell-open",
  "retreat", "done"
};

/* 阶段类型：直线插值移动 / 夹爪角度渐变 / 原地停顿 */
enum { PICK_KIND_MOVE = 0, PICK_KIND_TOOL, PICK_KIND_DWELL };

static int s_obj = -1;                          /* 当前物体编号，-1 = 空闲 */
static int s_stage = PICK_ST_IDLE;
static int s_kind = PICK_KIND_DWELL;
static unsigned long s_stageStartMs = 0;        /* 本阶段开始时刻 */
static unsigned long s_stageMs = 0;             /* 本阶段计划耗时 */

static double s_fromX = 0.0, s_fromY = 0.0, s_fromZ = 0.0;   /* 移动段起点 */
static double s_toX = 0.0, s_toY = 0.0, s_toZ = 0.0;         /* 移动段终点 */
static double s_fromTool = 0.0, s_toTool = 0.0;              /* 夹爪段起止角 */

/* ==================== 小工具 ==================== */

/* 反解一个工作区点，成功时给出 b/r/c。
 * 注意必须给 angle4 一个合法值：getAngleEx() 会把四个关节一起夹，
 * angle4 非法（例如 0）会让 clamped 恒为 true（.selfcheck 的历史教训）。 */
static bool solveJoint(double x, double y, double z,
                       double *b, double *r, double *c) {
  pos p;
  p.rec.x = x;
  p.rec.y = y;
  p.rec.z = z;
  p.ser = Pos.ser;                 /* 顺带带上传一个合法的 angle4 */
  bool clamped = false;
  if (!getAngleEx(&p, &clamped)) return false;
  if (clamped) return false;       /* 靠吸附才能表示的姿态不算可达 */
  if (b) *b = p.ser.angle1;
  if (r) *r = p.ser.angle2;
  if (c) *c = p.ser.angle3;
  return true;
}

/* 这个工作区点能不能用：在 limit 内 + isReachable() + 反解成功且没被吸附。 */
static bool pointOk(double x, double y, double z,
                    double *b, double *r, double *c) {
  if (x < limit.minX || x > limit.maxX) return false;
  if (y < limit.minY || y > limit.maxY) return false;
  if (z < limit.minZ || z > limit.maxZ) return false;

  REC rec;
  rec.x = x;
  rec.y = y;
  rec.z = z;
  if (!isReachable(&rec)) return false;

  return solveJoint(x, y, z, b, r, c);
}

/* 校验一条直线段：逐点检查能不能用，并且相邻采样点的反解分支不跳变。 */
static bool segmentOk(double x0, double y0, double z0,
                      double x1, double y1, double z1) {
  double dx = x1 - x0;
  double dy = y1 - y0;
  double dz = z1 - z0;
  double len = sqrt(dx * dx + dy * dy + dz * dz);

  int steps = (int)(len / PICK_PATH_SAMPLE_STEP) + 1;
  if (steps < 2) steps = 2;
  if (steps > PICK_PATH_SAMPLE_MAX) steps = PICK_PATH_SAMPLE_MAX;

  double pb = 0.0, pr = 0.0, pc = 0.0;
  for (int i = 0; i <= steps; i++) {
    double t = (double)i / (double)steps;
    double b = 0.0, r = 0.0, c = 0.0;
    if (!pointOk(x0 + dx * t, y0 + dy * t, z0 + dz * t, &b, &r, &c)) {
      return false;
    }
    if (i > 0) {
      if (fabs(b - pb) > PICK_BRANCH_JUMP_DEG ||
          fabs(r - pr) > PICK_BRANCH_JUMP_DEG ||
          fabs(c - pc) > PICK_BRANCH_JUMP_DEG) {
        return false;   /* 反解在段中间换了分支，说明这条直线不能走 */
      }
    }
    pb = b;
    pr = r;
    pc = c;
  }
  return true;
}

/* 当前调速档位对应的关节角速度 */
static double pickRateDps(void) {
  int level = speedGetLevel();
  if (level == SPEED_SLOW) return PICK_RATE_SLOW_DPS;
  if (level == SPEED_FAST) return PICK_RATE_FAST_DPS;
  return PICK_RATE_NORMAL_DPS;
}

/* 走完 degrees 度需要多少毫秒（带上下限） */
static unsigned long durationMs(double degrees, double rateDps) {
  double ms = 0.0;
  if (degrees > 0.0 && rateDps > 0.0) {
    ms = degrees / rateDps * 1000.0;
  }
  if (ms < (double)PICK_MIN_SEG_MS) ms = (double)PICK_MIN_SEG_MS;
  if (ms > (double)PICK_MAX_SEG_MS) ms = (double)PICK_MAX_SEG_MS;
  return (unsigned long)(ms + 0.5);
}

/* 三个轴一次写完，只做一次正解刷新（与串口角度指令同一约定） */
static void setJoints(double b, double r, double c) {
  Pos.ser.angle1 = b;
  Pos.ser.angle2 = r;
  Pos.ser.angle3 = c;
  (void) recFromServo(&Pos.rec, &Pos.ser);
}

/* ==================== 阶段切换 ==================== */

static void beginMove(int stage, double x, double y, double z) {
  s_stage = stage;
  s_kind = PICK_KIND_MOVE;
  s_fromX = Pos.rec.x;
  s_fromY = Pos.rec.y;
  s_fromZ = Pos.rec.z;
  s_toX = x;
  s_toY = y;
  s_toZ = z;
  s_stageStartMs = millis();

  /* 这段要走多久：按"变化最大的那个关节"算，保证任何关节都不超过设定角速度。
   * 反解失败时给 0，落到耗时下限，具体在推进时还会再查一次。 */
  double dMax = 0.0;
  double b = 0.0, r = 0.0, c = 0.0;
  if (solveJoint(x, y, z, &b, &r, &c)) {
    double d;
    d = fabs(b - Pos.ser.angle1); if (d > dMax) dMax = d;
    d = fabs(r - Pos.ser.angle2); if (d > dMax) dMax = d;
    d = fabs(c - Pos.ser.angle3); if (d > dMax) dMax = d;
  }
  s_stageMs = durationMs(dMax, pickRateDps());
}

static void beginTool(int stage, double targetAngle) {
  s_stage = stage;
  s_kind = PICK_KIND_TOOL;
  s_fromTool = Pos.ser.angle4;
  s_toTool = targetAngle;
  s_stageStartMs = millis();
  s_stageMs = durationMs(fabs(s_toTool - s_fromTool), PICK_TOOL_RATE_DPS);
}

static void beginDwell(int stage, unsigned long ms) {
  s_stage = stage;
  s_kind = PICK_KIND_DWELL;
  s_stageStartMs = millis();
  s_stageMs = ms;
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
static void abortSequence(const __FlashStringHelper *why) {
#if WEARM_DEBUG_SERIAL
  Serial.print(F("[pick] ERROR: "));
  Serial.print(why);
  Serial.println(F(", sequence aborted (arm holds position)"));
#else
  (void) why;
#endif
  s_obj = -1;
  s_stage = PICK_ST_IDLE;
  s_kind = PICK_KIND_DWELL;
  s_stageMs = 0;
  s_stageStartMs = millis();
}

/* 本阶段跑完，进入下一阶段 */
static void advanceStage(void) {
  const struct pickPoint *src = &PICK_SRC[s_obj];
  const struct pickPoint *dst = &PICK_DST[s_obj];

  switch (s_stage) {
    case PICK_ST_TO_SRC_APPROACH:
      beginMove(PICK_ST_DESCEND_SRC, src->x, src->y, src->z);
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
      beginMove(PICK_ST_LIFT, src->x, src->y, src->z + PICK_APPROACH_DZ);
      break;

    case PICK_ST_LIFT:
      beginMove(PICK_ST_TRAVERSE, dst->x, dst->y, dst->z + PICK_APPROACH_DZ);
      break;

    case PICK_ST_TRAVERSE:
      beginMove(PICK_ST_DESCEND_DST, dst->x, dst->y, dst->z);
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
      beginMove(PICK_ST_RETREAT, dst->x, dst->y, dst->z + PICK_APPROACH_DZ);
      break;

    case PICK_ST_RETREAT:
      finishSequence();
      break;

    default:
      abortSequence(F("unexpected stage"));
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

  const struct pickPoint *src = &PICK_SRC[object];
  const struct pickPoint *dst = &PICK_DST[object];
  double srcRise = src->z + PICK_APPROACH_DZ;
  double dstRise = dst->z + PICK_APPROACH_DZ;

  /* 1) 当前位姿可用，而且手里的关节角与它的反解是同一个分支 */
  double b = 0.0, r = 0.0, c = 0.0;
  if (!pointOk(Pos.rec.x, Pos.rec.y, Pos.rec.z, &b, &r, &c)) return -3;
  if (fabs(b - Pos.ser.angle1) > PICK_START_TOL_DEG ||
      fabs(r - Pos.ser.angle2) > PICK_START_TOL_DEG ||
      fabs(c - Pos.ser.angle3) > PICK_START_TOL_DEG) {
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[pick] rejected: current pose is not on the ik branch of its own"
                     " coordinates, run posInit()/回到初始位姿后再试"));
#endif
    return -3;
  }

  /* 2) 整条路径采样校验（6 段直线），有一条不合格就整个拒绝 */
  if (!segmentOk(Pos.rec.x, Pos.rec.y, Pos.rec.z, src->x, src->y, srcRise) ||
      !segmentOk(src->x, src->y, srcRise, src->x, src->y, src->z) ||
      !segmentOk(src->x, src->y, src->z, src->x, src->y, srcRise) ||
      !segmentOk(src->x, src->y, srcRise, dst->x, dst->y, dstRise) ||
      !segmentOk(dst->x, dst->y, dstRise, dst->x, dst->y, dst->z) ||
      !segmentOk(dst->x, dst->y, dst->z, dst->x, dst->y, dstRise)) {
#if WEARM_DEBUG_SERIAL
    Serial.print(F("[pick] rejected: path check failed for "));
    Serial.println(PICK_LETTER[object]);
#endif
    return -3;
  }

  s_obj = object;
#if WEARM_DEBUG_SERIAL
  Serial.print(F("[pick] start "));
  Serial.print(PICK_LETTER[object]);
  Serial.print(F(": src=("));
  Serial.print(src->x, 2);
  Serial.print(F(", "));
  Serial.print(src->y, 2);
  Serial.print(F(", "));
  Serial.print(src->z, 2);
  Serial.print(F(") dst=("));
  Serial.print(dst->x, 2);
  Serial.print(F(", "));
  Serial.print(dst->y, 2);
  Serial.print(F(", "));
  Serial.print(dst->z, 2);
  Serial.print(F(") speed="));
  Serial.println(speedLevelName(speedGetLevel()));
#endif

  beginMove(PICK_ST_TO_SRC_APPROACH, src->x, src->y, srcRise);
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
    if (!solveJoint(x, y, z, &b, &r, &c)) {
      abortSequence(F("ik failed mid-path"));
      return;
    }
    setJoints(b, r, c);
  } else if (s_kind == PICK_KIND_TOOL) {
    (void) posSetAngle4(s_fromTool + (s_toTool - s_fromTool) * t);
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
  if (x) *x = PICK_SRC[object].x;
  if (y) *y = PICK_SRC[object].y;
  if (z) *z = PICK_SRC[object].z;
  return true;
}

bool pickPlaceGetTarget(int object, double *x, double *y, double *z) {
  if (object < 0 || object >= PICK_OBJECT_COUNT) return false;
  if (x) *x = PICK_DST[object].x;
  if (y) *y = PICK_DST[object].y;
  if (z) *z = PICK_DST[object].z;
  return true;
}

double pickPlaceApproachDz(void) {
  return PICK_APPROACH_DZ;
}
