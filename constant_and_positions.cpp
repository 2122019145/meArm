//
// constant_and_positions.cpp
// 机械臂核心实现：运动学反解、范围边界、全局调速
//
#include "Arduino.h"   /* 提供 Serial / F() 宏，以及 isnan / isinf */
#include "constant_and_positions.h"

/* ---------- 编译开关 ---------- */
/* 置 1: 打开调试串口输出（波特率由 serial_protocol 模块初始化）。
 * 本开关需与 joystick_control.cpp 中的同名开关保持一致，否则串口输出会缺失。 */
#define WEARM_DEBUG_SERIAL 1

#if WEARM_DEBUG_SERIAL
  #define WEARM_LOG(msg)   Serial.println(F(msg))
  #define WEARM_LOGN(val)  Serial.println(val)
#else
  #define WEARM_LOG(msg)   ((void)0)
  #define WEARM_LOGN(val)  ((void)0)
#endif

/* 复位目标点：选在工作空间内部，既不会触发钳制也保证可解算。
 * 选 (20,0,20) 的理由:
 *   ① 它正是用户标定的初始位姿 —— 舵机 (b,r,c)=(90,90,90,0) 时
 *      上臂竖直向上、下臂水平朝前，末端恰好落在这里；
 *   ② 该点离边界有余量（对当前 limit: 到 maxX 还有 20.0、到 maxZ 还有 20.0、
 *      到 minZ 还有 40.0），开机不会撞限位；
 *   ③ 避免了 x=0 的轴线退化点（rho=0 时回转角无法确定）。
 * 它同时作为 Pos 的初始坐标，避免开机时舵机角度为 0 乱动。 */
static const REC POS_HOME = { 20, 0, 20 };

/* ---------- 全局配置实例 ---------- */

/* 机械臂几何参数，请根据实际硬件修改 (上臂长 / 下臂长 / 末端伸出量)。
 * armheight 在"平面两连杆"模型里表示末端/手爪沿下臂方向再伸出的长度。
 * 本机按实测标定为 0 —— 也就是"下臂末端"本身就是被控点：
 *   (r,c)=(90,90) 时上臂竖直向上、下臂水平朝前，末端正好落在 (20,0,20)。 */
struct arm arm1 = { 20, 20, 0 };

/* 全局位姿实例。
 * 注意: 头文件里 Pos 只是 extern 声明，真正的定义必须在这里，
 * 否则链接阶段会报 undefined reference to `Pos'。
 * 这里先用安全点初始化，posInit() 会再钳制并解算一次角度。 */
pos Pos = { { 0, 0, 0, 0 }, { POS_HOME.x, POS_HOME.y, POS_HOME.z } };

/* 末端位置运动范围（单位与坐标一致）。
 * 该配置必须满足 min <= max，否则由 rangeClampConfig() 自动纠正并报警。
 *
 * 【本机实测可达空间】(肩关节在原点, L1=L2=20, armheight=0,
 *   关节限位见 servoLimit: b[0,180] r[0,180] c[0,180] f[60,150])
 *   用 probe_axes（**直接调用固件 recFromServo()**，不重写公式）
 *   对关节角全域 1° 步长扫描 5929741 个组合，实测:
 *     x[-40.00, 40.00]   y[-40.00, 40.00]   z[-20.00, 40.00]
 *   四个极端姿态（b=90° 时 θ=0，末端都落在 x-z 平面内）:
 *     x=+40  b=90 r=0   c=0     上臂、下臂一起水平朝前伸直
 *     x=-40  b=90 r=180 c=0     上臂、下臂一起水平朝后伸直
 *     z=+40  b=90 r=90  c=0     上臂竖直向上、下臂水平朝前
 *     z=-20  b=90 r=0   c=90    上臂水平朝前、下臂向下折 90°
 *   逐高度扫描（每 2° 一层，x/y 为该层上的完整可达区间）:
 *     z= -20.0: x[  0.00, 26.18]  y[-26.18, 26.18]
 *     z= -18.0: x[  0.00, 30.30]  y[-30.30, 30.30]
 *     z= -16.0: x[  0.00, 33.12]  y[-33.12, 33.12]
 *     z= -14.0: x[  0.00, 35.09]  y[-35.09, 35.09]
 *     z= -12.0: x[  0.00, 36.58]  y[-36.58, 36.58]
 *     z= -10.0: x[  0.00, 37.82]  y[-37.82, 37.82]
 *     z=  -8.0: x[  0.00, 38.67]  y[-38.67, 38.67]
 *     z=  -6.0: x[  0.00, 39.32]  y[-39.32, 39.32]
 *     z=  -4.0: x[  0.00, 39.75]  y[-39.75, 39.75]
 *     z=  -2.0: x[  0.00, 39.97]  y[-39.97, 39.97]
 *     z=   0.0: x[-40.00, 40.00]  y[-40.00, 40.00]   <- x/y 最远
 *     z=   2.0: x[-39.98, 39.98]  y[-39.98, 39.98]
 *     z=   4.0: x[-39.88, 39.88]  y[-39.88, 39.88]
 *     z=   6.0: x[-39.66, 39.66]  y[-39.66, 39.66]
 *     z=   8.0: x[-39.33, 39.33]  y[-39.33, 39.33]
 *     z=  10.0: x[-38.89, 38.89]  y[-38.89, 38.89]
 *     z=  12.0: x[-38.45, 38.45]  y[-38.45, 38.45]
 *     z=  14.0: x[-37.82, 37.82]  y[-37.82, 37.82]
 *     z=  16.0: x[-36.95, 36.95]  y[-36.95, 36.95]
 *     z=  18.0: x[-36.10, 36.10]  y[-36.10, 36.10]
 *     z=  20.0: x[-35.15, 35.15]  y[-35.15, 35.15]
 *     z=  22.0: x[-33.92, 33.92]  y[-33.92, 33.92]
 *     z=  24.0: x[-32.56, 32.56]  y[-32.56, 32.56]
 *     z=  26.0: x[-31.09, 31.09]  y[-31.09, 31.09]
 *     z=  28.0: x[-29.49, 29.49]  y[-29.49, 29.49]
 *     z=  30.0: x[-27.53, 27.53]  y[-27.53, 27.53]
 *     z=  32.0: x[-25.17, 25.17]  y[-25.17, 25.17]
 *     z=  34.0: x[-22.37, 22.37]  y[-22.37, 22.37]
 *     z=  36.0: x[-19.09, 19.09]  y[-19.09, 19.09]
 *     z=  38.0: x[-14.98, 14.98]  y[-14.98, 14.98]
 *     z=  40.0: x[ -8.66,  8.66]  y[ -8.66,  8.66]
 *   注 1：每一层是 |z - 层高| <= 1 的**一层**，不是"z 恰好等于层高"的精确平面。
 *   所以 z=40.0 那一层并不是"仅直立一点"，它还含了 z=39.05 的姿态
 *   （r=103,c=1 给 x=-8.66；r=78,c=1 给 x=+8.66）。真正的 z=40 只有
 *   (r,c)=(90,0) 一个姿态，此时 x=y=0。
 *   注 2：负 x 来自 ρ = 20cos(r) + 20cos(r-c) 变成负值的"朝后伸直/反折"
 *   姿态（例如 r=180、c=0 时 ρ=-40，b=90° 就落在 x=-40），不是坐标算错。
 *
 * 【limit 怎么定的】按用户选择"完全按包络，向外取整到 0.5"：
 *     minX = -40.0 (包络 -40.00)            maxX = 40.0 (包络 40.00)
 *     minY = -40.0 (包络 -40.00)            maxY = 40.0 (包络 40.00)
 *     minZ = -20.0 (包络 -20.00)            maxZ = 40.0 (包络 40.00)
 *   注意：这次包络的六个端值恰好都落在 0.5 的整数倍上，所以"向外取整到 0.5"
 *   之后余量是 0（不是上一个行程时的 0.5）。余量 0 仍然安全，因为包络本身
 *   是解析可证的（|20cos(r)+20cos(r-c)| <= 40、-20 <= 20sin(r)+20sin(r-c) <= 40
 *   在 r,c ∈ [0,180] 时恒成立），1° 步长扫描没有漏掉极端值 —— probe_axes
 *   实测"落在 limit 外的采样点 0 / 5929741 = 0.0%"，且三路关节都能走满行程。
 *   为什么不沿用更小的方盒子: 角度模式下被控量是**关节角**，坐标只是
 *   正运动学算出来的派生量。limit 在这里的作用是"别让机械臂进到没标定的
 *   区域"，而不是"帮用户规划路径"。如果 limit 比真实包络小，那么关节角
 *   明明还有行程、摇杆却会突然停住（表现为"推到头了"），反而更难用。
 *   limit 因此按包络取，四路关节都能走满 servoLimit 行程。
 *
 * 【代价 / 注意】做到这一点后 limit 只是"软护栏"：
 *   - 基座可以摆到 x<0 的后方（上臂 r 与下臂角 (r-c) 一起朝后伸直的姿态）；
 *   - 末端最低到 z≈-20.00，比桌面低，存在撞台面的可能。
 *   真正的最后防线仍然是 servoLimit（每个关节的机械行程）。
 *   上机请先在慢速档、空载、抬离台面的情况下单步试。
 *
 *   落地时 run_axes 自查（probe_axes [3] 段）:
 *     b 基座: 向小端/大端都能走满行程
 *     r 上臂: 向小端/大端都能走满行程（新行程 0~180）
 *     c 下臂: 向小端/大端都能走满行程
 *
 * 为什么 minX = -40.0 而不是 0: 基座舵机 b 的行程是 0~180°，而 b=90° 朝 +x，
 *   所以 x 的正负由平面半径 ρ = 20cos(r) + 20cos(r-c) 决定。r 与 (r-c) 各自
 *   都能到 180°（两节臂一起朝后伸直），此时 ρ 最小是 -40（r=180、c=0），
 *   只要 b=90°（θ=0）末端就落在 x=-40。旧行程 r∈[45,105] 时 ρ 最小只到
 *   -10.35（r=105、c=0），那一侧很窄，所以当时 minX 才敢取到 -10.5。
 *   r 放开到 0~180 之后这一侧明显扩大，limit 必须相应扩展，
 *   否则四路关节走不满行程（会被软护栏拦腰截住）。 */
struct rangeLimit limit = {
  .minX = -40.0, .maxX = 40.0,
  .minY = -40.0, .maxY = 40.0,
  .minZ = -20.0, .maxZ = 40.0
};

/* 全局调速默认值：中速 */
struct speedCfg speed = {
  .stepSize    = 1.0,
  .minDelayMs  = 10,
  .fullDelayMs = 40
};

/* 【关节硬限位】四个舵机的机械允许行程（度）
 *   b = angle1 底部回转   c = angle3 下臂俯仰
 *   r = angle2 上臂俯仰   f = angle4 末端
 * 按实测机械结构填写；反解结果超出这里会被夹住或拒绝（见 servoLimitMode）。 */
struct servoLimitCfg servoLimit = {
  .minB = 0,   .maxB = 180,
  .minR = 0,   .maxR = 180,
  /* r（上臂）按用户要求放开到 0~180：r=0 上臂水平朝前，r=180 上臂朝后水平。
   * 与 c 组合后末端最低可到肩关节以下约 20，上机请先在慢速档、空载、
   * 抬离台面的情况下单步试。 */
  /* c（下臂）按用户要求放开到 0~180。注意 c 接近 180° 时下臂会往后折回、
   * 末端重新向前伸，存在下臂与上臂/底座干涉的风险 —— 这是机械行程的理论
   * 上限，不是"保证不撞"的区间，上机请先用慢速档单步试。
   * 因为关节角现在是直接被控量（角度模式），这条限位就是唯一的软件边界。 */
  .minC = 0,   .maxC = 180,
  .minF = 60,  .maxF = 150
};

/* 关节越界策略：吸附到最近限位（动作到极限为止，不会突然停住） */
int servoLimitMode = SERVO_LIMIT_CLAMP;

/* 关节越界提示的限流时间戳（避免持续越限把串口刷爆） */
static unsigned long lastServoLogTime = 0;

/* 当前档位。-1 = 自定义(由 setSpeed 直接写入)，否则为 SPEED_* 之一 */
static int speedLevel = SPEED_NORMAL;

/* ---------- 内部小工具 ---------- */

/* 判断浮点数是否有效（排除 NaN 与 Inf）。 */
static bool isFiniteNum(double v) {
  return !isnan(v) && !isinf(v);
}

/* 把一个 double 限定到 [lo, hi]，且 lo/hi 写反时自动交换。 */
static double clampDouble(double v, double lo, double hi) {
  if (lo > hi) { double t = lo; lo = hi; hi = t; }
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* ---------- 关节硬限位 ---------- */

/* 各关节的限位映射表：把 servoLimit 里的 b/r/c/f 区间与 SER 字段对应起来。
 * 用表格而不是四段重复代码，加关节时只改这张表。 */
struct jointRule {
  char   name;          /* 单字符关节名，仅用于串口提示 */
  double minAngle;
  double maxAngle;
};

static struct jointRule jointMin[4] = { {'b', 0, 0}, {'r', 0, 0}, {'c', 0, 0}, {'f', 0, 0} };
static struct jointRule jointMax[4] = { {'b', 0, 0}, {'r', 0, 0}, {'c', 0, 0}, {'f', 0, 0} };

/* 把当前 servoLimit 刷新进映射表，并返回被修正（写反或超出 0~180）的项数。
 * 舵机物理行程只有 0~180°，所以越界的限位本身也是配置错误。 */
int servoSelfCheck(void) {
  int bad = 0;

  /* 1) 四项限位先各自排序、并夹进 0~180 */
  if (servoLimit.minB > servoLimit.maxB) { double t = servoLimit.minB; servoLimit.minB = servoLimit.maxB; servoLimit.maxB = t; bad++; }
  if (servoLimit.minR > servoLimit.maxR) { double t = servoLimit.minR; servoLimit.minR = servoLimit.maxR; servoLimit.maxR = t; bad++; }
  if (servoLimit.minC > servoLimit.maxC) { double t = servoLimit.minC; servoLimit.minC = servoLimit.maxC; servoLimit.maxC = t; bad++; }
  if (servoLimit.minF > servoLimit.maxF) { double t = servoLimit.minF; servoLimit.minF = servoLimit.maxF; servoLimit.maxF = t; bad++; }

  double *mins[4] = { &servoLimit.minB, &servoLimit.minR, &servoLimit.minC, &servoLimit.minF };
  double *maxs[4] = { &servoLimit.maxB, &servoLimit.maxR, &servoLimit.maxC, &servoLimit.maxF };
  for (int i = 0; i < 4; i++) {
    if (*mins[i] < 0.0)   { *mins[i] = 0.0;   bad++; }
    if (*maxs[i] > 180.0) { *maxs[i] = 180.0; bad++; }
  }

  /* 2) 刷新映射表 */
  char names[4] = {'b', 'r', 'c', 'f'};
  for (int i = 0; i < 4; i++) {
    jointMin[i].name = names[i];
    jointMin[i].minAngle = *mins[i];
    jointMin[i].maxAngle = *maxs[i];
    jointMax[i].name = names[i];
    jointMax[i].minAngle = *mins[i];
    jointMax[i].maxAngle = *maxs[i];
  }

  if (bad > 0) {
    WEARM_LOG("[servo] ERROR: joint limit config invalid, auto-corrected");
  }
  return bad;
}

/* 判断某一关节角是否在限位内（容差 ANGLE_EPS 度） */
static bool jointOk(int i, double angle) {
  if (i < 0 || i > 3) return false;
  return (angle >= jointMin[i].minAngle - ANGLE_EPS &&
          angle <= jointMin[i].maxAngle + ANGLE_EPS);
}

/* 把 SER 的四个角度指针按关节顺序取出来，便于统一处理 */
static double *serAnglePtr(SER *ser, int i) {
  switch (i) {
    case 0: return &ser->angle1;
    case 1: return &ser->angle2;
    case 2: return &ser->angle3;
    case 3: return &ser->angle4;
    default: return NULL;
  }
}

bool isServoInRange(const SER *ser) {
  if (ser == NULL) return false;
  if (!jointOk(0, ser->angle1)) return false;
  if (!jointOk(1, ser->angle2)) return false;
  if (!jointOk(2, ser->angle3)) return false;
  if (!jointOk(3, ser->angle4)) return false;
  return true;
}

bool clampServoAngles(SER *ser) {
  if (ser == NULL) return false;
  bool changed = false;
  for (int i = 0; i < 4; i++) {
    double *ap = serAnglePtr(ser, i);
    if (ap == NULL) continue;
    double v = clampDouble(*ap, jointMin[i].minAngle, jointMin[i].maxAngle);
    if (v != *ap) { *ap = v; changed = true; }
  }
  return changed;
}

/* 按当前策略处理关节限位，返回是否可接受该姿态。
 * 参数 clamped 用于回传"是否发生了吸附"（NULL 表示不关心）：
 *   CLAMP 策略下即使返回 true，只要 clamped=true，就说明算出的姿态被改过了、
 *   末端实际到不了目标点 —— 调用方（如 moveAxisStep）据此回退整步坐标，
 *   否则坐标系会和真实姿态越差越远。 */
static bool applyJointLimits(SER *ser, bool *clamped) {
  if (clamped != NULL) *clamped = false;
  if (ser == NULL) return false;

  /* 先找出第一个越界关节（只用于提示）。
   * 【容差为什么比 ANGLE_EPS 大】反解是 acos/atan2 拼出来的，落在限位边界上的
   * 解常有 -1e-14 这种量级的舍入误差（例如 c 的理论值是 0，算出来是
   * -1.1e-14）。若按 1e-6 判越界，几乎每一个"正好在行程端点"的姿态都会
   * 被记成"被吸附"，CLAMP 警告与 clamped 标志天天误报，
   * 调用方也就无法用它区分"真被限位挡住"和"只是端点舍入"。
   * 1e-9 度远小于舵机可分辨的步进（0.1° 量级），不会漏掉真实的越限。 */
  const double LIM_EPS = 1e-9;
  int badIdx = -1;
  double badVal = 0.0;
  for (int i = 0; i < 4; i++) {
    double *ap = serAnglePtr(ser, i);
    if (ap != NULL && (*ap < jointMin[i].minAngle - LIM_EPS ||
                       *ap > jointMin[i].maxAngle + LIM_EPS)) {
      badIdx = i; badVal = *ap; break;
    }
  }
  if (badIdx < 0) return true;      /* 全部在限位内 */

  if (servoLimitMode == SERVO_LIMIT_REJECT) {
    unsigned long now = millis();
    if (now - lastServoLogTime >= 500) {
      lastServoLogTime = now;
      Serial.print(F("[servo] reject joint "));
      Serial.print(jointMin[badIdx].name);
      Serial.print(F(" = "));
      Serial.print(badVal);
      Serial.print(F(" (allow "));
      Serial.print(jointMin[badIdx].minAngle);
      Serial.print('-');
      Serial.print(jointMin[badIdx].maxAngle);
      Serial.println(F(")"));
    }
    return false;
  }

  /* CLAMP 策略：吸附到最近限位 */
  clampServoAngles(ser);
  if (clamped != NULL) *clamped = true;
  unsigned long now = millis();
  if (now - lastServoLogTime >= 500) {
    lastServoLogTime = now;
    Serial.print(F("[servo] clamp joint "));
    Serial.print(jointMin[badIdx].name);
    Serial.print(F(" = "));
    Serial.print(badVal);
    Serial.print(F(" -> "));
    Serial.println(clampDouble(badVal, jointMin[badIdx].minAngle, jointMin[badIdx].maxAngle));
  }
  return true;
}
/* ---------- 范围边界 ---------- */

/* 对每个轴做 (min,max) 排序，返回原来写反了的轴数 */
int rangeClampConfig(void) {
  int bad = 0;
  if (limit.minX > limit.maxX) { double t = limit.minX; limit.minX = limit.maxX; limit.maxX = t; bad++; }
  if (limit.minY > limit.maxY) { double t = limit.minY; limit.minY = limit.maxY; limit.maxY = t; bad++; }
  if (limit.minZ > limit.maxZ) { double t = limit.minZ; limit.minZ = limit.maxZ; limit.maxZ = t; bad++; }
  if (bad > 0) {
    WEARM_LOG("[limit] ERROR: range min>max, auto-corrected:");
    WEARM_LOG("[limit]   X/Y/Z = [min,max] -> check constant_and_positions.cpp");
    WEARM_LOGN(bad);
  }
  return bad;
}

/* 检查某个坐标是否贴在边界上；贴边记为命中并记录轴名 */
static bool edgeHit(double v, double lo, double hi, char axis, char *outAxis) {
  if (v <= lo + RANGE_EPS || v >= hi - RANGE_EPS) {
    if (outAxis != NULL) *outAxis = axis;
    return true;
  }
  return false;
}

bool atRangeEdge(const pos *pos1, char *axis) {
  if (axis != NULL) *axis = 0;
  if (pos1 == NULL) return false;
  if (edgeHit(pos1->rec.x, limit.minX, limit.maxX, 'x', axis)) return true;
  if (edgeHit(pos1->rec.y, limit.minY, limit.maxY, 'y', axis)) return true;
  if (edgeHit(pos1->rec.z, limit.minZ, limit.maxZ, 'z', axis)) return true;
  return false;
}

/* 将位置钳制到配置的范围内，若有轴越界则返回 true。
 * 三个轴各自独立钳制，不会因为一个轴越界而影响其它轴。 */
bool clampToRange(pos *pos1) {
  if (pos1 == NULL) return false;
  bool changed = false;

  double x = clampDouble(pos1->rec.x, limit.minX, limit.maxX);
  double y = clampDouble(pos1->rec.y, limit.minY, limit.maxY);
  double z = clampDouble(pos1->rec.z, limit.minZ, limit.maxZ);

  if (x != pos1->rec.x) { pos1->rec.x = x; changed = true; }
  if (y != pos1->rec.y) { pos1->rec.y = y; changed = true; }
  if (z != pos1->rec.z) { pos1->rec.z = z; changed = true; }
  return changed;
}

/* ---------- 运动学 ---------- */

/* 可达性检查：判断反解时会不会出现 acos 越域或 0/0。
 *
 * 本机械臂是"肩关节在原点、两杆各 20"的平面两连杆结构：
 *   肩关节轴心 = 基座回转轴上的 (0,0,0)，末端坐标 z 以它为原点。
 *   末端与肩关节的距离 R 必须落在 [|L1-L2|, L1+L2] 内；
 *   R = |L1-L2| 只有在两杆完全折回（c = 180°，本机 c 上限已放开到 180）
 *   时才取到，此时末端落回肩关节轴线上（rho = 0）。rho 极小时回转角
 *   atan2(y,x) 在数学上不可确定，由 getAngleEx 取默认中立位 90°，
 *   是否真的可达交给关节限位与正解残差判定。
 * 关节行程（servoLimit）的最终把关在 getAngle() 里做，
 * 这里只挡掉"根本没有几何解"的目标点。 */
bool isReachable(const REC *rec) {
  if (rec == NULL) return false;
  if (!isFiniteNum(rec->x) || !isFiniteNum(rec->y) || !isFiniteNum(rec->z)) return false;

  double r = sqrt(rec->x * rec->x + rec->y * rec->y);
  double zv = rec->z - arm1.armheight;
  double R = sqrt(r * r + zv * zv);

  double L1 = arm1.armLength1;
  double L2 = arm1.armLength2;
  if (L1 <= 0 || L2 <= 0) return false;

  /* 条件 1: 上臂/下臂/空间半径能构成三角形。
   * 【必须带容差】两杆完全伸直时 R 在数学上恰好等于 L1+L2，但浮点算出来
   * 常是 L1+L2 再大最后一位（例如 40.000000000000004 > 40），
   * 严格比较会把"全伸直"这一整类姿态（c=0 时的所有 r）判成不可达 ——
   * 实测有 19 组 rho=16.905、z=36.252 的目标因此反解失败。
   * 1e-9 的容差远小于机构精度，不会把真正够不着的点放进来。 */
  const double GEOM_EPS = 1e-9;
  if (R > L1 + L2 + GEOM_EPS || R < fabs(L1 - L2) - GEOM_EPS) return false;
  /* 允许末端正好落在基座轴线上（r = 0）: 此时回转角不可确定，
   * 由 getAngleEx 取默认朝 +x，并由关节限位/残差校验决定是否真的可达。 */
  return true;
}

/* 正运动学：四个舵机角度 -> 末端笛卡尔坐标。
 *
 * 【为什么需要它】角度模式下被控量直接是关节角，Pos.rec 不再是"用户设的目标"，
 * 而是"当前角度算出来的真实位置"。每次改角度后都调一次，让坐标永远与角度自洽。
 * 四个关节角就是四个自由度：b（回转）决定末端绕基座轴的方位，
 * r/c（上臂/下臂）决定平面内的半径与高度，f 只转末端夹具、不影响被控点。
 * 与 getAngleEx 里的正解保持严格一致（同一组公式）：
 *   alpha = r, beta = r - c
 *   x_planar = L1 cos alpha + L2 cos beta
 *   z        = L1 sin alpha + L2 sin beta + armheight
 *   theta    = (90 - b)  ->  x = x_planar cos theta,  y = x_planar sin theta */
bool recFromServo(REC *rec, const SER *ser) {
  if (rec == NULL || ser == NULL) return false;

  double b = ser->angle1;
  double r = ser->angle2;
  double c = ser->angle3;
  if (!isFiniteNum(b) || !isFiniteNum(r) || !isFiniteNum(c)) return false;

  double alpha = r / RADtoDEG;          /* 上臂方向角（弧度） */
  double beta  = (r - c) / RADtoDEG;    /* 下臂方向角 = r - c */
  double xPlanar = arm1.armLength1 * cos(alpha) + arm1.armLength2 * cos(beta);
  double z       = arm1.armLength1 * sin(alpha) + arm1.armLength2 * sin(beta) + arm1.armheight;

  double theta = (90.0 - b) / RADtoDEG; /* b=90 朝 +x；b<90 转向 +y */
  /* 先算到局部变量再写回，允许 rec 与 ser 指向同一个 pos 结构体 */
  double x = xPlanar * cos(theta);
  double y = xPlanar * sin(theta);
  if (!isFiniteNum(x) || !isFiniteNum(y) || !isFiniteNum(z)) return false;

  rec->x = x;
  rec->y = y;
  rec->z = z;
  return true;
}

/* 由笛卡尔坐标反解舵机角度，不可达时拒绝写入。
 *
 * 【本机运动学模型】—— 以用户实测标定为准
 *   肩关节轴心在坐标原点 (0,0,0)；末端坐标 z 以肩关节轴为原点。
 *   上臂 L1 = 20，下臂 L2 = 20；armheight 表示末端/手爪沿下臂方向再伸出的
 *   长度（本机标定为 0，即"下臂末端"就是控制点）。
 *
 *   舵机中立位（90°）的实测含义：
 *     angle1 (b) 90° -> 基座朝 +x 方向（可达空间只占 x >= 0 那一侧）
 *     angle2 (r) 90° -> 上臂竖直向上 (0, +z)
 *     angle3 (c) 90° -> 下臂水平朝前 (+x)，即与上臂成 90°
 *
 *   设平面内以 +x 为 0°、抬向 +z 为正，两个连杆的方向角为:
 *     上臂方向角   alpha = r          （r 就是上臂的绝对方向角）
 *     下臂方向角   beta  = r - c      （c 是下臂相对上臂往前转的折角）
 *   正运动学:
 *     x_planar = L1 cos alpha + L2 cos beta
 *     z        = L1 sin alpha + L2 sin beta + armheight
 *   标定校验（armheight = 0, L1 = L2 = 20，两处误差均为 0.00）:
 *     (r,c) = (90, 90) -> alpha=90°, beta=  0° -> 末端 (20.00, 20.00)  ← 初始位姿
 *     (r,c) = (90,105) -> alpha=90°, beta=-15° -> 末端 (19.32, 14.82)  用户实测
 *     (r,c) = (45, 90) -> alpha=45°, beta=-45° -> 末端 (28.28,  0.00)  水平朝前
 *     (r,c) = (45,  0) -> alpha=45°, beta= 45° -> 末端 (28.28, 28.28)  斜举伸直
 *   r 增大 = 上臂抬高；c 增大 = 折角收小、末端"往前 + 往下"，
 *   与用户实测（c 从 90 加到 105 下臂往前转）一致。
 *
 *   反解（闭式，无迭代）:
 *     angle1 = 90° - atan2(y, x)                    （b=90 朝 +x，b<90 转向 +y）
 *     R²     = rho² + (z - armheight)²              末端到肩的距离，只与 c 有关
 *       cos(c) = (R² - L1² - L2²) / (2 L1 L2)        c ∈ [0,180°] 唯一候选
 *     由 P = e^{i·alpha}·W(c)、W(c) = L1 + L2·e^{-i·c} 得
 *       alpha  = atan2(zv, rho) + atan2(L2 sin c, L1 + L2 cos c)
 *     再由 (alpha, c) 回代正运动学校验残差，超差整组丢弃。
 *
 *   平面分支: 上面用的是 rhoP = +rho（末端朝平面 +x）。
 *     当 rhoP = -rho 时得到 "下臂反折" 的另一个姿态（alpha、beta 各差 180°），
 *     对应的回转角是 90° - atan2(-y, -x)。c 放开到 180° 后
 *     20cos(r) + 20cos(r-c) 可能为负，可达点会落到 x<0 一侧，
 *     此时只有反向分支能给出落在 [0,180] 内的回转角 —— 见 getAngleEx 里
 *     "选平面分支" 一段的注释。两个分支都不合法时返回 false。
 *
 * 返回值:
 *   true  —— 运动学可解，且（按 servoLimitMode）关节角都落在 servoLimit 内
 *            （CLAMP 策略下越限的角度会被就地吸附到最近限位）
 *   false —— 目标点不可达、角度出现 NaN/Inf，或 REJECT 策略下有关节越限
 * 返回 false 时不修改 pos1->ser，调用方应保留上一次的有效角度。
 * angle4(末端 f) 不参与反解，保持调用前已有的值不变。 */
bool getAngle(pos *pos1) {
  return getAngleEx(pos1, NULL);
}

/* getAngle 的扩展版：额外回传"是否因关节限位被吸附"。
 * 需要区分"精确到达目标"与"被限位挡住"的调用方（moveAxisStep）用这个版本。 */
/* getAngleEx 内部用的分支求解。
 *
 * 【为什么不能用 alpha + beta 凑】
 * 曾试过令 s = alpha + beta、由 L1·e^{iα} = (rhoP + i·zv) - L2·e^{iβ} 解出 s，
 * 结果整表反解失败（1425/1425）—— 那个式子是硬凑的，把 e^{iβ} 的模与幅角混在一起了。
 * 正确做法是把向量方程两边乘 e^{-iβ}：
 *     L1·e^{i(α-β)} + L2 = (rhoP + i·zv)·e^{-iβ}
 * 于是 e^{-iβ} 的幅角关系直接给出 beta，不需要任何"和角"技巧。
 *
 * 【固件运动学模型（两处实测标定核对，误差 0.00）】
 *   alpha = r / RADtoDEG = 上臂绝对方向角（弧度）
 *   beta  = (r - c) / RADtoDEG = 下臂绝对方向角
 *   x_planar = L1·cos(alpha) + L2·cos(beta)
 *   z        = L1·sin(alpha) + L2·sin(beta) + armheight
 * 注意 **舵机角 c = alpha - beta（弧度）**，它才是被 servoLimit 限位的量。
 *
 * 【反解分支】
 * 令 delta = alpha - beta（即舵机角，弧度），P = rhoP + i·zv：
 *     e^{i·beta} = P / (L1 + L2·e^{-i·delta})
 *     beta  = atan2(zv, rhoP) - atan2(L2·sin(delta), L1 + L2·cos(delta))
 *     alpha = beta + delta
 * 其中 |L1 + L2·e^{-iδ}|² = L1² + L2² + 2·L1·L2·cos(delta) = R²，
 * 所以 delta 的大小由余弦定理定：|delta| = acos((R²-L1²-L2²)/(2·L1·L2))，
 * 而正负号 k = ±1 对应肘部在上/在下的两个镜像姿态：
 *   k = +1 → delta = +|c|（常见姿态）
 *   k = -1 → delta = -|c|（镜像姿态）
 * 两个都必须试：镜像姿态里常常只有一个满足机械行程。 */
static void ikBranch(double rhoP, double zv, double delta, double L1, double L2,
                     double *alpha, double *beta, double *cServo, double *err) {
  double phiBase = atan2(zv, rhoP);                  /* 末端在平面内的方向角 */
  double argW    = atan2(L2 * sin(delta), L1 + L2 * cos(delta));
  *beta  = phiBase - argW;
  *alpha = *beta + delta;
  *cServo = delta;                                   /* 舵机角 c（弧度） */
  double fx = L1 * cos(*alpha) + L2 * cos(*beta);
  double fz = L1 * sin(*alpha) + L2 * sin(*beta);
  *err = sqrt(pow(fx - rhoP, 2) + pow(fz - zv, 2));
}

bool getAngleEx(pos *pos1, bool *clamped) {
  if (clamped != NULL) *clamped = false;
  if (pos1 == NULL) return false;

  /* 必须先把关节限位刷成配置里的值再选分支 —— 选分支要用 b 的行程
   * 判断回转角是否可行。漏掉这一句的话，开机时（限位还是 0）会把所有分支
   * 都判成不可行，然后 applyJointLimits 又把角度全部吸附到 0，
   * 表现为"反解永远返回 (0,0,0) 且 clamped 恒为 true"。 */
  (void) servoSelfCheck();

  if (!isReachable(&pos1->rec)) {
    WEARM_LOG("[kin] reject: target out of reachable workspace");
    return false;
  }

  const double L1 = arm1.armLength1;
  const double L2 = arm1.armLength2;

  /* 平面半径 rho 与"肩->末端"距离 R（末端竖直方向等于坐标 z，由 armheight 标定） */
  double rho = sqrt(pow(pos1->rec.x, 2) + pow(pos1->rec.y, 2));
  double zv  = pos1->rec.z - arm1.armheight;
  double R2  = rho * rho + zv * zv;

  /* ---------- 运动学模型（已用两个实测标定点校准）----------
   * 平面内以 +x 为 0°、抬向 +z 为正，两杆各 L1 / L2：
   *     alpha = 上臂方向角 = r            （r 就是上臂的绝对方向角）
   *     beta  = 下臂方向角 = r - c        （c 增大 = 下臂往前/往下转）
   *     x = L1·cos(alpha) + L2·cos(beta)
   *     z = L1·sin(alpha) + L2·sin(beta) + armheight
   * 标定校验（armheight = 0, L1 = L2 = 20，两处误差均为 0.00）：
   *     (r=90, c=90)  -> alpha=90°, beta=  0°  -> 末端 (20.00, 20.00)  ✔ 初始位姿
   *     (r=90, c=105) -> alpha=90°, beta=-15°  -> 末端 (19.32, 14.82)  ✔ 用户实测
   * 因此 beta = r - c。注意另两个候选 r+c-180 与 r+c 都只能命中其中一个点
   * （r+c-180 在 c=105 给 (19.32,25.18)，r+c 连初始位姿的符号都反了）。
   * 由 d4.cpp 对三个候选逐个代入两处实测标定后唯一命中 A。
   * ---------------------------------------------------------- */

  /* ---------- 四个平面候选 ----------
   * 反解在平面内是"两杆构成三角形"的标准问题：肘角大小 |c| 由余弦定理唯一确定，
   * 但有两个独立的二值自由度：
   *   ① 肘部在上 / 在下  →  delta = ±|c|（两个镜像姿态）
   *   ② 平面朝前 / 反折  →  rhoP  = +rho 或 -rho
   * ② 为什么必要：回转角 b = 90 - atan2(y,x)。目标落在 x<0 一侧时
   * atan2(y,x) 在 (90°,270°)，b 会算出 190°~270°，超出 b 的行程 [0,180]，
   * 被 CLAMP 静默吸附到 180° 并丢掉最多 5.4 个单位的坐标精度
   * （dbg_rt 证据：tgt=(-0.61,-3.43,39.85) rawB=190.000 -> retB=180.000 err=0.6077）。
   * 改用反向平面 rhoP = -rho 后，回转角变成 b = 90 - atan2(-y,-x)，
   * 对 x<0 恰好落回 [0,180]。两个平面分支都要试。
   * 选出候选后仍要用正解回代核对残差（见下面 errs[] 的用法）。 */
  double cosC = (R2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  if (cosC >  1.0) cosC =  1.0;
  if (cosC < -1.0) cosC = -1.0;
  double cAbs = acos(cosC);                          /* |c|，0~180° */

  const double BASE_EPS = 1e-9;                      /* 度：远小于舵机可分辨的 0.1° */
  const double JOINT_EPS = 1e-9;

  double bestB = 0.0, bestR = 0.0, bestC = 0.0, bestErr = 1e30;
  int pick = -1;

  for (int i = 0; i < 4; i++) {
    const bool reversePlane = (i >= 2);              /* ② 平面朝前 / 反折 */
    const double k = ((i & 1) == 0) ? 1.0 : -1.0;    /* ① 肘部在上 / 在下 */
    const double rhoP  = reversePlane ? -rho : rho;
    const double delta = k * cAbs;                   /* = 舵机角 c（弧度） */

    /* 回转角：正向平面用 (x,y)，反向平面用 (-x,-y)。rho 极小时方向无意义，取 90° */
    double b = 90.0;
    if (rho > 1e-9) {
      b = reversePlane ? (90.0 - RADtoDEG * atan2(-pos1->rec.y, -pos1->rec.x))
                       : (90.0 - RADtoDEG * atan2( pos1->rec.y,  pos1->rec.x));
    }
    while (b < 0.0)    b += 360.0;
    while (b >= 360.0) b -= 360.0;
    /* 【归一化后的回折】90 - RADtoDEG*atan2() 的浮点误差会把"正好 0°"算成
     * -2.4e-10，上面那句 += 360 于是把它变成 359.9999999998，
     * 再和 maxB = 180 一比就把这个分支丢掉了 ——
     * 实测 (x=0,y=20,z=20) 的正确回转角恰好是 0°（b=0 朝 +y），就踩在这个坑里，
     * 表现为反解返回 false 且 Pos.ser 停在 (0,0,0)。
     * 因此把"贴着 360°"的值折回 0°，容差远大于浮点噪声、远小于 1° 步进。 */
    if (b > 360.0 - 1e-6) b = 0.0;
    if (b < servoLimit.minB - BASE_EPS || b > servoLimit.maxB + BASE_EPS) continue;

    double alpha, beta, cServo, err;
    ikBranch(rhoP, zv, delta, L1, L2, &alpha, &beta, &cServo, &err);
    if (!isFiniteNum(alpha) || !isFiniteNum(beta) || !isFiniteNum(err)) continue;

    double rDeg = RADtoDEG * alpha;
    double cDeg = RADtoDEG * cServo;
    /* r、c 必须落在各自行程内 —— 必须在进 applyJointLimits 之前显式筛掉，
     * 否则 CLAMP 会把越限的候选静默吸附成"看起来能用"的解。 */
    if (rDeg < servoLimit.minR - JOINT_EPS || rDeg > servoLimit.maxR + JOINT_EPS) continue;
    if (cDeg < servoLimit.minC - JOINT_EPS || cDeg > servoLimit.maxC + JOINT_EPS) continue;

    if (pick < 0 || err < bestErr) {
      pick = i; bestErr = err;
      bestB = b; bestR = rDeg; bestC = cDeg;
    }
  }

  if (pick < 0) {
    WEARM_LOG("[kin] reject: no planar branch fits the joint travel");
    return false;
  }

  /* 正运动学回代自检：选中的分支必须真的落在目标点上，否则整组丢弃 */
  if (!(bestErr <= 0.05)) {
    WEARM_LOG("[kin] reject: residual too large, no valid solution");
    return false;
  }

  double angle1  = bestB;
  double angle2  = bestR;                            /* r = 上臂绝对方向角 */
  double angle3  = bestC;                            /* c = alpha - beta（舵机角） */

  /* 数值保护: 任何一项不是有限数就整组丢弃，避免舵机收到 NaN */
  if (!isFiniteNum(angle1) || !isFiniteNum(angle2) || !isFiniteNum(angle3)) {
    WEARM_LOG("[kin] reject: non-finite angle solution");
    return false;
  }

  /* 容差内的微小负角归零，避免 -0.0 这种值被 (int) 截断后传给舵机 */
  if (angle1 > -ANGLE_EPS && angle1 < 0) angle1 = 0;
  if (angle2 > -ANGLE_EPS && angle2 < 0) angle2 = 0;
  if (angle3 > -ANGLE_EPS && angle3 < 0) angle3 = 0;

  /* 【为什么先备份再写、失败要回滚】
   * 本函数的契约是"返回 false 时不修改 pos1->ser，调用方保留上一个有效姿态"。
   * 但 applyJointLimits 在 CLAMP 策略下会就地吸附角度，写完才发现要拒绝时，
   * 角度已经被改掉一半了 —— 老代码就踩过这个坑：一个反解失败的目标点会把
   * Pos.ser 留成 (0,0,0)，上电时机械臂直接甩向原点。
   * 因此这里把三角度先存本地，确认成功后再落回 ser。 */
  const double oldA1 = pos1->ser.angle1;
  const double oldA2 = pos1->ser.angle2;
  const double oldA3 = pos1->ser.angle3;
  const double oldA4 = pos1->ser.angle4;

  pos1->ser.angle1 = angle1;
  pos1->ser.angle2 = angle2;
  pos1->ser.angle3 = angle3;
  /* angle4(末端 f) 不由反解决定，保持调用前已有的值不动 */

  /* 最后一道防线：四个关节角都要落在 servoLimit 的机械行程内。
   * CLAMP 策略下越限会被吸附到最近限位并返回 true（同时置 clamped）；
   * REJECT 策略下越限直接返回 false，调用方应回退坐标。 */
  bool limClamped = false;
  if (!applyJointLimits(&pos1->ser, &limClamped)) {
    pos1->ser.angle1 = oldA1;
    pos1->ser.angle2 = oldA2;
    pos1->ser.angle3 = oldA3;
    pos1->ser.angle4 = oldA4;
    return false;
  }
  if (clamped != NULL) *clamped = limClamped;
  return true;
}

/* 复位到工作空间内的安全初始点。
 * 顺带自检 rangeLimit / servoLimit 配置，避免错误配置一直潜伏。 */
void posInit(void) {
  (void) rangeClampConfig();   /* 修正写反的 min/max，并在串口报警 */
  (void) servoSelfCheck();     /* 修正非法的关节限位，并刷新限位映射表 */

  Pos.rec.x = POS_HOME.x;
  Pos.rec.y = POS_HOME.y;
  Pos.rec.z = POS_HOME.z;
  clampToRange(&Pos);          /* 即使 POS_HOME 被改动越界也能拉回来 */

  /* 末端舵机取行程中位，避免开机时停在极限位置 */
  Pos.ser.angle4 = (servoLimit.minF + servoLimit.maxF) * 0.5;

  if (!getAngle(&Pos)) {
    WEARM_LOG("[pos] ERROR: POS_HOME unreachable or out of joint limits");
  }
#if WEARM_DEBUG_SERIAL
  /* 开机自检输出: 直接调用固件的正解，把"反解出的角度"再正解回坐标，
   * 与 POS_HOME 比较 —— 上机时一眼就能确认标定是否正确。
   *
   * 【教训】这里曾经自己重写公式并写成 beta = r + c（正确是 r - c），
   * 属于会骗人的自检：公式错了照样打印一串"看起来很合理"的数字。
   * 现在统一走 recFromServo()，固件只有一处运动学实现，改一处不会漏另一处。 */
  {
    REC chk;
    if (recFromServo(&chk, &Pos.ser)) {
      Serial.print(F("[pos] home fk check: x="));
      Serial.print(chk.x);
      Serial.print(F(" y="));
      Serial.print(chk.y);
      Serial.print(F(" z="));
      Serial.print(chk.z);
      Serial.print(F("  (POS_HOME x="));
      Serial.print(POS_HOME.x);
      Serial.print(F(" y="));
      Serial.print(POS_HOME.y);
      Serial.print(F(" z="));
      Serial.print(POS_HOME.z);
      Serial.println(F(")"));
    } else {
      Serial.println(F("[pos] home fk check: recFromServo failed"));
    }
  }
#endif
}

/* 取"开机初始位姿"对应的关节角，写给按键4 回中用。
 * 角度真值由 POS_HOME 反解得到，不在别处写死 90/90/90 —— 以后改了 POS_HOME
 * 或关节限位，回中仍然会回到真正的初始位姿。
 * angle4（末端开合）不是坐标反解的自由度：传进来什么就保持什么，这里不动它。 */
bool posGetHomeAngles(SER *ser) {
  if (ser == NULL) return false;

  pos p;
  p.ser = *ser;                 /* 带上调用者的 angle4 */
  p.rec.x = POS_HOME.x;
  p.rec.y = POS_HOME.y;
  p.rec.z = POS_HOME.z;
  (void) clampToRange(&p);      /* 即使 POS_HOME 被改到界外也能拉回来 */
  if (!getAngle(&p)) return false;

  ser->angle1 = p.ser.angle1;
  ser->angle2 = p.ser.angle2;
  ser->angle3 = p.ser.angle3;
  return true;
}

/* ---------- 全局调速 ---------- */
/* 设置全局调速参数 (带合法性校验)
 * 校验规则: stepSize > 0；minDelayMs > 0；fullDelayMs >= minDelayMs。
 * 不合法的项保持原值，只有确实改动了参数才把档位标记为自定义。-1。 */
void setSpeed(double stepSize, int minDelayMs, int fullDelayMs) {
  bool changed = false;
  if (stepSize > 0 && stepSize != speed.stepSize) {
    speed.stepSize = stepSize; changed = true;
  }
  if (minDelayMs > 0 && minDelayMs <= fullDelayMs && minDelayMs != speed.minDelayMs) {
    speed.minDelayMs = minDelayMs; changed = true;
  }
  if (fullDelayMs >= minDelayMs && fullDelayMs >= 0 && fullDelayMs != speed.fullDelayMs) {
    speed.fullDelayMs = fullDelayMs; changed = true;
  }
  /* 参数被手动改动后，当前档位名已不再代表实际参数 */
  if (changed) speedLevel = -1;
}

/* 按档位调整速度，返回生效档位 (-1 表示档位非法) */
int adjustSpeed(int level) {
  switch (level) {
    case SPEED_SLOW:   /* 慢速：小步长、长间隔，适合精细操作 */
      setSpeed(0.5, 20, 80);
      speedLevel = SPEED_SLOW;
      return SPEED_SLOW;
    case SPEED_NORMAL: /* 中速：默认参数 */
      setSpeed(1.0, 10, 40);
      speedLevel = SPEED_NORMAL;
      return SPEED_NORMAL;
    case SPEED_FAST:   /* 快速：大步长、短间隔 */
      setSpeed(2.0, 5, 20);
      speedLevel = SPEED_FAST;
      return SPEED_FAST;
    default:           /* 不合法档位：不改参数 */
      return -1;
  }
}

int speedGetLevel(void) {
  return speedLevel;
}

const char *speedLevelName(int level) {
  switch (level) {
    case SPEED_SLOW:   return "慢速";
    case SPEED_NORMAL: return "中速";
    case SPEED_FAST:   return "快速";
    default:           return "自定义";
  }
}

/* 慢 → 中 → 快 → 慢 循环降一档 */
int speedStepDown(void) {
  int cur = speedLevel;
  /* 自定义档位时从慢速重新开始，保证按键一定能看到变化 */
  if (cur < SPEED_SLOW || cur > SPEED_FAST) cur = SPEED_FAST;
  return adjustSpeed((cur + 1) % 3);
}

/* ---------- 末端舵机 (angle4 / f) ---------- */

/* 设置末端舵机角度。
 * 反解 getAngle() 不会改动 angle4，所以它只能由这里（或直接改 Pos.ser.angle4）
 * 单独设置。这里会把角度夹在 servoLimit 的 f 行程内，保证机械不顶死。
 * 返回 true 表示角度确实变了（调用方据此决定是否发串口提示、是否刷新时间门控）。 */
bool posSetAngle4(double angleDeg) {
  if (isnan(angleDeg) || isinf(angleDeg)) return false;

  double v = clampDouble(angleDeg, servoLimit.minF, servoLimit.maxF);
  if (v == Pos.ser.angle4) return false;
  Pos.ser.angle4 = v;
  return true;
}

/* 末端张开：朝 f 的行程上限方向走一步。
 * stepDeg <= 0 时用默认步长 5 度，方便串口单条命令直接调用。 */
bool posToolOpen(double stepDeg) {
  if (stepDeg <= 0) stepDeg = 5.0;
  return posSetAngle4(Pos.ser.angle4 + stepDeg);
}

/* 末端收回：朝 f 的行程下限方向走一步。 */
bool posToolClose(double stepDeg) {
  if (stepDeg <= 0) stepDeg = 5.0;
  return posSetAngle4(Pos.ser.angle4 - stepDeg);
}
