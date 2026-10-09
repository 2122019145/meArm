//
// constant_and_positions.cpp
// 机械臂核心实现：运动学反解、范围边界、全局调速
//
#include "Arduino.h"   /* 提供 Serial / F() 宏，以及 isnan / isinf */
#include "constant_and_positions.h"

/* ---------- 编译开关 ---------- */
/* 置 1: 打开调试串口输出（波特率由 serial_protocol 模块初始化）。
 * 本开关需与 joystick_control.cpp 中的同名开关保持一致，否则串口输出会缺失。 */
#include "weArm_config.h"

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

/* 全局调速默认值：中速（1.0 度/格、固定 40ms 一格 = 25°/s） */
struct speedCfg speed = {
  .stepSize    = 1.0,
  .stepDelayMs = 40
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

/* 关节越界提示的限流时间戳（避免持续越限把串口刷爆）。
 * 这条提示只在 WEARM_DEBUG_SERIAL 打开时才有意义，所以连变量一起裁掉，
 * 免得关掉调试后它变成"定义了但没人用"的告警源。 */
#if WEARM_DEBUG_SERIAL
static unsigned long lastServoLogTime = 0;
#endif

/* 当前档位。-1 = 自定义(由 setSpeed 直接写入)，否则为 SPEED_* 之一 */
static int speedLevel = SPEED_NORMAL;

/* ---------- 内部小工具 ---------- */

/* True for every finite value (NaN and +-Inf rejected).
 * "v - v == 0" is bit-exact equivalent to "!isnan(v) && !isinf(v)": a finite
 * value minus itself is +0.0, while NaN/Inf minus itself is NaN, which compares
 * unequal to 0.0. The build uses -Os -flto without -ffast-math /
 * -ffinite-math-only, so the compiler may not fold v - v away.
 * Kept out-of-line on purpose: inlining it at all ~13 call sites costs more
 * flash than one tiny shared function plus a call. */
static bool __attribute__((noinline)) isFiniteNum(double v) {
  return (v - v) == 0.0;
}

/* 把一个 double 限定到 [lo, hi]，且 lo/hi 写反时自动交换。
 * Kept out-of-line on purpose: it is used by three different loops
 * (clampServoAngles / clampToRange / posSetAngle4) and inlining the
 * "swap the bounds, then two comparisons" sequence in each of them costs
 * more flash than one shared copy plus a call. */
static double __attribute__((noinline)) clampDouble(double v, double lo, double hi) {
  if (lo > hi) { double t = lo; lo = hi; hi = t; }
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* ---------- 关节硬限位 ---------- */

/* The four joints are stored as eight consecutive doubles inside servoLimitCfg
 * (minB,maxB,minR,maxR,minC,maxC,minF,maxF). All members have the same type and
 * alignment, so the struct has no padding and the four (min,max) pairs can be
 * walked with one pointer instead of a four-entry mapping table.
 *
 * This is also why the old jointMin[]/jointMax[] mirror tables are gone: reading
 * servoLimit directly yields exactly the values those mirrors were refreshed to
 * (servoSelfCheck normalizes servoLimit in place and nothing else writes it),
 * while the mirrors cost 72 bytes of RAM plus a refresh loop in every call. */
#define JOINT_PAIR(i) (&servoLimit.minB + 2 * (i))

/* 单字符关节名，仅用于串口提示（调试关闭时整块被裁掉） */
#if WEARM_DEBUG_SERIAL
static const char jointName[4] PROGMEM = { 'b', 'r', 'c', 'f' };
#endif

/* 修正写反或超出 0~180 的关节限位，返回被修正的项数。
 * 舵机物理行程只有 0~180°，所以越界的限位本身也是配置错误。
 * Sorting and clamping each joint in turn gives exactly the same final values
 * and the same corrected-item count as the old "sort all four, then clamp all
 * four" two-pass version, because the four joints are independent. */
int servoSelfCheck(void) {
  int bad = 0;
  double *p = &servoLimit.minB;
  for (int i = 0; i < 4; i++, p += 2) {
    if (p[0] > p[1]) { double t = p[0]; p[0] = p[1]; p[1] = t; bad++; }
    if (p[0] < 0.0)   { p[0] = 0.0;   bad++; }
    if (p[1] > 180.0) { p[1] = 180.0; bad++; }
  }
  if (bad > 0) {
    WEARM_LOG("[servo] ERROR: joint limit config invalid, auto-corrected");
  }
  return bad;
}

/* The four SER angle fields are also consecutive doubles (angle1..angle4), so a
 * single pointer walks them in joint order -- the old serAnglePtr() switch and
 * the jointOk() wrapper are gone; the explicit NULL checks on the result could
 * never fire because the switch always returned a valid member. */

/* 判断四个关节角是否都在限位内（容差 ANGLE_EPS 度）。 */
/* Kept out-of-line: several modules call it, and duplicating the 4-joint scan
 * in every caller costs more flash than one shared copy plus a call. */
bool __attribute__((noinline)) isServoInRange(const SER *ser) {
  if (ser == NULL) return false;
  const double *ap  = &ser->angle1;
  const double *lim = JOINT_PAIR(0);
  for (int i = 0; i < 4; i++, ap++, lim += 2) {
    if (!(*ap >= lim[0] - ANGLE_EPS && *ap <= lim[1] + ANGLE_EPS)) return false;
  }
  return true;
}

/* 【为什么把两个钳制循环合成一份实现】
 * clampServoAngles（四个关节，走 servoLimit 的 8 个连续 double）和
 * clampToRange（三个轴，走 limit 的 6 个连续 double）原本各写了一遍
 * "取一对 (min,max) -> clampDouble -> 变了才写回"的循环，两份机器码几乎逐字相同，
 * 只差循环次数。这里把循环体收进 clampPairRun()，两个对外函数的
 * 顺序（关节 b,r,c,f / 轴 x,y,z）、容差、返回语义完全不变。 */
static bool __attribute__((noinline)) clampPairRun(double *v, const double *lim, int n) {
  bool changed = false;
  for (int i = 0; i < n; i++, v++, lim += 2) {
    double c = clampDouble(*v, lim[0], lim[1]);
    if (c != *v) { *v = c; changed = true; }
  }
  return changed;
}

bool clampServoAngles(SER *ser) {
  if (ser == NULL) return false;
  return clampPairRun(&ser->angle1, JOINT_PAIR(0), 4);
}

/* 【applyJointLimits 为什么被删掉】
 * 原来这里有一个 static applyJointLimits(ser, clamped)：逐个扫描四个关节，
 * 命中第一个越限关节后按 servoLimitMode 决定"拒绝"还是"四关节一起吸附"。
 * 它唯一的调用点在 getAngleEx 的末尾（全固件没有第二个调用点）。
 *
 * 但反解在调用它之前，已经用**逐字相同**的容差表达式
 *     rDeg >= minR - EPS  && rDeg <= maxR + EPS      （EPS = 1e-9）
 * 把 angle1..angle3 筛进了 [min - 1e-9, max + 1e-9]（c 同理），随后那三个角
 * 只多了一次"微小负角归零"，而归零只会把值推向区间**内部**：
 *   能让负角通过 rDeg >= minR - 1e-9 的只可能是 minR < 1e-9，
 *   此时归零后的 0 仍然 >= minR - 1e-9；
 *   上界方向 maxR >= minR > 原值，0 更不可能越界。
 * 所以那个四关节扫描在这一步**只可能命中 angle4** —— 一个从反解里原样带过来、
 * 本函数从不修改的关节。这段扫描是纯冗余：删掉后
 *   · REJECT 策略：只有 f 越限才返回 false，与"扫到第一个越限关节"等价；
 *   · CLAMP 策略：f 越限时照样调用 clampServoAngles()，
 *     它**仍然是四个关节一起吸附**，所以端点上的 b/r/c 该被吸附的依旧被吸附。
 * （那段 500ms 限流的串口提示也一并搬到 getAngleEx 里，文本逐字不变，
 *  只是关节名固定是 jointName[3]='f'、限位固定取 minF/maxF。） */
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

/* 将位置钳制到配置的范围内，若有轴越界则返回 true。
 * 三个轴各自独立钳制，不会因为一个轴越界而影响其它轴。 */
bool clampToRange(pos *pos1) {
  if (pos1 == NULL) return false;
  /* rec 的 x/y/z 与 limit 的 (min,max) 对都是连续存放的，一个指针就能走完三个轴。
   * 处理顺序仍是 x -> y -> z，每个轴独立钳制、独立比较。 */
  return clampPairRun(&pos1->rec.x, &limit.minX, 3);
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

  double zv = rec->z - arm1.armheight;
  double R = sqrt(rec->x * rec->x + rec->y * rec->y + zv * zv);

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
/* 【为什么把平面正解抽成一份 noinline 实现】
 * 同一组公式（x = L1·cosα + L2·cosβ，z = L1·sinα + L2·sinβ）在本文件里出现了两次：
 *   · recFromServo：由四个舵机角算末端坐标（alpha = r/RADtoDEG, beta = (r-c)/RADtoDEG）；
 *   · getAngleEx 内联的 ikBranch：由反解出的 alpha/beta 回代核对残差。
 * 两次各是"4 次 cos/sin + 4 次乘法 + 2 次加法"共约 160 字节机器码，且逐字相同。
 * 合成这一份后两个调用点共用同一段代码：
 *   · 表达式与求值顺序逐字不变（同一组乘加，舍入结果完全一致）；
 *   · L1/L2 每次现读 arm1（与两处原来的读法一致，调用期间没人会改 arm1）；
 *   · 出参走指针，调用方仍先落到自己的局部变量再写回结构体，
 *     所以 rec 与 ser 指向同一结构体（pos）时也照旧安全。
 * noinline 是刻意的：被内联回两处就退化成原来那两份重复机器码了。 */
static void __attribute__((noinline)) fkPlanar(double alpha, double beta,
                                               double *outX, double *outZ) {
  const double L1 = arm1.armLength1;
  const double L2 = arm1.armLength2;
  *outX = L1 * cos(alpha) + L2 * cos(beta);
  *outZ = L1 * sin(alpha) + L2 * sin(beta);
}

bool recFromServo(REC *rec, const SER *ser) {
  if (rec == NULL || ser == NULL) return false;

  double b = ser->angle1;
  double r = ser->angle2;
  double c = ser->angle3;
  if (!isFiniteNum(b) || !isFiniteNum(r) || !isFiniteNum(c)) return false;

  double alpha = r / RADtoDEG;          /* 上臂方向角（弧度） */
  double beta  = (r - c) / RADtoDEG;    /* 下臂方向角 = r - c */
  double xPlanar;
  double z;
  fkPlanar(alpha, beta, &xPlanar, &z);  /* 与反解里的回代共用同一份正解 */
  z += arm1.armheight;

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
 * 需要区分"精确到达目标"与"被限位挡住"的调用方（moveToPoint / 绘图轨迹校验）用这个版本。 */
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
/* 平面分支的正解残差。
 * delta 本身就是舵机角 c（弧度），所以调用者只需要 beta（下臂绝对方向角）与
 * 残差：alpha 仍按 beta + delta 现算，而 *alpha / *cServo 两个出参原来回传的
 * 都是调用者手里已经有的值（alpha = beta + delta，cServo = delta）。 */
static void ikBranch(double rhoP, double zv, double delta, double L1, double L2,
                     double *beta, double *err) {
  double phiBase = atan2(zv, rhoP);                  /* 末端在平面内的方向角 */
  double argW    = atan2(L2 * sin(delta), L1 + L2 * cos(delta));
  *beta = phiBase - argW;
  double alpha = *beta + delta;
  double fx, fz;
  fkPlanar(alpha, *beta, &fx, &fz);                  /* 与 recFromServo 共用同一份正解 */
  double dx = fx - rhoP, dz = fz - zv;
  *err = sqrt(dx * dx + dz * dz);                    /* 原来是 pow(dx,2)+pow(dz,2) */
}

/* 【回转角归一化】90 - RADtoDEG*atan2() 的值域是 [-90,270]，但两种浮点精度各有一个
 * 收尾的坑，所以三句都要留着：
 *   · 32 位 float（固件）：-1e-10 + 360 会被舍入成整 360.0，靠 >= 360 那句折回 0；
 *   · 64 位 double（PC 端自检）：同样算出来是 359.9999999998，>= 360 命中不了，
 *     靠最后那句 360-1e-6 折回 0（实测 (0,20,20) 的目标正好踩这个坑）。
 * 原来 bFwd/bRev 各写了一遍这三句，这里合成一份 noinline 实现给两个值共用，
 * 表达式、顺序、常量逐字不变。 */
static double __attribute__((noinline)) normRevAngle(double b) {
  if (b < 0.0)    b += 360.0;
  if (b >= 360.0) b -= 360.0;
  if (b > 360.0 - 1e-6) b = 0.0;
  return b;
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
  double rho = sqrt(pos1->rec.x * pos1->rec.x + pos1->rec.y * pos1->rec.y);
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

  /* 容差 1e-9 度：远小于舵机可分辨的 0.1°，只用来吸收 acos/atan2 的舍入噪声。
   * 原来分成 BASE_EPS / JOINT_EPS 两个同名常量，值相同，合并成一个即可。 */
  const double EPS = 1e-9;

  /* 限位值取一次：servoLimit 在本函数内不会被改动 */
  double minB = servoLimit.minB, maxB = servoLimit.maxB;
  double minR = servoLimit.minR, maxR = servoLimit.maxR;
  double minC = servoLimit.minC, maxC = servoLimit.maxC;

  double bestB = 0.0, bestR = 0.0, bestC = 0.0, bestErr = 1e30;
  int pick = -1;

  /* 回转角只跟"平面朝前/反折"有关，跟肘部镜像无关，所以两个分支各算一次即可
   * （原来在循环里对四个分支各算一次，其中两个是重复的）。
   * 表达式与原来逐字相同，结果逐位一致。
   * 正向平面用 (x,y)，反向平面用 (-x,-y)；rho 极小时方向无意义，取中立位 90°。
   * 90 - RADtoDEG*atan2() 的值域是 [-90,270]，所以 +=/-= 360 各最多发生一次。 */
  double bFwd = 90.0, bRev = 90.0;
  if (rho > 1e-9) {
    bFwd = 90.0 - RADtoDEG * atan2( pos1->rec.y,  pos1->rec.x);
    bRev = 90.0 - RADtoDEG * atan2(-pos1->rec.y, -pos1->rec.x);
  }
  /* 两个值走同一套归一化；rho 极小时给的默认 90.0 经过它原样返回 */
  bFwd = normRevAngle(bFwd);
  bRev = normRevAngle(bRev);

  for (int i = 0; i < 4; i++) {
    const bool reversePlane = (i >= 2);              /* ② 平面朝前 / 反折 */
    const double k = ((i & 1) == 0) ? 1.0 : -1.0;    /* ① 肘部在上 / 在下 */
    const double rhoP  = reversePlane ? -rho : rho;
    const double delta = k * cAbs;                   /* = 舵机角 c（弧度） */
    /* 【归一化后的回折】90 - RADtoDEG*atan2() 的浮点误差会把"正好 0°"算成
     * -2.4e-10，上面那句 += 360 于是把它变成 359.9999999998，
     * 再和 maxB = 180 一比就把这个分支丢掉了 ——
     * 实测 (x=0,y=20,z=20) 的正确回转角恰好是 0°（b=0 朝 +y），就踩在这个坑里，
     * 表现为反解返回 false 且 Pos.ser 停在 (0,0,0)。
     * 因此把"贴着 360°"的值折回 0°，容差远大于浮点噪声、远小于 1° 步进。 */
    const double b = reversePlane ? bRev : bFwd;
    
    if (b < minB - EPS || b > maxB + EPS) continue;

    double beta, err;
    ikBranch(rhoP, zv, delta, L1, L2, &beta, &err);
    /* rDeg 按 beta + delta 现算，与原 alpha = beta + delta 逐位一致。
     * alpha/beta 的"是否有限"检查由下面的正向区间判断覆盖：NaN/Inf 一样 continue。 */
    double rDeg = RADtoDEG * (beta + delta);
    double cDeg = RADtoDEG * delta;
    /* r、c 必须落在各自行程内 —— 必须在进 applyJointLimits 之前显式筛掉，
     * 否则 CLAMP 会把越限的候选静默吸附成"看起来能用"的解。 */
    if (!(rDeg >= minR - EPS && rDeg <= maxR + EPS)) continue;
    if (!(cDeg >= minC - EPS && cDeg <= maxC + EPS)) continue;
    if (!isFiniteNum(err)) continue;

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

  /* 到这里 angle1/2/3 必然都是有限数：bestB 来自 atan2（有限），
   * bestR/bestC 已通过上面的区间判断。原来这里还有一次 isFiniteNum 兜底，
   * 那是不可能走到的死代码（与调试开关无关），删掉不改变可观察行为。 */
  double angle1 = bestB;
  double angle2 = bestR;                             /* r = 上臂绝对方向角 */
  double angle3 = bestC;                             /* c = alpha - beta（舵机角） */

  /* 容差内的微小负角归零，避免 -0.0 这种值被 (int) 截断后传给舵机。
   * 【为什么只归零 r 和 c】b 在赋值前已经过归一化：
   *   b < 0        -> b += 360   （落到 [270,360)）
   *   b >= 360     -> b -= 360   （把 360-1e-10 这种恰好在 32 位浮点上
   *                               取整成 360 的值折回 0）
   *   b > 360-1e-6 -> b = 0      （64 位 double 下 +=360 保留的小尾巴）
   * 走完这三步的 b 只可能落在 [0, 270] ∪ (269.x, 360) ∪ {0}，
   * 永远不可能落在 (-1e-6, 0) —— 原来那句 angle1 归零是不可能命中的死代码。 */
  if (angle2 > -ANGLE_EPS && angle2 < 0) angle2 = 0;
  if (angle3 > -ANGLE_EPS && angle3 < 0) angle3 = 0;

  /* 【先判 f 再写回】本函数的契约是"返回 false 时不修改 pos1->ser，
   * 调用方保留上一个有效姿态"。原来的写法是先把 angle1..3 写进 ser，
   * 再交给 applyJointLimits（见上面的说明，它只可能命中 f），
   * 一旦 REJECT 就得把四个角全部回滚 —— 老代码就踩过这个坑：
   * 一个反解失败的目标点会把 Pos.ser 留成 (0,0,0)，上电时机械臂直接甩向原点。
   * 把 f 的判定提到写回之前，失败路径一个字节都不碰 ser，
   * 备份/回滚那 4 个 double 也就不用存在了，语义完全相同。 */
  const double LIM_EPS = 1e-9;
  const double a4 = pos1->ser.angle4;
  const bool toolBad = (a4 < servoLimit.minF - LIM_EPS ||
                        a4 > servoLimit.maxF + LIM_EPS);

  if (toolBad && servoLimitMode == SERVO_LIMIT_REJECT) {
    /* 越限提示：500ms 限流，避免持续越限把串口刷爆。
     * 整块（含 now/时间戳读写）都放在调试开关里：Serial.print(double) 会把
     * avr-libc 的浮点格式化整段链进固件（实测约 0.5KB flash），Uno 上不划算。
     * 容差 1e-9 的道理见文件开头：反解在限位端点上会有 -1e-14 量级的舍入误差，
     * 按 1e-6 判会让"正好停在行程端点"的姿态被误报成越限。 */
#if WEARM_DEBUG_SERIAL
    unsigned long now = millis();
    if (now - lastServoLogTime >= 500) {
      lastServoLogTime = now;
      Serial.print(F("[servo] reject joint "));
      Serial.print((char)pgm_read_byte(&jointName[3]));
      Serial.print(F(" = "));
      Serial.print(a4);
      Serial.print(F(" (allow "));
      Serial.print(servoLimit.minF);
      Serial.print('-');
      Serial.print(servoLimit.maxF);
      Serial.println(F(")"));
    }
#endif
    return false;
  }

  pos1->ser.angle1 = angle1;
  pos1->ser.angle2 = angle2;
  pos1->ser.angle3 = angle3;
  /* angle4(末端 f) 不由反解决定，保持调用前已有的值不动 */

  bool limClamped = false;
  if (toolBad) {
    /* CLAMP 策略：四个关节一起吸附到最近限位（与原来调用的同一个函数） */
    clampServoAngles(&pos1->ser);
    limClamped = true;
#if WEARM_DEBUG_SERIAL
    unsigned long now = millis();
    if (now - lastServoLogTime >= 500) {
      lastServoLogTime = now;
      Serial.print(F("[servo] clamp joint "));
      Serial.print((char)pgm_read_byte(&jointName[3]));
      Serial.print(F(" = "));
      Serial.print(a4);
      Serial.print(F(" -> "));
      Serial.println(clampDouble(a4, servoLimit.minF, servoLimit.maxF));
    }
#endif
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
 * 校验规则: stepSize > 0；stepDelayMs > 0。
 * 不合法的项保持原值，只有确实改动了参数才把档位标记为自定义。-1。 */
void setSpeed(double stepSize, int stepDelayMs) {
  bool changed = false;
  if (stepSize > 0 && stepSize != speed.stepSize) {
    speed.stepSize = stepSize; changed = true;
  }
  if (stepDelayMs > 0 && stepDelayMs != speed.stepDelayMs) {
    speed.stepDelayMs = stepDelayMs; changed = true;
  }
  /* 参数被手动改动后，当前档位名已不再代表实际参数 */
  if (changed) speedLevel = -1;
}

/* 按档位调整速度，返回生效档位 (-1 表示档位非法)
 * 三档参数本来就有 2 的幂倍数关系：步长 0.5/1/2 度、固定间隔 80/40/20 ms（慢/中/快），
 * 全部可以由档位精确算出（0.5·2^level 在二进制浮点里是精确的，
 * 整数右移也是精确的），于是三份 setSpeed 调用点收成一份。
 * SPEED_SLOW/NORMAL/FAST 就是 0/1/2，所以 speedLevel = level、return level
 * 与原 switch 里逐条赋值逐位相同。
 * 【实测】逐档 switch 写法整机 Program = 34970 B，本写法 34932 B，
 * 所以即使 adjustSpeed 自身的符号从 112 B 涨到 246 B，整机仍净省 38 B。 */
int adjustSpeed(int level) {
  if (level < SPEED_SLOW || level > SPEED_FAST) return -1;
  setSpeed(0.5 * (1 << level), 80 >> level);
  /* 慢/中/快 → 步长 0.5/1/2 度、固定间隔 80/40/20 ms */
  speedLevel = level;
  return level;
}

int speedGetLevel(void) {
  return speedLevel;
}

/* Kept out-of-line: it is called from several modules, and an inlined copy would
 * carry its own duplicate of the four name literals (flash and RAM). */
const char * __attribute__((noinline)) speedLevelName(int level) {
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
  /* wasnan/isinf pair -> one shared helper (same rejection set). */
  if (!isFiniteNum(angleDeg)) return false;

  double v = clampDouble(angleDeg, servoLimit.minF, servoLimit.maxF);
  if (v == Pos.ser.angle4) return false;
  Pos.ser.angle4 = v;
  return true;
}
