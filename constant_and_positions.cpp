//
// constant_and_positions.cpp
// 机械臂核心实现：运动学反解、几何可达性、全局调速（精简版）
//
#include "Arduino.h"   /* 提供 Serial / F() 宏，以及 isnan / isinf */
#include "constant_and_positions.h"

/* ---------- 编译开关 ---------- */
/* 置 1: 打开调试串口输出（波特率由 serial_protocol 模块初始化）。
 * 本开关需与 joystick_control.cpp 中的同名开关保持一致，否则串口输出会缺失。 */
#include "weArm_config.h"

#if WEARM_DEBUG_SERIAL
  #define WEARM_LOG(msg)   Serial.println(F(msg))
#else
  #define WEARM_LOG(msg)   ((void)0)
#endif

/* 复位目标点 = **上电初始位姿**（用户标定：初始状态下三个位姿舵机的角度必须是
 * x90, y90, z90，即 (b,r,c) = (90,90,90)；旧版 v1.6.5 的注释也是这么写的）。
 * 修正模型（alpha = 90 - r、beta = c - 90）下这个姿态对应末端 (40, 0, 0)：
 * alpha = beta = theta = 0，两节都水平朝前 —— 完全伸直的姿态，恰好落在可达
 * 外边界 rho = L1 + L2 = 40 上（probe_rt 的 anchor "(40.00, 0.00, 0.00) PASS"）。
 * 落在边界上的代价只有一条：从这一点**无法再向外伸**（rho 已经取到最大），
 * 因此"就地垂直抬升"也无解 —— rho = 40 只在 z = 0 成立。取放模块为此改成
 * 起点不可达就直接插值到"物体正上方(飞行高度)"，见 pick_place.cpp 的 s_riseAtSrc。
 * x = 40 同时在 limit 盒子的边界上，判据是闭区间，仍算"在盒内"。
 * 【历史】曾改成 (20,0,-10)（反解 b=90、r=60.55、c=7.43 的折叠位姿）以避开边界，
 * 但那会让上电姿态偏离 x90,y90,z90，已按用户要求改回。 */
static const REC POS_HOME = { 40, 0, 0 };

/* 机械臂几何参数（上臂长 / 下臂长 / 末端伸出量） */
struct arm arm1 = { 20, 20, 0 };

/* 全局位姿实例（头文件里 Pos 是 extern 声明，定义必须放这里） */
pos Pos = { { 0, 0, 0, 0 }, { POS_HOME.x, POS_HOME.y, POS_HOME.z } };

/* 末端笛卡尔坐标范围（"工作空间盒子"，见头文件里的用法说明）。
 * 【本版】三轴统一 ±40。这只是给外部的**粗坐标范围**（画布/取放点的合法性判据），
 * 不是真实包络：修正小臂模型为 beta = c - 90 后 r、c 各 0~180，使
 * alpha、beta 都落在 [-90,90]，两节 cos 都 ≥ 0，所以真实可达集在平面上是
 * x ≥ 0 的一枚"透镜"（两杆夹角由余弦定理定），不是圆盘、更不是球。
 * 盒子比真实包络大一圈是有意的：真正的可达性判据是 getAngleEx() 的回代残差。 */
struct rangeLimit limit = {
  .minX = -40.0, .maxX = 40.0,
  .minY = -40.0, .maxY = 40.0,
  .minZ = -40.0, .maxZ = 40.0
};

/* 全局调速默认值：中速（1.0 度/格、固定 40ms 一格 = 25°/s） */
struct speedCfg speed = {
  .stepSize    = 1.0,
  .stepDelayMs = 40
};

/* 当前档位。-1 = 自定义，否则为 SPEED_* 之一 */
static int speedLevel = SPEED_NORMAL;

/* ---------- 内部小工具 ---------- */

/* True for every finite value (NaN / ±Inf rejected)。 */
static bool __attribute__((noinline)) isFiniteNum(double v) {
  return (v - v) == 0.0;
}

/* 把一个 double 限定到 [lo, hi]，且 lo/hi 写反时自动交换。 */
static double __attribute__((noinline)) clampDouble(double v, double lo, double hi) {
  if (lo > hi) { double t = lo; lo = hi; hi = t; }
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* ---------- 可达性（圆柱工作空间模型） ----------
 *
 * 【与旧版的区别】旧版是球壳：末端到肩关节的三维距离
 *     R = sqrt(x² + y² + z²)
 * 必须落在 [|L1-L2|, L1+L2] = [0, 40] 内。
 * 新版改为"以肩关节竖直轴为中心、半径 40 的圆柱"：
 *     sqrt(x² + y²)  <=  L1 + L2 = 40   （水平边界）
 * 【本版本再删一刀】竖向边界 |z - armheight| <= L1+L2 也已按需求删除：
 * z 方向不再单独设卡，能不能到由 getAngleEx() 的残差判据与路径校验决定。
 *
 * 【为什么是圆柱】"末端到肩关节的距离不会超过两杆之和"对任何姿位都成立，是一条
 * 与模型无关的必要条件，所以"水平半径 ≤ 40 的圆柱"是真实可达集的外包围盒。
 *   - 真实可达集内的点必然满足水平半径 ≤ R ≤ 40，**不会误杀**；
 *   - 圆柱内、真实包络外的点（例如 (35, 0, 25)：R=43 > 40），isReachable 会放行，
 *     但下游 getAngleEx() 用余弦定理反解时仍会因为 R > 40 被拒（残差检查）。
 *     所以这一改对最终行为影响很小，主要作用是"圆柱内的点不再被提前拒掉，
 *     让反解去说能不能到"。
 *
 * 【与反解的分工】本函数只做粗粒度的工作空间外边界检查，
 * 用来尽早拒掉明显不可达的点；能不能真正反解出 b/r/c 由
 * getAngleEx() 判定。圆柱内的奇异点（例如 (0,0,40) 附近）仍可能
 * 被反解拒掉，这是几何事实，不是 bug。
 *
 * 【调用方无需改动】签名与返回值语义不变（true = 大致可达），
 * path_core.h / pick_place.cpp / draw_control.cpp 直接复用。 */
bool isReachable(const REC *rec) {
  if (rec == NULL) return false;
  if (!isFiniteNum(rec->x) || !isFiniteNum(rec->y) || !isFiniteNum(rec->z)) return false;

  double L1 = arm1.armLength1;
  double L2 = arm1.armLength2;
  if (L1 <= 0 || L2 <= 0) return false;

  /* 水平半径 = 臂展 = L1 + L2。竖向不再设卡（见上面的说明）。 */
  const double RHO_MAX = L1 + L2;
  const double GEOM_EPS = 1e-9;

  /* 水平半径检查（用平方比较，省一次 sqrt） */
  double rho2 = rec->x * rec->x + rec->y * rec->y;
  if (rho2 > RHO_MAX * RHO_MAX + GEOM_EPS) return false;

  return true;
}

/* 平面正解（两杆模型）：
 *   x = L1·cos(alpha) + L2·cos(beta)
 *   z = L1·sin(alpha) + L2·sin(beta)
 * 与反解残差回代共用。 */
static void __attribute__((noinline)) fkPlanar(double alpha, double beta,
                                               double *outX, double *outZ) {
  const double L1 = arm1.armLength1;
  const double L2 = arm1.armLength2;
  *outX = L1 * cos(alpha) + L2 * cos(beta);
  *outZ = L1 * sin(alpha) + L2 * sin(beta);
}

/* 正运动学：四个舵机角度 -> 末端笛卡尔坐标。
 *   alpha = 90 - r     （上臂：r=0 竖直朝上、r=90 水平朝前、r=180 竖直朝下）
 *   beta  = c - 90     （下臂绝对角：c=0 竖直朝下、c=90 水平朝前、c=180 竖直朝上）
 *   theta = 90 - b
 * f 只转末端夹具，不影响被控点。
 *
 * 【两轴方向相反：肩轴反转、肘轴正转】依据 RC-robot-hand 参考工程（与真机同型号）
 * 里记录的真实任务位姿：提臂只改 F(D7) 且 F↑=末端上升（COIN_A 提起 F 20→80、
 * PEN_B 提起 F 34→79），下降只改 F 且 F↓=末端下降 —— 即小臂舵机"角度↑=末端升"，
 * 故 beta = c - 90（c=90 → beta=0 水平朝前）。上臂 R(D8) 相反：descend 时 R↑
 * （COIN 93→107、OBJECT 78→95→102），lift 时 R↓（OBJECT 102→72）—— 即
 * "角度↑=末端降"，故 alpha = 90 - r。旧式子 beta = c - 180 把"下降"反解成
 * "上升"，正是取放过程中上下方向搞反的根因（详见 .selfcheck/README.md）。 */
bool recFromServo(REC *rec, const SER *ser) {
  if (rec == NULL || ser == NULL) return false;

  double b = ser->angle1;
  double r = ser->angle2;
  double c = ser->angle3;
  if (!isFiniteNum(b) || !isFiniteNum(r) || !isFiniteNum(c)) return false;

  double alpha = (90.0 - r) / RADtoDEG;
  double beta  = (c - 90.0) / RADtoDEG;
  double xPlanar;
  double z;
  fkPlanar(alpha, beta, &xPlanar, &z);
  z += arm1.armheight;

  double theta = (90.0 - b) / RADtoDEG;
  double x = xPlanar * cos(theta);
  double y = xPlanar * sin(theta);
  if (!isFiniteNum(x) || !isFiniteNum(y) || !isFiniteNum(z)) return false;

  rec->x = x;
  rec->y = y;
  rec->z = z;
  return true;
}

/* 反解一个平面分支并计算残差。
 * delta = alpha - beta 是两杆的**几何夹角**（±acos，弧度），不是某个舵机角；
 * 舵机角由调用方按 alpha = 90 - r、beta = c - 90 换算。 */
static void ikBranch(double rhoP, double zv, double delta, double L1, double L2,
                     double *beta, double *err) {
  double phiBase = atan2(zv, rhoP);
  /* 【勘误】这里原先是 atan2(L2*sin(d), L1 + L2*cos(d))，把两杆的角色写反了。
   * 正解是 R = e^{i·beta}·(L1·e^{i·delta} + L2)，所以
   *   beta = phiBase - atan2(L1·sin(delta), L2 + L1·cos(delta))
   * 只有当 L1 == L2 时两种写法的 atan2 结果才恰好互补
   * （atan2(sin, 1+cos) 恒等于 delta/2），所以本工程 arm1 = {20,20,0}
   * 一直没暴露；一旦按 constant_and_positions.h 的说明把某一节做长，
   * 四个候选的残差全都会超限、getAngleEx() 恒返回 false（手臂卡死）。
   * 依据：L2==L1 时两式恒等，故本修正不改变当前任何一条已验行为。 */
  double argW    = atan2(L1 * sin(delta), L2 + L1 * cos(delta));
  *beta = phiBase - argW;
  double alpha = *beta + delta;
  double fx, fz;
  fkPlanar(alpha, *beta, &fx, &fz);
  double dx = fx - rhoP, dz = fz - zv;
  *err = sqrt(dx * dx + dz * dz);
}

/* 回转角归一化：90 - RADtoDEG*atan2() 值域 [-90,270]，
 * 需要把贴 360 的值折回 0。 */
static double __attribute__((noinline)) normRevAngle(double b) {
  if (b < 0.0)    b += 360.0;
  if (b >= 360.0) b -= 360.0;
  if (b > 360.0 - 1e-6) b = 0.0;
  return b;
}

bool getAngle(pos *pos1) {
  return getAngleEx(pos1, NULL);
}

bool getAngleEx(pos *pos1, bool *clamped) {
  if (clamped != NULL) *clamped = false;
  if (pos1 == NULL) return false;

  if (!isReachable(&pos1->rec)) {
    WEARM_LOG("[kin] reject: target out of reachable workspace");
    return false;
  }

  const double L1 = arm1.armLength1;
  const double L2 = arm1.armLength2;

  double rho = sqrt(pos1->rec.x * pos1->rec.x + pos1->rec.y * pos1->rec.y);
  double zv  = pos1->rec.z - arm1.armheight;
  double R2  = rho * rho + zv * zv;

  /* 两杆几何夹角 |alpha - beta| 由余弦定理唯一确定 */
  double cosC = (R2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  if (cosC >  1.0) cosC =  1.0;
  if (cosC < -1.0) cosC = -1.0;
  double cAbs = acos(cosC);

#ifdef WEARM_IK_TRACE
  Serial.print(F("[ik] tgt rho=")); Serial.print(rho, 6);
  Serial.print(F(" zv=")); Serial.print(zv, 6);
  Serial.print(F(" R2=")); Serial.print(R2, 6);
  Serial.print(F(" cosC=")); Serial.print(cosC, 12);
  Serial.print(F(" cAbs=")); Serial.println(cAbs, 12);
#endif

  /* 行程判据的容差：必须显著大于"弧度<->角度换算"的舍入误差
   * （~180 * 1e-16 ≈ 2e-14），否则恰好落在行程端点（0/180）的解会被误杀。
   * 统一用 ANGLE_EPS(1e-6)，与后面负角归零的口径一致。 */
  const double EPS = ANGLE_EPS;

  /* 【硬编码的关节行程】b/r/c 三轴物理行程都是 0~180。
   * 原版本从 servoLimit 结构体读取；本版本已删除该结构体，改为本地常量。
   * f 不再参与反解判定（60~150 的硬限位已删除，只有 posSetAngle4() 里的
   * 舵机电气行程 0~180）。 */
  const double minB = 0.0, maxB = 180.0;
  const double minR = 0.0, maxR = 180.0;
  const double minC = 0.0, maxC = 180.0;

  /* 回转角只跟"平面朝前/反折"有关，两个平面各算一次 */
  double bFwd = 90.0, bRev = 90.0;
  if (rho > 1e-9) {
    bFwd = 90.0 - RADtoDEG * atan2( pos1->rec.y,  pos1->rec.x);
    bRev = 90.0 - RADtoDEG * atan2(-pos1->rec.y, -pos1->rec.x);
  }
  bFwd = normRevAngle(bFwd);
  bRev = normRevAngle(bRev);

  double bestB = 0.0, bestR = 0.0, bestC = 0.0, bestErr = 1e30;
  int bestPrio = 99;
  int pick = -1;

  /* 四个候选分支，**按优先级**挑：δ = +cAbs 的正/反平面，然后才轮到 δ = -cAbs 的正/反平面。
   *
   * 【为什么不是"四个分支取残差最小"】两杆平面机构对同一个点本来就有两个镜像解
   * （delta = +cAbs 与 -cAbs），在这个模型下两个都常常能落进 r/c 的 0~180 行程、
   * 残差又都是 0，于是"取残差最小"会靠浮点尾数随机挑一个 —— 同一条直线上相邻
   * 采样点来回换分支，关节角一次跳 60~80 度，取放/绘图的"段内不许换分支"校验必然
   * 失败（修复前的固件里 c 就是 delta、且受 c ∈ [0,180] 限制，等效于只允许 +cAbs
   * 这一族，所以历史上没有暴露过）。
   *
   * 【为什么还要保留 -cAbs 这一族】有些点只有镜像解落在行程里：只留 +cAbs 时
   * probe_rt 的往返验证会多出 13% 的"反解 false"（2028 组里 273 组），绘图的
   * 轨迹校验也会被这些点整条否掉。所以策略是"**优先级**"而不是"只留一族"：
   * 先取 +cAbs 一族（与修复前逐位一致，保证段内不跳），只有它一个都不合格时才
   * 退到 -cAbs（保证仍然解得出来）。同一族里正/反平面取残差小的。
   * 注意候选必须自己满足 err <= 0.05 才算合格，否则"在行程内但解是错的"会被选中。 */
  for (int i = 0; i < 4; i++) {
    const bool reversePlane = (i & 1) != 0;
    const bool plusFamily   = (i < 2);
    const double rhoP  = reversePlane ? -rho : rho;
    const double delta = plusFamily ? cAbs : -cAbs;
    const int    prio  = plusFamily ? 0 : 1;
    const double b = reversePlane ? bRev : bFwd;

    /* 舵机 b 硬限位 0~180 */
    if (b < minB - EPS || b > maxB + EPS) {
#ifdef WEARM_IK_TRACE
      Serial.print(F("[ik] cand ")); Serial.print(i); Serial.println(F(" reject: b out of travel"));
#endif
      continue;
    }

    double beta, err;
    ikBranch(rhoP, zv, delta, L1, L2, &beta, &err);

    /* 平面几何角 -> 舵机角（正解 recFromServo 的逆）：
     *   alpha = 90 - r   =>   r = 90 - alpha   （alpha = beta + delta）
     *   beta  = c - 90   =>   c = beta + 90                          */
    const double alphaDeg = RADtoDEG * (beta + delta);
    const double betaDeg  = RADtoDEG * beta;
    double rDeg = 90.0 - alphaDeg;
    double cDeg = betaDeg + 90.0;

    /* 【为什么要把 r/c 折回基本区间】alpha - beta 是两杆的几何夹角，正解里周期是
     * 360°，而本模型的 alpha - beta = 180 - r - c 取值跨到 [-180, 180] 这一侧。
     * atan2/asin 分支给出的 betaDeg 可能落到 [0,360) 之外（例如 betaDeg = +180
     * => cDeg = 270，或 betaDeg = -180 => cDeg = -90），它们与折回后的值对应同一
     * 个位姿；不折回来就会有一个分支被 0~180 的行程判据直接否掉（r 同理，可能是
     * 360 的整数倍偏移）。normRevAngle() 会把贴 360 的值折回 0、把负值折进
     * [0,360)。 */
    rDeg = normRevAngle(rDeg);
    cDeg = normRevAngle(cDeg);

#ifdef WEARM_IK_TRACE
    Serial.print(F("[ik] cand ")); Serial.print(i);
    Serial.print(F(" b=")); Serial.print(b, 6);
    Serial.print(F(" rhoP=")); Serial.print(rhoP, 9);
    Serial.print(F(" delta=")); Serial.print(delta, 12);
    Serial.print(F(" betaDeg=")); Serial.print(betaDeg, 6);
    Serial.print(F(" rDeg=")); Serial.print(rDeg, 6);
    Serial.print(F(" cDeg=")); Serial.print(cDeg, 6);
    Serial.print(F(" err=")); Serial.println(err, 12);
#endif

    /* 舵机 r、c 硬限位 0~180 */
    if (!(rDeg >= minR - EPS && rDeg <= maxR + EPS)) {
#ifdef WEARM_IK_TRACE
      Serial.println(F("[ik] reject: r out of travel"));
#endif
      continue;
    }
    if (!(cDeg >= minC - EPS && cDeg <= maxC + EPS)) {
#ifdef WEARM_IK_TRACE
      Serial.println(F("[ik] reject: c out of travel"));
#endif
      continue;
    }
    if (!isFiniteNum(err)) {
#ifdef WEARM_IK_TRACE
      Serial.println(F("[ik] reject: err not finite"));
#endif
      continue;
    }
    if (!(err <= 0.05)) {
#ifdef WEARM_IK_TRACE
      Serial.println(F("[ik] reject: err too big"));
#endif
      continue;
    }

    if (pick < 0 || prio < bestPrio ||
        (prio == bestPrio && err < bestErr)) {
      pick = i; bestPrio = prio; bestErr = err;
      bestB = b; bestR = rDeg; bestC = cDeg;
    }
  }

  if (pick < 0) {
    WEARM_LOG("[kin] reject: no planar branch fits the joint travel");
    return false;
  }

  /* 正解回代自检 */
  if (!(bestErr <= 0.05)) {
    WEARM_LOG("[kin] reject: residual too large, no valid solution");
    return false;
  }

  /* 【本函数契约：返回 false 时不修改 pos1->ser】走到这里说明 b/r/c 已经有解，
   * 下面一次性写回（bestB/bestR/bestC 是候选循环里挑出的那一组）。
   * angle4(f) 不由反解决定，保持调用前已有的值不动。
   * f 的 60~150 硬限位与"越限就吸附/拒绝"的策略已按需求删除（见
   * constant_and_positions.h 的 TOOL_ANGLE_*），所以这里不再碰 angle4，
   * *clamped 也保持函数开头设的 false。 */
  pos1->ser.angle1 = bestB;
  pos1->ser.angle2 = bestR;
  pos1->ser.angle3 = bestC;

  return true;
}

/* 配置自检：把 limit 里写反的 (min,max) 交换回来，返回被修正的轴数。
 * 盒子写反（例如 minZ = 60、maxZ = -60）会让**每个**点都被判越界，
 * 机械臂表面上"完全不能动"，所以开机当场纠正并留一条调试日志。
 * 与调速档位自检同一风格：只修配置，不碰 Pos。 */
int rangeLimitConfig(void) {
  int bad = 0;

  if (limit.minX > limit.maxX) { double t = limit.minX; limit.minX = limit.maxX; limit.maxX = t; bad++; }
  if (limit.minY > limit.maxY) { double t = limit.minY; limit.minY = limit.maxY; limit.maxY = t; bad++; }
  if (limit.minZ > limit.maxZ) { double t = limit.minZ; limit.minZ = limit.maxZ; limit.maxZ = t; bad++; }

#if WEARM_DEBUG_SERIAL
  if (bad) {
    WEARM_LOG("[limit] ERROR: range min>max, auto-corrected:");
    WEARM_LOG("[limit]   X/Y/Z = [min,max] -> check constant_and_positions.cpp");
  }
#endif
  return bad;
}

/* 复位到初始位姿 POS_HOME（= 上电姿态，三个位姿舵机 x90,y90,z90）。
 * 【本版本恢复】开机先跑 rangeLimitConfig() 对 limit 配置做一次自检；
 * rangeClampConfig()、servoSelfCheck()、clampToRange() 仍是删除状态
 * （servoLimit 那套没恢复，POS_HOME 本身就能反解，不需要夹取）。 */
void posInit(void) {
  (void) rangeLimitConfig();

  Pos.rec.x = POS_HOME.x;
  Pos.rec.y = POS_HOME.y;
  Pos.rec.z = POS_HOME.z;

  /* 末端舵机取名义半开位置 105 度（TOOL_ANGLE_OPEN 与 TOOL_ANGLE_CLOSE 的中点），
   * 开机既不夹紧也不到极限，避免上电瞬间把夹在里面的东西压坏。
   * 逻辑角范围已是 0~180，105 仍在区间内。 */
  Pos.ser.angle4 = (TOOL_ANGLE_OPEN + TOOL_ANGLE_CLOSE) * 0.5;

  if (!getAngle(&Pos)) {
    WEARM_LOG("[pos] ERROR: POS_HOME unreachable");
  }
#if WEARM_DEBUG_SERIAL
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
 * 【本版本已删除 clampToRange() 调用】：坐标范围限制已整体移除，
 * POS_HOME 直接反解即可（它在可达外边界上，属于闭区间内，能解出 90/90/90）。 */
bool posGetHomeAngles(SER *ser) {
  if (ser == NULL) return false;

  pos p;
  p.ser = *ser;                 /* 带上调用者的 angle4 */
  p.rec.x = POS_HOME.x;
  p.rec.y = POS_HOME.y;
  p.rec.z = POS_HOME.z;
  if (!getAngle(&p)) return false;

  ser->angle1 = p.ser.angle1;
  ser->angle2 = p.ser.angle2;
  ser->angle3 = p.ser.angle3;
  return true;
}

/* ---------- 全局调速 ---------- */

void setSpeed(double stepSize, int stepDelayMs) {
  bool changed = false;
  if (stepSize > 0 && stepSize != speed.stepSize) {
    speed.stepSize = stepSize; changed = true;
  }
  if (stepDelayMs > 0 && stepDelayMs != speed.stepDelayMs) {
    speed.stepDelayMs = stepDelayMs; changed = true;
  }
  if (changed) speedLevel = -1;
}

/* 慢/中/快 → 步长 0.5/1/2 度、固定间隔 80/40/20 ms */
int adjustSpeed(int level) {
  if (level < SPEED_SLOW || level > SPEED_FAST) return -1;
  setSpeed(0.5 * (1 << level), 80 >> level);
  speedLevel = level;
  return level;
}

int speedGetLevel(void) {
  return speedLevel;
}

const char * __attribute__((noinline)) speedLevelName(int level) {
  switch (level) {
    case SPEED_SLOW:   return "慢速";
    case SPEED_NORMAL: return "中速";
    case SPEED_FAST:   return "快速";
    default:           return "自定义";
  }
}

/* 快 → 中 → 慢 → 快 循环降一档
 * 【勘误】这里原先是 (cur + 1) % 3，而档位 0/1/2 = 慢/中/快，于是"降一档"实际
 * 上是 慢→中→快→慢，方向搞反了（只有 快→慢 那一跳碰巧是对的）。改成 +2：
 * 快(2)→中(1)→慢(0)→快(2)。本函数当前无调用点（protocol 的调速走 adjustSpeed），
 * 修好是为了让将来接入时语义与函数名一致。 */
int speedStepDown(void) {
  int cur = speedLevel;
  if (cur < SPEED_SLOW || cur > SPEED_FAST) cur = SPEED_FAST;
  return adjustSpeed((cur + 2) % 3);
}

/* ---------- 末端舵机 (angle4 / f) ---------- */

bool posSetAngle4(double angleDeg) {
  if (!isFiniteNum(angleDeg)) return false;

  /* 末端 f：只夹在舵机电气行程 0~180 内。
   * 60~150 的"夹爪范围限制"已删除 —— 正是这条限制让"夹紧"（< 60）写不进去。 */
  double v = clampDouble(angleDeg, TOOL_ANGLE_MIN, TOOL_ANGLE_MAX);
  if (v == Pos.ser.angle4) return false;
  Pos.ser.angle4 = v;
  return true;
}