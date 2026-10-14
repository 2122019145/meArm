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

/* 复位目标点：工作空间内部的安全点。
 * 选 (20,0,20)：
 *   ① 是用户标定的初始位姿 —— 舵机 (b,r,c)=(90,90,90) 时末端正好在这里；
 *   ② 远离边界，开机不会撞限位；
 *   ③ 避免 x=0 的轴线退化点（rho=0 时回转角无法确定）。 */
static const REC POS_HOME = { 20, 0, 20 };

/* 机械臂几何参数（上臂长 / 下臂长 / 末端伸出量） */
struct arm arm1 = { 20, 20, 0 };

/* 全局位姿实例（头文件里 Pos 是 extern 声明，定义必须放这里） */
pos Pos = { { 0, 0, 0, 0 }, { POS_HOME.x, POS_HOME.y, POS_HOME.z } };

/* 全局调速默认值：中速（1.0 度/格、固定 40ms 一格 = 25°/s） */
struct speedCfg speed = {
  .stepSize    = 1.0,
  .stepDelayMs = 40
};

/* 关节越界策略：吸附到最近限位（当前仅对 f 生效） */
int servoLimitMode = SERVO_LIMIT_CLAMP;

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

/* ---------- 运动学 ---------- */

/* 几何可达性：末端到肩的距离 R 必须落在 [|L1-L2|, L1+L2] 内，
 * 否则 acos 会越域。这是反解的前提，不是用户可配的"范围限制"。 */
bool isReachable(const REC *rec) {
  if (rec == NULL) return false;
  if (!isFiniteNum(rec->x) || !isFiniteNum(rec->y) || !isFiniteNum(rec->z)) return false;

  double zv = rec->z - arm1.armheight;
  double R = sqrt(rec->x * rec->x + rec->y * rec->y + zv * zv);

  double L1 = arm1.armLength1;
  double L2 = arm1.armLength2;
  if (L1 <= 0 || L2 <= 0) return false;

  const double GEOM_EPS = 1e-9;
  if (R > L1 + L2 + GEOM_EPS || R < fabs(L1 - L2) - GEOM_EPS) return false;
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
 *   alpha = r, beta = r - c
 *   theta = 90 - b
 * f 只转末端夹具，不影响被控点。 */
bool recFromServo(REC *rec, const SER *ser) {
  if (rec == NULL || ser == NULL) return false;

  double b = ser->angle1;
  double r = ser->angle2;
  double c = ser->angle3;
  if (!isFiniteNum(b) || !isFiniteNum(r) || !isFiniteNum(c)) return false;

  double alpha = r / RADtoDEG;
  double beta  = (r - c) / RADtoDEG;
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
 * delta 本身就是舵机角 c（弧度）。 */
static void ikBranch(double rhoP, double zv, double delta, double L1, double L2,
                     double *beta, double *err) {
  double phiBase = atan2(zv, rhoP);
  double argW    = atan2(L2 * sin(delta), L1 + L2 * cos(delta));
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

  /* 肘角 |c| 由余弦定理唯一确定 */
  double cosC = (R2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);
  if (cosC >  1.0) cosC =  1.0;
  if (cosC < -1.0) cosC = -1.0;
  double cAbs = acos(cosC);

  const double EPS = 1e-9;

  /* 【硬编码的关节行程】b/r/c 三轴物理行程都是 0~180。
   * 原版本从 servoLimit 结构体读取；本版本已删除该结构体，改为本地常量。
   * f 的 60~150 在下面 f 越限处理与 posSetAngle4() 里单独硬编码。 */
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
  int pick = -1;

  /* 四个候选分支：正/反平面 × 肘上/肘下 */
  for (int i = 0; i < 4; i++) {
    const bool reversePlane = (i >= 2);
    const double k = ((i & 1) == 0) ? 1.0 : -1.0;
    const double rhoP  = reversePlane ? -rho : rho;
    const double delta = k * cAbs;
    const double b = reversePlane ? bRev : bFwd;

    /* 舵机 b 硬限位 0~180 */
    if (b < minB - EPS || b > maxB + EPS) continue;

    double beta, err;
    ikBranch(rhoP, zv, delta, L1, L2, &beta, &err);
    double rDeg = RADtoDEG * (beta + delta);
    double cDeg = RADtoDEG * delta;

    /* 舵机 r、c 硬限位 0~180 */
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

  /* 正解回代自检 */
  if (!(bestErr <= 0.05)) {
    WEARM_LOG("[kin] reject: residual too large, no valid solution");
    return false;
  }

  double angle1 = bestB;
  double angle2 = bestR;
  double angle3 = bestC;

  /* 容差内的微小负角归零，避免 -0.0 传给舵机 */
  if (angle2 > -ANGLE_EPS && angle2 < 0) angle2 = 0;
  if (angle3 > -ANGLE_EPS && angle3 < 0) angle3 = 0;

  /* 【先判 f 再写回】本函数契约：返回 false 时不修改 pos1->ser */
  const double LIM_EPS = 1e-9;
  const double a4 = pos1->ser.angle4;
  /* f 的 60~150 硬编码（原 servoLimit.minF / maxF） */
  const double minF = 60.0, maxF = 150.0;
  const bool toolBad = (a4 < minF - LIM_EPS || a4 > maxF + LIM_EPS);

  if (toolBad && servoLimitMode == SERVO_LIMIT_REJECT) {
    WEARM_LOG("[servo] reject joint f out of tool range");
    return false;
  }

  pos1->ser.angle1 = angle1;
  pos1->ser.angle2 = angle2;
  pos1->ser.angle3 = angle3;
  /* angle4(f) 不由反解决定，保持调用前已有的值不动 */

  bool limClamped = false;
  if (toolBad) {
    /* CLAMP 策略：只对 f 吸附到最近限位（b/r/c 已被上面的区间判断筛过） */
    double v = a4;
    if (v < minF) v = minF;
    if (v > maxF) v = maxF;
    pos1->ser.angle4 = v;
    limClamped = true;
  }
  if (clamped != NULL) *clamped = limClamped;
  return true;
}

/* 复位到工作空间内的安全初始点。
 * 【本版本已删除】rangeClampConfig()、servoSelfCheck()、clampToRange() 的调用：
 * 三个函数都随 rangeLimit / servoLimit 配置一起被移除。
 * POS_HOME 本身就在工作空间内部，直接写入即可。 */
void posInit(void) {
  Pos.rec.x = POS_HOME.x;
  Pos.rec.y = POS_HOME.y;
  Pos.rec.z = POS_HOME.z;

  /* 末端舵机取行程中位（f 硬编码 60~150，中位 = 105），
   * 避免开机时停在极限位置 */
  Pos.ser.angle4 = (60.0 + 150.0) * 0.5;

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
 * POS_HOME 本身就在工作空间内部，直接反解即可。 */
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

/* 慢 → 中 → 快 → 慢 循环降一档 */
int speedStepDown(void) {
  int cur = speedLevel;
  if (cur < SPEED_SLOW || cur > SPEED_FAST) cur = SPEED_FAST;
  return adjustSpeed((cur + 1) % 3);
}

/* ---------- 末端舵机 (angle4 / f) ---------- */

bool posSetAngle4(double angleDeg) {
  if (!isFiniteNum(angleDeg)) return false;

  /* f 的 60~150 硬编码（原 servoLimit.minF / maxF） */
  double v = clampDouble(angleDeg, 60.0, 150.0);
  if (v == Pos.ser.angle4) return false;
  Pos.ser.angle4 = v;
  return true;
}