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
 * x90, y90, z90，即 (b,r,c) = (90,90,90)）。
 * 【v1.6.7 运动逻辑替换】运动学已整体换成 others/f/MeArm 里对应 GitHub 最新
 * 一版的模型（Fifiiiii-123/MeArm，commit a7d2ee1；Task5_LineN 及其后每个版本
 * 的 solveIK/forwardKin/servoToGeo 逐字节相同）：
 *   theta = b - 90（底座转角，从 +y 量起：b = 90 = 正前方）
 *   alpha = 180 - r（上臂仰角）   beta = c（前臂相对肘角）
 *   L1 = 38（底盘高）/ L2 = 80（上臂）/ L3 = 80（前臂），全部为 mm
 * 这个模型下 (b,r,c) = (90,90,90) 的正解是：
 *   R_h = L2·cos90 - L3·cos180 = 80
 *   x = -R_h·sin0 = 0、y = R_h·cos0 = 80、z = 38 + 80·sin90 - 80·sin180 = 118
 * 即 (0, 80, 118) —— 上臂竖直、前臂水平前伸，落在工作空间中央，不再像旧
 * 40mm 模型那样贴在可达外边界 (40,0,0) 上（那时从 home 既不能向外伸也不能
 * 就地抬升，取放模块不得不为它写特例，见 pick_place.cpp 的 s_riseAtSrc）。
 * 反解 (0,80,118) 恰好回到 (90,90,90)。 */
static const REC POS_HOME = { 0, 80, 118 };

/* 机械臂几何参数（上臂长 / 下臂长 / 底盘高）—— 参照仓库最新版的实物标定值
 * （注释里写"JoyFK_alpha2 实测"）：
 *   armLength1 = L2 = 80mm（上臂：肩 → 肘）
 *   armLength2 = L3 = 80mm（前臂：肘 → 腕）
 *   armheight  = L1 = 38mm（底盘高度：底座顶面 → 肩轴）
 * 旧版是 { 20, 20, 0 }（两杆各 20mm、无底盘高），与实物的 ~160mm 臂展差一个
 * 数量级，也是旧模型下坐标全部挤在 ±40 里、方向又对不上的根因之一。 */
struct arm arm1 = { 80, 80, 38 };

/* 全局位姿实例（头文件里 Pos 是 extern 声明，定义必须放这里） */
pos Pos = { { 0, 0, 0, 0 }, { POS_HOME.x, POS_HOME.y, POS_HOME.z } };

/* 末端笛卡尔坐标范围（"工作空间盒子"，见头文件里的用法说明）。
 * 【v1.6.7 随运动学一起替换】x/y 取新模型两杆 L2 = L3 = 80 的**几何外包围盒**
 * ±(L2+L3) = ±160；z 取**实测可达竖直带**（从底盘顶面量）：
 *   z ∈ [-18, +171] mm  即 -1.8cm ~ +17.1cm
 * 实测值是 [-18.5, +171.5]（`.selfcheck/picktable_check.cpp` 的 [1b] 段全程扫描），
 * 盒子两端各留 0.5mm 余量。两端**都不是臂长的几何极限** [-122, +198]，而是
 * **关节行程**决定的：r ∈ [45,160] ⇒ α ∈ [20°,135°]（上臂压不到水平线以下），
 * c ∈ [30,110] ⇒ d ≤ 131.1mm。所以盒子外的点报"越界"、盒内但姿态不可达的点
 * 报"够不到"，语义各自清楚。旧版盒子是 ±40，按 40mm 臂长写的，新臂长下连
 * POS_HOME (0,80,118) 都会被判越界。
 * 【注意这个盒子比反解严】反解的回代残差只查水平 dx/dy（照抄参照工程，见 IK 尾部），
 * 在"前臂已顶到行程端点、水平没问题、竖直却差很远"的近满伸区它会放行：
 * 实测 (0,122,-42) 解出 r=160/c=110，真实到达 (0,126.598,+4.078)，竖直误差 +46.1mm；
 * (0,122,-60) 落到同一组角，竖直误差 +64.1mm（`.selfcheck/fkcheck.cpp`）。
 * 也就是"反解接受"不等于"真能到"。盒子故意留在**物理可达**的 [-18, +171]：
 * 路径校验（`path_core.h` 的 pathCorePointOk）与点动（`move.cpp` 的 posOutOfRange）
 * 都按盒子判，于是这些"竖直撒谎"的解进不了取放/绘图路径，直接下发的 x/y/z
 * 也仍按物理带报越界。
 * 它仍然只是给外部的**粗坐标范围**：真正的可达性由 getAngleEx() 的三维距离
 * 判据（|L2-L3| < d < L2+L3）、奇异判据（R_h >= 5mm）、行程窗口
 * （b 0~180、r 45~160、c 30~110）与 5mm 回代残差（只查水平 x/y，见 IK 尾部）
 * 共同把关。
 * 注意新模型的底座 b ∈ [0,180] 意味着 **y >= 0 的半空间**（b=90 指向 +y），
 * y < 0 的点反解会因 b 越出行程被拒 —— 盒子保留负半轴只是"声明"，
 * 拦不住任何真实姿态。 */
struct rangeLimit limit = {
  .minX = -160.0, .maxX = 160.0,
  .minY = -160.0, .maxY = 160.0,
  .minZ = -18.0, .maxZ = 171.0
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

/* ---------- 可达性（三维距离判据，参照仓库 solveIK 的第一步） ----------
 *
 * 【v1.6.7 随运动逻辑一起替换】旧版是"以肩关节竖直轴为中心、半径 40 的圆柱"
 * （只查水平半径，竖向完全不设卡），那是给 40mm 两杆模型配的。
 * 新版逐字照搬参照仓库的判据：把末端点换算到**肩轴为原点**的平面几何里，
 *     rho = sqrt(x² + y²)            （水平投影）
 *     zv  = z - armheight = z - L1    （相对肩轴的高度）
 *     d   = sqrt(rho² + zv²)          （腕点到肩轴的三维距离）
 * 要求  |L2 - L3| + 1 < d < (L2 + L3) - 1  （L2 = L3 = 80，故 1 < d < 159）。
 *   - 上界 L2+L3 是"两杆伸直"的几何极限，贴上去（d = 160）是奇异位姿，
 *     参照仓库用 1mm 余量拒掉；
 *   - 下界 |L2-L3| 是"两杆对折"的极限，本机 L2 = L3 时为 0，加 1mm 余量后
 *     要求 d > 1，即"末端不能压在肩轴正下方/正上方"。
 *
 * 【与反解的分工】本函数只做粗粒度的工作空间外边界检查，用来尽早拒掉明显
 * 不可达的点；能不能真正反解出 b/r/c 仍由 getAngleEx() 判定（它内部同样查
 * 一遍 d，不依赖调用序）。盒子（limit 结构体）比这里更粗，只供外部使用。
 *
 * 【调用方无需改动】签名与返回值语义不变（true = 大致可达），
 * path_core.h / pick_place.cpp / draw_control.cpp 直接复用。 */
bool isReachable(const REC *rec) {
  if (rec == NULL) return false;
  if (!isFiniteNum(rec->x) || !isFiniteNum(rec->y) || !isFiniteNum(rec->z)) return false;

  double L2 = arm1.armLength1;
  double L3 = arm1.armLength2;
  if (L2 <= 0 || L3 <= 0) return false;

  double rho = sqrt(rec->x * rec->x + rec->y * rec->y);
  double zv  = rec->z - arm1.armheight;
  double d   = sqrt(rho * rho + zv * zv);

  const double dMin = fabs(L2 - L3) + 1.0;
  const double dMax = (L2 + L3) - 1.0;
  if (!(d > dMin)) return false;
  if (!(d < dMax)) return false;

  return true;
}

/* 平面正解（两杆模型，参照仓库 forwardKin 的平面部分）：
 *   R_h = L2·cos(alpha) - L3·cos(alpha + beta)
 *   z   = L1 + L2·sin(alpha) - L3·sin(alpha + beta)
 * 与原版的关键差别：参照模型的 beta 是**前臂相对上臂的肘关节内角**，不是
 * 下臂的绝对角，所以两节是 (a) 与 (a+beta)，而不是 (alpha) 与 (beta)。
 * R_h 是腕点到肩轴竖直轴的水平距离（**带符号**，负值表示腕点落在肩轴后方，
 * 由反解的 R_h >= 5mm 奇异判据拦下）；z 是含底盘高 L1 的大地高度。
 * 正解与反解的回代核对共用这一份算式，保证两边逐位一致。 */
static void __attribute__((noinline)) fkPlanar(double alpha, double betaRel,
                                               double *outR, double *outZ) {
  const double L2 = arm1.armLength1;
  const double L3 = arm1.armLength2;
  *outR = L2 * cos(alpha) - L3 * cos(alpha + betaRel);
  *outZ = arm1.armheight + L2 * sin(alpha) - L3 * sin(alpha + betaRel);
}

/* 正运动学：四个舵机角度 -> 末端笛卡尔坐标（参照仓库最新版的 servoToGeo +
 * forwardKin，逐式照搬）。
 *   theta = b - 90    （底座转角：从 +y 轴量起，b = 90 = 正前方）
 *   alpha = 180 - r   （上臂仰角：r = 90 竖直朝上，r 增大 = 前倾/下垂）
 *   beta  = c         （前臂相对上臂的肘角：c = 90 与上臂成直角，
 *                       c 增大 = 前臂下垂、末端变低）
 *   x = -R_h·sin(theta)、y = R_h·cos(theta)
 * 【X 镜像】实物 base 角度增大 = 左转，而代码约定 x 正 = 实物右，所以 x 带负号，
 * 与参照仓库一致（他们每个文件头的坐标约定就是"x = 左右（正 = 实物右）"）。
 * f 只转末端夹具，不影响被控点。 */
bool recFromServo(REC *rec, const SER *ser) {
  if (rec == NULL || ser == NULL) return false;

  double b = ser->angle1;
  double r = ser->angle2;
  double c = ser->angle3;
  if (!isFiniteNum(b) || !isFiniteNum(r) || !isFiniteNum(c)) return false;

  double theta = (b - 90.0) / RADtoDEG;
  double alpha = (180.0 - r) / RADtoDEG;
  double beta  = c / RADtoDEG;

  double rh;
  double z;
  fkPlanar(alpha, beta, &rh, &z);

  double x = -rh * sin(theta);
  double y =  rh * cos(theta);
  if (!isFiniteNum(x) || !isFiniteNum(y) || !isFiniteNum(z)) return false;

  rec->x = x;
  rec->y = y;
  rec->z = z;
  return true;
}

bool getAngle(pos *pos1) {
  return getAngleEx(pos1, NULL);
}

/* 运动学逆解（参照仓库最新版 solveIK，逐式照搬，再加上本工程的契约）：
 *   1) rho = sqrt(x² + y²)、zv = z - armheight、d = sqrt(rho² + zv²)；
 *      要求 |L2-L3| + 1 < d < L2 + L3 - 1，否则不可达。
 *   2) 余弦定理求肘角 beta（前臂相对上臂的内角，唯一解，只取 +acos：
 *      参照仓库的机械臂前臂只能朝下方折，镜像解没有物理行程，故不做候选分支）。
 *   3) phi   = atan2(zv, rho)
 *      alpha = phi + atan2(L3·sin(beta), L2 - L3·cos(beta))
 *   4) 奇异判据：R_h < 5mm（腕点越过肩轴正下方/后方）判不可达。
 *   5) theta = -atan2(x, y)（X 镜像，与正解互逆）。
 *   6) 几何角 -> 舵机角：b = theta + 90、r = 180 - alpha、c = beta。
 *   7) 按各自的行程窗口夹取（b 0~180、r 45~160、c 30~110），夹到了就把
 *      *clamped 置 true；再用正解回代核对，误差 > 5mm 判不可达。
 * 返回 false 时**不改动 pos1->ser**（本函数的契约）。
 * 与旧版的区别：旧版是"±acos × 正/反平面"四个候选按优先级挑选、再把 r/c 折回
 * 基本区间；新模型只有一个解，分支跳变问题从根上消失。 */
bool getAngleEx(pos *pos1, bool *clamped) {
  if (clamped != NULL) *clamped = false;
  if (pos1 == NULL) return false;

  if (!isReachable(&pos1->rec)) {
    WEARM_LOG("[kin] reject: target out of reachable workspace");
    return false;
  }

  const double L2 = arm1.armLength1;
  const double L3 = arm1.armLength2;

  const double x   = pos1->rec.x;
  const double y   = pos1->rec.y;
  const double rho = sqrt(x * x + y * y);
  const double zv  = pos1->rec.z - arm1.armheight;
  const double d   = sqrt(rho * rho + zv * zv);

  if (!isFiniteNum(d)) return false;

  /* 与 isReachable() 同一判据，这里再查一遍：本函数被单独调用时也必须能拒绝 */
  if (!(d > fabs(L2 - L3) + 1.0)) {
    WEARM_LOG("[kin] reject: target too close to the shoulder axis");
    return false;
  }
  if (!(d < (L2 + L3) - 1.0)) {
    WEARM_LOG("[kin] reject: target beyond full extension");
    return false;
  }

  double cosB = (L2 * L2 + L3 * L3 - d * d) / (2.0 * L2 * L3);
  if (cosB >  1.0) cosB =  1.0;
  if (cosB < -1.0) cosB = -1.0;
  const double beta = acos(cosB);

  const double phi   = atan2(zv, rho);
  const double alpha = phi + atan2(L3 * sin(beta), L2 - L3 * cos(beta));

  /* 奇异检测：腕点水平投影越过肩轴（R_h < 5mm，含负值）时坐标失去唯一方向 */
  double rhChk = 0.0, zChk = 0.0;
  fkPlanar(alpha, beta, &rhChk, &zChk);
  (void) zChk;
  if (!(rhChk >= 5.0)) {
    WEARM_LOG("[kin] reject: wrist behind the shoulder axis (singular)");
    return false;
  }

  const double theta = -atan2(x, y);

  const double bRaw = RADtoDEG * theta + 90.0;
  const double rRaw = 180.0 - RADtoDEG * alpha;
  const double cRaw = RADtoDEG * beta;

  const double nb = clampDouble(bRaw, KIN_BASE_MIN, KIN_BASE_MAX);
  const double nr = clampDouble(rRaw, KIN_RARM_MIN, KIN_RARM_MAX);
  const double nc = clampDouble(cRaw, KIN_FARM_MIN, KIN_FARM_MAX);
  if (nb != bRaw || nr != rRaw || nc != cRaw) {
    if (clamped != NULL) *clamped = true;
  }

#ifdef WEARM_IK_TRACE
  Serial.print(F("[ik] tgt rho=")); Serial.print(rho, 6);
  Serial.print(F(" zv=")); Serial.print(zv, 6);
  Serial.print(F(" d=")); Serial.print(d, 6);
  Serial.print(F(" beta=")); Serial.print(RADtoDEG * beta, 6);
  Serial.print(F(" alpha=")); Serial.print(RADtoDEG * alpha, 6);
  Serial.print(F(" b=")); Serial.print(nb, 6);
  Serial.print(F(" r=")); Serial.print(nr, 6);
  Serial.print(F(" c=")); Serial.println(nc, 6);
#endif

  /* 回代核对：行程夹取可能让实际到达点偏离目标。判据照抄参照仓库
   * （`Task5_LineN2_TeachSmooth.ino:237-239`）：**只查水平 x/y**，不看 z。
   * 这样行为与参照逐点等价（z 会被夹取推偏而 x/y 正常的点参照也接受）；
   * 但 z 偏差大时留一条日志，方便上机时看出"解在水平上成立、竖直上偷了懒"。 */
  const double vTheta = (nb - 90.0) / RADtoDEG;
  const double vAlpha = (180.0 - nr) / RADtoDEG;
  const double vBeta  = nc / RADtoDEG;
  double vRh = 0.0, vZ = 0.0;
  fkPlanar(vAlpha, vBeta, &vRh, &vZ);
  const double vx = -vRh * sin(vTheta);
  const double vy =  vRh * cos(vTheta);
  const double dx = vx - x;
  const double dy = vy - y;
  const double dz = vZ - pos1->rec.z;
  if (sqrt(dx * dx + dy * dy) > 5.0) {
    WEARM_LOG("[kin] reject: residual too large after clamping");
    return false;
  }
  if (dz > 5.0 || dz < -5.0) {
    WEARM_LOG("[kin] note: accepted with large vertical offset after clamping");
  }

  /* 【本函数契约：返回 false 时不修改 pos1->ser】到这里 b/r/c 已确认，
   * 一次性写回；angle4(f) 不由反解决定，保持调用前已有的值不动。 */
  pos1->ser.angle1 = nb;
  pos1->ser.angle2 = nr;
  pos1->ser.angle3 = nc;
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
 * POS_HOME (0,80,118) 直接反解即可 —— 它的 d = √(80² + 80²) ≈ 113.1mm，
 * 离可达外边界 159mm 还远，能精确解出 90/90/90（旧版 POS_HOME (40,0,0) 是
 * 真的贴在 ρ = L1+L2 = 40 边界上，那一版才需要额外说明）。 */
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