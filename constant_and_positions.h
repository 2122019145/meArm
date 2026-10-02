//
// constant_and_positions.h
// 机械臂核心数据结构、运动学、范围边界与全局调速配置
//
// 内容:
//   1. 常量 (RADtoDEG, PI, RANGE_EPS)
//   2. 结构体 (arm / servoAngle / rectangularCoordinate / position /
//      rangeLimit / speedCfg)
//   3. 全局实例 (Pos / arm1 / limit / speed)
//   4. 范围边界: 配置自检 rangeClampConfig() / 坐标钳制 clampToRange()
//      / 边界判定 atRangeEdge() / 可达性 isReachable()
//   5. 反解算法 getAngle()（越界或不可达时拒绝写入并返回 false）
//   6. 全局调速 setSpeed()、档位 adjustSpeed()、降档 speedStepDown()
//
// 坐标轴约定（全工程统一）:
//   x: 左右 (右为正)   y: 前后 (前为正)   z: 上下 (上为正)
//
// 【范围边界规则】
//   所有会改变末端坐标的操作都必须遵守下面三条，缺一不可：
//     (1) 先判断 limit 配置本身合法 (min 不大于 max)，见 rangeClampConfig()；
//     (2) 每一步移动后调用 clampToRange() 把坐标夹回 [min,max] 区间，
//         越界的方向会被"挡住"而不是继续往外走；
//     (3) 钳制后的目标点还要能用运动学反解出有效角度、且角度不越关节硬限位，
//         见 getAngleEx()；解不出来或角度被限位吸附(move.cpp 用的语义)就整步
//         回退，绝不写入无效角度、也不让坐标系与真实姿态脱节。
//
//   实测真可达包络（肩在原点, L1=L2=20, 关节限位 b[0,180] r[0,180] c[0,180] f[60,150]）:
//     x[-40.00, 40.00]  y[-40.00, 40.00]  z[-20.00, 40.00]
//   当前 limit 完全覆盖包络，余量为 0（x[-40.0,40.0] y[-40.0,40.0] z[-20.0,40.0]）。
//
#ifndef WEARM_CONSTANT_AND_POSITIONS_H
#define WEARM_CONSTANT_AND_POSITIONS_H

#include <math.h>
#include <stdbool.h>

#define RADtoDEG (180 / 3.1415926535)
/* 避免与 Arduino 核心的 PI 宏冲突 */
#ifndef PI
#define PI 3.1415926535
#endif
/* 浮点边界比较容差：小于它视为"贴在边界上" */
#define RANGE_EPS 1e-9
/* 关节角容差（度）：小于它的微小负角（-1e-15 / -0.0）按 0 处理，
 * 避免在奇异点附近误判合法姿态为不可达 */
#define ANGLE_EPS 1e-6

/* 机械臂几何参数：上臂长、下臂长、臂高（单位与坐标一致） */
struct arm {
  double armLength1;
  double armLength2;
  double armheight;
};

/* 各舵机角度 (度) —— 对应你手上的四个关节记号:
 *   angle1 = b  底部回转
 *   angle2 = r  上臂(肩)俯仰
 *   angle3 = c  下臂(肘)俯仰
 *   angle4 = f  末端
 * 注意：这里存的是"解算出来的原始角度"，是否越限由 isServoInRange() 判断。 */
typedef struct servoAngle {
  double angle1;   /* b 水平回转舵机 */
  double angle2;   /* r 上臂俯仰舵机 */
  double angle3;   /* c 下臂俯仰舵机 */
  double angle4;   /* f 末端舵机（反解不改变它，由指令单独设置） */
} SER;

/* 笛卡尔坐标 (末端位姿) */
typedef struct rectangularCoordinate {
  double x;
  double y;
  double z;
} REC;

/* 位置状态: 同时保存笛卡尔坐标与对应舵机角度 */
typedef struct position {
  SER ser;
  REC rec;
} pos;

extern pos Pos;
extern struct arm arm1;

/* 末端位置的运动范围上下限（每个轴必须有 min <= max） */
struct rangeLimit {
  double minX, maxX;
  double minY, maxY;
  double minZ, maxZ;
};
extern struct rangeLimit limit;

/* 每个舵机的机械允许角度区间（度）——【关节硬限位】
 * 这四个区间来自实测机械结构能安全到达的范围，是防止舵机顶死/连杆别死的
 * 最后一道防线。反解出的角度必须全部落在这里面，姿态才被接受。
 * 与 limit 的分工:
 *   limit         管"末端坐标"不许跑出工作空间（粗边界，挡得快）
 *   servoLimit    管"每个关节角"不许超出机械行程（细边界，最终防线） */
struct servoLimitCfg {
  double minB, maxB;   /* angle1 底部回转 */
  double minR, maxR;   /* angle2 上臂俯仰 */
  double minC, maxC;   /* angle3 下臂俯仰 */
  double minF, maxF;   /* angle4 末端 */
};
extern struct servoLimitCfg servoLimit;

/* 关节限位越界策略:
 *   SERVO_LIMIT_CLAMP  越界时把角度吸附到最近限位（动作到极限为止，不报错）
 *   SERVO_LIMIT_REJECT 越界时直接拒绝该姿态（末端停在原位，更保守） */
#define SERVO_LIMIT_CLAMP  0
#define SERVO_LIMIT_REJECT 1
extern int servoLimitMode;

/* 校验并修正 servoLimit 配置（min<=max，且都落在舵机物理 0~180 内）。
 * 返回被修正的项数，0 表示配置本来就正确。 */
int servoSelfCheck(void);

/* 判断四个关节角是否都在 servoLimit 允许区间内（容差 ANGLE_EPS 度）。 */
bool isServoInRange(const SER *ser);

/* 把四个关节角吸附到 servoLimit 区间内，返回是否有角度被改动。 */
bool clampServoAngles(SER *ser);

/* 全局调速配置：所有移动源 (摇杆/步进/手动) 共用。
 * stepSize      —— 每次步进移动的坐标单位 (越大移动越快)
 * minDelayMs    —— 输入较弱时的最小步间间隔 (ms)，控制最高速度
 * fullDelayMs   —— 满偏时的步间间隔 (ms)，满偏最慢最安全 */
struct speedCfg {
  double stepSize;
  int    minDelayMs;
  int    fullDelayMs;
};
extern struct speedCfg speed;

/* 将 pos1->rec 钳制到 [min,max] 范围内。若有轴被钳制则返回 true。
 * 内部对 min/max 做了交换保护，即使配置写反也不会把坐标推出范围。 */
bool clampToRange(pos *pos1);

/* 校验并修正 limit 配置：对每个轴做 (min,max) 排序，并统计写反的轴数。
 * 返回写反（现已自动纠正）的轴数，0 表示配置本来就正确。 */
int rangeClampConfig(void);

/* 判断 pos1->rec 是否贴在范围边界上（容差 RANGE_EPS）。
 * *axis 非空时写入出界轴名 'x'/'y'/'z'，多轴同时贴边时取第一个。 */
bool atRangeEdge(const pos *pos1, char *axis);

/* 判断坐标 rec 是否在机械臂可达工作空间内（反解不会出现 acos 越域）。 */
bool isReachable(const REC *rec);

/* 由笛卡尔坐标 rec 反解出舵机角度 ser (逆运动学)。
 *
 * 【本机运动学模型】肩关节在原点，上臂 L1、下臂 L2，armheight = 0。
 *   舵机中立位 (90°) 的实测含义:
 *     angle1 (b) 90° -> 基座朝 +x（可达空间只占 x >= 0 一侧）
 *     angle2 (r) 90° -> 上臂竖直向上
 *     angle3 (c) 90° -> 下臂水平朝前（与上臂成 90°）
 *   平面内以 +x 为 0°、抬向 +z 为正，两个连杆的方向角为:
 *     上臂方向角 alpha = r          （r 直接就是上臂的绝对方向角）
 *     下臂方向角 beta  = r - c      （c 是下臂相对上臂往前转的折角）
 *   正运动学:
 *     x_planar = L1 cos alpha + L2 cos beta
 *     z        = L1 sin alpha + L2 sin beta + armheight
 *   标定校验（L1 = L2 = 20、armheight = 0，两处误差均为 0.00）:
 *     (r,c) = (90, 90) -> (20.00, 20.00)   初始位姿
 *     (r,c) = (90,105) -> (19.32, 14.82)   用户实测（c 增大末端往前+往下）
 *
 * 反解（闭式，无迭代）:
 *     angle1 = 90° - atan2(y, x)                    b=90 朝 +x，b<90 转向 +y
 *     cos(c) = (R² - L1² - L2²) / (2 L1 L2)         R = 末端到肩的距离
 *     alpha  = atan2(z - armheight, rho) + atan2(L2 sin c, L1 + L2 cos c)
 *   算完用正运动学回代自检，残差 > 0.05 就整组丢弃。
 *
 * 返回值:
 *   true  —— 运动学可解，且（按 servoLimitMode）关节角都落在 servoLimit 内
 *            （CLAMP 策略下越限的角度会被就地吸附到最近限位）
 *   false —— 目标点不可达、角度出现 NaN/Inf，或 REJECT 策略下有关节越限
 * 返回 false 时不修改 pos1->ser，调用方应保留上一次的有效角度。
 * angle4(末端 f) 不参与反解，保持调用前已有的值不变。 */
bool getAngle(pos *pos1);

/* getAngle 的扩展版：*clamped 回传"是否因关节硬限位被吸附"。
 * CLAMP 策略下可能出现"返回 true 但角度被改过"（clamped=true），
 * 此时末端实际到不了目标点。需要"精确到达"语义的调用方
 * （moveAxisStep 的每一步移动）应改用本函数并在 clamped 时回退坐标，
 * 否则坐标系会与真实姿态越差越远。
 * clamped 传 NULL 时行为与 getAngle 完全一致。 */
bool getAngleEx(pos *pos1, bool *clamped);

/* 正运动学：由四个舵机角度算出末端笛卡尔坐标，写入 *rec。
 * 【角度模式下的用途】现在被控量是关节角本身，末端 x/y/z 只是显示量，
 * 每次改完角度都要用它刷新 Pos.rec，让"坐标"永远等于"角度的真实结果"，
 * 而不是一个越用越偏的独立状态。
 * angle4(f) 是末端夹具的自转，不影响被控点位置，不参与本计算。
 * rec 与 ser 允许指向同一个 pos 结构体（内部先读后写）。
 * 返回 true 表示算出了有限数；参数为空时返回 false。 */
bool recFromServo(REC *rec, const SER *ser);

/* 把 Pos 复位到工作空间内的一个安全初始点，并解算一次舵机角度。
 * 应在 setup() 里 writeServo() 之前调用，避免开机时舵机角度为 0 乱动。 */
void posInit(void);

/* 设置末端舵机 angle4 (f) 的角度（度）。
 * 反解不会改动 angle4，它只能由这些接口或直接写 Pos.ser.angle4 改变，
 * 一个坐标点里 angle4 与 x/y/z 是彼此独立的自由度。
 * 角度会被夹在 servoLimit 的 f 行程内；返回 true 表示确实发生了变化。 */
bool posSetAngle4(double angleDeg);

/* 末端张开 / 收回一步（默认步长 5 度，传入 >0 的值可自定义）。
 * 返回 true 表示角度确实变了；已在限位上则返回 false。 */
bool posToolOpen(double stepDeg);
bool posToolClose(double stepDeg);

/* 设置全局调速参数 (带合法性校验)。
 * stepSize 必须 > 0；minDelay 必须为正且不超过 fullDelay。
 * 不合法的项保持原值；只有参数确实被改动时才把档位标记为自定义 (-1)。 */
void setSpeed(double stepSize, int minDelayMs, int fullDelayMs);

/* 按档位调整速度:
 *   SPEED_SLOW   —— 慢速: 小步长 + 长间隔, 精细移动
 *   SPEED_NORMAL —— 中速: 默认参数
 *   SPEED_FAST   —— 快速: 大步长 + 短间隔, 高速移动
 * 也可直接 setSpeed(...) 自定义。
 * 返回生效的档位；level 非法时返回 -1 且不改动任何参数。 */
#define SPEED_SLOW   0
#define SPEED_NORMAL 1
#define SPEED_FAST   2
int  adjustSpeed(int level);

/* 返回当前档位 (-1 表示自定义/被 setSpeed 覆盖过)。 */
int  speedGetLevel(void);

/* 档位名，用于串口提示："慢速"/"中速"/"快速"/"自定义"。 */
const char *speedLevelName(int level);

/* 在 慢→中→快→慢 之间循环降一档，返回生效的档位。 */
int  speedStepDown(void);

#endif /* WEARM_CONSTANT_AND_POSITIONS_H */
