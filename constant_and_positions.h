//
// constant_and_positions.h
// 机械臂核心数据结构、运动学与全局调速配置（精简版）
//
// 内容:
//   1. 常量 (RADtoDEG, PI, ANGLE_EPS)
//   2. 结构体 (arm / servoAngle / rectangularCoordinate / position / speedCfg)
//   3. 全局实例 (Pos / arm1 / speed)
//   4. 可达性 isReachable()（几何前提，不是范围限制）
//   5. 反解算法 getAngle()（不可达时拒绝写入并返回 false）
//   6. 全局调速 setSpeed()、档位 adjustSpeed()、降档 speedStepDown()
//
// 坐标轴约定（全工程统一）:
//   x: 左右 (右为正)   y: 前后 (前为正)   z: 上下 (上为正)
//
// 【本版本删去的范围限制】
//   - 笛卡尔坐标范围：struct rangeLimit / limit、rangeClampConfig()、
//     clampToRange() 全部移除。末端 x/y/z 只是角度正解出来的派生量。
//   - 可配置的关节硬限位：struct servoLimitCfg / servoLimit、servoSelfCheck()、
//     isServoInRange()、clampServoAngles() 全部移除。
//   - b/r/c 三轴的物理行程 0~180 与 f 的 60~150 分别在 moveJointStep()、
//     posSetAngle4() 与 writeServo() 里硬编码。
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
/* 关节角容差（度）：小于它的微小负角按 0 处理 */
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
 * 注意：这里存的是"解算出来的原始角度"，越界由各自的写入路径夹取。 */
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

/* 关节限位越界策略（当前仅对 f 生效；b/r/c 在反解阶段就被显式筛掉） */
#define SERVO_LIMIT_CLAMP  0
#define SERVO_LIMIT_REJECT 1
extern int servoLimitMode;

/* 全局调速配置：所有移动源 (摇杆/示教点动/手动步进) 共用。 */
struct speedCfg {
  double stepSize;
  int    stepDelayMs;
};
extern struct speedCfg speed;

/* 几何可达性：末端到肩的距离 R 必须落在 [|L1-L2|, L1+L2] 内，
 * 否则 acos 会越域。这是反解的数学前提，不属于"范围限制"，保留。 */
bool isReachable(const REC *rec);

/* 由笛卡尔坐标 rec 反解出舵机角度 ser（逆运动学）。
 * 返回值:
 *   true  —— 运动学可解，且 b/r/c 都落在硬编码的 0~180 内
 *            （f 的越界按 servoLimitMode 处理）
 *   false —— 目标点不可达、角度出现 NaN/Inf，或 REJECT 策略下 f 越限
 * 返回 false 时不修改 pos1->ser，调用方应保留上一次的有效角度。
 * angle4(末端 f) 不参与反解，保持调用前已有的值不变。 */
bool getAngle(pos *pos1);

/* getAngle 的扩展版：*clamped 回传"是否因 f 越限被吸附"。
 * clamped 传 NULL 时行为与 getAngle 完全一致。 */
bool getAngleEx(pos *pos1, bool *clamped);

/* 正运动学：由四个舵机角度算出末端笛卡尔坐标，写入 *rec。
 * angle4(f) 是末端夹具的自转，不影响被控点位置，不参与本计算。
 * rec 与 ser 允许指向同一个 pos 结构体（内部先读后写）。
 * 返回 true 表示算出了有限数；参数为空时返回 false。 */
bool recFromServo(REC *rec, const SER *ser);

/* 把 Pos 复位到工作空间内的一个安全初始点，并解算一次舵机角度。 */
void posInit(void);

/* 取"开机初始位姿"（POS_HOME）对应的关节角，写到 ser->angle1/2/3。
 * ser->angle4 不会被改动。返回 true 表示反解成功。 */
bool posGetHomeAngles(SER *ser);

/* 设置末端舵机 angle4 (f) 的角度（度）。
 * 角度会被夹在硬编码的 60~150 内；返回 true 表示确实发生了变化。 */
bool posSetAngle4(double angleDeg);

/* 全局调速 */
void setSpeed(double stepSize, int stepDelayMs);

#define SPEED_SLOW   0
#define SPEED_NORMAL 1
#define SPEED_FAST   2
int  adjustSpeed(int level);
int  speedGetLevel(void);
const char *speedLevelName(int level);
int  speedStepDown(void);

#endif /* WEARM_CONSTANT_AND_POSITIONS_H */