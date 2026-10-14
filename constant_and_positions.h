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
//   - 【本版本新增删除】夹爪 f 的 60~150 硬限位：getAngleEx() 里的越限吸附、
//     posSetAngle4() 的夹取、button_control 录制/回放里的夹取全部删掉，
//     f 现在只受舵机电气行程 TOOL_ANGLE_MIN..TOOL_ANGLE_MAX(0~180) 约束，
//     于是"夹紧"（压到 60 以下）才可能做到。
//   - 【本版本新增删除】纸面高度的标定范围 DRAW_PEN_Z_MIN/MAX(-5~30)。
//   - 【本版本新增删除】isReachable() 的竖向边界 |z - armheight| <= L1+L2：
//     可达性只看水平半径，竖向交给反解残差与路径校验把关。
//   - b/r/c 三轴的物理行程 0~180 在 moveJointStep() 内硬编码。
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

/* ---------- 末端夹爪（f）关节角：逻辑角，单位度 ----------
 * 【本版本已删除 60~150 的"夹爪范围限制"】现在只剩舵机自身的电气行程
 * 0~180（与 b/r/c 同一套边界），夹爪可以一直往里压到 0 也不会被软件拦下。
 * TOOL_ANGLE_TIGHT 是"夹紧"角：比名义合拢角再往里压几度，靠舵机的保持力矩
 * 把物体/笔夹住——取放序列与绘图循环每一帧都会重申这个角（见 pick_place.cpp
 * 的 pickHoldingStage() 与 draw_control.cpp 的 drawLoop()）。
 * 注意：压得越深堵转电流越大，舵机最好单独供电；夹不住再一点点调小。 */
#define TOOL_ANGLE_MIN   0.0     /* f 逻辑角下限（舵机电气行程） */
#define TOOL_ANGLE_MAX   180.0   /* f 逻辑角上限 */
#define TOOL_ANGLE_OPEN  150.0   /* 张开到位，与串口 'O' 一致 */
#define TOOL_ANGLE_CLOSE 60.0    /* 名义合拢，与串口 'S' 一致 */
#define TOOL_ANGLE_TIGHT 55.0    /* 夹紧：再往里压 5 度 */
/* 夹爪的装配镜像：physical = TOOL_MIRROR_SUM - logical（与基座的 180- 同理）。
 * 210 是这台机器上舵机摆臂的装配角，与"行程"无关，换装配才需要改。 */
#define TOOL_MIRROR_SUM  210.0

/* 全局调速配置：所有移动源 (摇杆/示教点动/手动步进) 共用。 */
struct speedCfg {
  double stepSize;
  int    stepDelayMs;
};
extern struct speedCfg speed;

/* 几何可达性：末端的水平半径 sqrt(x²+y²) 必须 <= L1+L2。
 * 这是反解的数学前提（尽早拒掉明显够不到的点），不属于"范围限制"，保留。
 * 【本版本】竖向边界 |z - armheight| <= L1+L2 已按需求删除：z 不再单独设卡，
 * 竖向靠 getAngleEx() 的残差判据与路径校验把关（cosC 在 acos 前已夹到 ±1，
 * 不会越域）。 */
bool isReachable(const REC *rec);

/* 由笛卡尔坐标 rec 反解出舵机角度 ser（逆运动学）。
 * 返回值:
 *   true  —— 运动学可解，且 b/r/c 都落在硬编码的 0~180 内（f 不参与判定）
 *   false —— 目标点不可达，或角度出现 NaN/Inf
 * 返回 false 时不修改 pos1->ser，调用方应保留上一次的有效角度。
 * angle4(末端 f) 不参与反解，保持调用前已有的值不变。 */
bool getAngle(pos *pos1);

/* getAngle 的扩展版：*clamped 回传"解出的姿态是否被吸附到关节限位上"。
 * 【本版本】f 的 60~150 吸附已删除，b/r/c 走的是显式区间筛选（不合格就换
 * 分支/拒绝），不做吸附，所以 *clamped 目前恒为 false；参数保留给将来的
 * 姿态吸附策略，调用方（path_core.h）的判据不用改。clamped 传 NULL 时与
 * getAngle 完全一致。 */
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
 * 只夹在舵机电气行程 TOOL_ANGLE_MIN..TOOL_ANGLE_MAX(0~180) 内（60~150 的
 * "夹爪范围限制"已删除，所以比名义合拢角 60 更小的"夹紧角"能写进去）；
 * 返回 true 表示确实发生了变化。 */
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