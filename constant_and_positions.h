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
// 运动学模型（v1.6.7 起 = 参照仓库 GitHub Fifiiiii-123/MeArm@a7d2ee1 最新一版）:
//   几何：L1 = armheight = 38（底座顶面→肩轴）、L2 = armLength1 = 80（上臂）、
//         L3 = armLength2 = 80（前臂）
//   舵机角 -> 几何角：theta = b - 90（从 +y 量起，b=90 = 正前方）、
//                     alpha = 180 - r（上臂仰角）、beta = c（前臂相对肘角）
//   正解：R_h = L2·cos(alpha) - L3·cos(alpha+beta)
//         x = -R_h·sin(theta)、y = R_h·cos(theta)（X 镜像：实物 base 增大 = 左转）
//         z = L1 + L2·sin(alpha) - L3·sin(alpha+beta)
//   初始位姿 (b,r,c) = (90,90,90) <=> 末端 (0, 80, 118)，即 POS_HOME
//
// 【范围限制现状】（v1.6.7 运动逻辑替换后）
//   - 笛卡尔坐标范围（"工作空间盒子"）：struct rangeLimit / limit 定义在
//     constant_and_positions.cpp，x/y 为 ±160（= ±(L2+L3)），z 为 [-122, 198]
//     （= [L1-(L2+L3), L1+(L2+L3)]），即新几何的**外包围盒**。
//     越界点由 path_core.h 的点校验与 move.cpp 的关节步进拒绝，详见 meArm.ino
//     的"安全边界"一节。真正的可达性判据是 isReachable()/getAngleEx()（三维
//     距离 d ∈ (|L2-L3|, L2+L3) + 行程窗口 + 5mm 回代残差）。
//   - 关节行程窗口：b 0~180、r 45~160、c 30~110（参照仓库最新版的实机标定值，
//     见下面的 KIN_* 宏）。反解超出行程时夹取并回传 *clamped = true。
//   - 可配置的关节硬限位：struct servoLimitCfg / servoLimit、servoSelfCheck()、
//     isServoInRange()、clampServoAngles() 仍然是删除状态。
//   - 夹爪 f 的 60~150 硬限位：已删除（见下面的 TOOL_ANGLE_*）。f 只受舵机
//     电气行程 TOOL_ANGLE_MIN..TOOL_ANGLE_MAX(0~180) 约束，于是"夹紧"
//     （压到 60 以下）才可能做到。
//   - 纸面高度的标定范围 DRAW_PEN_Z_MIN/MAX(-5~30)：已删除。
//
#ifndef WEARM_CONSTANT_AND_POSITIONS_H
#define WEARM_CONSTANT_AND_POSITIONS_H

#include <math.h>
#include <stdbool.h>

/* 【v1.6.6 修正】原来写的是 180/3.1415926535（π 只截了 10 位），相对误差
 * ~3.5e-11，换算到 90° 是 ~3.2e-9 度的偏差。当时行程容差是 1e-9，偏差比容差还大
 * —— r/c 恰好落在 180/0 的端点解会被 0~180 的行程判据误杀（probe_rt 里
 * (0,0,-40) => r=180,c=0 这一条就是这么被拒的）。π 改成完整精度（常量、编译期
 * 折叠，不占 flash）；行程容差也统一成下面的 ANGLE_EPS(1e-6)，两头都不再留隐患。 */
#define RADtoDEG (180.0 / 3.14159265358979323846)
/* 避免与 Arduino 核心的 PI 宏冲突 */
#ifndef PI
#define PI 3.1415926535
#endif
/* 关节角容差（度）：反解的回代残差判据与 r/c 行程边界判据共用这一个口径 */
#define ANGLE_EPS 1e-6

/* ---------- 关节行程窗口（v1.6.7 运动逻辑替换） ----------
 * 直接照搬参照仓库最新版（others/f/MeArm，GitHub Fifiiiii-123/MeArm@a7d2ee1）
 * 里每个 Task5 系列 / JoyFK 系列草图都在用的实机标定值：
 *     BASE 0~180、RARM 45~160、FARM 30~110
 * 它们不是"舵机电气行程"（那是 0~180，见 TOOL_ANGLE_*），而是这台实物机械臂
 * 三个位姿关节的**机械行程** —— 参照仓库把它们作为 solveIK 的最后一道防线，
 * 目的是别让结构自己顶死（上臂完全放平、前臂反折等姿态会撞到自身）。
 * 反解超出行程时的策略也照搬：夹到窗口边界，并通过 *clamped 告诉调用方"这个解
 * 被吸附过，实际到达点与目标可能有偏差"（随后还有 5mm 回代核对）。
 * 想放宽/收紧结构行程，改这里一处即可（move.cpp / button_control.cpp /
 * serial_protocol.cpp 的钳制都引用这几个宏）。 */
#define KIN_BASE_MIN 0.0
#define KIN_BASE_MAX 180.0
#define KIN_RARM_MIN 45.0
#define KIN_RARM_MAX 160.0
#define KIN_FARM_MIN 30.0
#define KIN_FARM_MAX 110.0

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

/* ---------- 末端笛卡尔坐标范围（"工作空间盒子"）----------
 * 末端坐标 x/y/z 的上下限，每个轴都必须 min <= max（写反由 rangeLimitConfig()
 * 在开机时自动交换回来）。数值在 constant_and_positions.cpp 里，
 * 【v1.6.7】x/y = ±160、z = [-122, 198]，即新几何（L1=38、L2=L3=80）的外包围盒。
 *
 * 越界怎么处理（两条"整步/整条回退"路径）:
 *   - 取放与绘图：轨迹采样点越界 -> 整条轨迹校验不过，取放启动被拒（BUSY）、
 *     绘图启动/绘制中被拒（见 path_core.h 的 pathCorePointOk()）；
 *   - 摇杆 / 按键 / 示教的关节步进：走完一步后正解出的末端越界 -> 这一步整步
 *     回退（返回 MOVE_AT_LIMIT，摇杆表现为"推到头了"，见 move.cpp）。
 *
 * 【盒子 vs 真实可达集】盒子是**外包围盒**，比真实可达集宽，所以不会有可达姿态
 * 落在盒子外被误杀；反过来盒子内的点也可能够不到，那由 isReachable() 与
 * getAngleEx()（三维距离 + 行程窗口 + 5mm 回代残差）拦下。
 * 注意新模型的底座只有 b ∈ [0,180]，b = 90 指向 +y（正前方），所以**真实可达集
 * 全在 y >= 0 的半空间**里，盒子里的负半轴只是声明、任何姿态都到不了；
 * 这是参照仓库的实机约定（他们的 x 正 = 实物右、y 正 = 正前方），不是笔误。 */
struct rangeLimit {
  double minX, maxX;
  double minY, maxY;
  double minZ, maxZ;
};
extern struct rangeLimit limit;

/* 校验并修正 limit 配置：把每个轴写反的 (min,max) 交换回来，返回被修正的轴数。
 * 在 posInit() 里开机调用一次（配置自检，与调速/行程的自检同一风格）：
 * 盒子写反会让每个点都被判越界，必须当场纠正。 */
int rangeLimitConfig(void);

/* ---------- 末端夹爪（f）关节角：逻辑角，单位度 ----------
 * 【本版本已删除 60~150 的"夹爪范围限制"】现在只剩舵机自身的电气行程
 * 0~180（与 b/r/c 同一套边界），夹爪可以一直往里压到 0 也不会被软件拦下。
 *
 * 三个角的分工（能不能夹住东西，全靠最后这一个差值）：
 *   TOOL_ANGLE_OPEN   150   完全张开
 *   TOOL_ANGLE_CLOSE   60   名义合拢角（= 串口 'S'）：夹爪“刚碰到物体”时大致的位置
 *   TOOL_ANGLE_TIGHT   45   夹紧角：在名义合拢角的基础上**再往里压 15 度**
 *
 * 舵机是位置伺服：它只会一直朝目标角使劲，直到堵转。所以"夹得紧不紧"在固件里
 * 只有一个可调量 —— 目标角比接触点再往里压多少度：
 *   · 压得太少（v1.6.5 的 55 只有 5 度）：稍小一点的物体，夹爪还没碰到它，
 *     舵机就已经"到位"了；此时看着是合上了，其实两爪之间是空的，一抬臂就掉。
 *     这就是取物容易松开的直接原因。
 *   · 压得够多（本版 15 度）：不管物体大小，夹爪都必然压到物体上并继续用力。
 *     夹持力 = 舵机的堵转力矩，与压进去几度无关（多压只是多走一段行程，
 *     保证"一定碰上"）。
 *   · 压得太多（逼近 0）：两爪先自己顶死，物体反而可能被斜面挤出去，
 *     而且堵转电流最大、发热最快。
 * 调法：夹不住就按 5 度一档往下调本值；舵机发烫 / 供电掉压就往回调大。
 * 取放序列也是按这个思路走的：先快速合到 TOOL_ANGLE_CLOSE，再用很慢的速度
 * 蠕动到本角（见 pick_place.cpp 的 SQUEEZE_TOOL），免得一上来就用力把物体顶飞。
 * 取放与绘图每一帧都会重申这个角（pickHoldingStage() / drawLoop()）。 */
#define TOOL_ANGLE_MIN   0.0     /* f 逻辑角下限（舵机电气行程） */
#define TOOL_ANGLE_MAX   180.0   /* f 逻辑角上限 */
#define TOOL_ANGLE_OPEN  150.0   /* 张开到位，与串口 'O' 一致 */
#define TOOL_ANGLE_CLOSE 60.0    /* 名义合拢，与串口 'S' 一致 */
#define TOOL_ANGLE_TIGHT 45.0    /* 夹紧：在名义合拢角 60 的基础上再往里压 15 度 */
/* 夹爪的装配镜像：physical = TOOL_MIRROR_SUM - logical（与基座的 180- 同理）。
 * 210 是这台机器上舵机摆臂的装配角，与"行程"无关，换装配才需要改。 */
#define TOOL_MIRROR_SUM  210.0

/* 全局调速配置：所有移动源 (摇杆/示教点动/手动步进) 共用。 */
struct speedCfg {
  double stepSize;
  int    stepDelayMs;
};
extern struct speedCfg speed;

/* 几何可达性：把末端点换算到肩轴为原点的平面几何里，三维距离
 *     d = sqrt(x² + y² + (z - armheight)²)
 * 必须落在 (|L2-L3|, L2+L3) 内（两端各留 1mm 余量避开伸直的奇异姿态）。
 * 这是反解的数学前提（尽早拒掉明显够不到的点），不属于"范围限制"，保留。
 * 【v1.6.7】旧版只看水平半径 sqrt(x²+y²) <= 40（竖向完全不管，那时是 40mm
 * 两杆模型），已随运动逻辑一起换成参照仓库的三维判据。 */
bool isReachable(const REC *rec);

/* 由笛卡尔坐标 rec 反解出舵机角度 ser（逆运动学）。
 * 返回值:
 *   true  —— 运动学可解，且 b/r/c 都落在硬编码的 0~180 内（f 不参与判定）
 *   false —— 目标点不可达，或角度出现 NaN/Inf
 * 返回 false 时不修改 pos1->ser，调用方应保留上一次的有效角度。
 * angle4(末端 f) 不参与反解，保持调用前已有的值不变。 */
bool getAngle(pos *pos1);

/* getAngle 的扩展版：*clamped 回传"解出的姿态是否被吸附到关节行程窗口上"。
 * 【v1.6.7】反解改成参照仓库的单分支解 + 行程夹取：b 0~180、r 45~160、
 * c 30~110（见上面的 KIN_* 宏）。只要任一轴被夹到窗口边界，*clamped 就是
 * true（此时若实际到达点与目标相差超过 5mm，反解直接返回 false）。
 * clamped 传 NULL 时与 getAngle 完全一致。 */
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