//
// move.h
// 关节角步进接口（【角度模式】主接口 moveJointStep）+ 兼容的旧方向轴接口
//
// 【本工程的控制方式】控制的是机械臂的四个关节角，不是末端坐标。
//   被控量（状态量）: Pos.ser.angle1..angle3（b / r / c）与 angle4（f）
//   派生量（显示量）: Pos.rec.x/y/z —— 由正运动学实时算出，不是目标值
//   软件边界        : servoLimit（每个关节的机械行程）是最终防线；
//                     rangeLimit 只是"这个角度算出来的位置跑出工作空间就挡住"
//                     的额外保险，角度模式下坐标已不再是用户输入。
//
// 设计说明:
//   - moveJointStep 是主接口：把某个关节角加减一个步长，返回三态结果，
//     便于摇杆模块区分"正常转动 / 顶到限位 / 位置超出工作空间"。
//   - 六个定长函数 (moveup/movedown/...) 是历史上"末端坐标点动"的接口，
//     现在语义变成"对应关节角走 1 度"，保留给程序化顺序动作使用。
//
// 坐标轴与关节的对应（角度模式）:
//   DIR_UP/DOWN   <-> 下臂 c (angle3)
//   DIR_FWD/BWD   <-> 上臂 r (angle2)
//   DIR_LEFT/RIGHT<-> 基座 b (angle1)
//
// 【边界行为】
//   每一步都做两重校验，任一失败则整步回退，Pos 保持在上一次的有效位置：
//     1) 关节限位：角度夹到 servoLimit 内；夹完与原来相同说明已到机械行程
//        尽头 -> MOVE_AT_LIMIT。
//     2) 位置边界：由新角度正解出的末端坐标必须仍在 rangeLimit 内，
//        否则整步回退 -> MOVE_AT_LIMIT（避免臂跑到工作空间外）。
//
#ifndef WEARM_MOVE_H
#define WEARM_MOVE_H

#include "Arduino.h"
#include "constant_and_positions.h"
#include <math.h>

/* 固定 1 单位的六个方向步进（自动钳制 + 反解舵机角度）。
 * 越界或不可达时什么都不改，函数无返回值。 */
void moveup(void);       /* z +1 上升 */
void movedown(void);     /* z -1 下降 */
void moveleft(void);     /* x -1 左 */
void moveright(void);    /* x +1 右 */
void moveforward(void);  /* y +1 前 */
void movebackward(void); /* y -1 后 */

/* 方向枚举：JointDir 的数值基础，也是定长函数 moveup/movedown/... 的取值来源 */
enum MoveDir {
  DIR_UP = 1, DIR_DOWN,
  DIR_LEFT, DIR_RIGHT,
  DIR_FWD, DIR_BWD
};

/* 【角度模式】关节方向枚举：每个关节"加/减"两个方向，语义直接对应舵机角。
 * 数值刻意与 MoveDir 的对应项一致（UP=下臂加、DOWN=下臂减、FWD=上臂加、
 * BWD=上臂减、LEFT=基座减、RIGHT=基座加），这样两套枚举可以互相传参而不错位。 */
enum JointDir {
  JOINT_C_UP    = DIR_UP,     /* 下臂 c (angle3) 角度增大 */
  JOINT_C_DOWN  = DIR_DOWN,   /* 下臂 c (angle3) 角度减小 */
  JOINT_B_LEFT  = DIR_LEFT,   /* 基座 b (angle1) 角度减小 */
  JOINT_B_RIGHT = DIR_RIGHT,  /* 基座 b (angle1) 角度增大 */
  JOINT_R_FWD   = DIR_FWD,    /* 上臂 r (angle2) 角度增大 */
  JOINT_R_BWD   = DIR_BWD     /* 上臂 r (angle2) 角度减小 */
};

/* moveJointStep 的执行结果 */
enum MoveResult {
  MOVE_NONE        = 0,  /* 方向非法 / 步长无效，未动作 */
  MOVE_OK          = 1,  /* 正常移动了一步 */
  MOVE_AT_LIMIT    = 2,  /* 方向被范围边界挡住（已在边界上） */
  MOVE_UNREACHABLE = 3   /* 目标点超出机械臂臂展，已整步回退 */
};

/* 按指定步长移动一个关节角（【角度模式下的主接口】）。
 *
 * 本工程现在控制的是机械臂的关节角，不是末端坐标。四个关节与摇杆的对应关系
 * 见 joystick_control.h；本函数就是"把某一个关节角加/减 stepSize 度"的底层动作。
 *
 * 每步的动作流程（保证 Pos.ser 与 Pos.rec 永远自洽）:
 *   1. 先把关节角按 dir 加减 stepSize 度（坐标轴同时跟着试探前移/后退，
 *      这一步只是为了让 Pos.rec 有个落点，真正的坐标以第 4 步为准）；
 *   2. 关节角夹到 servoLimit 区间内 —— 夹完等于原值说明这个方向已到限位，
 *      返回 MOVE_AT_LIMIT；
 *   3. 用正运动学 recFromServo() 重算末端坐标，并夹到 rangeLimit 内。
 *      坐标被夹回说明"这个角度在几何上跑出了位置边界"，整步回退，
 *      返回 MOVE_AT_LIMIT（摇杆表现为"推到头了"）；
 *   4. 接受本次移动，返回 MOVE_OK。Pos.ser 与 Pos.rec 此时严格对应。
 *
 *   dir 取 JointDir 的值。stepSize 传 <= 0 时使用全局 speed.stepSize（随调速档位变化）。
 *   返回 enum MoveResult；返回 MOVE_OK 以外的值时 Pos 完全不变。 */
int moveJointStep(int dir, double stepSize);

/* ==================== 移动到指定 x,y,z（绘图 / 取放共用） ====================
 *
 * 「移动到指定坐标」在本工程只有这一份实现：反解目标点 -> 按关节速率上限把
 * b/r/c 朝解算结果推进一格。绘图的三处点位写入（直线平移 moveTick、轨迹跟随
 * pathAdvance、示教点动 teachJogTick）与取放模块都调用这里，不再各自展开一份
 * "反解 + 限速 + 写角度"的代码。串口 x/y/z 按题目要求直接写关节角，不经过这里。
 *
 * 参数:
 *   goal[3]  目标点，与 Pos.rec 同一坐标系（肩关节为原点的内部坐标，z 从肩算起）。
 *            调用者：绘图模块（轨迹采样点）与取放模块；串口 x/y/z 指令按题目
 *            要求直接写关节角，不再走这里。
 *   seen     位掩码：位 0/1/2 分别表示 x/y/z 被本次请求提及。**没被提及的轴
 *            沿用 Pos.rec 的当前值**（串口 "x10" 这类单轴命令就是这么处理的），
 *            传 0x07 表示三个轴都要走到 goal。
 *   maxDps   关节角速度上限（度/秒）。**只在 TRACK / JOG 下有意义**：
 *            <= 0（或 dtSec <= 0）时本步上限退化到下限 0.2 度 —— 也就是最严格的
 *            限速，超出的点照样被拒绝 / 夹取。只有 MOVE_XYZ_NOW 是真正不限速、
 *            一次调用直接到位，它根本不读这两个参数。
 *   dtSec    本轮时长（秒）；与 maxDps 相乘就是本步允许的最大关节变化
 *            （下限 0.2 度，极短的一轮也给一点步长）。
 *   mode     逼近策略，见下面三个 MOVE_XYZ_*。
 *
 * 返回 enum MoveXyzResult。**只有 MOVE_XYZ_OK 会写 Pos**，其余情况 Pos.ser 与
 * Pos.rec 一个字节都不改，调用方可以直接把失败当成"这一轮什么也没发生"。
 *
 * 【与角度模式的关系】本函数是"坐标模式"的入口：写进去的仍然是关节角，
 * 写完立刻用正运动学刷新 Pos.rec，所以不变量（坐标 == 角度的真实结果）不变。 */
enum MoveXyzResult {
  MOVE_XYZ_REJECTED = 0,   /* 解不出来 / 被硬限位吸附 / 本步超出速率上限（严格策略） */
  MOVE_XYZ_OK       = 1    /* 已按策略写入新的关节角，并刷新了 Pos.rec */
};

/* 逼近策略 —— 三种，正好对应三个调用方:
 *   MOVE_XYZ_TRACK 轨迹跟随（绘图的直线平移与绘制）：解算结果里任一关节本步
 *                  需要的变化超过 maxDps×dtSec 就整点拒绝（轨迹参数不推进，
 *                  调用方会折半位移重试）；反解被关节硬限位吸附同样算拒绝。
 *   MOVE_XYZ_JOG   示教点动：超出上限的部分夹到上限（尽量走一点），被吸附
 *                  仍然算失败 —— 手动操作宁可到不了请求点，也绝不原地卡住。
 *   MOVE_XYZ_NOW   即时到位（串口 x/y/z）：不限速、一次调用直接落到目标姿态，
 *                  并允许硬限位吸附 —— 与串口原来的"严格移动"逐位相同。
 *                  【唯一差异】旧代码在 recFromServo() 失败时会打一句
 *                  WEARM_DEBUG_SERIAL 调试警告，这里不打了（返回值本来就被丢弃，
 *                  姿态、回包与结果码完全不变）。 */
#define MOVE_XYZ_TRACK 0
#define MOVE_XYZ_JOG   1
#define MOVE_XYZ_NOW   2

int moveToPoint(const double *goal, uint8_t seen, double maxDps, double dtSec, int mode);

#endif //WEARM_MOVE_H
