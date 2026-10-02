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

/* 方向枚举，供 moveAxisStep 使用 */
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

/* moveAxisStep 的执行结果 */
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

/* 与上面同一个实现，保留旧名字供"按方向轴理解"的调用方使用。
 * DIR_UP/DOWN -> 下臂 c，DIR_FWD/BWD -> 上臂 r，DIR_LEFT/RIGHT -> 基座 b。
 * 新代码请直接用 moveJointStep + JOINT_*，语义更清楚。 */
int moveAxisStep(int dir, double stepSize);

#endif //WEARM_MOVE_H
