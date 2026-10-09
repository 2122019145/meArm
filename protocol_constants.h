/*
// protocol_constants.h
// 串口命令协议层常量（固定指令通信 + x/y/z 三舵机同步角度指令）
//
// 【为什么单独一个文件】
//   协议相关的"可调数字"集中在这里：命令字符、轴字母、轴与舵机的对应表、
//   行缓冲大小、波特率、速度档位边界。
//   上位机的命令文档要改时，只改这一个文件即可。
//
// 【不重复定义真值】
//   1. 每个关节的机械行程（例如爪子的 60/150）唯一真值是
//      constant_and_positions.cpp 里的 servoLimit，本文件不写死，
//      只通过 protoAxisGetLimit() 转发，避免出现两处真值。
//   2. 速度档位的具体参数（0.5/20/80 等）唯一真值在 adjustSpeed()，
//      本文件只引用档位号。
//
// 【角度上下限只在 servoLimit 里定义】
//   本文件不定义角度上下限（没有 PROTO_ANGLE_MIN/MAX 这类名义区间）。解析
//   阶段不会因为角度超出某个区间而拒绝指令（那样 "x200" 就变成废指令了），
//   而是在落地前按该关节真实的 servoLimit 行程夹取，例如 x200 最终写 180。
//
*/

#ifndef WEARM_PROTOCOL_CONSTANTS_H
#define WEARM_PROTOCOL_CONSTANTS_H

#include "constant_and_positions.h"

/* ---------- 1) 单字符固定指令 ---------- */
#define PROTO_CMD_GRIPPER_OPEN   'O'
#define PROTO_CMD_GRIPPER_CLOSE  'S'
#define PROTO_CMD_SPEED_UP       'H'
#define PROTO_CMD_SPEED_DOWN     'L'

/* A/B/C 自动取放指令：分别启动物体 A/B/C 的取放序列 */
#define PROTO_CMD_PICK_A    'A'
#define PROTO_CMD_PICK_B    'B'
#define PROTO_CMD_PICK_C    'C'

/* 四个物理按键的串口等价命令（效果与按下按键完全一样）。
 * 用字母而不是数字：数字键留给将来扩展，且与绘图任务的弹点时序无关。 */
#define PROTO_CMD_BTN_CYCLE    'N'   /* 按键1 循环执行：下一次按顺序夹 A/B/C */
#define PROTO_CMD_BTN_RECORD   'R'   /* 按键2 录制：第一次开始，第二次结束并保存 */
#define PROTO_CMD_BTN_PLAY     'P'   /* 按键3 播放上一次录制的动作 */
#define PROTO_CMD_BTN_HOME     'M'   /* 按键4 回中：回到开机初始位姿 */
#define PROTO_CMD_BTN_HOME_ALT '0'   /* 回中的别名（'0' 没有被别的命令占用） */

/* 绘图命令（v1.0.0 新增，实现见 draw_control.cpp）。
 * 选这些字母的理由：都是此前未被占用的字符，且不与 x/X/y/Y/z/Z 三个角度轴字母冲突
 * （轴字母开头的行会走角度解析，绘图命令一律用别的字母）。 */
#define PROTO_CMD_DRAW_TASK    'F'   /* 切换绘制任务：直线/字母V/字母N/三角形/字母Z/五点折线/五点曲线 */
#define PROTO_CMD_DRAW_START   'D'   /* 开始绘制（内置图形直接画；示教任务进入五点示教） */
#define PROTO_CMD_DRAW_RECORD  'G'   /* 记录一个示教点（等价于示教中按按键1） */
#define PROTO_CMD_DRAW_UNDO    'E'   /* 撤销一个示教点（等价于示教中按按键2） */
#define PROTO_CMD_DRAW_PAUSE   'Q'   /* 暂停（等价于绘制中按按键1） */
#define PROTO_CMD_DRAW_RESUME  'U'   /* 继续（等价于绘制中按按键2） */
#define PROTO_CMD_DRAW_CANCEL  'W'   /* 取消（等价于绘制中按按键3） */

/* 纸面标定命令（多字符，形如 p12.5 / n6 / o20,0），用于上机时把"纸面"告诉固件 */
#define PROTO_CMD_DRAW_PAPER_Z 'p'   /* p<纸面高度>：铅笔尖落在纸上时的 z */
#define PROTO_CMD_DRAW_HALF    'n'   /* n<半宽>：内置图形的半宽 */
#define PROTO_CMD_DRAW_CENTER  'o'   /* o<中心x>,<中心y>：内置图形的中心 */

/* ---------- 2) x/y/z 三舵机同步角度指令 ---------- */
#define PROTO_AXIS_COUNT 3
/* 轴的字符（大小写都接受，见 protoAxisIndexFromChar） */
extern const char protoAxisChar[PROTO_AXIS_COUNT];
/* 轴对应的关节字母：b = 基座(angle1) r = 上臂(angle2) c = 下臂(angle3)，
 * 只用于串口提示；写入本身按"轴 a 写 angle(a+1)"进行。 */
extern const char protoAxisJoint[PROTO_AXIS_COUNT];

/* ---------- 3) 解析与缓冲区 ---------- */
#define PROTO_LINE_BUF_SIZE 40
#define PROTO_BAUD          115200
/* 一行指令迟迟收不到结束符时，静默这么久就按"整行已到"处理，
 * 兼容串口助手不勾"发送新行"的情况（详见 serial_protocol.cpp）。 */
#define PROTO_LINE_TIMEOUT_MS 300

/* ---------- 4) 速度档位边界 ---------- */
#define PROTO_SPEED_LEVEL_MIN SPEED_SLOW
#define PROTO_SPEED_LEVEL_MAX SPEED_FAST
#define PROTO_SPEED_LEVEL_DEF SPEED_NORMAL

/* 把 x/X/y/Y/z/Z 转成轴下标 0..2，其它字符返回 -1 */
int protoAxisIndexFromChar(int c);

/* 取第 axis 个轴（0..2）的关节行程上下限，直接转发 servoLimit。
 * axis 越界或指针为空时返回 false。 */
bool protoAxisGetLimit(int axis, double *minAngle, double *maxAngle);

#endif