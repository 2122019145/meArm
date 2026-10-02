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

/* 兼容旧命令（v0.2.0 之前就在用，保留不删） */
#define PROTO_CMD_SPEED_SLOW      '1'
#define PROTO_CMD_SPEED_NORMAL    '2'
#define PROTO_CMD_SPEED_FAST      '3'
#define PROTO_CMD_TOOL_OPEN_STEP  'k'
#define PROTO_CMD_TOOL_CLOSE_STEP 'K'

/* 'k' 与 'K' 每次步进的角度（度） */
#define PROTO_TOOL_STEP_DEG      5.0

/* ---------- 2) x/y/z 三舵机同步角度指令 ---------- */
#define PROTO_AXIS_COUNT 3
/* 轴的字符（大小写都接受，见 protoAxisIndexFromChar） */
extern const char protoAxisChar[PROTO_AXIS_COUNT];
/* 轴对应哪个舵机：1 = angle1(b 基座) 2 = angle2(r 上臂) 3 = angle3(c 下臂) */
extern const int  protoAxisServoIndex[PROTO_AXIS_COUNT];
/* 轴对应的关节字母，仅用于串口提示 */
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