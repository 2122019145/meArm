/*
// serial_protocol.h
// 串口命令协议：固定指令通信 + 多舵机协同（x/y/z 三舵机同步角度）指令
//
// 命令表:
//   O                             爪子张开（angle4 走到 f 行程上限）
//   S                             爪子关闭（angle4 走到 f 行程下限）
//   H / L                         整体运行速度 提升 / 降低 一档
//   x角度,y角度,z角度              三舵机同步角度指令，例: x10,y30,z20
//                                 可只写其中一部分（如 y45），未出现的轴不动
//   1 / 2 / 3                     兼容旧命令：直接切到 慢/中/快 档
//   k / K                         兼容旧命令：爪子步进张开/收回
//
// 舵机对应关系（可改 protocol_constants.cpp 里的 protoAxisServoIndex）:
//   x -> angle1 = b 基座回转
//   y -> angle2 = r 上臂俯仰
//   z -> angle3 = c 下臂俯仰
//   angle4 = f 末端夹具不在 x/y/z 之列，只由 O/S/k/K 控制
//
// 【单一读者】串口字节只准由 serialProtocolLoop() 读取。旧的
// handleSerialSpeedCmd() 已被本模块取代并删除 —— 两个读者会把同一串
// 数据各吃掉一半，行缓冲永远拼不出完整指令。
//
*/

#ifndef WEARM_SERIAL_PROTOCOL_H
#define WEARM_SERIAL_PROTOCOL_H

#include "constant_and_positions.h"

/* protoHandleLine() 的返回值，同时也是"这条命令做了什么"的记号 */
#define PROTO_RES_NONE          0  /* 空行或只有空白，什么都没做 */
#define PROTO_RES_GRIPPER_OPEN  1  /* O 爪子张开 */
#define PROTO_RES_GRIPPER_CLOSE 2  /* S 爪子关闭 */
#define PROTO_RES_SPEED_UP      3  /* H 速度提升 */
#define PROTO_RES_SPEED_DOWN    4  /* L 速度降低 */
#define PROTO_RES_SPEED_LEVEL   5  /* 1 或 2 或 3 直接指定档位 */
#define PROTO_RES_TOOL_STEP     6  /* k 或 K 末端步进开合 */
#define PROTO_RES_ANGLES_SET    7  /* x/y/z 角度指令已同步写入 */
#define PROTO_RES_UNKNOWN       8  /* 无法识别的命令（未改动任何状态） */
#define PROTO_RES_BAD_SYNTAX    9  /* 像角度指令但语法错（未改动任何状态） */

/* 初始化串口：Serial.begin(PROTO_BAUD) 并打印命令表。
 * 在 setup() 里调用一次，要放在其它会往串口打印的初始化之前。 */
void serialProtocolBegin(void);

/* 每轮 loop() 调用一次：把串口收到的字符攒成一行并执行。
 * 注意: 旧接口 handleSerialSpeedCmd() 已被本函数取代并删除，不要再同时调用两者
 *       —— 两个读者会把同一串数据各吃掉一半，行缓冲永远拼不出完整指令。 */
void serialProtocolLoop(void);

/* 直接处理一整行命令（不含换行符）。
 * 串口主循环与 .selfcheck 的探针都走这里，保证"探针测的就是固件真正执行的那段代码"。
 * 返回上面的 PROTO_RES_ 之一。 */
int protoHandleLine(const char *line);

#endif