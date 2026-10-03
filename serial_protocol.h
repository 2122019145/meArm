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
//   A / B / C                     启动物体 A/B/C 的自动取放序列，序列执行期间只有调速指令仍然有效、其它动作指令返回 BUSY
//   !                             查询编译且当前启用的功能 (P=取放 B=按键 D=绘图)
//   !P / !B / !D                  切换已编译的对应功能；未编译的功能不能运行时开启
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

/* ===== second-level switch: responses the host must keep =====
 * WEARM_SERIAL_RESPONSES=1 makes the firmware print the boot command table and
 * answer every command with a very short UPPERCASE line (OK / REJECTED / ERR /
 * the angle echo of O and S / the calibration echo of p,n,o), so the host still
 * sees what happened even when the verbose WEARM_DEBUG_SERIAL traces are off.
 * The per-step traces stay behind WEARM_DEBUG_SERIAL in serial_protocol.cpp.
 * The response layer avoids Serial.print() for numbers: that would drag the
 * Arduino number/float formatting layer (~9.5 KB) into the image. It writes
 * through Serial.write() straight out of PROGMEM. */
#ifndef WEARM_SERIAL_RESPONSES
#define WEARM_SERIAL_RESPONSES 1
#endif

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
#define PROTO_RES_PICK_STARTED 10 /* A/B/C 取放序列已启动 */
#define PROTO_RES_BUSY         11 /* 动作指令被序列挡下：序列正在执行，或本次请求无法启动 */

/* 按键命令的返回值（N/R/P/M，实现见 button_control.cpp） */
#define PROTO_RES_REC_STARTED    12 /* R：开始录制 */
#define PROTO_RES_REC_SAVED      13 /* R：结束录制且数据合格，已保存 */
#define PROTO_RES_REC_REJECTED   14 /* R：结束录制但数据不合格（太短/没位移/缓冲满），已丢弃 */
#define PROTO_RES_PLAY_STARTED   15 /* P：开始播放 */
#define PROTO_RES_PLAY_NO_RECORD 16 /* P：还没有录制数据可播 */
#define PROTO_RES_HOME_STARTED   17 /* M/0：开始回中 */

/* 绘图命令的返回值（F/D/G/E/Q/U/W 与 p/n/o，实现见 draw_control.cpp） */
#define PROTO_RES_DRAW_TASK_SELECTED 18 /* F：切换绘制任务（直线/N/三角形/Z/V/五点折线/五点曲线） */
#define PROTO_RES_DRAW_STARTED       19 /* D：绘图任务已启动（内置图形，或进入五点示教） */
#define PROTO_RES_DRAW_PAUSED        20 /* Q：绘制已暂停（停在原地，任务状态保留） */
#define PROTO_RES_DRAW_RESUMED       21 /* U：从暂停处继续绘制 */
#define PROTO_RES_DRAW_CANCELED      22 /* W：取消本次绘图，抬笔后回待机 */
#define PROTO_RES_DRAW_TEACH_POINT   23 /* G：记录一个示教点 */
#define PROTO_RES_DRAW_TEACH_UNDO    24 /* E：撤销一个示教点 */
#define PROTO_RES_DRAW_REJECTED      25 /* 绘图请求被拒：轨迹校验不过 / 示教点不够 / 标定值非法 */
#define PROTO_RES_DRAW_CALIBRATED    26 /* p/n/o：纸面高度、图形半宽、图形中心已更新 */

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