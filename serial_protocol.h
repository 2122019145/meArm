/*
// serial_protocol.h
// 串口命令协议：固定指令通信 + 三舵机同步角度指令（x/y/z）
//
// 命令表:
//   O                             爪子张开（angle4 走到 f 行程上限）
//   S                             爪子关闭（angle4 走到 f 行程下限）
//   H / L                         整体运行速度 提升 / 降低 一档
//   x角度,y角度,z角度              三舵机同步角度指令，例: x10,y30,z20
//                                 轴与舵机对应: x = 基座舵机(angle1/b)
//                                              y = 上臂舵机(angle2/r)
//                                              z = 下臂舵机(angle3/c)
//                                 一行里写到的轴在同一次正解刷新中一起写入（同步控制）
//                                 可只写其中一部分（如 y30），未写出的关节保持不动
//                                 超出机械行程的值按 servoLimit 夹取（x200 最终写 180）
//                                 语法：字母、可选 '='、逗号分隔、支持小数
//   A / B / C                     启动物体 A/B/C 的自动取放序列，序列执行期间只有调速指令仍然有效、其它动作指令返回 BUSY
//   N / R / P / M / 0             四个实体按键的串口孪生（需要「按键」模块已编译且启用，默认已启用）：
//                                 N=循环取放  R=录制（第一次按下开始，再按一次结束并保存）
//                                 P=播放录制轨迹  M 或 0=回中。R 不会"无反应"：
//                                 能开始时回 OK，正忙回 BUSY，没走出位移被丢弃回 DISCARD。
//   F / D / G / E / Q / U / W     绘图（需要「绘图」模块已编译且启用，默认已启用）：
//                                 选图形 / 开始 / 记一个示教点 / 撤销示教点 / 暂停 / 继续 / 取消
//   p / n / o                     绘图标定：纸面高度 / 图形半宽 / 图形中心（如 p12.5、n6、o20,0）
//   !                             查询编译且当前启用的功能 (P=取放 B=按键 D=绘图)
//   !P / !B / !D                  切换已编译的对应功能；未编译的功能不能运行时开启
//
// 每条命令都会回一行短回复（OK / BUSY / OFF / DISCARD / EMPTY / ERR / REJECTED），
// 含义见下面 WEARM_SERIAL_RESPONSES 的说明。
//
// 【x/y/z 就是三个舵机的角度，不做逆解】
//   一条 x10,y30,z20 直接写基座/上臂/下臂三个舵机的角度（单位：度），
//   写入前按各关节的真实机械行程 servoLimit 夹取，写完后做一次正解刷新
//   Pos.rec，所以"给出的角度"与"机械臂的位置"始终是同一件事。
//   一行里出现多个轴 = 三个舵机在同一轮 PWM 输出里同步到位。
//   历史：v1.3.0 ~ v1.6.3 曾把同一串指令当作末端空间坐标（内部做逆解），
//   现按题目要求改回角度语义。坐标版源码备份在 git 分支
//   backup/xyz-cartesian（= tag v1.6.3）以及 dist/ 目录下的压缩包里。
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
 * answer every command with a very short UPPERCASE line, so the host still
 * sees what happened even when the verbose WEARM_DEBUG_SERIAL traces are off:
 *
 *   OK           命令已执行（O/S 带夹爪角度：OK #；F 带图形模式：OK F=<模式>）
 *   BUSY         命令看懂了但机械臂正忙（取放序列/录放/绘图进行中）
 *   REJECTED     这台固件没编进该模块；或绘图轨迹校验不过
 *                （x/y/z 角度指令只会按行程夹取，不会因为数值大小被拒）
 *   OFF          固件里有该模块，但被 !P/!B/!D 关掉了
 *   DISCARD      R 结束录制时数据不合格（没位移/没有动作/缓冲满），已丢弃
 *   EMPTY        P 播放时还没有录制数据
 *   ERR          语法错（比如把 x,y,z 写坏，或给单字符命令加了尾巴）
 *
 * The per-step traces stay behind WEARM_DEBUG_SERIAL in serial_protocol.cpp.
 * The response layer avoids Serial.print() for numbers: that would drag the
 * Arduino number/float formatting layer (~9.5 KB) into the image. It writes
 * through protoPut() (a raw UDR0 write on the Uno release build). */
#ifndef WEARM_SERIAL_RESPONSES
#define WEARM_SERIAL_RESPONSES 1
#endif

/* protoHandleLine() 的返回值，同时也是"这条命令做了什么"的记号 */
#define PROTO_RES_NONE          0  /* 空行或只有空白，什么都没做 */
#define PROTO_RES_GRIPPER_OPEN  1  /* O 爪子张开 */
#define PROTO_RES_GRIPPER_CLOSE 2  /* S 爪子关闭 */
#define PROTO_RES_SPEED_UP      3  /* H 速度提升 */
#define PROTO_RES_SPEED_DOWN    4  /* L 速度降低 */
#define PROTO_RES_SPEED_LEVEL   5  /* 保留未用：原 '1'/'2'/'3' 档位直达（与题目无关，已删） */
#define PROTO_RES_FEATURE       6  /* ! 查询、!P/!B/!D 切换运行时功能位 */
#define PROTO_RES_ANGLES_SET    7  /* x/y/z 角度指令已写入三个舵机（同一次正解刷新） */
#define PROTO_RES_UNKNOWN       8  /* 无法识别的命令（未改动任何状态） */
#define PROTO_RES_BAD_SYNTAX    9  /* 像角度指令但语法错（未改动任何状态） */
#define PROTO_RES_PICK_STARTED 10 /* A/B/C 取放序列已启动 */
#define PROTO_RES_BUSY         11 /* 动作指令被序列挡下：序列正在执行，或本次请求无法启动 */

/* 按键命令的返回值（N/R/P/M，实现见 button_control.cpp） */
#define PROTO_RES_REC_STARTED    12 /* R：开始录制 */
#define PROTO_RES_REC_SAVED      13 /* R：结束录制且数据合格，已保存 */
#define PROTO_RES_REC_REJECTED   14 /* R：结束录制但数据不合格（没位移/缓冲满），已丢弃 */
#define PROTO_RES_PLAY_STARTED   15 /* P：开始播放 */
#define PROTO_RES_PLAY_NO_RECORD 16 /* P：还没有录制数据可播 */
#define PROTO_RES_HOME_STARTED   17 /* M/0：开始回中 */

/* 绘图命令的返回值（F/D/G/E/Q/U/W 与 p/n/o，实现见 draw_control.cpp） */
#define PROTO_RES_DRAW_TASK_SELECTED 18 /* F：切换绘制任务（直线/字母V/字母N/三角形/字母Z/五点折线/五点曲线） */
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