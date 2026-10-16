/*
 * servo_drive.h —— 4 路舵机驱动的统一接口（v1.6.6 起封装 Arduino 官方 Servo 库）
 *
 * 【为什么改用官方 Servo 库】
 *   v1.3.0~v1.6.5 这里是一份自研的 Timer1 顺序脉冲驱动（省 flash）。
 *   改回官方库的两个原因：
 *     1) 刷新周期：自研版为塞进 flash 把帧长改到了 30ms，夹爪的保持刚度
 *        直接受刷新率影响（舵机只在收到脉冲时被"顶"住）；官方库是标准
 *        20ms 帧。夹物易松的问题里，这是固件侧能改的第一个杠杆。
 *     2) 一致性：others/ 下的全部参考工程（MeArm-Arduino / Spark-MeArm /
 *        meArm / meArm_MovementRecorder）用的都是官方 Servo 库 —— 同一套
 *        脉宽映射（544~2400us，map(angle,0,180,544,2400)，TRIM 2us），
 *        实机行为可直接与参考工程对表。
 *   代价是 flash（实测见 weArm_config.h 顶部容量表），换来的是标准时序
 *   与久经实机验证的实现。
 *
 * 【接口】保持与自研版完全相同的三个函数，调用方（meArm.ino）一行不用改：
 *   servoDriveBegin()          预留的初始化钩子（官方库在 attach() 里自备时序，
 *                              这里保留空实现以维持 setup() 的调用序）
 *   servoDriveAttach(ch, pin)  把通道 ch 绑到引脚 pin（Servo::attach）
 *   servoDriveWrite(ch, deg)   写目标角度（度）：整数部分截断 + 夹到 0~180，
 *                              与原来 write((int)angle) 的行为一致
 *
 * 【通道与引脚】通道 0/1/2/3 依次对应关节 b/r/c/f；接线（meArm.ino 的
 *   setup()）：D9 = 基座、D8 = 上臂、D7 = 下臂、D6 = 夹爪。
 *   官方库在 Uno 上用 Timer1（与旧自研版相同），D9/D10 的 analogWrite 会被
 *   禁用 —— 本工程没有用到它们，无影响。
 */
#ifndef SERVO_DRIVE_H
#define SERVO_DRIVE_H

#include <Arduino.h>

/* 通道数。通道 0/1/2/3 依次对应关节 b/r/c/f。 */
#define SERVO_CH_COUNT 4u

/* 初始化钩子（保留调用序；官方 Servo 库无需预初始化）。 */
void servoDriveBegin(void);

/* 把通道 ch 绑到引脚 pin（内部 Servo::attach）。重复绑定同一路是允许的。 */
void servoDriveAttach(uint8_t ch, uint8_t pin);

/* 写入目标角度（度）。整数部分截断并夹到 0~180，与原来的 write((int)angle) 一致。 */
void servoDriveWrite(uint8_t ch, double deg);

#endif /* SERVO_DRIVE_H */
