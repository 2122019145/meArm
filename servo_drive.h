/*
 * servo_drive.h —— 4 路舵机驱动（Timer1 顺序脉冲，替代 Arduino Servo 库）
 *
 * 【为什么要自己写（v1.3.0 容量整合）】
 *   Servo 库是全平台通用实现：它要为"任意通道数、任意引脚、运行期 attach/detach、
 *   读回角度、多定时器实例"付出代价。本工程只用到其中极小一部分
 *   （固定 4 路、固定引脚、只写 0~180°、不 detach、不读回）。在 ATmega328P 上
 *   Servo 库实际占用 flash：
 *       __vector_11 (Timer1 ISR) 376 B
 *       Servo::write             248 B
 *       Servo::attach            240 B
 *       turnOffPWM                82 B
 *       Servo 全局数组静态构造     40 B
 *       合计约 986 B
 *   本文件用同一套 Timer1 机制重写等价功能，约 300 B，净省约 650 B。
 *
 * 【脉宽映射与 Servo 库逐位一致（这是不改变机械行为的硬要求）】
 *   meArm.ino（当时还叫 wearm.ino）原来是 servos[i].write((int)angle)，走 Servo::write(int)：
 *     1) angle < 544 视为角度 -> 夹到 0~180 -> map(angle, 0, 180, 544, 2400)
 *                              = angle * 1856 / 180 + 544  （32 位长整型运算）
 *     2) writeMicroseconds(): 夹到 [544, 2400] -> 减去 TRIM_DURATION(2us)
 *                              -> usToTicks(): (16MHz * us) / 8 = us * 2
 *   所以最终比较值 = ((us - 2) * 2) 个 Timer1 tick，tick = 0.5us（预分频 8）。
 *   本文件的 servoDriveTicksForDeg() 逐位复刻上式；另外本工程四个关节角由
 *   servoLimit 保证恒在 0~180 内，不会落进">= 544 视为微秒"的分支，
 *   故只实现角度分支（少一处判断，也少一处歧义）。
 *
 * 【时序（与 Servo 库相同的 20ms 帧）】
 *   每个 20ms 帧：先按通道 0,1,2,3 顺序连续输出 4 个脉冲（脉冲之间无空隙，
 *   与 Servo 库 handle_interrupts() 一致），4 路发完后等帧尾。
 *   Timer1 跑**普通模式**（WGM12 = 0，比较匹配不清零 TCNT1），所有比较值都是
 *   绝对计数：帧边界把 TCNT1 清零，并在**同一次中断**里拉起第 0 路 ——
 *   即"帧边界 = 第 0 路脉冲上升沿"，帧长因此严格 40 000 tick = 20ms。
 *   （帧尾的比较值在"4 路都发完"那一次中断里就算好 = 绝对计数 SERVO_FRAME_TICKS，
 *   那时计数器只有一万多，一定还在未来。**不能**等到帧尾中断里现算
 *   "还差多少"：中断入口读到的 TCNT1 已越过 40000，uint16 下溢会把下一帧
 *   推到约 52.8ms 之后 —— 帧长变成 19Hz 并抖动。见 servo_drive.cpp 的长注释。）
 *
 * 【可测性】
 *   硬件相关代码只有 servoDriveBegin() 里的 5 行寄存器操作和 ISR 壳子
 *   （用 #if defined(__AVR__) 隔开）。脉冲状态机 servoDriveStep() 是纯函数：
 *   给定"当前计数器值"，它决定本次事件做什么（拉高/拉低哪个引脚）并返回
 *   下一次比较值。PC 端自检（probe_servo_drive）直接按 tick 步进这个状态机，
 *   用 mock 的 digitalWrite 记录验证"引脚顺序 + 脉宽 + 帧长"，
 *   所以时序逻辑不是"只能上硬件才能验证"。
 */
#ifndef SERVO_DRIVE_H
#define SERVO_DRIVE_H

#include <Arduino.h>

/* 通道数。通道 0/1/2/3 依次对应关节 b/r/c/f（原来的 servos[1..4]）。 */
#define SERVO_CH_COUNT 4u

/* 启动 Timer1 顺序脉冲发生器。必须在 servoDriveAttach() 之前调用一次。 */
void servoDriveBegin(void);

/* 把通道 ch 绑到引脚 pin（内部 pinMode(OUTPUT) + 拉低）。未绑定的通道不会被驱动。 */
void servoDriveAttach(uint8_t ch, uint8_t pin);

/* 写入目标角度（度）。整数部分截断，与原来的 write((int)angle) 一致。 */
void servoDriveWrite(uint8_t ch, double deg);

/* 纯函数：角度 -> Timer1 比较值（0.5us/tick）。与 Servo 库逐位一致，供自检比对。 */
uint16_t servoDriveTicksForDeg(double deg);

/* 纯函数：脉冲状态机。推进一个事件：
 *   nowTicks   当前 TCNT1 计数（上一次事件若返回 1，调用者已把 TCNT1 清零）
 *   nextDelayTicks 出参，下一次比较值：
 *                  返回 1 时 —— 先 TCNT1 = 0，再 OCR1A = *nextDelayTicks
 *                  返回 0 时 —— OCR1A = *nextDelayTicks（已经是绝对计数）
 *   返回 1 表示"本次是帧起点（需要把计数器清零）"，返回 0 表示其它事件。 */
uint8_t servoDriveStep(uint16_t nowTicks, uint16_t *nextDelayTicks);

#endif /* SERVO_DRIVE_H */
