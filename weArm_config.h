/*
 * weArm_config.h -- 全局编译期配置（v1.1.0 引入）
 *
 * ============================ 为什么会有这个文件 ============================
 * Arduino Uno（ATmega328P）只有 32 KB flash（avr-size 按 32768 B 计算，
 * 其中 512 B 留给 bootloader，所以实际可用 32256 B）和 2 KB SRAM。
 * v1.0.0 把取放、四按键、绘图三大功能全部编进同一个固件，编译结果是
 * 51452 B / 2319 B，直接超限（avr-size 157% / 113%）。
 *
 * v1.1.0 先把调试日志关掉、再逐函数压缩，三功能全开降到 32842 B，
 * 但仍然差 586 B，所以那一版出厂默认关掉四按键（只留取放 + 绘图）。
 *
 * 本轮又拆掉两个大头，终于让「全功能」成为出厂默认：
 *   1) 不再用 Arduino 的 HardwareSerial：release 构建直接写 UDR0 / UCSR0A，
 *      接收走 USART_RX 中断 + 64 字节环形缓冲（见 serial_protocol.cpp 的
 *      "serial backend" 段）。只要还链接 Serial 对象，它的 vtable 就会把
 *      整个 HardwareSerial 拖进来，逐函数裁剪做不到；整块换掉省了
 *      852 B flash 和 109 B SRAM。想退回官方串口库：-DWEARM_SERIAL_ARDUINO=1。
 *   2) 自研 servo_drive（Timer1 硬件 PWM）取代 Servo 库，省 550 B。
 *
 * 三功能全开现在是 31936 B / 1015 B，离可用 flash 还剩 320 B，默认全开。
 *
 * 这里提供两个维度的编译期开关：
 *   1) WEARM_DEBUG_SERIAL -- 关掉全部串口调试日志（默认关）
 *   2) WEARM_ENABLE_xxx   -- 按功能裁剪（关掉的功能会被链接器完全丢弃，
 *                            既不占 flash 也不占 SRAM）
 *
 * 关闭某个功能时，该模块的头文件会退化成一组空实现的 inline 桩函数，
 * 因此**调用方（weArm.ino / serial_protocol.cpp / joystick_control.cpp /
 * button_control.cpp）一行都不用改**，编译器会把空调用直接优化掉。
 *
 * ============================== 怎么用 =====================================
 * 想改配置，**只需要改本文件里的数字**（0 = 关，1 = 开），其它文件不用动：
 *
 *   WEARM_DEBUG_SERIAL        1 -> 在串口上打印各模块的中文调试日志
 *                             （实测 +9260 B，很占 flash，排故时才开；
 *                              打开后会退回 Arduino Serial，见上面 1)）
 *   WEARM_ENABLE_PICK_PLACE   1 -> 取放序列（串口 A/B/C、按键1 循环取放）
 *   WEARM_ENABLE_BUTTONS      1 -> 四按键（录制/播放/回中/循环取放）
 *   WEARM_ENABLE_DRAW         1 -> 绘图（铅笔轨迹，v1.0.0 的功能）
 *
 * 三个功能开关默认都是 1（全功能）。万一以后代码又长胖到装不下，
 * 照下面的容量表关掉最不常用的那一个即可。
 *
 * 摇杆、串口协议解析、反解/限位、舵机驱动这些底层能力没有开关，永远编译进去。
 *
 * 如果要批量改动（例如 PC 端自检要打开全部功能，或做容量消融实验），
 * 可以在命令行上用 -D 覆盖，不会被本文件的默认值挡住：
 *     -DWEARM_ENABLE_PICK_PLACE=1 -DWEARM_ENABLE_BUTTONS=1 -DWEARM_ENABLE_DRAW=1
 * 注意 avr_build.ps1 的 -ExtraDefs 收的是字符串**数组**：
 *     -ExtraDefs @('-DWEARM_ENABLE_BUTTONS=1','-DWEARM_ENABLE_DRAW=1')
 * 把多个 -D 塞进一个带空格的字符串会被当成一个参数，报
 * "token "=" is not valid in preprocessor expressions"。
 *
 * 实测容量（Program / Data，本轮实测；avr-size 的百分比按 32768 / 2048 算）：
 *     取放 + 按键 + 绘图       : 31936 B (97.5%) / 1015 B (49.6%)  <- 出厂默认
 *     取放 + 绘图   + 按键关   : 28534 B (87.1%) /  490 B (23.9%)
 *     取放 + 按键   + 绘图关   : 19226 B (58.7%) /  807 B (39.4%)
 *     绘图 + 按键   + 取放关   : 29184 B (89.1%) /  981 B (47.9%)
 * 默认配置相对 Uno 可用 flash（32256 B）还剩 320 B 余量。
 * 对照 v1.1.0 的旧数字（29476 / 20126 / 30074 / 32842）看版本演进。
 */
#ifndef WEARM_CONFIG_H
#define WEARM_CONFIG_H

/* ---------------------------------------------------------------------------
 * 1) 串口调试日志总开关
 *    打开后 constant_and_positions / joystick_control / serial_protocol /
 *    pick_place / button_control / draw_control 六个模块都会打印日志。
 *    中文字符串按 UTF-8 每个字 3 字节，实测全开会让固件大 9260 B
 *    （42192 -> 51452）。注意 AVR 的 .rodata 是并进 .text 的，
 *    所以这些字符串只占 flash、**不占 SRAM**（v1.0.0 的注释写错了）。
 *
 *    打开这个开关还会让串口退回 Arduino 的 HardwareSerial（调试日志和
 *    协议回复要共用一条有序输出流，裸寄存器的 TX 不参与 printf 缓冲），
 *    所以排故构建的体积会明显大于 release 构建，属正常现象。
 * ------------------------------------------------------------------------ */
#ifndef WEARM_DEBUG_SERIAL
#  define WEARM_DEBUG_SERIAL 0
#endif

/* ---------------------------------------------------------------------------
 * 2) 功能裁剪开关
 * ------------------------------------------------------------------------ */

/* 取放序列：把 A/B/C 三个物体夹到各自的放置点（串口 A/B/C，按键1 循环） */
#ifndef WEARM_ENABLE_PICK_PLACE
#  define WEARM_ENABLE_PICK_PLACE 1
#endif

/* 四按键：录制 / 播放 / 回中 / 循环取放（D2~D5）
 * 默认开启；串口侧的孪生命令是 N / R / P / M */
#ifndef WEARM_ENABLE_BUTTONS
#  define WEARM_ENABLE_BUTTONS 1
#endif

/* 绘图：铅笔轨迹绘制（v1.0.0 新增，串口 F/D/G/E/Q/U/W/p/n/o） */
#ifndef WEARM_ENABLE_DRAW
#  define WEARM_ENABLE_DRAW 1
#endif

#endif /* WEARM_CONFIG_H */
