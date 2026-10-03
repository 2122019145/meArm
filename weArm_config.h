/*
 * weArm_config.h -- 全局编译期配置（v1.1.0 引入）
 *
 * ============================ 为什么会有这个文件 ============================
 * Arduino Uno（ATmega328P）只有 32 KB flash（可用 32256 B）和 2 KB SRAM。
 * v1.0.0 把取放、四按键、绘图三大功能全部编进同一个固件后，
 * 编译结果是 51452 B / 2319 B，直接超限（159% / 113%）。
 *
 * v1.1.0 做了一轮逐函数级的体积压缩（做法见 README 的「v1.1.0 容量与配置」），
 * 三功能全开降到 32842 B / 1163 B，比 v1.0.0 小 36%，但**仍然差 586 B**。
 * 所以出厂默认配置改成「取放 + 绘图」，四按键默认关闭。
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
 *                             （很占 flash，排故时才开）
 *   WEARM_ENABLE_PICK_PLACE   1 -> 取放序列（串口 A/B/C、按键1 循环取放）
 *   WEARM_ENABLE_BUTTONS      1 -> 四按键（录制/播放/回中/循环取放）
 *   WEARM_ENABLE_DRAW         1 -> 绘图（铅笔轨迹，v1.0.0 的功能）
 *
 * 摇杆、串口协议解析、反解/限位这些底层能力没有开关，永远编译进去。
 *
 * 如果要批量改动（例如 PC 端自检要打开全部功能），可以在命令行上用
 * -D 覆盖，不会被本文件的默认值挡住：
 *     -DWEARM_ENABLE_PICK_PLACE=1 -DWEARM_ENABLE_BUTTONS=1 -DWEARM_ENABLE_DRAW=1
 *
 * 实测容量（Program / Data，上限 32256 / 2048）：
 *     取放 + 绘图     + 按键关 : 29476 B (91.4%) /  638 B (31.2%)  <- 出厂默认
 *     取放 + 四按键   + 绘图关 : 20126 B (62.4%) /  953 B (46.5%)
 *     绘图 + 四按键   + 取放关 : 30074 B (93.2%) / 1129 B (55.1%)
 *     取放 + 四按键   + 绘图   : 32842 B (101.8%) / 1163 B (56.8%)
 *                                ^ 三功能同时编进去还是装不下，差 586 B
 * 本文件的默认值 = 实测能装进 Uno 的配置。
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
 * v1.1.0 默认关闭。想用按键，就把下面其它开关里的某一项改成 0：
 *     关取放（WEARM_ENABLE_PICK_PLACE 0）-> 30074 B，装得下
 *     关绘图（WEARM_ENABLE_DRAW       0）-> 20126 B，装得下 */
#ifndef WEARM_ENABLE_BUTTONS
#  define WEARM_ENABLE_BUTTONS 0
#endif

/* 绘图：铅笔轨迹绘制（v1.0.0 新增，串口 F/D/G/E/Q/U/W/p/n/o） */
#ifndef WEARM_ENABLE_DRAW
#  define WEARM_ENABLE_DRAW 1
#endif

#endif /* WEARM_CONFIG_H */
