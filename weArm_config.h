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
 * 三功能全开现在是 31604 B / 994 B，离可用 flash 只剩 652 B —— 已经很挤了，
 * 后面再加功能之前，先用下面的容量表挑一个开关关掉，或者重新做一轮瘦身。
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
 *   WEARM_ENABLE_BUTTONS      1 -> 按键模块（录制/播放/回中/循环取放，串口 N/R/P/M/0）
 *   WEARM_ENABLE_DRAW         1 -> 绘图（铅笔轨迹，v1.0.0 的功能）
 *   WEARM_BUTTON_PINS         0 -> 不读 D2~D5 的物理按键（默认，本机没有独立按键）
 *                             1 -> 四个独立按键接 D2~D5（INPUT_PULLUP、按下接地）
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
 * 实测容量（Program / Data，v1.4.0 实编；avr-size 的百分比按 32768 / 2048 算）：
 *     取放 + 按键 + 绘图       : 31604 B (96.4%) /  994 B (48.5%)  <- 出厂默认
 *     取放 + 绘图   + 按键关   : 28756 B (87.8%) /  497 B (24.3%)
 *     取放 + 按键   + 绘图关   : 18884 B (57.6%) /  786 B (38.4%)
 *     绘图 + 按键   + 取放关   : 28848 B (88.0%) /  960 B (46.9%)
 * 默认配置相对 Uno 可用 flash（32256 B）还剩 652 B 余量，改动前务必复测。
 * 对照 v1.1.0 的旧数字（29476 / 20126 / 30074 / 32842）看版本演进。
 * （v1.3.0 的四行是 31936 / 28534 / 19226 / 29184：摇杆中位自标定、方向镜像、
 *   录制中按 P 先收尾再播放这三个实机修复，一共让体积涨了 200~240 B；
 *   本轮默认 WEARM_BUTTON_PINS=0、不再读 D2~D5，省回 510 B flash / 28 B SRAM，
 *   删掉"录制时长必须 >10 秒"的判定又省 58 B（那三行含按键模块的配置各减 58 B），
 *   最终默认配置比 v1.3.0 净省 332 B。）
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

/* 按键模块：录制 / 播放 / 回中 / 循环取放。
 * 默认开启；触发方式默认是串口 N / R / P / M（或 0），物理引脚见第 4 节
 * 的 WEARM_BUTTON_PINS（默认 0 = 不读 D2~D5）。 */
#ifndef WEARM_ENABLE_BUTTONS
#  define WEARM_ENABLE_BUTTONS 1
#endif

/* 绘图：铅笔轨迹绘制（v1.0.0 新增，串口 F/D/G/E/Q/U/W/p/n/o） */
#ifndef WEARM_ENABLE_DRAW
#  define WEARM_ENABLE_DRAW 1
#endif

/* ---------------------------------------------------------------------------
 * 3) 机械装配方向镜像
 *    实机装配时，基座回转（通道 0 / b）与末端夹爪（通道 3 / f）的舵机是
 *    反向装上的：逻辑角增大时它们物理上朝反方向走。实机表现：
 *      - 左右旋转的舵机转向反了（摇杆/坐标指令给的方向与机械实际相反）；
 *      - 串口 O（张开 = 逻辑角 maxF）实际把爪子夹紧、S（夹紧 = minF）反而张开。
 *    修法不动 0~180 的整套映射，也不动任何逻辑角，只在唯一写出口
 *    writeServo() 里对这两个通道做一次"在自身行程窗口内镜像"：
 *        physical = (min + max) - logical
 *    逻辑角（限位、反解、录制回放、绘图、示教）仍是唯一真值，变的只是最后
 *    送给舵机的那个数。镜像后 physical 仍落在 [min,max] 区间内：
 *    b 是 [0,180] 对称、f 是 [60,150] 对称（60+150=210），所以夹爪不可能被
 *    推到机械危险区。
 *    换一套装配（舵机正着装）就把对应开关改成 0，其它文件一行都不用动。
 * ------------------------------------------------------------------------ */
#ifndef WEARM_MIRROR_BASE
#  define WEARM_MIRROR_BASE 1   /* 1 = 基座回转（通道 0 / b）反向装配，输出镜像 */
#endif
#ifndef WEARM_MIRROR_TOOL
#  define WEARM_MIRROR_TOOL 1   /* 1 = 末端夹爪（通道 3 / f）反向装配，输出镜像 */
#endif

/* ---------------------------------------------------------------------------
 * 4) 物理按键引脚开关（默认不用物理按键）
 *    本机只接了两根摇杆（A0~A3 的 X/Y）：D2~D5 上没有独立按键，曾经把
 *    KY-023 摇杆自带的按压开关（SW 脚）接到 D2 = 按键1 = 循环取放。实机问题：
 *    把**右摇杆推到最前**时机械上会压到那颗轻触开关 → 假触发一次"按键1"
 *    → 正在手动推杆的操作被 pickPlaceStart() 抢走（摇杆随即被 locked 锁住），
 *    现象就是"右摇杆向前本来该转下臂，结果触发了自动取放"。
 *
 *    所以默认 0：**完全不读 D2~D5**（连 pinMode 都不做，省下 4 路扫描与
 *    24 字节消抖状态），四个按键的全部功能改由串口字符触发：
 *        N      循环执行：夹 A -> B -> C -> A（= 原按键1）
 *        R      录制 开/关（= 原按键2）
 *        P      播放上一次录制（= 原按键3；录制中直接发 P 会先收尾再播放）
 *        M / 0  回中：平滑回到开机初始位姿（= 原按键4）
 *    改成 1 就恢复"四个独立按键接 D2~D5、INPUT_PULLUP、按下接地"的老接线。
 *
 *    注意：这个开关只裁剪"读引脚"那一段（buttonSetup/buttonLoop/btnEdge）。
 *    button_control.cpp 的录制 / 播放 / 回中 / 循环取放逻辑、上面四条串口
 *    命令、（绘图示教中的）按键语义都不受影响；WEARM_ENABLE_BUTTONS 也不必动。
 *    PC 端探针想测物理按键，在包含 button_control.cpp 之前定义 1 即可：
 *        #define WEARM_BUTTON_PINS 1
 * ------------------------------------------------------------------------ */
#ifndef WEARM_BUTTON_PINS
#  define WEARM_BUTTON_PINS 0   /* 0 = 不读 D2~D5（全部走串口），1 = 物理按键 */
#endif

#endif /* WEARM_CONFIG_H */
