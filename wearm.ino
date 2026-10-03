//
// wearm.ino
// 机械臂主程序 (Arduino)
//
// 结构:
//   setup() —— 复位到安全初始角度、初始化串口协议、自标定摇杆中位与状态灯、
//              初始化按键/绘图模块、启动 Timer1 舵机驱动并把 4 路 attach 到引脚
//              （顺序有讲究，见 setup() 里的编号注释）
//   loop()  —— 每轮:
//     1. serialProtocolLoop() 处理串口指令（唯一读者，放在最前面保证及时响应）
//     2. pickPlaceLoop()   推进取放序列（非阻塞状态机，独占 b/r/c 三个关节角）
//     3. buttonLoop()      扫描四个按键 + 推进录制/播放/回中的插值（非阻塞）
//     4. drawLoop()        推进绘图任务（回待机/走位/落笔/绘制/抬笔；示教时读摇杆点动。
//                          每个采样点最终都交给 move.h 的 moveToPoint() 落地）
//     5. joystickLoop()    读手柄（MeArm 套件手柄：两根双轴摇杆，共占 A0~A3），
//                          按全局速度参数直接改变 4 个关节角
//     6. writeServo()      把当前 4 个关节角写入物理舵机（servo_drive，Timer1 硬件 PWM）
//   （取放/按键/绘图三者互斥：各自的 IsBusy()/Locked() 会让另外两个与摇杆整段让位）
//
// 【控制方式：关节角是唯一被控量】
//   被控量: Pos.ser.angle1..angle4（b / r / c / f 四个关节角）—— 摇杆直接加减它们。
//   派生量: Pos.rec.x/y/z —— 由正运动学 recFromServo() 实时算出，用于显示与工作空间
//           校验，不是"用户设定的目标"。
//   坐标输入只有两个入口，最终都落到上面这组关节角上:
//     1) 串口 x/y/z 指令（见下，一次到位）；
//     2) 绘图轨迹的每个采样点（draw_control.cpp）。
//   两者调用的是**同一个函数** move.cpp 的 moveToPoint()——"移动到指定 x,y,z 坐标"
//   在全工程只有这一份实现（内部 getAngleEx() 反解 → 写 Pos.ser → recFromServo()
//   刷新 Pos.rec），所以"坐标 == 角度的真实结果"这条不变量在每个入口上都成立。
//   （v1.0.0 那种"把坐标当目标直接控制"的方式已废弃；坐标入口一律是**严格移动**：
//     解不出来或超出关节速率上限的点整条拒绝、一个字节都不写，绝不会半路停在
//     没人要的姿态上。旧坐标控制的代码已备份到工作区外。）
//
// 手柄操作（详见 joystick_control.h）:
//   左手柄 左右推 (A0) -> b = angle1 基座回转   角度增大 = 末端往 +y 侧转
//   左手柄 前后推 (A1) -> r = angle2 上臂俯仰   角度增大 = 上臂继续抬高
//   右手柄 前后推 (A3) -> c = angle3 下臂俯仰   角度增大 = 下臂往前收
//   右手柄 左右推 (A2) -> f = angle4 末端夹具   角度增大 = 张开
//   ★ 摇杆中位在 setup() 里自标定（各轴取平均、扣掉静态偏差，见 joystick_control.cpp
//     的 JOY_CAL_*）：真实手柄的中位很少正好是 ADC 512，偏差超过死区就会让机械臂
//     在没人碰摇杆时持续慢慢乱走。标定期间别碰摇杆；若某路偏差大得离谱（开机就被
//     压住），该路会退回标准中位 512 而不是按错的偏差算。
//   串口命令（波特率 115200，由 serial_protocol 模块解析，详见 serial_protocol.h）：
//     发 O             爪子张开（angle4 走到 f 行程上限；实机装配方向见下面的镜像说明）
//     发 S             爪子关闭（angle4 走到 f 行程下限）
//     发 H             整体运行速度提升一档
//     发 L             整体运行速度降低一档
//     发 x坐标,y坐标,z坐标   末端**空间直角坐标**（例: x20,y0,z40），不是关节角度：
//                      固件把这行交给 move.h 的 moveToPoint()（与绘图轨迹用的是同一个
//                      核心）反解出 b/r/c 三个关节角并一次到位，夹爪角 f 保持不变；
//                      可只写其中一部分（如 y10），没写到的轴沿用当前坐标。
//                      原点 = 过肩关节的地面垂足，x+ 面朝方向、z+ 向上；
//                      z 是离地高度，负值直接 REJECTED。
//                      解不出来或超出关节行程的点整条拒绝（REJECTED），一个字节都不写。
//                      【破坏性变更】v1.0.0 的 x/y/z 是"三个关节角度"，
//                      同样一条 x10,y30,z20 现在表示一个坐标点。
//     发 A / B / C     自动取放：夹起物体 A/B/C 放到各自的放置点（详见 pick_place.h）
//                      序列执行期间摇杆与其它动作指令让位，只有 H/L/1/2/3 调速仍生效
//                      （旧的 '1'/'2'/'3' 调速与 k/K 末端开合仍兼容）
//     发 N / R / P / M 四个物理按键的等价命令（详见 button_control.h）：
//                      N = 按键1 循环执行，R = 按键2 录制开/关，
//                      P = 按键3 播放（若录制还没结束，P 会先帮你了结录制再播放），
//                      M（或 '0'）= 按键4 回中
//     发 !             查询当前编译且启用的功能（回复 P=… B=… D=…）
//     发 !P / !B / !D  运行时开关 取放 / 按键 / 绘图（没编译进去的回 REJECTED）
//                      注意串口命令与按键有一行短回复（OK / BUSY / DISCARD / …），
//                      看回复才知道命令是否被接受。
//
//   ★ 物理按键（D2~D5）默认不用：本机只接了两根摇杆，D2~D5 上没有独立按键，
//     而摇杆自带的 SW 脚曾被接到 D2（= 按键1 循环取放）——右摇杆推到最前时机械上
//     会压合那颗轻触开关，假触发一次取放、把手动推杆抢走。
//     所以固件默认 WEARM_BUTTON_PINS = 0：**完全不读 D2~D5**（连 pinMode 都不做），
//     按键功能一律用上面那四条串口命令 N / R / P / M（或 0）触发。
//     要恢复"四个独立按键接 D2~D5"的老接线：把 weArm_config.h 里
//     WEARM_BUTTON_PINS 改成 1，别的文件一行都不用动。
//     按键模块本身（录制/播放/回中/循环取放）仍然编译在里面（WEARM_ENABLE_BUTTONS=1），
//     因为串口 N/R/P/M 走的就是它；录制期间摇杆照常可用，播放/回中期间摇杆让位，
//     忙让位规则详见 button_control.h。
//
//   绘图（铅笔固定在末端夹具上，详见 draw_control.h；实现只有 draw_control.cpp 一个文件。
//   每个采样点都交给 move.h 的 moveToPoint()：反解、关节速率限制、写 Pos 与正解刷新都在
//   那里，与上面 x/y/z 指令共用同一份实现，draw_control.cpp 只决定"下一步走到哪个坐标"）:
//     发 F             切换绘制任务：直线 -> 字母N -> 三角形 -> 字母Z -> 字母V -> 五点折线 -> 五点曲线
//     发 D             开始绘制（内置图形直接画；五点折线/五点曲线先进入五点示教）
//     发 G / E         示教中记录 / 撤销一个示教点（等价于示教时按按键1 / 按键2）
//     发 Q / U / W     暂停 / 继续 / 取消（等价于绘制时按按键1 / 按键2 / 按键3）
//     发 p12.5         标定纸面高度（铅笔尖刚好落在纸上时的 z）
//     发 n6            标定内置图形的半宽（顶点落在 ±半宽 的正方形里）
//     发 o20,0         标定内置图形的中心 (x,y)
//   绘图时的按键语义（与空闲时的 循环取放/录制/播放/回中 同一套"按键编号"，只是换了解释）:
//     示教中  "按键1"记录示教点（记满 5 个自动开始）  "按键2"撤销  "按键3"取消  "按键4"手动开始
//     绘制中  "按键1"暂停（停在原地、任务状态保留）  "按键2"继续（从暂停处接续）  "按键3"取消（抬笔回待机）
//     上面的"按键1~4"= 空闲时的那四个编号（BTN_KEY_CYCLE/RECORD/PLAY/HOME = DRAW_KEY_1~4），
//     所以默认不接物理按键时，这四条语义同样由串口 N / R / P / M（或 0）触发，
//     另有专门的绘图命令 G（=记录）/ E（=撤销）/ Q（=暂停）/ U（=继续）/ W（=取消）；
//     示教中的"手动开始"发 D。
//   绘图期间的排他: 取放序列与按键录制/播放/回中期间不允许启动绘图，反之亦然；
//                   绘图进行中摇杆整段让位（示教时由 draw_control.cpp 自己读摇杆做笛卡尔点动）。
//   D13 指示灯            任一关节在动 -> 快闪；全部静止 -> 灭
//                         （取放序列与录制播放期间也算在动，灯会一直闪）
//
//   4 个关节各占一路摇杆轴，可以同时操作，因此不需要切模式。
//   ★ 若发现左右手反了，把 joystick_control.cpp 里 JOY_L/JOY_R 两组引脚对调即可。
//
// 【方向镜像：基座与夹爪在这台机器上是反向装配的】
//   实机装配把 b（基座回转，通道 0）和 f（末端夹爪，通道 3）的舵机装反了：
//   逻辑角增大时它们物理上朝反方向走。表现就是"左右旋转的舵机转向反了"和
//   "抓取时 O/S 反了"（O 走 maxF 本该张开，实际夹紧）。
//   修法只在唯一写出口 writeServo() 里对这两个通道做一次窗口内镜像
//   physical = (min + max) - logical，逻辑角（限位/反解/录制/绘图）仍是唯一真值；
//   镜像后物理角仍落在 [min,max] 内，夹爪不会被推到危险区。
//   换一套装配就把 weArm_config.h 里的 WEARM_MIRROR_BASE / WEARM_MIRROR_TOOL 改成 0。
//
// 安全边界:
//   1. servoLimit 关节硬限位 —— b[0,180] r[0,180] c[0,180] f[60,150]，唯一的软件边界。
//                              摇杆/按键走 moveJointStep()：角度夹到限位内，夹不动了
//                              就停住（MOVE_AT_LIMIT），不会顶死舵机。
//                              坐标入口走 moveToPoint()：反解结果要靠限位吸附才成立的
//                              点，绘图/示教一律拒绝，串口 x/y/z（NOW 策略）允许吸附后到位。
//   2. rangeLimit 位置边界   —— 由新角度正解出的末端坐标若跑出 limit 区间，
//                              这一步整步回退，避免臂跑到没标定的区域。
//   3. 正运动学自洽          —— 每步都用 recFromServo() 重算坐标，
//                              Pos.ser 与 Pos.rec 永远严格对应（坐标入口也一样，
//                              见 move.cpp 的 moveToPoint()）。
//
// 坐标模型（用于显示与工作空间校验，与 constant_and_positions.cpp 一致）:
//   肩关节在坐标原点，上臂 L1 = 下臂 L2 = 20，armheight = 0。
//   舵机中立位 90° 的实测含义:
//     b (angle1) 90° -> 基座朝 +x，b 偏离 90° 就转向 ±y；
//     r (angle2) 90° -> 上臂竖直向上；
//     c (angle3) 90° -> 下臂水平朝前（与上臂成 90°）。
//   初始位姿 (b,r,c,f) = (90,90,90,f 中位) 对应末端 (20, 0, 20)，
//   也就是 constant_and_positions.cpp 里的 POS_HOME。
//   关节限位内的实测可达包络（probe_axes 扫 5929741 个组合）:
//     x[-40.00, 40.00] y[-40.00, 40.00] z[-20.00, 40.00]
//   limit 按包络"向外取整到 0.5"取 x[-40.0,40.0] y[-40.0,40.0] z[-20.0,40.0]
//   （这次包络端值恰好落在 0.5 的整数倍上，所以余量是 0），
//   理由是角度模式下坐标只是派生量，limit 不该比真实包络更早挡住摇杆
//   （详见 constant_and_positions.cpp 里 limit 上方的注释）。
//   ★ 代价: limit 现在是"软护栏"，基座可摆到 x<0 后方、末端最低到 z≈-20.0，
//     上机请先在慢速档、空载、抬离台面的情况下单步试。
//
// 调速:
//   调用 adjustSpeed(level) 切换速度档位:
//     SPEED_SLOW   慢速 (小步长 + 长间隔 ≈ 0.5°/步，精细操作)
//     SPEED_NORMAL 中速 (默认，≈ 1°/步)
//     SPEED_FAST   快速 (大步长 + 短间隔 ≈ 2°/步)
//   或 setSpeed(stepSize, minDelayMs, fullDelayMs) 自定义（stepSize 单位现在是"度"）。
//   调速即时生效，下一轮 loop 的摇杆转动就会使用新参数。
//
#include "servo_drive.h"             /* 4 路舵机（Timer1 硬件 PWM，取代 Servo 库） */
#include "constant_and_positions.h"  /* Pos（唯一被控量）与 servoLimit / rangeLimit */
#include "joystick_control.h"        /* 摇杆读取、板载 LED、全局调速 */
#include "serial_protocol.h"         /* 串口命令解析（x/y/z 指令在这里换算坐标系） */
#include "pick_place.h"              /* 取放序列 A/B/C */
#include "button_control.h"          /* 按键 / 录制 / 播放 / 回中 */
#include "draw_control.h"            /* 绘图任务（采样点交给 move.h 的 moveToPoint()） */

/* 4 个舵机: 通道 0~3 对应各关节（见 servo_drive.h，替代 Arduino Servo 库）。
 * 字母记号（与 constant_and_positions.h 的 servoLimit 一致）:
 *   通道0 = b 水平回转   通道1 = r 上臂俯仰
 *   通道2 = c 下臂俯仰   通道3 = f 末端 */

/* 把当前 4 个关节角写入物理舵机（通道 0..3）。
 *
 * 【方向镜像】基座回转（b）与末端夹爪（f）在这台机器上是反向装配的，所以这两个
 * 通道在各自的行程窗口内做一次镜像：physical = (min + max) - logical。
 * 逻辑角仍是唯一真值（限位/反解/录制/绘图/示教全用逻辑角），只有送到舵机的
 * 那一个数被换向；镜像后仍落在 [min,max] 内，夹爪不会被推到危险区。
 * 换装配就改 weArm_config.h 里的 WEARM_MIRROR_BASE / WEARM_MIRROR_TOOL。 */
void writeServo(void){
#if WEARM_MIRROR_BASE
  servoDriveWrite(0, (servoLimit.minB + servoLimit.maxB) - Pos.ser.angle1);
#else
  servoDriveWrite(0, Pos.ser.angle1);
#endif
  servoDriveWrite(1, Pos.ser.angle2);
  servoDriveWrite(2, Pos.ser.angle3);
#if WEARM_MIRROR_TOOL
  servoDriveWrite(3, (servoLimit.minF + servoLimit.maxF) - Pos.ser.angle4);
#else
  servoDriveWrite(3, Pos.ser.angle4);
#endif
}

void setup() {
  /* 1. 把 Pos 复位到安全的初始角度（含 angle4 取行程中位）。
   *    必须在 writeServo() 之前，否则开机瞬间舵机会被拉到角度 0。 */
  posInit();

  /* 2. 初始化串口协议（Serial.begin(115200) 并打印命令表）。
   *    必须在其它会往串口打印的初始化之前，串口字节也只由本模块读取。 */
  serialProtocolBegin();

  /* 3. 初始化手柄引脚与指示灯（串口不在这里初始化） */
  joystickSetup();

  /* 4. 初始化按键模块（状态机/录制缓冲；D2~D5 的引脚扫描默认关闭，
   *    见 weArm_config.h 的 WEARM_BUTTON_PINS —— 本机不接物理按键） */
  buttonSetup();

  /* 4.5 初始化绘图模块（纸面参数用默认值，上机时用串口 p/n/o 标定） */
  drawSetup();

  /* 5. 启动舵机驱动并把 4 路 attach 到对应引脚。
   *    servoDriveBegin() 必须在 attach 之前：它初始化 Timer1 与各通道默认脉宽。
   *    引脚号请按实际硬件修改 (示例: 9, 7, 8, 6) */
  servoDriveBegin();
  servoDriveAttach(0, 9);   /* 水平回转 */
  servoDriveAttach(1, 7);   /* 上臂俯仰 */
  servoDriveAttach(2, 8);   /* 下臂俯仰 */
  servoDriveAttach(3, 6);   /* 末端 */

  /* 6. 上电后先让机械臂到位，再开始接收手柄输入 */
  writeServo();

  /* 可选: 设置初始速度档位 (默认中速) */
  // adjustSpeed(SPEED_NORMAL);
}

void loop() {
  /* 先处理串口指令（唯一读者），再处理手柄；
   * 串口放在最前面保证及时响应。 */
  serialProtocolLoop();

  /* 推进取放序列（A/B/C 触发的非阻塞状态机，每轮走一步） */
  pickPlaceLoop();

  /* 推进按键模块：扫描按键沿 + 录制采样 / 播放 / 回中的插值（非阻塞） */
  buttonLoop();

  /* 推进绘图任务（回待机/走位/落笔/绘制/抬笔的非阻塞状态机；示教时读摇杆做点动）。
   * 放在 buttonLoop() 之后：按键按下沿先由 button_control.cpp 派发，
   * 绘图任务进行中它会转交给 drawHandleButton()（暂停/继续/取消/记录示教点）。
   * 放在 joystickLoop() 之前：绘图任务进行中 drawControlLocked() 会让摇杆那一段整段让位，
   * 免得摇杆和绘图状态机同时改同一条关节角。 */
  drawLoop();

  /* 读手柄，按全局调速参数直接改变 4 个关节角度 */
  joystickLoop();

  /* 把当前关节角写入物理舵机 */
  writeServo();
}
