//
// wearm.ino
// 机械臂主程序 (Arduino)
//
// 结构:
//   setup() —— 复位到安全初始角度、初始化手柄引脚与串口、挂载 4 个舵机
//   loop()  —— 每轮:
//     1. joystickLoop()   读手柄（MeArm 套件手柄：两根双轴摇杆，共占 A0~A3），
//                         按全局速度参数直接改变 4 个舵机的目标角度
//     2. writeServo()     把当前角度写入 4 个 Servo
//
// 【控制方式：直接控制关节角】
//   摇杆推动 = 对应舵机角度增大/减小，不再通过末端坐标反解。
//   被控量: Pos.ser.angle1..angle4（b / r / c / f 四个关节角）
//   派生量: Pos.rec.x/y/z —— 由正运动学 recFromServo() 实时算出，只用于显示，
//           不再是"用户设定的目标"（原坐标控制方式的代码已备份到工作区外）。
//
// 手柄操作（详见 joystick_control.h）:
//   左手柄 左右推 (A0) -> b = angle1 基座回转   角度增大 = 末端往 +y 侧转
//   左手柄 前后推 (A1) -> r = angle2 上臂俯仰   角度增大 = 上臂继续抬高
//   右手柄 前后推 (A3) -> c = angle3 下臂俯仰   角度增大 = 下臂往前收
//   右手柄 左右推 (A2) -> f = angle4 末端夹具   角度增大 = 张开
//   串口发 '1'/'2'/'3'    切速度档：慢速 / 中速 / 快速（波特率 115200）
//   串口发 'k'/'K'        末端张开 / 收回
//   D13 指示灯            任一关节在动 -> 快闪；全部静止 -> 灭
//
//   4 个关节各占一路摇杆轴，可以同时操作，因此不需要切模式。
//   ★ 若发现左右手反了，把 joystick_control.cpp 里 JOY_L/JOY_R 两组引脚对调即可。
//
// 安全边界:
//   1. servoLimit 关节硬限位 —— b[0,180] r[0,180] c[0,180] f[60,150]，
//                              角度模式下的唯一软件边界；转到行程尽头就停住，
//                              不会顶死舵机（详见 moveJointStep）。
//   2. rangeLimit 位置边界   —— 由新角度正解出的末端坐标若跑出 limit 区间，
//                              这一步整步回退，避免臂跑到没标定的区域。
//   3. 正运动学自洽          —— 每步都用 recFromServo() 重算坐标，
//                              Pos.ser 与 Pos.rec 永远严格对应。
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
#include "Servo.h"
#include "move.h"
#include "joystick_control.h"

/* 4 个舵机: index 1~4 对应各关节。
 * 字母记号（与 constant_and_positions.h 的 servoLimit 一致）:
 *   index1 = b 水平回转   index2 = r 上臂俯仰
 *   index3 = c 下臂俯仰   index4 = f 末端 */
Servo servos[5];

/* 把当前 4 个关节角写入物理舵机 (index 1..4) */
void writeServo(void){
  servos[1].write((int) Pos.ser.angle1);
  servos[2].write((int) Pos.ser.angle2);
  servos[3].write((int) Pos.ser.angle3);
  servos[4].write((int) Pos.ser.angle4);
}

void setup() {
  /* 1. 把 Pos 复位到安全的初始角度（含 angle4 取行程中位）。
   *    必须在 writeServo() 之前，否则开机瞬间舵机会被拉到角度 0。 */
  posInit();

  /* 2. 初始化手柄引脚、指示灯与串口调速命令 */
  joystickSetup();

  /* 3. 将 4 个舵机 attach 到对应引脚。
   *    引脚号请按实际硬件修改 (示例: 9, 7, 8, 6) */
  servos[1].attach(9);   /* 水平回转 */
  servos[2].attach(7);   /* 上臂俯仰 */
  servos[3].attach(8);   /* 下臂俯仰 */
  servos[4].attach(6);   /* 末端 */

  /* 4. 上电后先让机械臂到位，再开始接收手柄输入 */
  writeServo();

  /* 可选: 设置初始速度档位 (默认中速) */
  // adjustSpeed(SPEED_NORMAL);
}

void loop() {
  /* 读手柄，按全局调速参数直接改变 4 个关节角度 */
  joystickLoop();

  /* 把当前关节角写入物理舵机 */
  writeServo();
}
