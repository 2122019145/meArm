//
// wearm.ino
// 机械臂主程序 (Arduino)
//
// 结构:
//   setup() —— 复位到安全初始角度、初始化串口协议与手柄引脚、挂载 4 个舵机
//   loop()  —— 每轮:
//     1. serialProtocolLoop() 处理串口指令（唯一读者，放在最前面保证及时响应）
//     2. pickPlaceLoop()   推进取放序列（非阻塞状态机，独占 b/r/c 三个关节角）
//     3. buttonLoop()      扫描四个按键 + 推进录制/播放/回中的插值（非阻塞）
//     4. drawLoop()        推进绘图任务（回待机/走位/落笔/绘制/抬笔；示教时读摇杆点动）
//     5. joystickLoop()    读手柄（MeArm 套件手柄：两根双轴摇杆，共占 A0~A3），
//                          按全局速度参数直接改变 4 个舵机的目标角度
//     6. writeServo()      把当前角度写入 4 个 Servo
//   （取放/按键/绘图三者互斥：各自的 IsBusy()/Locked() 会让另外两个与摇杆整段让位）
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
//   串口命令（波特率 115200，由 serial_protocol 模块解析，详见 serial_protocol.h）：
//     发 O             爪子张开（angle4 走到 f 行程上限）
//     发 S             爪子关闭（angle4 走到 f 行程下限）
//     发 H             整体运行速度提升一档
//     发 L             整体运行速度降低一档
//     发 x角度,y角度,z角度   同步设置三个舵机（例: x10,y30,z20）
//                      x -> angle1 基座、y -> angle2 上臂、z -> angle3 下臂，
//                      可只写其中一部分（如 y45），未出现的轴不动
//     发 A / B / C     自动取放：夹起物体 A/B/C 放到各自的放置点（详见 pick_place.h）
//                      序列执行期间摇杆与其它动作指令让位，只有 H/L/1/2/3 调速仍生效
//                      （旧的 '1'/'2'/'3' 调速与 k/K 末端开合仍兼容）
//     发 N / R / P / M 四个物理按键的等价命令（详见 button_control.h）：
//                      N = 按键1 循环执行，R = 按键2 录制开/关，
//                      P = 按键3 播放，M（或 '0'）= 按键4 回中
//
//   四个物理按键（接 D2~D5，INPUT_PULLUP，按下 = 接地；实现只有 button_control.cpp 一个文件）:
//     按键1 (D2) 循环执行  每按一次触发一次取放，依次夹 A -> B -> C -> 回到 A
//     按键2 (D3) 录制      第一次按下开始录制（此时用摇杆操控），第二次按下结束并保存；
//                          要求这段动作"时长大于 10 秒"且末端有明显位移，否则丢弃重录
//     按键3 (D4) 播放      复现上一次按键2 录下来的动作
//     按键4 (D5) 回中      平滑回到开机初始位姿（POS_HOME 对应的关节角）
//     录制期间摇杆照常可用；播放/回中期间摇杆让位。这些状态下的串口忙让位详见 button_control.h
//
//   绘图（铅笔固定在末端夹具上，详见 draw_control.h；实现只有 draw_control.cpp 一个文件）:
//     发 F             切换绘制任务：直线 -> 字母N -> 三角形 -> 字母Z -> 字母V -> 五点折线 -> 五点曲线
//     发 D             开始绘制（内置图形直接画；五点折线/五点曲线先进入五点示教）
//     发 G / E         示教中记录 / 撤销一个示教点（等价于示教时按按键1 / 按键2）
//     发 Q / U / W     暂停 / 继续 / 取消（等价于绘制时按按键1 / 按键2 / 按键3）
//     发 p12.5         标定纸面高度（铅笔尖刚好落在纸上时的 z）
//     发 n6            标定内置图形的半宽（顶点落在 ±半宽 的正方形里）
//     发 o20,0         标定内置图形的中心 (x,y)
//   绘图时的四个物理按键（空闲时仍是上面的循环取放/录制/播放/回中）:
//     示教中  按键1 记录示教点（记满 5 个自动开始）  按键2 撤销  按键3 取消  按键4 手动开始
//     绘制中  按键1 暂停（停在原地、任务状态保留）  按键2 继续（从暂停处接续）  按键3 取消（抬笔回待机）
//   绘图期间的排他: 取放序列与按键录制/播放/回中期间不允许启动绘图，反之亦然；
//                   绘图进行中摇杆整段让位（示教时由 draw_control.cpp 自己读摇杆做笛卡尔点动）。
//   D13 指示灯            任一关节在动 -> 快闪；全部静止 -> 灭
//                         （取放序列与录制播放期间也算在动，灯会一直闪）
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
#include "servo_drive.h"
#include "move.h"
#include "joystick_control.h"
#include "serial_protocol.h"
#include "pick_place.h"
#include "button_control.h"
#include "draw_control.h"

/* 4 个舵机: 通道 0~3 对应各关节（见 servo_drive.h，替代 Arduino Servo 库）。
 * 字母记号（与 constant_and_positions.h 的 servoLimit 一致）:
 *   通道0 = b 水平回转   通道1 = r 上臂俯仰
 *   通道2 = c 下臂俯仰   通道3 = f 末端 */

/* 把当前 4 个关节角写入物理舵机（通道 0..3） */
void writeServo(void){
  servoDriveWrite(0, Pos.ser.angle1);
  servoDriveWrite(1, Pos.ser.angle2);
  servoDriveWrite(2, Pos.ser.angle3);
  servoDriveWrite(3, Pos.ser.angle4);
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

  /* 4. 初始化四个按键（D2~D5，INPUT_PULLUP；按键功能全部封装在 button_control.cpp） */
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
