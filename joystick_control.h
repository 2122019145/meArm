//
// joystick_control.h
// 手柄控制模块对外接口（MeArm 套件自带手柄：两根双轴摇杆，共占 A0~A3）
//
// 【本工程的控制方式：直接控制关节角】
//   摇杆推动直接改变某个舵机（关节）的角度，不再控制末端坐标。
//   被控量: Pos.ser.angle1..angle4（b / r / c / f 四个关节角）
//   派生量: Pos.rec.x/y/z —— 由正运动学 recFromServo() 实时算出，只用于显示，
//           不再是"用户设定的目标值"。
//
// 【硬件】
//   MeArm Arduino 套件附带的操纵手柄板上有两根自复位双轴摇杆，
//   每根摇杆出 2 路模拟电压（X / Y），共 4 路，刚好对应 4 个关节。
//   板上的摇杆按键本工程暂不使用（见文件末尾"可选扩展"）。
//
// 【接线（Arduino Uno）】
//   左手柄:  X -> A0      Y -> A1      右手柄:  X -> A2      Y -> A3
//   手柄模块 VCC -> 5V，GND -> GND
//   指示灯 D13（板载 LED，可改 PIN_LED_MODE）
//
//   ★ 物理"左/右"与 A0~A3 的对应关系由实际接线决定：
//     若发现左右手反了，只要把 JOY_LX_PIN/JOY_LY_PIN 与
//     JOY_RX_PIN/JOY_RY_PIN 两组引脚对调即可，逻辑不用改。
//
// 【操作表 —— 4 个关节角各占一路轴，同时可用，不需要切模式】
//     操作                    关节                     推杆方向与角度变化
//     ----------------------  -----------------------  ----------------------------
//     左手柄 左右推 (A0)      b = angle1 基座回转       角度增大（末端往 +y 侧转）
//     左手柄 前后推 (A1)      r = angle2 上臂俯仰       前推 = 角度减小 / 后拉 = 角度增大
//     右手柄 前后推 (A3)      c = angle3 下臂俯仰       前推 = 角度减小 / 后拉 = 角度增大
//     右手柄 左右推 (A2)      f = angle4 末端夹具       角度增大（张开）
//     上臂与下臂这两路的方向是上机实测后调转过的，基座与末端未改。
//     串口发送 '1'/'2'/'3'     切速度档: 慢速 / 中速 / 快速  (波特率 115200)
//     串口发送 'k'/'K'          末端张开 / 收回 (与右手柄左右推等效)
//
//   每个关节的行程由全局 servoLimit 限制（b 0~180 / r 45~105 / c 0~180 / f 60~150），
//   推到行程尽头就停住（moveJointStep 返回 MOVE_AT_LIMIT），不会顶死舵机。
//
// 【为什么改成"控角度"而不是"控末端坐标"】
//   原方案是摇杆改末端 x/y/z，再反解舵机角度。缺点是:
//     1) 末端坐标可解的角度组合常常超出舵机行程，摇杆推到头会突然停住；
//     2) 姿态随坐标反解变化，操作者很难预判舵机会往哪转；
//     3) 4 个自由度里 angle4 本来就不参与反解，混在一起语义混乱。
//   直接控角度后每一路轴 ↔ 一个舵机，动作可预期、行程边界清晰，
//   末端坐标只作为显示量算出来。
//
// 【坐标模型（用于显示与工作空间校验）】
//   肩关节在坐标原点，上臂 L1 = 下臂 L2 = 20，armheight = 0。
//   舵机中立位 90° 的含义:
//     b (angle1) 90° -> 基座朝 +x 方向；b 行程 0~180° 使可达空间只占 x >= 0 一侧
//     r (angle2) 90° -> 上臂竖直向上
//     c (angle3) 90° -> 下臂水平朝前（与上臂成 90°）
//   初始位姿 (b,r,c,f) = (90,90,90,0) == 末端 (20, 0, 20)。
//   正运动学: alpha = r, beta = r - c,
//             x_planar = L1 cos alpha + L2 cos beta,
//             z = L1 sin alpha + L2 sin beta + armheight,
//             theta = 90 - b  ->  x = x_planar cos theta, y = x_planar sin theta。
//
// 【调速参数】存在全局 speedCfg (speed) 中，所有移动源共用：
//     speed.stepSize    每个控制周期转过的角度（度）
//     speed.minDelayMs  输入较弱时的最小步间间隔（最高速度）
//     speed.fullDelayMs 满偏时的步间间隔（最低速度，最安全）
//
// 【可选扩展】板载摇杆按键（把按键脚接到空闲数字口即可）：
//   可在 joystickSetup() 里 pinMode(pin, INPUT_PULLUP)，
//   然后按键控末端张开/收回或切档，参考 buttonDown() 的时间消抖实现。
//
#ifndef WEARM_JOYSTICK_CONTROL_H
#define WEARM_JOYSTICK_CONTROL_H

#include "Arduino.h"
#include "constant_and_positions.h"
#include "move.h"

/* 保留的兼容值：本硬件用不到模式切换，恒为 0 */
#define JOY_MODE_PLANE  0
#define JOY_MODE_VERT   1
#define JOY_MODE_COUNT  2

/* 摇杆一次采样的完整结果（按"被控关节"命名，便于直接使用） */
struct joyState {
  int base;     /* A0 轴：基座回转 b 的角度增量方向与幅度（正 = 角度增大） */
  int shoulder; /* A1 轴：上臂俯仰 r（正 = 角度增大） */
  int elbow;    /* A3 轴：下臂俯仰 c（正 = 角度增大） */
  int tool;     /* A2 轴：末端夹具 f（正 = 角度增大 / 张开） */

  int sx;       /* A0 原始 ADC 值（调试用） */
  int sy;       /* A1 原始 ADC 值 */
  int tx;       /* A2 原始 ADC 值 */
  int ty;       /* A3 原始 ADC 值 */

  int dir;      /* 本轮选中的方向编码（幅度最大的那一路，见 JOY_DIR_*） */
  int mag;      /* 选中方向对应的偏转幅度（ADC 单位，已扣死区） */
  int mask;     /* 本轮有动作的关节位图（JOY_ACT_*） */
};

/* 方向编码（joystickRead() 的返回值，沿用原约定） */
#define JOY_DIR_NONE   0   /* 无动作 */
#define JOY_DIR_RIGHT  2   /* 基座角度增大 */
#define JOY_DIR_BWD    3   /* 上臂角度减小 */
#define JOY_DIR_LEFT   4   /* 基座角度减小 */
#define JOY_DIR_FWD    5   /* 上臂角度增大 */
#define JOY_DIR_UP     6   /* 下臂角度增大 */
#define JOY_DIR_DOWN   7   /* 下臂角度减小 */

/* 活跃关节位图（joyState.mask） */
#define JOY_ACT_BASE     0x01   /* 基座 b */
#define JOY_ACT_SHOULDER 0x02   /* 上臂 r */
#define JOY_ACT_ELBOW    0x04   /* 下臂 c */
#define JOY_ACT_TOOL     0x08   /* 末端 f */

/* 初始化摇杆引脚、LED 与串口（在 setup() 中调用）。
 * 注意：内部会 Serial.begin(115200)，不要重复初始化。 */
void joystickSetup(void);

/* 读取当前手柄状态（4 个自由度的偏转量 + 原始值 + 选中的移动方向） */
void joystickReadState(struct joyState *st);

/* 读取本轮选中的方向编码（JOY_DIR_*；无动作返回 JOY_DIR_NONE） */
int joystickRead(void);

/* 兼容接口：本硬件不做模式切换，恒返回 JOY_MODE_PLANE */
int joystickGetMode(void);

/* 每轮 loop 调一次，非阻塞（内部不使用 delay）。
 * 内部依次处理：串口命令 -> 摇杆采样 -> 按全局调速参数步进 -> LED 指示。 */
void joystickLoop(void);

#endif /* WEARM_JOYSTICK_CONTROL_H */
