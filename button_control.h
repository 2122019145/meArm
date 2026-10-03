/*
 * button_control.h -- 四个按键的对外接口（按键1 循环执行 / 按键2 录制 / 按键3 播放 / 按键4 回中）
 *
 * 【这一版做了什么】
 *   把"四个按键"的全部调用接口集中在这一个模块里（实现只有 button_control.cpp 一个
 *   文件），主程序只要在 setup() 里调 buttonSetup()、在 loop() 里调 buttonLoop()，
 *   别的什么都不用管：
 *
 *     按键1（D2）循环执行：每按一次触发一次取放，第 1 次夹 A、第 2 次夹 B、
 *                          第 3 次夹 C，之后回到 A，如此循环。
 *     按键2（D3）录制    ：第 1 次按下开始录制，此时用摇杆手动操控机械臂；
 *                          第 2 次按下结束录制并保存。
 *                          保存条件：只管"动没动"—— 期间末端在 x/y/z 至少有一个
 *                          方向走出了 BTN_REC_MIN_TRAVEL（10.0，明显位移）。
 *                          **没有最小时长限制**（原来要求"大于 10 秒"已删除，
 *                          1 秒的动作只要走够了位移就能保存）；最长受 192 条
 *                          = 38.4 秒的缓冲上限约束，写满判废。
 *                          （采样周期与条数是 weArm_config.h 第 5 节的开关：
 *                          WEARM_REC_TICK_MS 默认 200 ms、WEARM_REC_ENTRIES
 *                          默认 192 —— 768 B SRAM。回放会把每条记录再拆成
 *                          4 个 50 ms 子步插值展开，不会一顿一顿。）
 *                          不满足就丢弃并串口报明原因（不会留下一段假数据）。
 *     按键3（D4）播放    ：把上一次录制的动作完整复现一遍（先平滑摆到录制起点，
 *                          再按录制时的真实时间轴复现，200 ms 一条记录在回放时
 *                          按 4 个子步插值展开）。
 *     按键4（D5）回中    ：平滑回到开机初始位姿（POS_HOME 对应的关节角）。
 *
 * 【串口等价命令】（上位机发单个字母，效果与按下物理按键完全一样）
 *     N -> 按键1      R -> 按键2      P -> 按键3      M 或 0 -> 按键4
 *   之所以用字母而不是 1/2/3/4：'1'/'2'/'3' 从 v0.2.0 起就是"慢/中/快"调速命令，
 *   不能抢过来，否则老的上位机脚本会突然开始动机械臂。
 *
 * 【按键电气约定】
 *   四个按键脚统一 pinMode(INPUT_PULLUP)，按下 = 拉到 GND = 读到 LOW。
 *   按键脚默认是 D2/D3/D4/D5（本工程舵机占了 D6~D9、指示灯占 D13、串口占 D0/D1、
 *   摇杆占 A0~A3，所以空闲且好接的就是 D2~D5）。接别的脚只改 BTN_PIN_* 那几个宏。
 *   软件消抖 25ms 并只认"按下沿"，一次物理按下只会触发一次动作。
 *
 *   ★ 本机默认 WEARM_BUTTON_PINS = 0（weArm_config.h 第 4 节）：**完全不读
 *     D2~D5**，四个功能一律走上面的串口命令。原因是本机没有独立按键，只有摇杆
 *     自带的 SW 脚；右摇杆推到最前时机械上会压合那颗轻触开关，接在 D2 就会假触发
 *     按键1（循环取放），把手动推杆的操作抢走。改回 1 即恢复物理按键接线。
 *
 * 【谁让位给谁】
 *   录制期间：摇杆照常可用（录制就是要录你推摇杆的动作），但串口的
 *             O/S/k/K 与 x,y,z 角度指令会被拒绝，取放序列也起不来 ——
 *             录制是"摇杆独占"，避免录进来一半是别人动的。
 *   播放/回中期间：摇杆被让位（buttonControlLocked() 为真，joystick_control.cpp
 *             的步进循环会跳过），串口动作指令同样被拒绝，只有 H/L/1/2/3 调速有效。
 *   取放序列执行期间：四个按键的动作全部拒绝（返回 PROTO_RES_BUSY）。
 */
#ifndef BUTTON_CONTROL_H
#define BUTTON_CONTROL_H

#include "Arduino.h"
#include "constant_and_positions.h"
#include "serial_protocol.h"
#include "weArm_config.h"

/* 按键编号（顺序与物理按键 1~4 一致，也就是 buttonName() 的下标） */
#define BTN_COUNT      4
#define BTN_KEY_CYCLE  0   /* 按键1：循环执行（A -> B -> C -> A ...） */
#define BTN_KEY_RECORD 1   /* 按键2：录制 开/关 */
#define BTN_KEY_PLAY   2   /* 按键3：播放 */
#define BTN_KEY_HOME   3   /* 按键4：回中 */

#if WEARM_ENABLE_BUTTONS
/* 初始化四个按键脚（在 setup() 里调用一次，放在 joystickSetup() 之后）。 */
void buttonSetup(void);

/* 每轮 loop() 调一次，非阻塞：
 *   扫描按键沿 -> 执行对应动作 -> 推进"录制采样 / 回中与预摆的插值 / 播放时间轴"。 */
void buttonLoop(void);

/* 串口等价命令入口（单个字符）。
 * 返回 PROTO_RES_* （见 serial_protocol.h）：
 *   N -> PROTO_RES_PICK_STARTED / PROTO_RES_BUSY
 *   R -> PROTO_RES_REC_STARTED / PROTO_RES_REC_SAVED / PROTO_RES_REC_REJECTED / PROTO_RES_BUSY
 *   P -> PROTO_RES_PLAY_STARTED / PROTO_RES_PLAY_NO_RECORD / PROTO_RES_BUSY
 *   M、0 -> PROTO_RES_HOME_STARTED / PROTO_RES_BUSY
 * 不是这四个字符时返回 PROTO_RES_UNKNOWN（调用方据此继续往下解析）。 */
int buttonHandleCommand(char c);

/* 该字符是不是本模块负责的按键命令（serial_protocol.cpp 用它决定要不要让本模块处理）。 */
bool buttonIsCommandChar(char c);

/* ---------- 状态查询（探针与调试用） ---------- */

/* 下一次按按键1 会夹哪个物体（PICK_OBJECT_A/B/C）。 */
int buttonPickNext(void);

/* 本模块正在做的事：录制 / 播放 / 回中 / 空闲。 */
bool buttonIsRecording(void);
bool buttonPlaybackActive(void);

/* 录制缓冲里当前的条目数（录制中 = 已写条数，空闲 = 上一次保存的条数）。 */
int buttonRecordingEntries(void);

/* 上一次录制（或正在进行的录制）的时长 ms，以及末端在 x/y/z 上的包围盒跨度
 * （取三轴跨度最大值 = 保存门槛用的"位移"，来回走会累加，不是净位移）。 */
unsigned long buttonRecordingMs(void);
double buttonRecordingTravel(void);

/* 是否有一份"已经保存好、可以播放"的录制。 */
bool buttonHasRecording(void);

/* 本模块是否正忙（录制 / 播放 / 回中任一）。串口动作指令的忙判定用这个：
 * 录制期间虽然摇杆还能用，但取放序列、开合爪、角度指令都不该插进来。 */
bool buttonControlBusy(void);

/* 本模块是否独占机械臂（播放 / 回中，不含录制）。
 * 摇杆步进必须让位的是这个 —— 录制期间摇杆当然不能停。 */
bool buttonControlLocked(void);

/* 按键名（"按键1 循环执行" 等）与当前状态名（"空闲"/"录制中"/"播放中"/"回中中"）。 */
const char *buttonName(int key);
const char *buttonStateName(void);
#else
/* 按键模块已禁用，提供空桩函数接口 */
static inline void buttonSetup(void)                 {}
static inline void buttonLoop(void)                  {}
static inline int  buttonHandleCommand(char c)       { (void)c; return -1; }
static inline bool buttonIsCommandChar(char c)       { (void)c; return false; }
static inline int  buttonPickNext(void)              { return -1; }
static inline bool buttonIsRecording(void)           { return false; }
static inline bool buttonPlaybackActive(void)        { return false; }
static inline int  buttonRecordingEntries(void)      { return 0; }
static inline unsigned long buttonRecordingMs(void)  { return 0UL; }
static inline double buttonRecordingTravel(void)     { return 0.0; }
static inline bool buttonHasRecording(void)          { return false; }
static inline bool buttonControlBusy(void)           { return false; }
static inline bool buttonControlLocked(void)         { return false; }
static inline const char *buttonName(int key)       { (void)key; return ""; }
static inline const char *buttonStateName(void)     { return ""; }
#endif

#endif /* BUTTON_CONTROL_H */
