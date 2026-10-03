/*
 * draw_control.h -- 纸面轨迹绘制（铅笔固定在末端夹具上）
 *
 * 【这一版做了什么】
 *   把"机械臂在纸面上画图"的全部逻辑封在一个模块里（实现只有 draw_control.cpp
 *   一个文件）。主程序只要在 setup() 里调 drawSetup()、在 loop() 里调 drawLoop()，
 *   别的什么都不用管：
 *
 *     1. 直线绘制      任务 DRAW_TASK_LINE：在纸面上画一条连续直线。
 *     2. 基础图形      任务 DRAW_TASK_N / TRIANGLE / Z / V：任选一种字母或图形。
 *     3. 绘图过程控制  暂停 / 继续 / 取消，三个物理按键分别触发（见下表）。
 *     4. 五点示教折线  任务 DRAW_TASK_POLYLINE：摇杆把笔尖依次移到 5 个目标点，
 *                      按键记录；记录完成后先回待机位、再走到第 1 点，然后依次
 *                      连线到第 5 点，画完 4 段连续折线。
 *     5. 五点示教曲线  任务 DRAW_TASK_CURVE：同样的示教方式，画一条依次经过
 *                      5 个目标点的连续平滑曲线（向心 Catmull-Rom 样条）。
 *
 * 【为什么是"先反解、再插值"】
 *   绘制轨迹在笛卡尔空间里定义（直线要真的直、曲线要真的经过示教点），
 *   每个采样点交给 move.h 的 moveToPoint()（"移动到指定 x,y,z 坐标"的公共核心，
 *   串口 x/y/z 指令用的是同一个函数；它内部用 getAngleEx() 反解成 b/r/c 并写回 Pos）。
 *   ★ 启动前会把整条轨迹按 DRAW_PATH_SAMPLE_STEP 采样校验一遍
 *     （在 limit 内 + isReachable() + 反解没被吸附 + 相邻点反解分支不跳变），
 *     任何一条不满足就拒绝启动、一个字节都不改（与 pick_place 同一套做法）。
 *
 * 【按键分工】（绘图任务进行中，四个物理按键被本模块接管；空闲时按键还是原来的
 *   循环取放 / 录制 / 播放 / 回中，不受影响）
 *     示教中   按键1 = 记录当前点为下一个示教点
 *              按键2 = 撤销最后一个示教点
 *              按键3 = 取消示教（回待机）
 *              按键4 = 结束示教并开始绘制（已记录 >= 2 点）
 *              第 5 个点记录完会自动开始（停顿 DRAW_TEACH_START_DELAY_MS 毫秒）
 *     绘制中   按键1 = 暂停（保持当前任务状态，笔尖停在原地）
 *              按键2 = 继续（从暂停位置接着画）
 *              按键3 = 取消（抬笔 -> 回待机位 -> 空闲）
 *              按键4 = 无动作（打印提示）
 *     暂停中   按键1 = 仍然暂停（无动作），按键2 = 继续，按键3 = 取消
 *
 * 【串口等价命令】（单字符，与上面一一对应；空闲时也能用，用来选任务/启动）
 *     F  循环选择绘图任务（直线 -> N -> 三角形 -> Z -> V -> 五点折线 -> 五点曲线）
 *     D  开始当前任务：内置图形直接开始绘制；两个示教任务进入示教模式（先回中）
 *     G  记录当前示教点（= 按键1）
 *     E  撤销最后一个示教点（= 按键2）
 *     Q  暂停（= 按键1）
 *     U  继续（= 按键2）
 *     W  取消（= 按键3）
 *   标定命令（都是在"上机对纸面"时用的，可随时发）：
 *     p<数字>        纸面高度（铅笔尖刚好落在纸上时的 Pos.rec.z），例 p12.5
 *     n<数字>        内置图形的半宽（工作区单位，默认 6.0），例 n7
 *     o<x>,<y>       内置图形的中心（工作区单位，默认 20,0），例 o22,-2
 *
 * 【谁让位给谁】
 *   本模块忙（示教/绘制/暂停/回待机）时：
 *     - 摇杆让位（joystick_control.cpp 用 drawControlLocked() 判断）；
 *       但示教模式下摇杆由本模块自己读，用来点动笔尖 —— 见"示教点动"一节。
 *     - 串口的动作指令（O/S/k/K、x,y,z 角度、A/B/C 取放）被拒绝，
 *       只有调速 H/L/1/2/3 与绘图/按键命令本身有效。
 *   反过来，取放序列执行中或按键模块录制/播放/回中时，本模块拒绝启动（回 BUSY）。
 *
 * 【示教点动：笛卡尔点动，不是关节角点动】
 *   示教时推动摇杆，改变的是"笔尖的 x/y/z"，不是某个舵机角 —— 因为考试要求
 *   把笔尖移到纸面上的目标点，按关节角推根本对不准。约定：
 *     左手柄 左右 (A0) -> 笔尖 x   增大 / 减小
 *     左手柄 前后 (A1) -> 笔尖 y   前推 = y 增大
 *     右手柄 前后 (A3) -> 笔尖 z   前推 = z 减小（往下压）
 *     右手柄 左右 (A2) -> 末端 f   保持原来的开合语义
 *   每步的位移量取全局调速参数（speed.stepSize × DRAW_JOG_SCALE），
 *   步进间隔也按偏转量在 speed.minDelayMs~fullDelayMs 之间插值 ——
 *   也就是示教点动同样吃 调速 档位，慢速档可以一点点挪。
 *   哪一路方向觉得反了，把 DRAW_JOG_INVERT_X/Y/Z 改成 1 即可（不用改逻辑）。
 *
 * 【纸面坐标与几何】
 *   纸面是水平面 z = 纸面高度（默认 DRAW_PEN_Z）。内置图形画在
 *   以 (DRAW_CENTER_X, DRAW_CENTER_Y) 为中心、半宽 DRAW_HALF 的正方形里，
 *   顶点用"归一化坐标 u,v ∈ [-1,1]"定义，这样改大小只动一个数。
 *   默认区域 x[14,26] y[-6,6]（中心 20,0，半宽 6），整块都在可达空间内部：
 *   该区域 |rho| 最大 26.7，R = sqrt(rho^2+z^2) 最大约 29.6 << L1+L2 = 40。
 */
#ifndef DRAW_CONTROL_H
#define DRAW_CONTROL_H

#include "Arduino.h"
#include "constant_and_positions.h"
#include "serial_protocol.h"
#include "weArm_config.h"

/* ---------- 绘图任务编号 ---------- */
#define DRAW_TASK_LINE      0   /* 直线（内置） */
#define DRAW_TASK_N         1   /* 字母 N（内置） */
#define DRAW_TASK_TRIANGLE  2   /* 三角形（内置） */
#define DRAW_TASK_Z         3   /* 字母 Z（内置） */
#define DRAW_TASK_V         4   /* 字母 V（内置） */
#define DRAW_TASK_POLYLINE  5   /* 五点示教折线 */
#define DRAW_TASK_CURVE     6   /* 五点示教平滑曲线 */
#define DRAW_TASK_COUNT     7

/* 示教点个数（任务 4/5 固定 5 个目标点） */
#define DRAW_TEACH_MAX_POINTS 5

/* ---------- 按键角色（与 button_control.h 的 BTN_KEY_* 下标一致） ----------
 * 绘图任务进行中 button_control 会把四个按键转交到这里，本模块按当前状态
 * 决定每个按键干什么（见文件头"按键分工"）。 */
#define DRAW_KEY_1 0
#define DRAW_KEY_2 1
#define DRAW_KEY_3 2
#define DRAW_KEY_4 3

/* ---------- 绘图阶段（drawGetPhase() 的返回值） ---------- */
#define DRAW_PHASE_IDLE      0  /* 空闲 */
#define DRAW_PHASE_HOME      1  /* 关闭/开局：关节空间插值回待机位 */
#define DRAW_PHASE_TRAVEL    2  /* 直线移动到轨迹起点正上方（抬笔高度） */
#define DRAW_PHASE_PLUNGE    3  /* 垂直下降到轨迹起点（落笔） */
#define DRAW_PHASE_PATH      4  /* 沿轨迹绘制（内置图形或示教轨迹） */
#define DRAW_PHASE_LIFT      5  /* 垂直抬起（抬笔） */
#define DRAW_PHASE_TEACH     6  /* 示教：摇杆点动 + 按键记录 */
#define DRAW_PHASE_TEACH_WAIT 7 /* 5 点记录完的小停顿，之后自动开始绘制 */
#define DRAW_PHASE_RETURN    8  /* 绘制结束后回待机位 */

#if WEARM_ENABLE_DRAW
/* ---------- 生命周期 ---------- */
/* 初始化（在 setup() 里调用一次）。不占用任何引脚：按键与摇杆都由各自模块读。 */
void drawSetup(void);

/* 每轮 loop() 调一次，非阻塞：推进状态机 / 示教点动 / 读取按键（由 button_control 转交）。 */
void drawLoop(void);

/* ---------- 任务选择与启动 ---------- */
/* 选择绘图任务，返回生效的任务编号；task 非法时返回 -1 且不改动。 */
int drawSelectTask(int task);
/* 切到下一个任务（直线 -> N -> 三角形 -> Z -> V -> 折线 -> 曲线 -> 直线），返回新编号。 */
int drawTaskCycle(void);
/* 当前选中的任务编号。 */
int drawGetTask(void);
/* 任务名（"直线"/"字母N"/"三角形"/"字母Z"/"字母V"/"五点折线"/"五点曲线"）。 */
const char *drawTaskName(int task);

/* 开始当前选中的任务：
 *   内置图形（0~4）：直接进入 回待机 -> 抬笔移动到起点 -> 落笔 -> 绘制 -> 抬笔 -> 回待机。
 *   示教任务（5~6）：进入示教模式（先回待机位），之后用摇杆点动 + 按键记录。
 * 返回 PROTO_RES_DRAW_STARTED；状态不对或轨迹校验失败时返回
 * PROTO_RES_BUSY / PROTO_RES_DRAW_REJECTED（原因见串口日志与 drawLastResult()）。 */
int drawStartTask(void);

/* 串口单字符入口（F/D/G/E/Q/U/W）。不是本模块的字符时返回 PROTO_RES_UNKNOWN。 */
int drawHandleCommand(char c);
/* 该字符是不是本模块负责的绘图命令（serial_protocol.cpp 用它决定要不要让本模块处理）。 */
bool drawIsCommandChar(char c);

/* 标定：纸面高度 / 内置图形半宽 / 内置图形中心（工作区单位）。
 * 数值非法（非有限、超出合理范围）时返回 false 且不改动。 */
bool drawSetPaperZ(double z);
double drawGetPaperZ(void);
bool drawSetHalfSize(double half);
double drawGetHalfSize(void);
bool drawSetCenter(double x, double y);
double drawGetCenterX(void);
double drawGetCenterY(void);

/* ---------- 示教 ---------- */
/* 记录当前笔尖位置为下一个示教点（= 按键1）。点数已满 5 时返回 PROTO_RES_BUSY。 */
int drawTeachRecord(void);
/* 撤销最后一个示教点（= 按键2）。没有点可撤时返回 PROTO_RES_DRAW_REJECTED。 */
int drawTeachUndo(void);
/* 结束示教并开始绘制（= 按键4，需要 >= 2 个点）。 */
int drawTeachFinish(void);
/* 当前已记录的示教点数。 */
int drawTeachCount(void);

/* ---------- 绘图过程控制 ---------- */
/* 暂停（= 按键1）。不在绘制/示教流程中时返回 PROTO_RES_BUSY。 */
int drawPause(void);
/* 继续（= 按键2）。不在暂停状态时返回 PROTO_RES_BUSY。 */
int drawResume(void);
/* 取消（= 按键3）：抬笔 -> 回待机位 -> 空闲。空闲时返回 PROTO_RES_BUSY。 */
int drawCancel(void);

/* 按键入口：button_control.cpp 在 btnAction() 里把按键转交过来。
 * 返回 PROTO_RES_*（与串口命令同一套回话）。 */
int drawHandleButton(int key);
/* 四个按键现在归本模块管吗（= 本模块不空闲）。 */
bool drawAcceptButton(int key);

/* ---------- 状态查询（探针与调试用） ---------- */
/* 本模块是否正忙（示教 / 绘制 / 暂停 / 回待机 任一）。 */
bool drawControlBusy(void);
/* 是否独占机械臂、摇杆必须让位（当前与 drawControlBusy() 同义，分开写是为了
 * 以后若要支持"绘制中允许摇杆微调"时不至于改调用方）。 */
bool drawControlLocked(void);
/* 当前阶段（DRAW_PHASE_*）。 */
int drawGetPhase(void);
/* 阶段名（"空闲"/"回待机"/"抬笔移动"/"落笔"/"绘制中"/"抬笔"/"示教中"/"准备绘制"/"结束回待机"）。 */
const char *drawPhaseName(void);
/* 状态名（比阶段名更贴近操作者："空闲"/"示教中(3/5)"/"绘制中"/"已暂停"/"取消中"...）。 */
const char *drawStateName(void);
/* 是否处于暂停。 */
bool drawIsPaused(void);

/* 上一次绘制的结果：用时 ms、过点个数 / 总点数（内置图形时总点数为 0）。 */
unsigned long drawLastRunMs(void);
int drawLastPointsHit(void);
int drawLastPointsTotal(void);
/* 最近一次操作的返回码（PROTO_RES_*），方便串口/探针回看。 */
int drawLastResult(void);
#else
/* 生命周期 */
/* 绘图功能已关闭，这里是空实现 */ static inline void drawSetup(void) { }
/* 绘图功能已关闭，这里是空实现 */ static inline void drawLoop(void) { }

/* ---------- 任务选择与启动 ---------- */
/* 绘图功能已关闭，这里是空实现 */ static inline int drawSelectTask(int task) { (void)task; return -1; }
/* 绘图功能已关闭，这里是空实现 */ static inline int drawTaskCycle(void) { return -1; }
/* 绘图功能已关闭，这里是空实现 */ static inline int drawGetTask(void) { return -1; }
/* 绘图功能已关闭，这里是空实现 */ static inline const char *drawTaskName(int task) { (void)task; return ""; }
/* 绘图功能已关闭，这里是空实现 */ static inline int drawStartTask(void) { return -1; }

/* 串口命令 */
/* 绘图功能已关闭，这里是空实现 */ static inline int drawHandleCommand(char c) { (void)c; return -1; }
/* 绘图功能已关闭，这里是空实现 */ static inline bool drawIsCommandChar(char c) { (void)c; return false; }

/* 标定 */
/* 绘图功能已关闭，这里是空实现 */ static inline bool drawSetPaperZ(double z) { (void)z; return false; }
/* 绘图功能已关闭，这里是空实现 */ static inline double drawGetPaperZ(void) { return 0.0; }
/* 绘图功能已关闭，这里是空实现 */ static inline bool drawSetHalfSize(double half) { (void)half; return false; }
/* 绘图功能已关闭，这里是空实现 */ static inline double drawGetHalfSize(void) { return 0.0; }
/* 绘图功能已关闭，这里是空实现 */ static inline bool drawSetCenter(double x, double y) { (void)x; (void)y; return false; }
/* 绘图功能已关闭，这里是空实现 */ static inline double drawGetCenterX(void) { return 0.0; }
/* 绘图功能已关闭，这里是空实现 */ static inline double drawGetCenterY(void) { return 0.0; }

/* ---------- 示教 ---------- */
/* 绘图功能已关闭，这里是空实现 */ static inline int drawTeachRecord(void) { return -1; }
/* 绘图功能已关闭，这里是空实现 */ static inline int drawTeachUndo(void) { return -1; }
/* 绘图功能已关闭，这里是空实现 */ static inline int drawTeachFinish(void) { return -1; }
/* 绘图功能已关闭，这里是空实现 */ static inline int drawTeachCount(void) { return 0; }

/* ---------- 绘图过程控制 ---------- */
/* 绘图功能已关闭，这里是空实现 */ static inline int drawPause(void) { return -1; }
/* 绘图功能已关闭，这里是空实现 */ static inline int drawResume(void) { return -1; }
/* 绘图功能已关闭，这里是空实现 */ static inline int drawCancel(void) { return -1; }

/* 按键入口 */
/* 绘图功能已关闭，这里是空实现 */ static inline int drawHandleButton(int key) { (void)key; return -1; }
/* 绘图功能已关闭，这里是空实现 */ static inline bool drawAcceptButton(int key) { (void)key; return false; }

/* ---------- 状态查询（探针与调试用） ---------- */
/* 绘图功能已关闭，这里是空实现 */ static inline bool drawControlBusy(void) { return false; }
/* 绘图功能已关闭，这里是空实现 */ static inline bool drawControlLocked(void) { return false; }
/* 绘图功能已关闭，这里是空实现 */ static inline int drawGetPhase(void) { return -1; }
/* 绘图功能已关闭，这里是空实现 */ static inline const char *drawPhaseName(void) { return ""; }
/* 绘图功能已关闭，这里是空实现 */ static inline const char *drawStateName(void) { return ""; }
/* 绘图功能已关闭，这里是空实现 */ static inline bool drawIsPaused(void) { return false; }

/* 上一次绘制的结果 */
/* 绘图功能已关闭，这里是空实现 */ static inline unsigned long drawLastRunMs(void) { return 0UL; }
/* 绘图功能已关闭，这里是空实现 */ static inline int drawLastPointsHit(void) { return 0; }
/* 绘图功能已关闭，这里是空实现 */ static inline int drawLastPointsTotal(void) { return 0; }
/* 绘图功能已关闭，这里是空实现 */ static inline int drawLastResult(void) { return -1; }
#endif

#endif /* DRAW_CONTROL_H */
