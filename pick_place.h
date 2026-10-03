/*
 * pick_place.h -- 物体 A / B / C 的自动取放序列
 *
 * 上位机在串口里发单个字母 A / B / C，本模块用一条非阻塞状态机自动完成整套动作：
 *     移动到物体上方 -> 垂直下降 -> 合爪 -> 停顿 -> 垂直抬起
 *     -> 平移到放置点上方 -> 垂直下降 -> 松爪 -> 停顿 -> 垂直撤离
 *
 * 位置表（每个物体的初始位置与放置位置）写在 pick_place.cpp 顶部，可直接改。
 * 三个物体的初始位置互不相同，三个放置位置也互不相同；而且对每个物体来说，
 * 放置位置相对它的初始位置在 x 与 y 两个方向上都有明显位移（表里逐条标了 Δx/Δy）。
 *
 * 【与摇杆、串口的关系】
 *  - 序列执行期间由本模块独占 b/r/c 三个关节角，摇杆步进被让位
 *    （joystick_control.cpp 里的轴步进会跳过）；
 *  - 序列执行期间串口的 O/S/k/K 与 x,y,z 角度指令会被拒绝（调用方拿到 PROTO_RES_BUSY），
 *    只有 H/L/1/2/3 这些调速指令仍然生效 —— 可以一边跑一边改速度；
 *  - 序列本身是非阻塞的：每轮 loop() 调一次 pickPlaceLoop() 推进一步，
 *    期间串口接收与指示灯照常工作。
 *
 * 【为什么序列要用反解走直线】
 * 每个阶段都是在 x/y/z 上线性插值（所以下降/抬起是垂直的），插值点用固件自己的
 * getAngleEx() 反解成关节角，写进 Pos.ser 后只刷新一次 Pos.rec。
 * 启动前会把整条路径采样验证一遍（在 limit 内、isReachable()、关节角没被吸附、
 * 相邻采样点的反解分支不跳变），有一条不满足就拒绝启动、一个字节都不改。
 */
#ifndef PICK_PLACE_H
#define PICK_PLACE_H

#include "weArm_config.h"

/* 物体编号（与上位机的 A / B / C 一一对应） */
#define PICK_OBJECT_A 0
#define PICK_OBJECT_B 1
#define PICK_OBJECT_C 2
#define PICK_OBJECT_COUNT 3

#if WEARM_ENABLE_PICK_PLACE
/* 启动一次取放序列。
 *   object : PICK_OBJECT_A / B / C
 * 返回：
 *    0  已开始（pickPlaceIsBusy() 变为 true）
 *   -1  物体编号非法
 *   -2  已经有一个序列在执行（本次忽略）
 *   -3  路径校验失败（当前位姿到物体、或物体到放置点的路径上有不可达/越界点），
 *       拒绝启动，Pos 一个字节都不改                                 */
int pickPlaceStart(int object);

/* 每轮 loop() 调用一次，推动序列前进（非阻塞，空闲时立即返回）。 */
void pickPlaceLoop(void);

/* 是否正在执行取放序列。 */
bool pickPlaceIsBusy(void);

/* 正在执行的物体编号；空闲时返回 -1。 */
int pickPlaceCurrentObject(void);

/* 当前阶段名（调试与探针用，空闲时是 "idle"）。 */
const char *pickPlaceStageName(void);

/* 取某个物体的初始位置 / 放置位置（工作区坐标，单位与 Pos.rec 相同）。
 * 探测点用：探针靠它核对"三个物体位置互不相同、x 与 y 都有明显位移"。
 * 物体编号非法时返回 false。 */
bool pickPlaceGetSource(int object, double *x, double *y, double *z);
bool pickPlaceGetTarget(int object, double *x, double *y, double *z);

/* 接近 / 撤离高度（相对取放点的 z 抬升量，也是平移时的飞行高度）。
 * 单值真值在 pick_place.cpp 的 PICK_APPROACH_DZ，探针与上位机靠这个取，不要各自写死。 */
double pickPlaceApproachDz(void);
#else
/* 取放功能已关闭，这里是空实现 */
static inline int  pickPlaceStart(int object)   { (void)object; return -1; }
static inline void pickPlaceLoop(void)          { }
static inline bool pickPlaceIsBusy(void)        { return false; }
static inline int  pickPlaceCurrentObject(void) { return -1; }
static inline const char *pickPlaceStageName(void) { return ""; }
static inline bool pickPlaceGetSource(int object, double *x, double *y, double *z)
    { (void)object; (void)x; (void)y; (void)z; return false; }
static inline bool pickPlaceGetTarget(int object, double *x, double *y, double *z)
    { (void)object; (void)x; (void)y; (void)z; return false; }
static inline double pickPlaceApproachDz(void)  { return 0.0; }
#endif

#endif
