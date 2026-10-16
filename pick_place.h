/*
 * pick_place.h -- 物体 A / B / C 的自动取放序列
 *
 * 上位机在串口里发单个字母 A / B / C，本模块用一条非阻塞状态机自动完成整套动作。
 * 路径按用户口径：以物体高度为基准（坐标单位就是 mm，5cm = 50.0、2cm = 20.0）
 *     先上移到「物体高度 + 5cm」-> 沿水平方向移到「物体后方 2cm」
 *     -> 下降到与物体同高 -> 沿水平方向「向前 2cm」把物体送进两爪之间
 *     -> 合爪（快合到名义合拢角，再慢压到夹紧角）-> 停顿
 *     -> 抬回「物体高度 + 5cm」-> 沿水平方向移到「目标点正上方」
 *     -> 下降到目标高度（名义上正好 5cm）-> 张爪 -> 停顿
 *     -> 抬回飞行高度 -> 回到开机初始位姿 POS_HOME
 *
 * 合爪之后到张爪之前的每一帧都重申夹紧角，搬运途中不会松开（见 pickHoldingStage）。
 * 「后方 / 向前」取的是基座的水平半径方向，不是固定 ±x（见 pickBehindPoint）。
 *
 * 位置表（每个物体的初始位置与放置位置）写在 pick_place.cpp 顶部，可直接改。
 * 物体与目标的放置高度允许 -18 ~ +171 mm（实测可达竖直带，见 pick_place.cpp 文件头）；
 * 物体越高可达的水平半径越小，
 * 所以「+5cm 的飞行高度」在物体很高时会整个落在可达包络之外，这时
 * planTravelZ() 会逐档降低飞行高度，最低不低于「最高物体 + 1cm」。
 * 飞行高度是否被降过可以用 pickPlacePlanZ() 查（探针用它断言取值区间）。
 *
 * 【与摇杆、串口的关系】
 *  - 序列执行期间由本模块独占 b/r/c 三个关节角，摇杆步进被让位
 *    （joystick_control.cpp 里的轴步进会跳过）；
 *  - 序列执行期间串口的 O/S 与 x,y,z 角度指令会被拒绝（调用方拿到 PROTO_RES_BUSY），
 *    只有 H/L 这些调速指令仍然生效 —— 可以一边跑一边改速度；
 *  - 序列本身是非阻塞的：每轮 loop() 调一次 pickPlaceLoop() 推进一步，
 *    期间串口接收与指示灯照常工作。
 *
 * 【为什么序列要用反解走直线】
 * 每个移动阶段都是在 x/y/z 上线性插值（所以下降/抬起是垂直的），插值点用固件自己的
 * getAngleEx() 反解成关节角，写进 Pos.ser 后只刷新一次 Pos.rec。最后回初始位姿
 * 那一段走的是关节空间插值（POS_HOME 本身就是关节姿态，不必再反解）。
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
 *   -3  路径校验失败（当前位姿不可用 / 不在自己的反解分支上，或路径上有不可达、
 *       越界、分支跳变的点），拒绝启动，Pos 一个字节都不改
 *   -4  位置表高度超出 PICK_HEIGHT_MIN/MAX（-18 ~ +171 mm），或整段区间都找不到可达的飞行高度 */
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

/* 理想的接近 / 撤离高度（相对取放点的 z 抬升量，即「物体高度 + 5cm」）。
 * 单值真值在 pick_place.cpp 的 PICK_LIFT_DZ，探针与上位机靠这个取，不要各自写死。
 * 物体很高时实际用的飞行高度会比它低（被 planTravelZ 降过），要用 pickPlacePlanZ()。 */
double pickPlaceApproachDz(void);

/* 「物体后方 2cm」的水平退让距离（单值真值 PICK_BACK_DZ）。 */
double pickPlaceBackDz(void);

/* 按当前位置与物体 object 的位置表，算出这次序列实际会用的飞行高度（工作区 z）。
 * 只做规划、不改变任何状态；找不到可达的飞行高度时返回 false。 */
bool pickPlacePlanZ(int object, double *z);
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
static inline double pickPlaceBackDz(void)      { return 0.0; }
static inline bool pickPlacePlanZ(int object, double *z)
    { (void)object; (void)z; return false; }
#endif

#endif
