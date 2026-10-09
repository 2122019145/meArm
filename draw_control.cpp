/*
 * draw_control.cpp -- 纸面轨迹绘制（实现）
 *
 * 【整体结构】
 *   ┌ 动作编排（一条非阻塞状态机，每轮 loop 推进一步）──────────────────┐
 *   │ IDLE -> HOME(回待机) -> TRAVEL(抬笔平移到起点上方) -> PLUNGE(落笔) │
 *   │      -> PATH(沿轨迹绘制) -> LIFT(抬笔) -> RETURN(回待机) -> IDLE    │
 *   │ 示教任务: IDLE -> HOME -> TEACH(摇杆点动 + 按键记录)                │
 *   │      -> TEACH_WAIT(停顿) -> HOME -> TRAVEL -> PLUNGE -> PATH -> ... │
 *   └────────────────────────────────────────────────────────────────────┘
 *
 * 【轨迹是怎么走的】
 *   轨迹在笛卡尔空间定义（折线 = 顶点连线，曲线 = 向心 Catmull-Rom 样条）。
 *   每个采样点交给 move.h 的 moveToPoint()（"移动到指定 x,y,z 坐标"的公共核心，
 *   取放序列与绘图内部都走它）：反解、速率限制、写入 Pos 与正解刷新
 *   都在那里，本文件只决定"下一步走到哪个坐标"。
 *   （与 pick_place 同一约定：坐标永远等于角度的真实结果；串口 x/y/z 指令
 *   按题目要求直接写关节角，不经过这里。）
 *
 *   速度规划用"按剩余距离刹车"的经典做法，不需要预先算速度表：
 *       v 允许的最大值 = min( DRAW_V_MAX, sqrt(2a·已走距离), 当前速度 + a·dt )
 *       （第一项是巡航上限，第二项保证到终点刚好刹停，第三项限制加速度）
 *   于是整条轨迹是一段梯形速度：起步加速、中段匀速、到终点减速停住。
 *   折线在顶点处不特意减速，靠 DRAW_V_MAX 取得比较温和来保证过弯平稳。
 *
 *   关节速率硬上限 DRAW_MAX_DPS 是最后一道保险：若某一步反解出的角度会让某个关节
 *   比上一轮跳得太多，就把这一步的位移折半重试（最多 4 次），还是超限就本轮不动、
 *   轨迹参数也不推进 —— 宁可慢一点，也不允许舵机猛跳或笔尖偏离轨迹。
 *
 * 【精细化插值（本分支）】
 *   上面算出的 step 送去执行前先过一道上限：单次写入的位移不超过
 *   DRAW_STEP_MAX（0.02 工作区单位）。配合放慢后的速度（3 单位/秒、加减速 6、
 *   关节上限 70°/s），笔尖的轨迹由几百个密排的点组成，舵机每步只动零点几度，
 *   看上去是"缓慢匀速滑过去"，顶点附近也不会甩过冲。
 *   正常 loop 周期（1~5 ms）下算出来的 step 只有 0.003~0.015 单位，上限不生效；
 *   只有 loop 被拖长的那一轮才限住（少走一点，剩下的下一轮补），宁慢不跳。
 *   路径校验的采样步长也同步从 0.5 收紧到 0.25。
 *
 * 【示教点动】
 *   示教模式下摇杆由本模块自己读（joystickReadState），推动改变的是笔尖 x/y/z
 *   （笛卡尔点动），不是关节角。每步都做一次完整校验（limit + 可达 + 反解不被吸附），
 *   校验不过就原地不动并在串口说一句，绝不会把机械臂带到解不出来的地方。
 *
 * 【为什么抬笔要单独一个阶段】
 *   铅笔固定在夹具上，笔尖贴着纸面平移会留下一道多余的线。所以凡是"不画"的移动
 *   （回待机、走到起点、画完撤离）都在 z + DRAW_LIFT_DZ 的高度上做，落笔/抬笔
 *   各自是一段垂直运动。抬笔高度会在可达性允许的范围内自动降低（见 liftZFor）。
 */

#include <math.h>
#include <Arduino.h>
#include "constant_and_positions.h"
#include "move.h"           /* moveToPoint(): "移动到指定 x,y,z" 的唯一实现 */
#include "path_core.h"
#include "joystick_control.h"
#include "pick_place.h"
#include "button_control.h"
#include "draw_control.h"

#define DRAW_DEBUG_SERIAL WEARM_DEBUG_SERIAL   /* 置 1: 打开绘图模块的串口日志 */

#if WEARM_ENABLE_DRAW

/* ==================== 可自定义的参数 ==================== */

/* --- 纸面几何（上机标定用，也可以随时发串口命令 p/n/o 改） --- */
#define DRAW_PEN_Z_DEFAULT    12.0   /* 纸面高度：铅笔尖刚好落在纸上时的 Pos.rec.z */
#define DRAW_CENTER_X_DEFAULT 20.0   /* 内置图形的中心 x */
#define DRAW_CENTER_Y_DEFAULT  0.0   /* 内置图形的中心 y */
#define DRAW_HALF_DEFAULT      6.0   /* 内置图形的半宽（顶点落在 ±半宽 的正方形里） */
#define DRAW_PEN_Z_MIN        -5.0
#define DRAW_PEN_Z_MAX        30.0
#define DRAW_HALF_MIN          2.0
#define DRAW_HALF_MAX         12.0
#define DRAW_CENTER_LIMIT     35.0   /* 中心允许的 |x|、|y| 上限（工作区单位） */

/* 抬笔高度：不画图的移动都在 纸面 + 这个高度 上做。
 * 若该高度在工作空间外会自动降低（见 liftZFor），所以取得高一点没关系。 */
#define DRAW_LIFT_DZ           4.0

/* --- 速度（本分支：整体放慢，见下） --- */
/* 【本分支改了什么】把绘制运动调慢调细：
 *   线速度 8 -> 3 单位/秒、加速度 12 -> 6、关节角硬上限 110 -> 70°/s，
 *   并且每一轮最多只走 DRAW_STEP_MAX（见 pathTick）。
 *   于是每条轨迹都变成"很多个很小的位移点"，舵机是一小格一小格叠上去的。 */
#define DRAW_V_MAX             3.0   /* 笔尖最大线速度（工作区单位/秒） */
#define DRAW_ACCEL             6.0   /* 加减速度（单位/秒^2） */
#define DRAW_MAX_DPS          70.0   /* 关节角速度硬上限（度/秒） */
#define DRAW_HOME_DPS         50.0   /* 回待机位的关节角速度（度/秒） */
#define DRAW_JOG_MAX_DPS     200.0   /* 示教点动的关节角速度上限（度/秒，比自动绘制宽松：手动操作宁快勿卡） */
/* 单次写入的最大位移（工作区单位）：一轮里要走的距离超过它就只走这么多，
 * 剩下的留给下一轮（正常 loop 周期下用不到它）。
 * 0.02 相当于"一条 12 单位的直线至少分 600 步"，关节角每步只动零点几度。 */
#define DRAW_STEP_MAX          0.02

/* --- 时间 --- */
#define DRAW_TICK_MAX_MS      50UL   /* 单次推进最多认 50ms（串口打印卡顿保护） */
#define DRAW_TEACH_START_DELAY_MS 700UL /* 记录满 5 点后的停顿（给操作者松手的时间） */
#define DRAW_HOME_TOL_DEG      0.4   /* 回待机位的到位容差（度） */

/* --- 轨迹校验（与 pick_place 同一套门限思想） --- */
#define DRAW_PATH_SAMPLE_STEP  0.25  /* 轨迹校验的采样步长（本分支 0.5 -> 0.25，校验更细） */
#define DRAW_PATH_SAMPLE_MAX  96
#define DRAW_BRANCH_JUMP_DEG  25.0

/* --- 示教 --- */
#define DRAW_POINT_TOL         0.8   /* 过点判定容差（工作区单位） */
#define DRAW_JOG_SCALE         0.5   /* 末端 f 的点动步长 = speed.stepSize × 这个系数（度） */
#define DRAW_JOG_CENTER      512     /* 摇杆中位 ADC 值 */
/* 【示教点动的坐标步长（本次修复）】
 * x/y/z 是"工作区单位"（0~40 左右），而 speed.stepSize 在手动模式里的含义是
 * "每个控制周期转过的角度（度）" —— 直接拿它当位移用会让笔尖一格跳出半个工作区。
 * 所以这里单独给一套坐标步长：满偏时每格最多 DRAW_JOG_STEP_MAX 工作区单位，
 * 并按偏转比例缩小；配上 10~40ms 的步进间隔，满偏速度约 6 单位/秒
 * （比自动绘制的 DRAW_V_MAX 3 快一倍，手动点动够灵敏，但不会"一推就飞"）。 */
#define DRAW_JOG_STEP_MAX      0.25  /* 满偏时每格位移（工作区单位） */
#define DRAW_JOG_STEP_MIN      0.02  /* 轻微偏转时的每格位移下限（再小就点不动了） */
/* 示教期间是否允许摇杆动末端夹爪：默认 0 = 不响应 A2。
 * 夹爪里夹着笔（或笔夹），示教时顺手推右摇杆很容易带开夹爪把笔顶歪/掉笔，
 * 而"点动"本身根本不需要它。要恢复旧行为把这里改成 1。 */
#define DRAW_JOG_TOOL_ENABLE   0
/* 哪一路觉得方向反了就把对应的宏改成 1（不用改逻辑） */
#define DRAW_JOG_INVERT_X      0
#define DRAW_JOG_INVERT_Y      0
#define DRAW_JOG_INVERT_Z      0

/* ==================== 状态 ==================== */

/* 内置图形的归一化顶点：u=v=0 是图形中心，[-1,1] 映射到 ±半宽。
 * 顶点按"一笔画完"的顺序排列（中间不抬笔），相邻两点画一段。 */
struct drawVertex {
  double u;
  double v;
};

/* The tables live in flash (PROGMEM): they are read-only lookups, so keeping a
 * RAM copy would waste data space for nothing. Read them with pgm_read_float. */
static const struct drawVertex SHAPE_LINE[2] PROGMEM = {
  { -1.0,  0.0 }, {  1.0,  0.0 }
};
/* 字母 N：左下 -> 左上 -> 右下 -> 右上（两个竖 + 一道斜） */
static const struct drawVertex SHAPE_N[4] PROGMEM = {
  { -1.0, -1.0 }, { -1.0,  1.0 }, {  1.0, -1.0 }, {  1.0,  1.0 }
};
/* 三角形：左下 -> 右下 -> 顶点 -> 回到左下（最后一点与第一点重合，闭合） */
static const struct drawVertex SHAPE_TRIANGLE[4] PROGMEM = {
  { -1.0, -1.0 }, {  1.0, -1.0 }, {  0.0,  1.0 }, { -1.0, -1.0 }
};
/* 字母 Z：左上 -> 右上 -> 左下 -> 右下（上横 + 斜 + 下横） */
static const struct drawVertex SHAPE_Z[4] PROGMEM = {
  { -1.0,  1.0 }, {  1.0,  1.0 }, { -1.0, -1.0 }, {  1.0, -1.0 }
};
/* 字母 V：左上 -> 底尖 -> 右上 */
static const struct drawVertex SHAPE_V[3] PROGMEM = {
  { -1.0,  1.0 }, {  0.0, -1.0 }, {  1.0,  1.0 }
};

/* 任务名（下标即 DRAW_TASK_*）。
 * 7 个任务：5 个内置图形 + 2 个五点示教。
 * 【恢复记录】字母N / 三角形 / 字母Z 曾在 v1.6.1 为省 flash 删掉过，
 * 现按题目"同组同学所选图形不得完全相同"恢复；编号见 draw_control.h。 */
static const char *const DRAW_TASK_NAME[DRAW_TASK_COUNT] = {
  "直线", "字母N", "三角形", "字母Z", "字母V", "五点折线", "五点曲线"
};

/* ASCII 任务短标签（下标即 DRAW_TASK_*），供串口回报当前图形模式用：
 * 终端和 ESP8266 都按单字节比较，所以这里不能用中文任务名。 */
static const char *const DRAW_TASK_TAG[DRAW_TASK_COUNT] = {
  "LINE", "N", "TRI", "Z", "V", "POLY", "CURVE"
};

/* 阶段名（下标即 DRAW_PHASE_*） */
static const char *const DRAW_PHASE_NAME[] = {
  "空闲", "回待机", "抬笔移动", "落笔", "绘制中", "抬笔", "示教中", "准备绘制", "结束回待机"
};

/* --- 任务与状态 --- */
/* Small-range state is stored in 1-byte types: ranges are (task 0..3), (phase 0..8),
 * (counts 0..5), (afterHome 0..2), so nothing observable changes. */
static int8_t s_task     = DRAW_TASK_LINE;      /* 当前选中的任务（本分支默认直线） */
static int8_t s_phase    = DRAW_PHASE_IDLE;
static bool s_paused   = false;
static int  s_lastRes  = PROTO_RES_NONE;
static unsigned long s_lastMs = 0UL;          /* drawLoop 上一次推进的时刻 */
static double s_dtSec  = 0.0;                 /* 本轮推进的时长（秒） */
static unsigned long s_applyMs = 0UL;         /* 上一次真正写入轨迹点位的时刻 */

/* --- 纸面标定 --- */
static double s_penZ    = DRAW_PEN_Z_DEFAULT;
static double s_centerX = DRAW_CENTER_X_DEFAULT;
static double s_centerY = DRAW_CENTER_Y_DEFAULT;
static double s_half    = DRAW_HALF_DEFAULT;

/* --- 轨迹（顶点表 / 控制点表） --- */
static double s_pts[DRAW_TEACH_MAX_POINTS][3];
static int8_t s_ptCount  = 0;      /* 顶点个数（示教任务 = 已记录的点数） */
static bool   s_curved   = false;  /* true = 样条曲线，false = 折线 */
static bool   s_isTeach  = false;  /* 当前任务是不是示教任务 */
static double s_totalLen = 0.0;                  /* 轨迹总弧长 */
static double s_ptArc[DRAW_TEACH_MAX_POINTS];    /* 各顶点在轨迹上的弧长位置 */

/* --- 轨迹跟随 --- */
static double s_done = 0.0;   /* 已走弧长 */
static double s_v    = 0.0;   /* 当前线速度（单位/秒） */
static int8_t s_span = 0;     /* 曲线：当前样条段号 */
static double s_u    = 0.0;   /* 曲线：段内参数 [0,1] */

/* --- 直线移动阶段（TRAVEL / PLUNGE / LIFT） --- */
static double s_mFrom[3], s_mTo[3];
static double s_mLen = 0.0, s_mDone = 0.0, s_mV = 0.0;
static int8_t s_nextAfterMove = DRAW_PHASE_IDLE;

/* 回待机结束之后进入哪里 */
enum { AFTER_HOME_IDLE = 0, AFTER_HOME_TRAVEL, AFTER_HOME_TEACH };

/* --- 回待机（关节空间插值） --- */
static double s_homeTarget[3] = { 0.0, 0.0, 0.0 };  /* 只用前三个 */
static int8_t s_afterHome = AFTER_HOME_IDLE;

/* --- 卡死保护 --- */
#define DRAW_STALL_LIMIT 500        /* 连续多少轮"原地不动又没到位"就放弃任务 */
static int    s_stallCount = 0;

/* --- 绘制结果统计 --- */
static unsigned long s_pathStartMs = 0UL;
static unsigned long s_lastRunMs   = 0UL;
static int8_t s_hitCount  = 0;
static int8_t s_hitTotal  = 0;
/* 5 个顶点是否已经过点：用 1 字节位图代替 bool[5]，判定逻辑逐字等价 */
static uint8_t s_ptHit = 0;

/* --- 示教 --- */
static unsigned long s_teachWaitMs = 0UL;
static unsigned long s_jogLastMs[4] = { 0UL, 0UL, 0UL, 0UL };  /* x/y/z/f 各自的步进计时 */

/* ==================== 小工具 ==================== */

/* 反解 / 点校验 / 直线段校验三件套原先在本文件有一份逐字实现，pick_place.cpp 里
 * 还有一份完全一样的。现在实现搬到 path_core.h 由两个模块共用（链接器只保留一份），
 * 这里保留同名同签名的转发函数，本文件所有调用点一行都不用改；采样步长与分支
 * 跳变门限仍用本文件的 DRAW_* 常量。转发函数是 static inline，会被就地展开，
 * 不会另外留下一份代码。 */
/* 注：原来这里还有一个 solveJoint() 转发，只被下面的点位写入用过；点位写入改成
 * 调 move.h 的 moveToPoint() 之后它没有调用方了，已删除（path_core.h 里的实现
 * 仍由 pointOk 与 pick_place.cpp 使用）。 */

/* 这个工作区点能不能用：在 limit 内 + isReachable() + 反解成功且没被吸附。 */
static inline bool pointOk(double x, double y, double z,
                           double *b, double *r, double *c) {
  return pathCorePointOk(x, y, z, b, r, c);
}

/* 校验一条直线段：逐点检查能不能用，并且相邻采样点的反解分支不跳变。
 * 本文件原先用六个 double 传两个端点，现在统一改成传两个点数组
 * （s_pts[] 的行本身就是连续三个 double，直接传地址即可），
 * 调用点少了一次压栈，采样与判据一个字没改。 */

/* 三个轴一次写完，只做一次正解刷新（与串口角度指令同一约定） */
static inline void setJoints(double b, double r, double c) {
  pathCoreSetJoints(b, r, c);
}

/* 把某个角度限制在 [lo,hi] */
static double clampDouble(double v, double lo, double hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* 两点距离 */
static double dist3(const double *a, const double *b)
{
  double dx = a[0] - b[0];
  double dy = a[1] - b[1];
  double dz = a[2] - b[2];
  return sqrt(dx * dx + dy * dy + dz * dz);
}

/* ==================== 内置图形 ==================== */

/* 取任务对应的归一化顶点表；返回顶点个数（0 = 任务没有内置图形） */
static int shapeTable(int task, const struct drawVertex **tbl)
{
  switch (task) {
    case DRAW_TASK_LINE:     *tbl = SHAPE_LINE;     return 2;
    case DRAW_TASK_N:        *tbl = SHAPE_N;        return 4;
    case DRAW_TASK_TRIANGLE: *tbl = SHAPE_TRIANGLE; return 4;
    case DRAW_TASK_Z:        *tbl = SHAPE_Z;        return 4;
    case DRAW_TASK_V:        *tbl = SHAPE_V;        return 3;
    default:                 *tbl = NULL;           return 0;
  }
}

/* 把内置图形展开成工作区坐标的顶点表（半宽 × 归一化 + 中心，z = 纸面高度） */
static void buildShapePath(int task)
{
  const struct drawVertex *tbl = NULL;
  int n = shapeTable(task, &tbl);
  if (n > DRAW_TEACH_MAX_POINTS) n = DRAW_TEACH_MAX_POINTS;   /* 表不会超，纯保险 */

  s_ptCount = (int8_t)n;
  s_curved  = false;
  s_isTeach = false;
  for (int i = 0; i < n; i++) {
    /* pgm_read_float reads the identical IEEE value that used to sit in RAM */
    s_pts[i][0] = s_centerX + pgm_read_float(&tbl[i].u) * s_half;
    s_pts[i][1] = s_centerY + pgm_read_float(&tbl[i].v) * s_half;
    s_pts[i][2] = s_penZ;
  }
}

/* ==================== 曲线（向心 Catmull-Rom） ==================== */

/* 节点间距：用 |P_a - P_b| 的平方根（向心参数化，点距不均匀时不会打圈） */
static double knotDelta(int a, int b)
{
  double d = dist3(s_pts[a], s_pts[b]);
  if (d < 1e-6) return 0.001;
  return sqrt(d);
}

/* 第 span 段（P[span] -> P[span+1]）在参数 u∈[0,1] 处的点 p[3] 与切线 d[3]。
 * 切线用"相邻点差 / 节点间距"的向心 Catmull-Rom 形式，
 * 端点用弦方向（不外推），保证曲线不会冲出五个目标点之外。
 * 三个坐标共用一个循环：每一项的算式与运算次序和逐坐标展开时逐字相同，
 * 所以结果与展开版 bit 级一致，只是省掉了 3 份重复代码。
 *
 * （实测记录：把"切线计算"抽成单独函数、让算点与算导数各走一条路
 *   Program 反而 +170 字节——切线块 1634B、curvePoint 1396B，
 *   拆开后光切线就比原来整个 curveSpan(1518B) 还大，故保持合并的写法。） */
static void curveSpan(int span, double u, double *p, double *d)
{
  int n = s_ptCount;
  int i = span;
  double dt = knotDelta(i, i + 1);

  /* t1/t2 = 两个端点的切线 × dt（先除后乘，次序不变，值不变） */
  double t1[3], t2[3];
  if (i == 0) {
    for (int k = 0; k < 3; k++) t1[k] = (s_pts[1][k] - s_pts[0][k]) / dt * dt;
  } else {
    double sum = knotDelta(i - 1, i) + dt;
    for (int k = 0; k < 3; k++) t1[k] = (s_pts[i + 1][k] - s_pts[i - 1][k]) / sum * dt;
  }
  if (i + 1 == n - 1) {
    for (int k = 0; k < 3; k++) t2[k] = (s_pts[n - 1][k] - s_pts[n - 2][k]) / dt * dt;
  } else {
    double sum = dt + knotDelta(i + 1, i + 2);
    for (int k = 0; k < 3; k++) t2[k] = (s_pts[i + 2][k] - s_pts[i][k]) / sum * dt;
  }

  double u2 = u * u;
  double u3 = u2 * u;
  /* 两个恒等式，IEEE 下逐位成立，各少几次软浮点调用：
   *   h01 = fl(-2u³+3u²) = -fl(2u³-3u²)   （舍入对符号左右对称，乘 2 精确）
   *   而 h00 = fl(fl(2u³-3u²)+1) = fl(1-h01)（x+1 与 1+x 同一次舍入）
   *   e1  = fl(-6u²+6u) = -fl(6u²-6u) = -e0
   * 所以 h00 可以写成 1.0 - h01、e1 可以写成 -e0，数值逐位不变。 */
  double h01 = -2.0 * u3 + 3.0 * u2;
  double h00 =  1.0 - h01;
  double h10 =         u3 - 2.0 * u2 + u;
  double h11 =         u3 -       u2;
  double g10 =  3.0 * u2 - 4.0 * u + 1.0;
  double g11 =  3.0 * u2 - 2.0 * u;
  double e0  =  6.0 * u2 - 6.0 * u;
  double e1  = -e0;

  /* p[k] 与 d[k] 共用一个 k 循环（实测：拆成两个循环各自少一点寄存器压力，
   * 但 Program 反而 +84 字节，所以保持合并——共用 a0/a1 与一次循环控制更划算）。 */
  for (int k = 0; k < 3; k++) {
    double a0 = s_pts[i][k];
    double a1 = s_pts[i + 1][k];
    p[k] = h00 * a0 + h10 * t1[k] + h01 * a1 + h11 * t2[k];
    d[k] = e0 * a0 + g10 * t1[k] + e1 * a1 + g11 * t2[k];
  }
}

/* 曲线在参数 (span,u) 处的点 */
static void curvePoint(int span, double u, double *p)
{
  double d[3];
  curveSpan(span, u, p, d);
}

/* 曲线在参数 (span,u) 处的 dP/du 模（推进时用来换算成"匀速"） */
static double curveSpeed(int span, double u)
{
  double p[3], d[3];
  curveSpan(span, u, p, d);
  return sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
}

/* 曲线总弧长与"各控制点在曲线上的弧长位置"（都是采样求和，够用且简单） */
static void curveMeasure(void)
{
  const int SAMPLE_PER_SPAN = 64;
  double len = 0.0;
  double prev[3];
  curvePoint(0, 0.0, prev);

  /* 第 0 个控制点就在曲线起点 */
  s_ptArc[0] = 0.0;
  for (int span = 0; span < s_ptCount - 1; span++) {
    for (int k = 1; k <= SAMPLE_PER_SPAN; k++) {
      double u = (double)k / (double)SAMPLE_PER_SPAN;
      double cur[3];
      curvePoint(span, u, cur);
      len += dist3(prev, cur);
      prev[0] = cur[0]; prev[1] = cur[1]; prev[2] = cur[2];
    }
    /* 这一段走完正好到第 span+1 个控制点 */
    s_ptArc[span + 1] = len;
  }
  s_totalLen = len;
}

/* 折线：算各段长度、总长、各顶点弧长位置 */
static void polyMeasure(void)
{
  double acc = 0.0;
  s_ptArc[0] = 0.0;
  for (int i = 0; i < s_ptCount - 1; i++) {
    /* s_segLen[] was dropped from RAM; dist3() is deterministic on the same
     * two points, so recomputing it yields bit-identical values. */
    acc += dist3(s_pts[i], s_pts[i + 1]);
    s_ptArc[i + 1] = acc;
  }
  s_totalLen = acc;
}

/* 按当前 s_curved 重新测量轨迹 */
static void measurePath(void)
{
  if (s_curved) curveMeasure();
  else          polyMeasure();
}

/* ==================== 轨迹校验 ==================== */

/* 抬笔高度：先试 纸面 + DRAW_LIFT_DZ，太高（不可达）就一点点降下来。
 * 返回可用的抬笔 z；连纸面本身都不可达时返回纸面高度（调用方随后会校验失败）。 */
static double liftZFor(const double *pt)
{
  double z = pt[2] + DRAW_LIFT_DZ;
  while (z > pt[2]) {
    if (pointOk(pt[0], pt[1], z, NULL, NULL, NULL)) return z;
    z -= 0.5;
  }
  return pt[2];
}

/* 校验整条轨迹（含回待机 -> 起点上方 -> 落笔 -> 轨迹 -> 抬笔）。
 * 任何一段不满足就返回 false，调用方拒绝启动、一个字节都不改。 */
static bool validatePath(void)
{
  if (s_ptCount < 2) return false;

  /* 1) 待机位（POS_HOME 的位置）-> 起点正上方 */
  SER home;
  home = Pos.ser;
  if (!posGetHomeAngles(&home)) return false;
  REC hrec;
  if (!recFromServo(&hrec, &home)) return false;

  double lz = liftZFor(s_pts[0]);
  double hp[3];                       /* 待机位 */
  hp[0] = hrec.x; hp[1] = hrec.y; hp[2] = hrec.z;
  double ap[3];                       /* 起点正上方 */
  ap[0] = s_pts[0][0]; ap[1] = s_pts[0][1]; ap[2] = lz;
  if (!pathCoreSegmentOk(hp, ap, DRAW_PATH_SAMPLE_STEP,
                         DRAW_PATH_SAMPLE_MAX, DRAW_BRANCH_JUMP_DEG)) return false;

  /* 2) 垂直落笔 */
  if (!pathCoreSegmentOk(ap, s_pts[0], DRAW_PATH_SAMPLE_STEP,
                         DRAW_PATH_SAMPLE_MAX, DRAW_BRANCH_JUMP_DEG)) {
    return false;
  }

  /* 3) 轨迹本体 */
  if (s_curved) {
    /* 曲线：按参数密集采样，逐对检查可达性与反解分支 */
    const int SAMPLE_PER_SPAN = 24;
    double prevB = 0.0, prevR = 0.0, prevC = 0.0;
    bool first = true;
    for (int span = 0; span < s_ptCount - 1; span++) {
      for (int k = 0; k <= SAMPLE_PER_SPAN; k++) {
        double u = (double)k / (double)SAMPLE_PER_SPAN;
        double p[3];
        curvePoint(span, u, p);
        double b = 0.0, r = 0.0, c = 0.0;
        if (!pointOk(p[0], p[1], p[2], &b, &r, &c)) return false;
        if (!first) {
          if (fabs(b - prevB) > DRAW_BRANCH_JUMP_DEG ||
              fabs(r - prevR) > DRAW_BRANCH_JUMP_DEG ||
              fabs(c - prevC) > DRAW_BRANCH_JUMP_DEG) {
            return false;
          }
        }
        prevB = b; prevR = r; prevC = c;
        first = false;
      }
    }
  } else {
    for (int i = 0; i < s_ptCount - 1; i++) {
      if (!pathCoreSegmentOk(s_pts[i], s_pts[i + 1], DRAW_PATH_SAMPLE_STEP,
                             DRAW_PATH_SAMPLE_MAX, DRAW_BRANCH_JUMP_DEG)) {
        return false;
      }
    }
  }

  /* 4) 终点抬笔 */
  const double *last = s_pts[s_ptCount - 1];
  double ez = liftZFor(last);
  double ep[3];
  ep[0] = last[0]; ep[1] = last[1]; ep[2] = ez;
  if (!pathCoreSegmentOk(last, ep, DRAW_PATH_SAMPLE_STEP,
                         DRAW_PATH_SAMPLE_MAX, DRAW_BRANCH_JUMP_DEG)) {
    return false;
  }
  return true;
}

/* ==================== 点位写入与关节速率限制 ==================== */

/* 距上一次真正写入点位的时长（秒），夹在 [5ms, 100ms]。
 * 【为什么按"距上次写入"而不是按循环周期算速率上限】
 *   路径跟随每轮都写点，两者一样；但示教点动是事件驱动的（每 10~40ms 才动一次），
 *   若按循环周期（探针里 5ms → 110°/s×0.005 = 0.55°）当上限，一次点动要走的
 *   1.4~2.9° 会被整条拒绝，摇杆根本点不动。上限也要夹在 100ms，
 *   否则长时间暂停后的第一轮会允许一次大跳变。 */
static double applyDtSec(void)
{
  unsigned long now = millis();
  double dt = (double)(now - s_applyMs) / 1000.0;
  if (dt > 0.1) dt = 0.1;
  if (dt < 0.005) dt = 0.005;
  return dt;
}

/* 反解 (x,y,z) 并按策略写进 Pos —— 本文件所有"点位移"都走这里，实现则是 move.h 的
 * moveToPoint()（"移动到指定 x,y,z 坐标"的公共核心，取放序列与绘图共用）。
 *
 * 原来本文件自己展开过两份（严格版 tryApplyPoint / 夹取版 applyPointClamped），
 * 每份都要重算 maxDps×dtSec、逐轴比较或夹取；现在这些算式只存在于 moveToPoint()
 * 里，两边不可能再走偏。参数与判据一个字没改，所以三个调用点仍然逐位等价：
 *   MOVE_XYZ_TRACK —— 任一关节本步要动超过 maxDps×dtSec 就整点拒绝（轨迹跟随）；
 *   MOVE_XYZ_JOG   —— 超出的部分夹到上限，尽量走一点（示教点动，绝不原地卡住）。
 *
 * 成功后才前移速率基准时刻 s_applyMs —— 与原先在 setJoints() 之后写它的次序一致。 */
static bool applyPointStep(double x, double y, double z, double maxDps, double dtSec, int mode)
{
  const double goal[3] = { x, y, z };
  if (moveToPoint(goal, 0x07u, maxDps, dtSec, mode) != MOVE_XYZ_OK) return false;
  s_applyMs = millis();
  return true;
}

/* ==================== 速度规划 ==================== */

/* 梯形速度：按"还要走多远才能刹住"给当前允许的最大速度 */
static double speedLimit(double done, double total, double vCur, double dt)
{
  double remain = total - done;
  if (remain < 0.0) remain = 0.0;

  double vEnd = sqrt(2.0 * DRAW_ACCEL * remain);   /* 现在这个位置开始刹车刚好够 */
  if (vEnd > DRAW_V_MAX) vEnd = DRAW_V_MAX;

  double vRamp = vCur + DRAW_ACCEL * dt;           /* 这一步最多能加到多少 */
  double v = (vRamp < vEnd) ? vRamp : vEnd;
  if (v < 0.0) v = 0.0;
  return v;
}

/* 卡死保护：连续多轮既没到位、也没有任何位移，就放弃本任务。
 * 正常情况下不会触发（速度规划本身就保证每轮都前进），
 * 万一出现（例如关节速率限制与几何互相顶住）也不能让机械臂永远停在那儿。 */
static bool stallGuard(double movedNow)
{
  if (movedNow > 1e-6) {
    s_stallCount = 0;
    return false;
  }
  s_stallCount++;
  if (s_stallCount > DRAW_STALL_LIMIT) {
#if DRAW_DEBUG_SERIAL
    Serial.println(F("[draw] 无法继续前进（关节速率或可达性受限），放弃本次绘图"));
#endif
    s_stallCount = 0;
    s_paused = false;
    s_phase  = DRAW_PHASE_IDLE;
    s_lastRes = PROTO_RES_DRAW_REJECTED;
    return true;
  }
  return false;
}

/* ==================== 阶段切换 ==================== */

/* 计算 POS_HOME 的关节角，写进 s_homeTarget（只改前三个，angle4 保持不动） */
static void loadHomeTarget(void)
{
  SER ser = Pos.ser;
  if (!posGetHomeAngles(&ser)) ser = Pos.ser;   /* 失败时同样回到当前角度 */
  s_homeTarget[0] = ser.angle1;
  s_homeTarget[1] = ser.angle2;
  s_homeTarget[2] = ser.angle3;
}

/* 进入"回待机"阶段；after 为回完之后进入的阶段，phase 为对外显示的阶段号
 * （正常回待机 DRAW_PHASE_HOME；绘制结束/取消时用 DRAW_PHASE_RETURN，便于串口观察。
 *  两个变体只差一个常量，合并成一个函数以免各展开一份代码。） */
static void beginHome(int after, int phase)
{
  loadHomeTarget();
  s_afterHome  = (int8_t)after;
  s_stallCount = 0;
  s_applyMs    = millis();     /* 速率上限的计时基准重新起算，避免长时间空闲后一次大跳变 */
  s_phase = (int8_t)phase;
}

/* 进入一段直线移动（抬笔平移 / 落笔 / 抬笔） */
static void beginMove(int next, double x, double y, double z)
{
  s_mFrom[0] = Pos.rec.x;
  s_mFrom[1] = Pos.rec.y;
  s_mFrom[2] = Pos.rec.z;
  s_mTo[0] = x;
  s_mTo[1] = y;
  s_mTo[2] = z;
  s_mLen  = dist3(s_mFrom, s_mTo);
  s_mDone = 0.0;
  s_mV    = 0.0;
  s_applyMs = millis();        /* 同上：每段移动都从"这一刻"重新起算速率基准 */
  s_nextAfterMove = (int8_t)next;
}

/* 开始绘制序列：回待机 -> 抬笔到起点上方 -> 落笔 -> 绘制 */
static void beginSequence(void)
{
  s_paused  = false;
  s_done    = 0.0;
  s_v       = 0.0;
  s_span    = 0;
  s_u       = 0.0;
  s_hitCount = 0;
  s_hitTotal = s_isTeach ? s_ptCount : 0;
  s_ptHit = 0;

  /* 先回待机；回完由 drawLoop 转入"抬笔去起点上方" */
  beginHome(AFTER_HOME_TRAVEL, DRAW_PHASE_HOME);
}

/* ==================== 各阶段推进 ==================== */

/* 关节空间回待机：每个关节按 DRAW_HOME_DPS 匀速靠近目标，全部到位返回 true */
static bool homeTick(void)
{
  double maxStep = DRAW_HOME_DPS * s_dtSec;
  if (maxStep < 0.2) maxStep = 0.2;

  /* 三个关节走同一个循环；算式与逐关节展开时一模一样。
   * （实测：展开成 db/dr/dc 三个标量后 Program 反而 +204 字节，
   *   编译器对数组版能复用寄存器，标量版会各留一份活跃值，故保留循环。） */
  double tgt[3], cur[3], d[3];
  tgt[0] = s_homeTarget[0]; tgt[1] = s_homeTarget[1]; tgt[2] = s_homeTarget[2];
  cur[0] = Pos.ser.angle1;  cur[1] = Pos.ser.angle2;  cur[2] = Pos.ser.angle3;

  bool atHome = true;
  for (int i = 0; i < 3; i++) {
    d[i] = tgt[i] - cur[i];
    if (fabs(d[i]) > DRAW_HOME_TOL_DEG) atHome = false;
  }

  if (atHome) {
    setJoints(tgt[0], tgt[1], tgt[2]);
    return true;
  }

  setJoints(cur[0] + clampDouble(d[0], -maxStep, maxStep),
            cur[1] + clampDouble(d[1], -maxStep, maxStep),
            cur[2] + clampDouble(d[2], -maxStep, maxStep));
  return false;
}

/* 直线移动一步；到位返回 true */
static bool moveTick(void)
{
  if (s_mLen < 1e-6) return true;

  double v = speedLimit(s_mDone, s_mLen, s_mV, s_dtSec);
  double step = v * s_dtSec;
  if (step > s_mLen - s_mDone) step = s_mLen - s_mDone;

  double t = (s_mDone + step) / s_mLen;
  double x = s_mFrom[0] + (s_mTo[0] - s_mFrom[0]) * t;
  double y = s_mFrom[1] + (s_mTo[1] - s_mFrom[1]) * t;
  double z = s_mFrom[2] + (s_mTo[2] - s_mFrom[2]) * t;

  if (!applyPointStep(x, y, z, DRAW_MAX_DPS, applyDtSec(), MOVE_XYZ_TRACK)) {
    /* 关节速率不够（理论上只在异常时才发生）：本轮不动，下轮再试 */
    return false;
  }

  s_mDone += step;
  s_mV = (s_dtSec > 1e-6) ? (step / s_dtSec) : 0.0;
  return (s_mDone >= s_mLen - 1e-6);
}

/* 折线：弧长 s 处的点 */
static void polyPointAt(double s, double *px, double *py, double *pz)
{
  if (s <= 0.0) {
    *px = s_pts[0][0]; *py = s_pts[0][1]; *pz = s_pts[0][2];
    return;
  }
  double acc = 0.0;
  for (int i = 0; i < s_ptCount - 1; i++) {
    bool last = (i == s_ptCount - 2);
    double seg = dist3(s_pts[i], s_pts[i + 1]);   /* same value s_segLen[] held */
    if (s <= acc + seg || last) {
      double t = (seg > 1e-9) ? (s - acc) / seg : 1.0;
      t = clampDouble(t, 0.0, 1.0);
      *px = s_pts[i][0] + (s_pts[i + 1][0] - s_pts[i][0]) * t;
      *py = s_pts[i][1] + (s_pts[i + 1][1] - s_pts[i][1]) * t;
      *pz = s_pts[i][2] + (s_pts[i + 1][2] - s_pts[i][2]) * t;
      return;
    }
    acc += seg;
  }
}

/* 沿轨迹走一步。成功返回 true 并把 s_done/s_span/s_u 推进，
 * s_v 按"本步实际走出去的位移 / 本轮时长"更新（若折半过，记下的就是
 * 真正走成的速度）；
 * 若关节速率限制不允许（折半 4 次仍超限）则什么都不改、返回 false。 */
static bool pathAdvance(double step)
{
  if (step <= 0.0) return true;
  if (step > s_totalLen - s_done) step = s_totalLen - s_done;

  double tryStep = step;
  for (int attempt = 0; attempt < 4; attempt++) {
    double p[3] = { 0.0, 0.0, 0.0 };
    int    nspan = s_span;
    double nu    = s_u;

    if (s_curved) {
      double sp = curveSpeed(s_span, s_u);
      double du = (sp > 1e-6) ? (tryStep / sp) : 1.0;
      nu = s_u + du;
      while (nu >= 1.0 && nspan < s_ptCount - 2) { nu -= 1.0; nspan++; }
      if (nu > 1.0) nu = 1.0;
      curvePoint(nspan, nu, p);
    } else {
      polyPointAt(s_done + tryStep, &p[0], &p[1], &p[2]);
    }

    if (applyPointStep(p[0], p[1], p[2], DRAW_MAX_DPS, applyDtSec(), MOVE_XYZ_TRACK)) {
      s_done += tryStep;
      s_span  = (int8_t)nspan;
      s_u     = nu;
      s_v     = (s_dtSec > 1e-6) ? (tryStep / s_dtSec) : 0.0;
      return true;
    }
    tryStep *= 0.5;
  }
  return false;
}

/* 绘制阶段一步 */
static void pathTick(void)
{
  double remain = s_totalLen - s_done;
  double v = speedLimit(s_done, s_totalLen, s_v, s_dtSec);
  double step = v * s_dtSec;
  if (step > remain) step = remain;

  /* 【精细化插值】这一轮最多只送出去 DRAW_STEP_MAX（0.02 工作区单位）的位移。
   *   - 正常情况（loop 周期 1~5 ms、巡航 3 单位/秒）算出来的 step 只有
   *     0.003~0.015 单位，**这个上限根本不会生效**，速度完全按梯形规划走。
   *   - 只有当某一轮 loop 被拖长（串口打印、别的模块占时间）时才会限住，
   *     那一轮就少走一点、剩下的留给下一轮 —— 宁可慢，也不让舵机一次跳一大步。
   *   - 为什么不做"一轮里拆成多小步循环送出"：本工程用 LTO + -Os，
   *     把 pathAdvance() 放进循环里会让它无法被内联，实测同样功能要多花
   *     662 B flash（Uno 只剩 512 B 余量，装不下）。限一步的写法只多 32 B，
   *     效果一样：**任何一次写入的位移都不会超过 0.02 单位**。 */
  if (step > DRAW_STEP_MAX) step = DRAW_STEP_MAX;
  (void) pathAdvance(step);

  /* 过点统计：走过了第 i 个目标点的弧长位置就量一下笔尖离它多远 */
  for (int i = 0; i < s_ptCount; i++) {
    uint8_t bit = (uint8_t)(1u << i);
    if (s_ptHit & bit) continue;
    if (s_done + 1e-6 < s_ptArc[i]) continue;
    double dx = Pos.rec.x - s_pts[i][0];
    double dy = Pos.rec.y - s_pts[i][1];
    double dz = Pos.rec.z - s_pts[i][2];
    if (sqrt(dx * dx + dy * dy + dz * dz) <= DRAW_POINT_TOL) {
      s_ptHit |= bit;
      s_hitCount++;
    }
  }
}

/* 抬笔：从当前笔尖位置垂直到 (x, y, z)；到位返回 true */
static void beginLift(double z)
{
  beginMove(DRAW_PHASE_LIFT, Pos.rec.x, Pos.rec.y, z);
}

/* 绘制完成 / 中途取消共用：先垂直抬笔（不然笔尖会拖着纸走出一道多余的线），
 * 抬笔到位后再回待机。
 * ★ 必须显式把阶段切到"抬笔"：beginLift 只装移动参数，不改阶段。
 *   漏了这一行，取消后阶段还停在原来的 TRAVEL/PLUNGE/PATH，
 *   状态机会接着把这一笔按原轨迹画下去 —— 取消等于没取消。 */
static void beginLiftToReturn(void)
{
  double lz = Pos.rec.z + DRAW_LIFT_DZ;
  if (!pointOk(Pos.rec.x, Pos.rec.y, lz, NULL, NULL, NULL)) lz = Pos.rec.z;
  beginLift(lz);
  s_nextAfterMove = DRAW_PHASE_RETURN;
  s_phase = DRAW_PHASE_LIFT;
}

/* ==================== 示教 ==================== */

/* 示教点动一步的步进间隔：现在使用固定间隔 speed.stepDelayMs，不再随偏转变化。
 * 旧版偏转越大间隔越短（推得越狠走得越快）的动态调速已被移除。 */

/* 示教点动：读摇杆 -> 一路一路按各自的计时门控动一点笔尖（笛卡尔点动）。
 * 三路坐标（A0→x、A1→y、A3→z）用"工作区单位"的步长与间隔，轻微偏转走小步、
 * 满偏走大步；末端 A2 默认整路不响应（DRAW_JOG_TOOL_ENABLE 0）。
 * 每格位移都被 DRAW_JOG_STEP_MAX 卡住 ⇒ 单轴推杆只让对应的那一个坐标变化。 */
static void teachJogTick(void)
{
  struct joyState st;
  joystickReadState(&st);

  unsigned long now = millis();
  /* 四路：0=x(A0) 1=y(A1) 2=z(A3) 3=f(A2) */
  const int amp[4]   = { st.base, st.shoulder, st.elbow, st.tool };
  const int raw[4]   = { st.sx,   st.sy,        st.ty,      st.tx };

  for (int i = 0; i < 4; i++) {
    if (amp[i] <= 0) continue;
#if !DRAW_JOG_TOOL_ENABLE
    /* 夹爪里夹着笔：示教点动不响应 A2（右摇杆的左右推），免得一推带歪笔尖 */
    if (i == 3) continue;
#endif
    int interval = speed.stepDelayMs;
    if (now - s_jogLastMs[i] < (unsigned long)interval) continue;
    s_jogLastMs[i] = now;

    int sign = (raw[i] > DRAW_JOG_CENTER) ? 1 : -1;

    if (i == 3) {
      /* 末端 f：这一路是角度（度），与摇杆模块 toolStep() 同一比例，按偏转缩放 */
      double deg = speed.stepSize * DRAW_JOG_SCALE *
                   ((double)amp[i] / (double)DRAW_JOG_CENTER);
      if (deg < 0.05) deg = 0.05;
      (void) posSetAngle4(Pos.ser.angle4 - (double)sign * deg);
      continue;
    }

    /* x/y/z：这一路是工作区单位，按偏转比例缩放（绝不用 speed.stepSize 当距离） */
    double step = DRAW_JOG_STEP_MAX * ((double)amp[i] / (double)DRAW_JOG_CENTER);
    if (step < DRAW_JOG_STEP_MIN) step = DRAW_JOG_STEP_MIN;

    double nx = Pos.rec.x;
    double ny = Pos.rec.y;
    double nz = Pos.rec.z;
    if (i == 0) {
      int dir = DRAW_JOG_INVERT_X ? -sign : sign;
      nx += (double)dir * step;
    } else if (i == 1) {
      int dir = DRAW_JOG_INVERT_Y ? -sign : sign;
      ny += (double)dir * step;
    } else {
      /* A3 前推 = 往下压（铅笔压向纸面） */
      int dir = DRAW_JOG_INVERT_Z ? sign : -sign;
      nz += (double)dir * step;
    }

    /* 示教点动用"夹取"版：一次点动最多走 DRAW_JOG_MAX_DPS × 本次步进间隔，
     * 超出部分就少走一点（手动操作允许笔尖略偏离请求点），绝不原地卡住。 */
    if (!applyPointStep(nx, ny, nz, DRAW_JOG_MAX_DPS, (double)interval / 1000.0, MOVE_XYZ_JOG)) {
#if DRAW_DEBUG_SERIAL
      Serial.println(F("[draw] 点动被挡：该方向不可达或超出工作空间"));
#endif
    }
  }
}

/* 记录当前笔尖位置为一个示教点 */
static int teachRecord(void)
{
  if (s_ptCount >= DRAW_TEACH_MAX_POINTS) return PROTO_RES_BUSY;

  s_pts[s_ptCount][0] = Pos.rec.x;
  s_pts[s_ptCount][1] = Pos.rec.y;
  s_pts[s_ptCount][2] = Pos.rec.z;
  s_ptCount++;

#if DRAW_DEBUG_SERIAL
  Serial.print(F("[draw] 记录示教点 "));
  Serial.print((int)s_ptCount);
  Serial.print(F("/"));
  Serial.print(DRAW_TEACH_MAX_POINTS);
  Serial.print(F("  x="));
  Serial.print(s_pts[s_ptCount - 1][0], 2);
  Serial.print(F(" y="));
  Serial.print(s_pts[s_ptCount - 1][1], 2);
  Serial.print(F(" z="));
  Serial.println(s_pts[s_ptCount - 1][2], 2);
#endif

  if (s_ptCount >= DRAW_TEACH_MAX_POINTS) {
    s_teachWaitMs = millis();
    s_phase = DRAW_PHASE_TEACH_WAIT;
  }
  return PROTO_RES_DRAW_TEACH_POINT;
}

/* 示教结束：从已记录的点生成轨迹并开始绘制 */
static int teachFinish(void)
{
  if (s_ptCount < 2) return PROTO_RES_DRAW_REJECTED;

  s_curved = (s_task == DRAW_TASK_CURVE);
  measurePath();
  if (!validatePath()) {
#if DRAW_DEBUG_SERIAL
    Serial.println(F("[draw] 轨迹校验失败，拒绝开始绘制"));
#endif
    return PROTO_RES_DRAW_REJECTED;
  }
  beginSequence();
  return PROTO_RES_DRAW_STARTED;
}

/* ==================== 对外接口 ==================== */

void drawSetup(void)
{
  s_task   = DRAW_TASK_LINE;   /* 本分支默认任务 = 直线 */
  s_phase  = DRAW_PHASE_IDLE;
  s_paused = false;
  s_lastMs = millis();
  s_applyMs = s_lastMs;
  s_lastRes = PROTO_RES_NONE;
  s_ptCount = 0;
  s_curved  = false;
  s_isTeach = false;

#if DRAW_DEBUG_SERIAL
  Serial.println(F("[draw] 绘图模块就绪（铅笔固定方案）："));
  Serial.print(F("[draw]   当前任务 "));
  Serial.print(DRAW_TASK_NAME[s_task]);
  Serial.print(F("，纸面 z="));
  Serial.print(s_penZ, 2);
  Serial.print(F("，中心 ("));
  Serial.print(s_centerX, 2);
  Serial.print(F(","));
  Serial.print(s_centerY, 2);
  Serial.print(F(")，半宽 "));
  Serial.println(s_half, 2);
  Serial.println(F("[draw]   按键: 示教中 1=记录 2=撤销 3=取消 4=开始；绘制中 1=暂停 2=继续 3=取消"));
  Serial.println(F("[draw]   串口: F 换任务  D 开始  G 记录  E 撤销  Q 暂停  U 继续  W 取消"));
  Serial.println(F("[draw]   标定: p<纸面高>  n<半宽>  o<中心x>,<中心y>"));
#endif
}

int drawSelectTask(int task)
{
  if (task < 0 || task >= DRAW_TASK_COUNT) return -1;
  if (s_phase != DRAW_PHASE_IDLE) return -1;      /* 忙的时候不许换任务 */
  s_task = (int8_t)task;
  return s_task;
}

int drawTaskCycle(void)
{
  if (s_phase != DRAW_PHASE_IDLE) return -1;
  s_task = (int8_t)((s_task + 1) % DRAW_TASK_COUNT);
  return s_task;
}

int drawGetTask(void) { return s_task; }

const char *drawTaskName(int task)
{
  if (task < 0 || task >= DRAW_TASK_COUNT) return "未知";
  return DRAW_TASK_NAME[task];
}

/* ASCII 短标签，供串口"当前图形模式"提示使用（终端/ESP8266 按字节比较，不能用中文） */
const char *drawTaskTag(int task)
{
  if (task < 0 || task >= DRAW_TASK_COUNT) return "?";
  return DRAW_TASK_TAG[task];
}

int drawStartTask(void)
{
  s_lastRes = PROTO_RES_BUSY;

  if (s_phase != DRAW_PHASE_IDLE) {
#if DRAW_DEBUG_SERIAL
    Serial.println(F("[draw] 忙：先取消当前绘图任务再开始新的"));
#endif
    return PROTO_RES_BUSY;
  }
  if (pickPlaceIsBusy() || buttonControlBusy()) {
#if DRAW_DEBUG_SERIAL
    Serial.println(F("[draw] 忙：取放序列或按键录制/播放/回中正在进行"));
#endif
    return PROTO_RES_BUSY;
  }

  s_paused  = false;
  s_isTeach = (s_task == DRAW_TASK_POLYLINE || s_task == DRAW_TASK_CURVE);

  if (s_isTeach) {
    s_ptCount = 0;
    s_curved  = (s_task == DRAW_TASK_CURVE);
    beginHome(AFTER_HOME_TEACH, DRAW_PHASE_HOME);      /* 先回待机，回完进入示教 */
#if DRAW_DEBUG_SERIAL
    Serial.print(F("[draw] 进入示教（"));
    Serial.print(DRAW_TASK_NAME[s_task]);
    Serial.print(F("），用摇杆把笔尖移到目标点，按键1 记录，共 "));
    Serial.print(DRAW_TEACH_MAX_POINTS);
    Serial.println(F(" 个点"));
#endif
    s_lastRes = PROTO_RES_DRAW_STARTED;
    return PROTO_RES_DRAW_STARTED;
  }

  buildShapePath(s_task);
  measurePath();
  if (!validatePath()) {
#if DRAW_DEBUG_SERIAL
    Serial.println(F("[draw] 轨迹校验失败，拒绝开始绘制"));
#endif
    s_lastRes = PROTO_RES_DRAW_REJECTED;
    return PROTO_RES_DRAW_REJECTED;
  }
  beginSequence();
#if DRAW_DEBUG_SERIAL
  Serial.print(F("[draw] 开始绘制 "));
  Serial.print(DRAW_TASK_NAME[s_task]);
  Serial.print(F("：总弧长 "));
  Serial.print(s_totalLen, 2);
  Serial.println(F(" 单位"));
#endif
  s_lastRes = PROTO_RES_DRAW_STARTED;
  return PROTO_RES_DRAW_STARTED;
}

int drawTeachRecord(void)
{
  if (s_phase != DRAW_PHASE_TEACH) {
    s_lastRes = PROTO_RES_BUSY;
    return PROTO_RES_BUSY;
  }
  s_lastRes = teachRecord();
  return s_lastRes;
}

int drawTeachUndo(void)
{
  if (s_phase != DRAW_PHASE_TEACH || s_ptCount <= 0) {
    s_lastRes = PROTO_RES_DRAW_REJECTED;
    return PROTO_RES_DRAW_REJECTED;
  }
  s_ptCount--;
#if DRAW_DEBUG_SERIAL
  Serial.print(F("[draw] 撤销一个示教点，剩 "));
  Serial.println((int)s_ptCount);
#endif
  s_lastRes = PROTO_RES_DRAW_TEACH_UNDO;
  return PROTO_RES_DRAW_TEACH_UNDO;
}

int drawTeachFinish(void)
{
  if (s_phase != DRAW_PHASE_TEACH && s_phase != DRAW_PHASE_TEACH_WAIT) {
    s_lastRes = PROTO_RES_BUSY;
    return PROTO_RES_BUSY;
  }
  s_lastRes = teachFinish();
  return s_lastRes;
}

int drawTeachCount(void) { return s_ptCount; }

int drawPause(void)
{
  if (s_phase == DRAW_PHASE_IDLE || s_phase == DRAW_PHASE_TEACH ||
      s_phase == DRAW_PHASE_TEACH_WAIT) {
    s_lastRes = PROTO_RES_BUSY;
    return PROTO_RES_BUSY;
  }
  s_paused = true;
#if DRAW_DEBUG_SERIAL
  Serial.println(F("[draw] 已暂停（按键2 继续，按键3 取消）"));
#endif
  s_lastRes = PROTO_RES_DRAW_PAUSED;
  return PROTO_RES_DRAW_PAUSED;
}

int drawResume(void)
{
  if (!s_paused) {
    s_lastRes = PROTO_RES_BUSY;
    return PROTO_RES_BUSY;
  }
  s_paused = false;
  s_v      = 0.0;      /* 从静止重新起步，避免继续时猛冲 */
  s_mV     = 0.0;
#if DRAW_DEBUG_SERIAL
  Serial.println(F("[draw] 继续绘制"));
#endif
  s_lastRes = PROTO_RES_DRAW_RESUMED;
  return PROTO_RES_DRAW_RESUMED;
}

int drawCancel(void)
{
  if (s_phase == DRAW_PHASE_IDLE) {
    s_lastRes = PROTO_RES_BUSY;
    return PROTO_RES_BUSY;
  }
  s_paused = false;

  if (s_phase == DRAW_PHASE_TEACH || s_phase == DRAW_PHASE_TEACH_WAIT) {
    s_ptCount = 0;
    beginHome(AFTER_HOME_IDLE, DRAW_PHASE_HOME);       /* 示教取消：直接回待机 */
  } else {
    /* 绘制中取消：先垂直抬笔（不然笔尖会拖着纸走出一道多余的线），再回待机 */
    beginLiftToReturn();
  }
#if DRAW_DEBUG_SERIAL
  Serial.println(F("[draw] 已取消：抬笔后回待机"));
#endif
  s_lastRes = PROTO_RES_DRAW_CANCELED;
  return PROTO_RES_DRAW_CANCELED;
}

/* 标定接口 */
bool drawSetPaperZ(double z)
{
  if (z != z) return false;                        /* NaN */
  if (z < DRAW_PEN_Z_MIN || z > DRAW_PEN_Z_MAX) return false;
  s_penZ = z;
  return true;
}
double drawGetPaperZ(void) { return s_penZ; }

bool drawSetHalfSize(double half)
{
  if (half != half) return false;
  if (half < DRAW_HALF_MIN || half > DRAW_HALF_MAX) return false;
  s_half = half;
  return true;
}
double drawGetHalfSize(void) { return s_half; }

bool drawSetCenter(double x, double y)
{
  if (x != x || y != y) return false;
  if (fabs(x) > DRAW_CENTER_LIMIT || fabs(y) > DRAW_CENTER_LIMIT) return false;
  s_centerX = x;
  s_centerY = y;
  return true;
}
double drawGetCenterX(void) { return s_centerX; }
double drawGetCenterY(void) { return s_centerY; }

/* 按键转交入口 */
int drawHandleButton(int key)
{
  switch (key) {
    case DRAW_KEY_1:
      if (s_phase == DRAW_PHASE_TEACH)  return drawTeachRecord();
      if (s_phase == DRAW_PHASE_TEACH_WAIT) return PROTO_RES_BUSY;
      return drawPause();
    case DRAW_KEY_2:
      if (s_phase == DRAW_PHASE_TEACH)  return drawTeachUndo();
      if (s_phase == DRAW_PHASE_TEACH_WAIT) return PROTO_RES_BUSY;
      return drawResume();
    case DRAW_KEY_3:
      return drawCancel();
    case DRAW_KEY_4:
      if (s_phase == DRAW_PHASE_TEACH || s_phase == DRAW_PHASE_TEACH_WAIT) {
        return drawTeachFinish();
      }
#if DRAW_DEBUG_SERIAL
      Serial.println(F("[draw] 绘制中按键4 无动作（按键1 暂停 / 2 继续 / 3 取消）"));
#endif
      return PROTO_RES_BUSY;
    default:
      return PROTO_RES_UNKNOWN;
  }
}

bool drawAcceptButton(int key)
{
  (void) key;
  return (s_phase != DRAW_PHASE_IDLE);
}

/* 串口命令 */
bool drawIsCommandChar(char c)
{
  return (c == 'F' || c == 'D' || c == 'G' || c == 'E' ||
          c == 'Q' || c == 'U' || c == 'W' ||
          c == 'p' || c == 'n' || c == 'o');
}

int drawHandleCommand(char c)
{
  switch (c) {
    case 'F': {
      int t = drawTaskCycle();
      if (t < 0) {
        s_lastRes = PROTO_RES_BUSY;
        return PROTO_RES_BUSY;
      }
#if DRAW_DEBUG_SERIAL
      Serial.print(F("[draw] 当前任务 -> "));
      Serial.println(DRAW_TASK_NAME[t]);
#endif
      s_lastRes = PROTO_RES_DRAW_TASK_SELECTED;
      return PROTO_RES_DRAW_TASK_SELECTED;
    }
    case 'D':
      return drawStartTask();
    case 'G':
      return drawTeachRecord();
    case 'E':
      return drawTeachUndo();
    case 'Q':
      return drawPause();
    case 'U':
      return drawResume();
    case 'W':
      return drawCancel();
    default:
      return PROTO_RES_UNKNOWN;
  }
}

/* 状态查询 */
bool drawControlBusy(void)  { return (s_phase != DRAW_PHASE_IDLE); }
bool drawControlLocked(void) { return (s_phase != DRAW_PHASE_IDLE); }
int  drawGetPhase(void)     { return s_phase; }
bool drawIsPaused(void)     { return s_paused; }

const char *drawPhaseName(void)
{
  if (s_phase < 0 || s_phase > DRAW_PHASE_RETURN) return "未知";
  return DRAW_PHASE_NAME[s_phase];
}

const char *drawStateName(void)
{
  if (s_phase == DRAW_PHASE_IDLE) return "空闲";
  if (s_phase == DRAW_PHASE_TEACH) return "示教中";
  if (s_phase == DRAW_PHASE_TEACH_WAIT) return "准备绘制";
  if (s_paused) return "已暂停";
  return DRAW_PHASE_NAME[s_phase];
}

unsigned long drawLastRunMs(void) { return s_lastRunMs; }
int drawLastPointsHit(void)   { return s_hitCount; }
int drawLastPointsTotal(void) { return s_hitTotal; }
int drawLastResult(void)      { return s_lastRes; }

/* ==================== 主循环 ==================== */

void drawLoop(void)
{
  unsigned long now = millis();
  unsigned long dtMs = now - s_lastMs;
  if (dtMs == 0UL) return;                 /* 同一毫秒内重复调用不推进 */
  s_lastMs = now;
  if (dtMs > DRAW_TICK_MAX_MS) dtMs = DRAW_TICK_MAX_MS;
  s_dtSec = (double)dtMs / 1000.0;

  if (s_phase == DRAW_PHASE_IDLE) return;

  /* 示教：摇杆点动（暂停概念在示教里没有意义，按键1/2 是记录/撤销） */
  if (s_phase == DRAW_PHASE_TEACH) {
    teachJogTick();
    return;
  }

  if (s_phase == DRAW_PHASE_TEACH_WAIT) {
    if (now - s_teachWaitMs >= DRAW_TEACH_START_DELAY_MS) {
      s_lastRes = teachFinish();
      if (s_lastRes != PROTO_RES_DRAW_STARTED) {
        s_phase = DRAW_PHASE_IDLE;       /* 校验失败：退回空闲，不硬来 */
      }
    }
    return;
  }

  if (s_paused) return;                    /* 暂停：保持任务状态，什么都不推进 */

  switch (s_phase) {
    case DRAW_PHASE_HOME: {
      if (homeTick()) {
        if (s_afterHome == AFTER_HOME_TEACH) {
          s_phase = DRAW_PHASE_TEACH;      /* 示教：停在待机位等人用摇杆点动 */
        } else if (s_afterHome == AFTER_HOME_IDLE) {
          s_phase = DRAW_PHASE_IDLE;
        } else {
          /* 抬笔移动到起点正上方 */
          double lz = liftZFor(s_pts[0]);
          beginMove(DRAW_PHASE_PLUNGE, s_pts[0][0], s_pts[0][1], lz);
          s_phase = DRAW_PHASE_TRAVEL;
        }
      }
      break;
    }

    case DRAW_PHASE_TRAVEL:
    case DRAW_PHASE_PLUNGE:
    case DRAW_PHASE_LIFT: {
      double px = Pos.rec.x, py = Pos.rec.y, pz = Pos.rec.z;
      bool arrived = moveTick();
      double moved = fabs(Pos.rec.x - px) + fabs(Pos.rec.y - py) + fabs(Pos.rec.z - pz);
      if (!arrived) {
        (void) stallGuard(moved);
        break;
      }
      if (s_phase == DRAW_PHASE_TRAVEL) {
        /* 抬笔到位 -> 落笔 */
        beginMove(DRAW_PHASE_PATH, s_pts[0][0], s_pts[0][1], s_pts[0][2]);
        s_phase = DRAW_PHASE_PLUNGE;
      } else if (s_phase == DRAW_PHASE_PLUNGE) {
        s_done = 0.0;
        s_v    = 0.0;
        s_span = 0;
        s_u    = 0.0;
        s_stallCount = 0;
        s_pathStartMs = millis();
        s_phase = DRAW_PHASE_PATH;
      } else {
        /* 抬笔结束：要么回待机（绘制完成/取消），要么继续走 s_nextAfterMove */
        if (s_nextAfterMove == DRAW_PHASE_RETURN) {
          beginHome(AFTER_HOME_IDLE, DRAW_PHASE_RETURN);
        } else {
          s_phase = s_nextAfterMove;
        }
      }
      break;
    }

    case DRAW_PHASE_PATH: {
      double doneBefore = s_done;
      pathTick();
      if (stallGuard(s_done - doneBefore)) break;
      if (s_done >= s_totalLen - 1e-6) {
        s_lastRunMs = millis() - s_pathStartMs;
#if DRAW_DEBUG_SERIAL
        Serial.print(F("[draw] 绘制完成：用时 "));
        Serial.print(s_lastRunMs);
        Serial.print(F(" ms，笔尖停在 x="));
        Serial.print(Pos.rec.x, 2);
        Serial.print(F(" y="));
        Serial.print(Pos.rec.y, 2);
        Serial.print(F(" z="));
        Serial.println(Pos.rec.z, 2);
        if (s_isTeach) {
          Serial.print(F("[draw] 过点 "));
          Serial.print((int)s_hitCount);
          Serial.print(F("/"));
          Serial.println((int)s_hitTotal);
        }
#endif
        /* 画完抬笔再回待机 */
        beginLiftToReturn();
      }
      break;
    }

    case DRAW_PHASE_RETURN: {
      if (homeTick()) {
#if DRAW_DEBUG_SERIAL
        Serial.println(F("[draw] 已回到待机位，绘图任务结束"));
#endif
        s_phase = DRAW_PHASE_IDLE;
      }
      break;
    }

    default:
      s_phase = DRAW_PHASE_IDLE;
      break;
  }
}

#endif
