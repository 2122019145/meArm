/* drawsweep.cpp -- 找出"三角形画完第一条线后卡住"的配置与卡住位置
 *
 * 背景（用户报告）：三角绘图时画完第一条线后会卡住。
 * drawcheck.cpp 的默认配置（纸面 z=-6、中心 (0,75)、半宽 25）能画完三角形，
 * 所以卡住一定和 **纸面高度 / 绘图中心 / 半宽** 的组合有关（也和时间步长有关：
 * drawcheck 用 10ms 一拍，真机 drawLoop 是几毫秒一拍，这里改用 2ms 更接近真机）。
 *
 * 本工具对每个 (paperZ, centerY, half) 组合：
 *   1) 选中三角形任务并启动（走真正的 validatePath）；
 *   2) 用 2ms 一拍的 mock 时钟把 drawLoop 推到空闲或超预算；
 *   3) 只有"启动成功但没画完"的组合才详细打印 —— 那就是卡住的候选，
 *      并给出停下时的笔尖位置、过点计数、阶段名、一次移动量（0 = 完全没动）。
 *
 * 编译（与 drawcheck 相同的链接方式；v1.6.8 起绘图类工具还需串口层，见下）：
 *   g++ -std=gnu++17 -O2 -I. -I.\.selfcheck\mock `
 *       -DWEARM_ENABLE_DRAW=1 -DWEARM_ENABLE_PICK_PLACE=0 -DWEARM_ENABLE_BUTTONS=1 `
 *       .\.selfcheck\drawsweep.cpp .\draw_control.cpp .\constant_and_positions.cpp `
 *       .\move.cpp .\servo_drive.cpp .\button_control.cpp .\joystick_control.cpp `
 *       .\protocol_constants.cpp .\serial_protocol.cpp `
 *       .\.selfcheck\mock\Arduino.cpp -o .\.selfcheck\out\drawsweep.exe
 *
 * 注意：`draw_control.cpp` 的"卡死保护"放弃时会调 `serialReportDrawStop()`
 * （实现在 `serial_protocol.cpp`），所以必须多链 `serial_protocol.cpp` +
 * `protocol_constants.cpp`，否则 undefined reference。
 *
 * 运行：.\.selfcheck\out\drawsweep.exe            （三角形 + 5 个内置图形全扫）
 *       .\.selfcheck\out\drawsweep.exe tri        （只扫三角形）
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <Arduino.h>
#include "constant_and_positions.h"
#include "path_core.h"
#include "draw_control.h"

extern unsigned long g_mockMillis;

#define TICK_MS 2UL          /* 一拍推进多少 mock 毫秒（真机 drawLoop 约 1~5ms 一次） */
#define BUDGET_MS 20000UL    /* 单个任务的模拟时间预算（够画完所有内置图形） */

/* 跑一个任务：返回 0=画完、2=启动被拒、1=启动成功但卡住/超时 */
static int runOne(int task, double pz, double cy, double half,
                  unsigned long *usedMs, double *stopPos, int *hit, int *tot,
                  const char **phase, int *lastRes, double *jumpMax)
{
    drawCancel();
    while (drawControlBusy()) { drawLoop(); g_mockMillis += TICK_MS; }

    drawSetPaperZ(pz);
    drawSetCenter(0.0, cy);
    drawSetHalfSize(half);
    drawSelectTask(task);
    if (drawStartTask() != PROTO_RES_DRAW_STARTED) return 2;

    unsigned long ms = 0;
    double prev[3] = { Pos.rec.x, Pos.rec.y, Pos.rec.z };
    double jump = 0.0;          /* 单拍最大位移（判断"完全没动"） */
    int    still = 0;           /* 连续没动的拍数 */
    int    stillMax = 0;
    while (drawControlBusy() && ms < BUDGET_MS) {
        drawLoop();
        g_mockMillis += TICK_MS;
        ms += TICK_MS;
        double d = fabs(Pos.rec.x - prev[0]) + fabs(Pos.rec.y - prev[1]) + fabs(Pos.rec.z - prev[2]);
        if (d > jump) jump = d;
        if (d <= 1e-9) { still++; if (still > stillMax) stillMax = still; }
        else still = 0;
        prev[0] = Pos.rec.x; prev[1] = Pos.rec.y; prev[2] = Pos.rec.z;
    }

    *usedMs  = ms;
    *stopPos = Pos.rec.x; stopPos[1] = Pos.rec.y; stopPos[2] = Pos.rec.z;
    *hit     = drawLastPointsHit();
    *tot     = drawLastPointsTotal();
    *phase   = drawStateName();
    *lastRes = drawLastResult();
    *jumpMax = stillMax;

    bool stuck = drawControlBusy();
    if (stuck) { drawCancel(); while (drawControlBusy()) { drawLoop(); g_mockMillis += TICK_MS; } }
    return stuck ? 1 : 0;
}

int main(int argc, char **argv)
{
    bool onlyTri = (argc > 1 && strcmp(argv[1], "tri") == 0);

    posInit();
    drawSetup();

    const double pzs[]   = { -38.0, -34.0, -30.0, -26.0, -22.0, -18.0, -14.0,
                             -10.0, -6.0, -2.0, 2.0, 10.0 };
    const double cys[]   = { 50.0, 60.0, 70.0, 75.0, 80.0, 90.0, 100.0, 110.0, 120.0 };
    const double halfs[] = { 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0 };
    const int    tasks[] = { DRAW_TASK_TRIANGLE };
    int nt = 1;
    int allTasks[5] = { DRAW_TASK_LINE, DRAW_TASK_N, DRAW_TASK_TRIANGLE, DRAW_TASK_Z, DRAW_TASK_V };
    if (!onlyTri) nt = 0;   /* nt=0 表示"每个组合都把 5 个内置图形跑一遍" */

    printf("=== 启动被拒的组合（validatePath 不过）统计 ===\n");
    int rej = 0, stuck = 0, done = 0, tried = 0;
    for (unsigned i = 0; i < sizeof(pzs) / sizeof(pzs[0]); i++) {
        for (unsigned j = 0; j < sizeof(cys) / sizeof(cys[0]); j++) {
            for (unsigned k = 0; k < sizeof(halfs) / sizeof(halfs[0]); k++) {
                for (int ti = 0; ti < (onlyTri ? nt : 5); ti++) {
                    int task = onlyTri ? tasks[ti] : allTasks[ti];
                    unsigned long used; double stop[3], stillMax;
                    int hit, tot, lastRes; const char *phase;
                    tried++;
                    int r = runOne(task, pzs[i], cys[j], halfs[k], &used, stop,
                                   &hit, &tot, &phase, &lastRes, &stillMax);
                    if (r == 2) { rej++; continue; }
                    if (r == 0) { done++; continue; }
                    stuck++;
                    printf("  ★ %-4s pz=%6.1f cy=%5.1f half=%4.1f : 启动成功但没画完"
                           "  %5lu ms  hit=%d/%d  停在(%.1f,%.1f,%.1f)  阶段=%s"
                           "  res=%d  最长静止 %d 拍(%.0f ms)\n",
                           drawTaskTag(task), pzs[i], cys[j], halfs[k], used,
                           hit, tot, stop[0], stop[1], stop[2], phase, lastRes,
                           (int)stillMax, (double)stillMax * (double)TICK_MS);
                }
            }
        }
    }
    printf("=== 合计：试 %d 组，画完 %d，启动被拒 %d，**卡住 %d** ===\n",
           tried, done, rej, stuck);
    return 0;
}
