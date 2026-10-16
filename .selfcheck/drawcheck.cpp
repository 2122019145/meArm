/* drawcheck.cpp -- 在 PC 上用固件本体试跑绘图模块（draw_control.cpp）
 *
 * 为什么需要它：探针（probe_draw.cpp）已按用户要求停用，而运动逻辑换成
 * others/f/MeArm 最新版模型后，绘图模块的"纸面高度 / 绘图区中心 / 半宽"这三个
 * 默认值都是按旧模型（1 单位 = 4mm、可达 40mm 臂展）写死的，必须实测：
 *   (1) 新模型下每个 z 平面沿 +y 的可达 rho 区间（选绘图区用）；
 *   (2) 7 个内置/示教任务在**当前代码默认值**下能不能启动、能不能画完；
 *   (3) 一组候选 (纸面 z, 中心 y, 半宽) 里哪些能通过 validatePath()。
 *
 * 编译（链接固件本体，不碰 avr）：
 *   g++ -std=gnu++17 -O2 -I. -I.\.selfcheck\mock `
 *       -DWEARM_ENABLE_DRAW=1 -DWEARM_ENABLE_PICK_PLACE=0 -DWEARM_ENABLE_BUTTONS=1 `
 *       .\.selfcheck\drawcheck.cpp .\draw_control.cpp .\constant_and_positions.cpp `
 *       .\move.cpp .\servo_drive.cpp .\button_control.cpp .\joystick_control.cpp `
 *       .\.selfcheck\mock\Arduino.cpp -o .\.selfcheck\out\drawcheck.exe
 *
 * 运行：.\.selfcheck\out\drawcheck.exe           （默认做 (1)(2)(3) 全部）
 *       .\.selfcheck\out\drawcheck.exe scan      （只做 (1)）
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <Arduino.h>
#include "constant_and_positions.h"
#include "path_core.h"
#include "draw_control.h"

extern unsigned long g_mockMillis;

/* ---------- (1) 每个 z 平面沿 +y 的可达 rho 区间 ---------- */
static void scanZ(double z)
{
    bool in = false, any = false;
    double start = 0.0;
    printf("  z=%7.2f  rho:", z);
    for (int rho = 0; rho <= 168; rho++) {
        double b = 0, r = 0, c = 0;
        bool ok = pathCorePointOk(0.0, (double) rho, z, &b, &r, &c);
        if (ok && !in) { in = true; start = rho; }
        if (!ok && in) { in = false; printf(" [%g..%d]", start, rho - 1); any = true; }
    }
    if (in) { printf(" [%g..168]", start); any = true; }
    if (!any) printf(" (none)");
    printf("\n");
}

/* ---------- 跑一个任务到结束 ---------- */
static void runTask(int task, const char *label, int budgetMs)
{
    drawCancel();
    while (drawControlBusy()) { drawLoop(); g_mockMillis += 10UL; }   /* 排空取消 */

    int sel = (task >= 0) ? drawSelectTask(task) : drawGetTask();
    int rc  = drawStartTask();
    printf("  %-12s (task %d) start=%d %-8s", label, sel, rc,
           (rc == PROTO_RES_DRAW_STARTED) ? "STARTED" : "REJECTED");
    if (rc != PROTO_RES_DRAW_STARTED) { printf("\n"); return; }

    int ms = 0;
    while (drawControlBusy() && ms < budgetMs) {
        drawLoop();
        g_mockMillis += 10UL;
        ms += 10;
    }
    bool timeout = drawControlBusy();
    if (timeout) drawCancel();
    while (drawControlBusy()) { drawLoop(); g_mockMillis += 10UL; }

    printf("  %6d ms  res=%-3d hit=%d/%d  %s  rec(%.1f,%.1f,%.1f)%s\n",
           ms, drawLastResult(), drawLastPointsHit(), drawLastPointsTotal(),
           drawStateName(), Pos.rec.x, Pos.rec.y, Pos.rec.z,
           timeout ? "  <<< TIMEOUT" : "");
}

/* 5 个内置图形能不能全部启动（真正走 validatePath） */
static int tryStartAll(double penZ, double cx, double cy, double half, char *detail)
{
    int ok = 0;
    detail[0] = 0;
    for (int t = 0; t <= DRAW_TASK_V; t++) {
        drawCancel();
        while (drawControlBusy()) { drawLoop(); g_mockMillis += 10UL; }
        drawSetPaperZ(penZ);
        drawSetCenter(cx, cy);
        drawSetHalfSize(half);
        drawSelectTask(t);
        int rc = drawStartTask();
        bool good = (rc == PROTO_RES_DRAW_STARTED);
        char c = good ? '#' : '.';
        int n = (int) strlen(detail);
        detail[n] = c; detail[n + 1] = 0;
        if (good) {
            ok++;
            drawCancel();
            while (drawControlBusy()) { drawLoop(); g_mockMillis += 10UL; }
        }
    }
    return ok;
}

int main(int argc, char **argv)
{
    bool onlyScan = (argc > 1);

    posInit();
    drawSetup();

    /* drawcheck one <pz> <cy> <half>：单个配置跑一遍直线并打日志（配
     * -DWEARM_DEBUG_SERIAL=1 -DWEARM_MOCK_SERIAL=1 看 validatePath 卡在哪一步） */
    if (argc > 4 && strcmp(argv[1], "one") == 0) {
        double pz = atof(argv[2]), cy = atof(argv[3]), h = atof(argv[4]);
        printf("=== one: pz=%.1f cy=%.1f half=%.1f ===\n", pz, cy, h);
        drawSetPaperZ(pz); drawSetCenter(0.0, cy); drawSetHalfSize(h);
        runTask(DRAW_TASK_LINE, "直线", 120000);
        return 0;
    }

    printf("=== 0) 初始姿态 ===\n");
    printf("  home rec(%.2f,%.2f,%.2f)  ser(%.2f,%.2f,%.2f)\n",
           Pos.rec.x, Pos.rec.y, Pos.rec.z,
           Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);
    printf("  纸面 z=%.1f  中心(%.1f,%.1f)  半宽 %.1f\n",
           drawGetPaperZ(), drawGetCenterX(), drawGetCenterY(),
           drawGetHalfSize());

    printf("\n=== 1) 各 z 平面的可达 rho（沿 +y） ===\n");
    const double zs[] = { -20.0, -18.0, -16.0, -15.0, -14.0, -13.0, -12.0, -11.0,
                          -10.0, -8.0, -6.0, -4.0, -2.0, 0.0, 20.0, 38.0, 60.0, 80.0, 118.0 };
    for (unsigned i = 0; i < sizeof(zs) / sizeof(zs[0]); i++) scanZ(zs[i]);

    if (onlyScan) return 0;

    printf("\n=== 2) 当前代码默认值下的 7 个任务 ===\n");
    for (int t = 0; t < DRAW_TASK_COUNT; t++) {
        /* 示教任务(5/6)没有示教点，启动会被拒 —— 只报告，不算失败 */
        runTask(t, drawTaskName(t), 120000);
    }

    printf("\n=== 3) 候选配置扫描：5 个内置图形各能否启动（# = 通过 . = 被拒） ===\n");
    const double penzs[] = { -14.0, -12.0, -10.0, -8.0, -6.0, -4.0, -2.0, 0.0, 10.0 };
    const double cys[]   = { 60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 100.0, 110.0, 118.0 };
    const double halfs[] = { 10.0, 15.0, 20.0, 25.0, 30.0, 35.0 };
    for (unsigned i = 0; i < sizeof(penzs) / sizeof(penzs[0]); i++) {
        for (unsigned j = 0; j < sizeof(cys) / sizeof(cys[0]); j++) {
            printf("  pz=%6.1f cy=%5.1f :", penzs[i], cys[j]);
            for (unsigned k = 0; k < sizeof(halfs) / sizeof(halfs[0]); k++) {
                char d[8];
                int ok = tryStartAll(penzs[i], 0.0, cys[j], halfs[k], d);
                printf("  h%-4.0f %s", halfs[k], ok == 5 ? "OK  " : d);
            }
            printf("\n");
        }
    }
    return 0;
}
