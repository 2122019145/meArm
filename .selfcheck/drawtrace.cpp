/* drawtrace.cpp -- 把绘图的"笔尖实际走过的轨迹"打出来，用来肉眼确认画的是什么图形
 *
 * 为什么需要它：用户报"切到三角模式后画的仍是直线"。drawcheck.cpp 只报
 * "启动成功 / 过点 n/n"，看不出轨迹形状；这个工具直接采样落笔阶段
 * （DRAW_PHASE_PATH）的 Pos.rec(x,y)，给出包围盒、转角数、简化折线。
 *   直线  -> 包围盒宽度 = 0，转角 0
 *   字母N/三角形/字母Z -> 转角 2~3，包围盒 x/y 都有宽度
 *   字母V -> 转角 1
 *
 * 编译（默认组合 = 取放 + 绘图、按键关，与 weArm_config.h 现默认一致）：
 *   g++ -std=gnu++17 -O2 -I. -I.\.selfcheck\mock `
 *       -DWEARM_ENABLE_PICK_PLACE=1 -DWEARM_ENABLE_DRAW=1 -DWEARM_ENABLE_BUTTONS=0 `
 *       .\.selfcheck\drawtrace.cpp .\draw_control.cpp .\constant_and_positions.cpp `
 *       .\move.cpp .\servo_drive.cpp .\button_control.cpp .\joystick_control.cpp `
 *       .\pick_place.cpp .\protocol_constants.cpp .\serial_protocol.cpp `
 *       .\.selfcheck\mock\Arduino.cpp -o .\.selfcheck\out\drawtrace.exe
 *
 * 注意 v1.6.8 起的**新依赖**：`draw_control.cpp` 的"卡死保护"放弃时会调
 * `serialReportDrawStop()`（在 `serial_protocol.cpp` 里），所以绘图类工具必须多链
 * `serial_protocol.cpp` + `protocol_constants.cpp`；现在默认组合还带取放，
 * `pickPlaceIsBusy()` 也要 `pick_place.cpp`。少链任何一个都会 undefined reference。
 *
 * 运行：.\.selfcheck\out\drawtrace.exe        （5 个内置图形全跑）
 *       .\.selfcheck\out\drawtrace.exe tri    （只跑三角形）
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <Arduino.h>
#include "constant_and_positions.h"
#include "path_core.h"
#include "draw_control.h"

extern unsigned long g_mockMillis;

/* 模拟节拍（ms/拍）。真机上 drawLoop() 是"每圈跑一次、dtMs = 距上次的毫秒数"，
 * 所以节拍越小越接近真机的快速主循环。命令行第 2 个参数可以改它：
 *   drawtrace tri 1     —— 1 ms/拍（真机量级）
 *   drawtrace tri 10    —— 10 ms/拍（默认，早期结论都用这个） */
static unsigned long s_tickMs = 10UL;

#define MAXP 4000
static double tx[MAXP], ty[MAXP];
static int    np;

static void drain(void)
{
  drawCancel();
  while (drawControlBusy()) { drawLoop(); g_mockMillis += s_tickMs; }
}

static void traceTask(int task, const char *label, int budgetMs)
{
  drain();
  int sel = drawSelectTask(task);
  int rc  = drawStartTask();
  printf("\n### %s (task=%d) start=%d", label, sel, rc);
  if (rc != PROTO_RES_DRAW_STARTED) { printf("   <<< 未启动\n"); return; }

  double penZ = drawGetPaperZ();
  np = 0;
  int ms = 0;
  while (drawControlBusy() && ms < budgetMs) {
    drawLoop();
    g_mockMillis += s_tickMs;
    ms += (int)s_tickMs;
    if (drawGetPhase() == DRAW_PHASE_PATH &&
        fabs(Pos.rec.z - penZ) < 1.5 && np < MAXP) {
      tx[np] = Pos.rec.x; ty[np] = Pos.rec.y; np++;
    }
  }
  printf("  用时 %d ms  res=%d hit=%d/%d 阶段=%s 落笔采样 %d 点\n",
         ms, drawLastResult(), drawLastPointsHit(), drawLastPointsTotal(),
         drawStateName(), np);
  if (drawControlBusy()) {
    printf("  <<< 预算 %d ms 用完仍未跑完（阶段=%s）\n", budgetMs, drawStateName());
  } else if (drawLastResult() != PROTO_RES_DRAW_STARTED) {
    printf("  <<< 异常结束：res=%d（%d = stallGuard 放弃本任务）\n",
           drawLastResult(), PROTO_RES_DRAW_REJECTED);
  }
  if (np < 2) { printf("  (落笔段没采到样)\n"); return; }

  double x0 = tx[0], x1 = tx[0], y0 = ty[0], y1 = ty[0];
  for (int i = 1; i < np; i++) {
    if (tx[i] < x0) x0 = tx[i];
    if (tx[i] > x1) x1 = tx[i];
    if (ty[i] < y0) y0 = ty[i];
    if (ty[i] > y1) y1 = ty[i];
  }
  printf("  包围盒 x[%.1f..%.1f] 宽 %.1f   y[%.1f..%.1f] 高 %.1f\n",
         x0, x1, x1 - x0, y0, y1, y1 - y0);

  double sx[64], sy[64];
  int ns = 0;
  sx[ns] = tx[0]; sy[ns] = ty[0]; ns++;
  for (int i = 1; i < np && ns < 63; i++) {
    double d = hypot(tx[i] - sx[ns - 1], ty[i] - sy[ns - 1]);
    if (d >= 3.0) { sx[ns] = tx[i]; sy[ns] = ty[i]; ns++; }
  }
  if (ns < 63 && hypot(tx[np - 1] - sx[ns - 1], ty[np - 1] - sy[ns - 1]) > 0.2) {
    sx[ns] = tx[np - 1]; sy[ns] = ty[np - 1]; ns++;
  }

  int corners = 0;
  for (int i = 1; i + 1 < ns; i++) {
    double ax = sx[i] - sx[i - 1], ay = sy[i] - sy[i - 1];
    double bx = sx[i + 1] - sx[i], by = sy[i + 1] - sy[i];
    double la = hypot(ax, ay), lb = hypot(bx, by);
    if (la < 1e-6 || lb < 1e-6) continue;
    if ((ax * bx + ay * by) / (la * lb) < 0.866) corners++;   /* 转角 > 30° */
  }
  printf("  简化折线 %d 点、转角(>30°) %d 个： ", ns, corners);
  int step = (ns > 12) ? (ns / 12) : 1;
  for (int i = 0; i < ns; i += step) printf("(%.1f,%.1f)", sx[i], sy[i]);
  if ((ns - 1) % step) printf("(%.1f,%.1f)", sx[ns - 1], sy[ns - 1]);
  printf("\n");
}

int main(int argc, char **argv)
{
  if (argc > 2) {
    unsigned long v = strtoul(argv[2], NULL, 10);
    s_tickMs = (v < 1UL) ? 1UL : v;
  }
  g_mockMillis = 1000;
  posInit();
  drawSetup();

  printf("模拟节拍 %lu ms/拍\n", s_tickMs);
  printf("纸面 z=%.1f  中心(%.1f,%.1f)  半宽=%.1f\n",
         drawGetPaperZ(), drawGetCenterX(), drawGetCenterY(), drawGetHalfSize());

  drawSelectTask(DRAW_TASK_LINE);
  printf("F 键循环顺序（从默认 %s 起，每次 F 之后的当前任务）：", drawTaskTag(drawGetTask()));
  for (int i = 0; i < 8; i++) {
    drawHandleCommand('F');
    printf(" %s", drawTaskTag(drawGetTask()));
  }
  printf("\n");
  drawSelectTask(DRAW_TASK_LINE);

  const char *names[5] = { "直线", "字母N", "三角形", "字母Z", "字母V" };
  if (argc > 1 && strcmp(argv[1], "tri") == 0) {
    traceTask(DRAW_TASK_TRIANGLE, "三角形", 120000);
    return 0;
  }
  for (int t = 0; t <= DRAW_TASK_V; t++) traceTask(t, names[t], 120000);
  return 0;
}
