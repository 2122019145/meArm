/* picktable_run.cpp -- 在 PC 上用固件本体跑一遍取放启动校验。
 *
 * 为什么需要它：`picktable_check.cpp` 只逐点问 getAngleEx()，
 * 而 pickPlaceStart() 还要求
 *   (1) 整条 7 段直线的采样点都能反解且不跳分支（pathCoreSegmentOk）；
 *   (2) planTravelZ() 能在 PICK_LIFT_DZ..PICK_CLEAR_DZ 之间找到一档飞行高度；
 *   (3) 当前位姿的关节角与它的反解属同一分支。
 * 肉眼看点位"能解"并不等于这三个条件成立，所以直接调真函数最省事。
 *
 * 编译（链接固件本体，不碰 avr）：
 *   g++ -std=gnu++17 -O2 -I. -I.\.selfcheck\mock `
 *       .\.selfcheck\picktable_run.cpp .\pick_place.cpp .\constant_and_positions.cpp `
 *       .\move.cpp .\servo_drive.cpp .\.selfcheck\mock\Arduino.cpp `
 *       -o .\.selfcheck\out\picktable_run.exe
 */

#include <stdio.h>
#include <math.h>

#include <Arduino.h>
#include "constant_and_positions.h"
#include "pick_place.h"

extern unsigned long g_mockMillis;

static void drain(int object, int budgetMs)
{
    int t = 0;
    while (pickPlaceIsBusy() && t < budgetMs) {
        pickPlaceLoop();
        g_mockMillis += 10UL;
        t += 10;
    }
    printf("      drain %5d ms -> busy=%d  stage=%s  rec(%.1f,%.1f,%.1f)\n",
           t, (int) pickPlaceIsBusy(), pickPlaceStageName(),
           Pos.rec.x, Pos.rec.y, Pos.rec.z);
    (void) object;
}

int main(void)
{
    printf("=== pick_place start/schedule check ===\n");
    posInit();
    printf("home: rec(%.2f,%.2f,%.2f)  ser(%.2f,%.2f,%.2f)\n",
           Pos.rec.x, Pos.rec.y, Pos.rec.z,
           Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);

    for (int o = 0; o < 3; o++) {
        double sx = 0, sy = 0, sz = 0, dx = 0, dy = 0, dz = 0, pz = 0;
        bool gs = pickPlaceGetSource(o, &sx, &sy, &sz);
        bool gt = pickPlaceGetTarget(o, &dx, &dy, &dz);
        bool gp = pickPlacePlanZ(o, &pz);
        printf("\n[%c] src(%s) (%.1f,%.1f,%.1f)  dst(%s) (%.1f,%.1f,%.1f)  travelZ(%s) %.1f\n",
               'A' + o, gs ? "ok" : "X", sx, sy, sz, gt ? "ok" : "X", dx, dy, dz,
               gp ? "ok" : "X", pz);
        printf("     rho_src=%.1f d_src=%.1f   rho_dst=%.1f d_dst=%.1f\n",
               sqrt(sx * sx + sy * sy), sqrt(sx * sx + sy * sy + (sz - 38.0) * (sz - 38.0)),
               sqrt(dx * dx + dy * dy), sqrt(dx * dx + dy * dy + (dz - 38.0) * (dz - 38.0)));
        int rc = pickPlaceStart(o);
        printf("     pickPlaceStart(%d) -> %d\n", o, rc);
        if (rc == 0) drain(o, 120000);
    }
    return 0;
}
