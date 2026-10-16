/* picktable_search.cpp -- 为 pick_place.cpp 的 PICK_SRC / PICK_DST 搜一组
 * 在新运动模型下真正能走通的点位。
 *
 * 判据与 pickPlaceStart() 完全一致（直接调用 path_core.h 的 inline 判据）：
 *   1) 每个关键点 pathCorePointOk()（盒子 + isReachable + 反解未被吸附）；
 *   2) planTravelZ() 的七点阶梯能在 [top+CLEAR, top+LIFT] 里找到一档；
 *   3) 8 个拐点组成的 7 段直线逐段 pathCoreSegmentOk()（含 25° 分支跳变）。
 * 起始位姿取 posInit() 之后的 POS_HOME。
 *
 * 物体位置用极坐标给：(x, y) = (rho*sin(theta), rho*cos(theta))，theta=0 指向 +y。
 */

#include <stdio.h>
#include <math.h>

#include <Arduino.h>
#include "constant_and_positions.h"
#include "path_core.h"

static const double PICK_BACK_DZ      = 20.0;
static const double PICK_LIFT_DZ      = 50.0;
static const double PICK_CLEAR_DZ     = 10.0;
static const double PICK_TRAVEL_STEP  = 5.0;
static const double PICK_SAMPLE_STEP  = 2.0;
static const int    PICK_SAMPLE_MAX   = 160;
static const double PICK_JUMP_DEG     = 25.0;

static void behindPoint(const double *p, double *out)
{
    double rho = sqrt(p[0] * p[0] + p[1] * p[1]);
    if (rho < 1e-6) { out[0] = p[0]; out[1] = p[1]; out[2] = p[2]; return; }
    double k = PICK_BACK_DZ / rho;
    out[0] = p[0] - p[0] * k;
    out[1] = p[1] - p[1] * k;
    out[2] = p[2];
}

static bool planTravelZ(const double *src, const double *dst, const double *behind,
                        double fromX, double fromY, double *outZ, bool *riseAtSrc)
{
    const double top = (src[2] > dst[2]) ? src[2] : dst[2];
    const double zMin = top + PICK_CLEAR_DZ;
    const int maxSteps = (int)((PICK_LIFT_DZ - PICK_CLEAR_DZ) / PICK_TRAVEL_STEP);
    double b, r, c;
    for (int i = 0; i <= maxSteps; i++) {
        const double z = top + PICK_LIFT_DZ - (double)i * PICK_TRAVEL_STEP;
        if (z < zMin - 1e-9) break;
        const bool riseInPlace = pathCorePointOk(fromX, fromY, z, &b, &r, &c);
        if (!riseInPlace && !pathCorePointOk(src[0], src[1], z, &b, &r, &c)) continue;
        if (!pathCorePointOk(behind[0], behind[1], z, &b, &r, &c)) continue;
        if (!pathCorePointOk(src[0], src[1], z, &b, &r, &c)) continue;
        if (!pathCorePointOk(dst[0], dst[1], z, &b, &r, &c)) continue;
        if (!pathCorePointOk(behind[0], behind[1], src[2], &b, &r, &c)) continue;
        if (!pathCorePointOk(src[0], src[1], src[2], &b, &r, &c)) continue;
        if (!pathCorePointOk(dst[0], dst[1], dst[2], &b, &r, &c)) continue;
        *riseAtSrc = !riseInPlace;
        *outZ = z;
        return true;
    }
    return false;
}

/* 与 pickPlaceStart() 第 3 步逐字相同的整条路径校验 */
static bool pathOk(const double *src, const double *dst, const double *behind,
                   double travelZ, bool riseAtSrc, double fromX, double fromY, double fromZ)
{
    double path[8][3];
    path[0][0] = fromX; path[0][1] = fromY; path[0][2] = fromZ;
    path[1][0] = riseAtSrc ? src[0] : fromX;
    path[1][1] = riseAtSrc ? src[1] : fromY;
    path[1][2] = travelZ;
    path[2][0] = behind[0]; path[2][1] = behind[1]; path[2][2] = travelZ;
    path[3][0] = behind[0]; path[3][1] = behind[1]; path[3][2] = src[2];
    path[4][0] = src[0];    path[4][1] = src[1];    path[4][2] = src[2];
    path[5][0] = src[0];    path[5][1] = src[1];    path[5][2] = travelZ;
    path[6][0] = dst[0];    path[6][1] = dst[1];    path[6][2] = travelZ;
    path[7][0] = dst[0];    path[7][1] = dst[1];    path[7][2] = dst[2];
    for (int i = 0; i + 1 < 8; i++) {
        if (!pathCoreSegmentOk(path[i], path[i + 1], PICK_SAMPLE_STEP,
                               PICK_SAMPLE_MAX, PICK_JUMP_DEG)) return false;
    }
    return true;
}

static bool fullCheck(const double *src, const double *dst, double *outTravelZ,
                      bool *outRise, int *failStep)
{
    double behind[3];
    behindPoint(src, behind);
    bool rise = false;
    double travelZ = 0;
    if (!pathCorePointOk(src[0], src[1], src[2], NULL, NULL, NULL)) { *failStep = 1; return false; }
    if (!pathCorePointOk(behind[0], behind[1], src[2], NULL, NULL, NULL)) { *failStep = 2; return false; }
    if (!pathCorePointOk(dst[0], dst[1], dst[2], NULL, NULL, NULL)) { *failStep = 3; return false; }
    if (!planTravelZ(src, dst, behind, Pos.rec.x, Pos.rec.y, &travelZ, &rise)) { *failStep = 4; return false; }
    if (!pathOk(src, dst, behind, travelZ, rise, Pos.rec.x, Pos.rec.y, Pos.rec.z)) { *failStep = 5; return false; }
    *outTravelZ = travelZ;
    *outRise = rise;
    return true;
}

static void polar(double thetaDeg, double rho, double z, double *p)
{
    double t = thetaDeg * M_PI / 180.0;
    p[0] = rho * sin(t);
    p[1] = rho * cos(t);
    p[2] = z;
}

int main(void)
{
    posInit();
    printf("from = (%.1f, %.1f, %.1f)\n", Pos.rec.x, Pos.rec.y, Pos.rec.z);

    /* ---- 候选表校验（改这里再跑，比改固件再编快得多）---- */
    struct { const char *n; double x, y, z; } cand[] = {
        { "A-src",  69.0,  58.0,  20.0 },
        { "A-dst",  36.0,  77.0,  20.0 },
        { "B-src", -69.0,  58.0,  60.0 },
        { "B-dst", -36.0,  77.0,  60.0 },
        { "C-src",   0.0, 105.0, 100.0 },
        { "C-dst",   0.0,  75.0, 100.0 },
    };
    printf("\n[候选表]\n");
    for (int o = 0; o < 3; o++) {
        double src[3], dst[3], travelZ = 0; bool rise = false; int fail = 0;
        src[0] = cand[o * 2].x; src[1] = cand[o * 2].y; src[2] = cand[o * 2].z;
        dst[0] = cand[o * 2 + 1].x; dst[1] = cand[o * 2 + 1].y; dst[2] = cand[o * 2 + 1].z;
        bool ok = fullCheck(src, dst, &travelZ, &rise, &fail);
        printf("  %c src(%.0f,%.0f,%.0f) dst(%.0f,%.0f,%.0f) -> %s (failStep=%d travelZ=%.0f rise=%d)\n",
               'A' + o, src[0], src[1], src[2], dst[0], dst[1], dst[2],
               ok ? "OK" : "FAIL", fail, travelZ, (int) rise);
    }

    if (getenv("PICKTABLE_GRID") == NULL) return 0;

    const double zs[3] = { 20.0, 60.0, 100.0 };
    const double rhos[] = { 70, 80, 90, 100, 110, 120 };
    const double ths[]  = { 60, 45, 30, 15, 0, -15, -30, -45, -60 };

    for (int obj = 0; obj < 3; obj++) {
        printf("\n===== object %c  z=%.0f =====\n", 'A' + obj, zs[obj]);
        int shown = 0;
        for (unsigned si = 0; si < sizeof(ths) / sizeof(ths[0]) && shown < 14; si++) {
            for (unsigned ri = 0; ri < sizeof(rhos) / sizeof(rhos[0]) && shown < 14; ri++) {
                double src[3];
                polar(ths[si], rhos[ri], zs[obj], src);
                for (unsigned di = 0; di < sizeof(ths) / sizeof(ths[0]); di++) {
                    for (unsigned rj = 0; rj < sizeof(rhos) / sizeof(rhos[0]); rj++) {
                        double dst[3];
                        polar(ths[di], rhos[rj], zs[obj], dst);
                        double hx = dst[0] - src[0], hy = dst[1] - src[1];
                        double hdist = sqrt(hx * hx + hy * hy);
                        if (hdist < 60.0) continue;
                        double travelZ = 0; bool rise = false; int fail = 0;
                        if (!fullCheck(src, dst, &travelZ, &rise, &fail)) continue;
                        printf("  src(th=%4.0f rho=%3.0f) (%.1f,%6.1f,%5.0f)  "
                               "dst(th=%4.0f rho=%3.0f) (%.1f,%6.1f,%5.0f)  "
                               "travelZ=%.0f rise@src=%d  dxy=%.0f\n",
                               ths[si], rhos[ri], src[0], src[1], src[2],
                               ths[di], rhos[rj], dst[0], dst[1], dst[2],
                               travelZ, (int) rise, hdist);
                        shown++;
                        if (shown >= 14) break;
                    }
                    if (shown >= 14) break;
                }
            }
        }
        if (shown == 0) printf("  (无可行组合)\n");
    }
    return 0;
}
