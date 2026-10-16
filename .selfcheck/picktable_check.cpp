/* picktable_check.cpp -- 用固件自己的正/反解检查 pick_place 的点位表。
 *
 * 目的：v1.6.7 换成参照仓库的运动模型后，工作空间被**关节行程**卡得比
 * 几何距离 d <= 159 更紧：c 只能 30~110 度，而 d 与 c 一一对应
 * （d^2 = L2^2 + L3^2 - 2*L2*L3*cos(c)），所以真正能到的 d 只有
 * [41.4, 131.1] mm。旧表（以及照抄旧表的"×4"换算）里的点大多在 131 之外。
 *
 * 本程序不需要爬墙：直接调 getAngleEx()（固件本体），
 *   [1] 打印每个 z 上"可达的 rho 区间"
 *   [2] 校验一组候选点位（src / dst / 后方点 / 抬升点 / 各段采样）
 * 编译命令见同目录 README 或本文件的 §编译。
 */

#include <stdio.h>
#include <math.h>

#include "constant_and_positions.h"

static bool solv(double x, double y, double z, double *b, double *r, double *c)
{
    pos p;
    p.rec.x = x;
    p.rec.y = y;
    p.rec.z = z;
    bool clamped = false;
    if (!getAngleEx(&p, &clamped)) return false;
    if (b) *b = p.ser.angle1;
    if (r) *r = p.ser.angle2;
    if (c) *c = p.ser.angle3;
    (void) clamped;
    return true;
}

static bool inBox(double x, double y, double z)
{
    return x >= limit.minX && x <= limit.maxX &&
           y >= limit.minY && y <= limit.maxY &&
           z >= limit.minZ && z <= limit.maxZ;
}

/* 直线段采样校验：与 path_core.h 的 pathCoreSegmentOk 同一套判据
 * （盒子 + 反解成功 + 相邻采样点关节角不跳变）。 */
static bool segOK(double *p, double *q, double step)
{
    double dx = q[0] - p[0], dy = q[1] - p[1], dz = q[2] - p[2];
    double len = sqrt(dx * dx + dy * dy + dz * dz);
    int n = (int)(len / step) + 1;
    if (n > 2000) n = 2000;
    double pb = 0, pr = 0, pc = 0;
    bool first = true;
    for (int i = 0; i <= n; i++) {
        double t = (n > 0) ? (double)i / (double)n : 0.0;
        double x = p[0] + dx * t, y = p[1] + dy * t, z = p[2] + dz * t;
        if (!inBox(x, y, z)) { printf("    seg: 采样点越出盒子 (%.1f,%.1f,%.1f)\n", x, y, z); return false; }
        double b = 0, r = 0, c = 0;
        if (!solv(x, y, z, &b, &r, &c)) { printf("    seg: 采样点无解 (%.1f,%.1f,%.1f)\n", x, y, z); return false; }
        if (!first) {
            if (fabs(b - pb) > 25.0 || fabs(r - pr) > 25.0 || fabs(c - pc) > 25.0) {
                printf("    seg: 分支跳变 (%.1f,%.1f,%.1f)\n", x, y, z);
                return false;
            }
        }
        first = false; pb = b; pr = r; pc = c;
    }
    return true;
}

static void reportPoint(const char *name, double x, double y, double z)
{
    double b = 0, r = 0, c = 0;
    if (!inBox(x, y, z)) { printf("  %-10s (%7.1f,%6.1f,%7.1f)  FAIL 越出盒子\n", name, x, y, z); return; }
    if (!solv(x, y, z, &b, &r, &c)) { printf("  %-10s (%7.1f,%6.1f,%7.1f)  FAIL 反解拒绝 (rho=%.1f d=%.1f)\n",
                                           name, x, y, z, sqrt(x * x + y * y), sqrt(x * x + y * y + (z - 38.0) * (z - 38.0))); return; }
    printf("  %-10s (%7.1f,%6.1f,%7.1f)  OK  b=%6.1f r=%6.1f c=%6.1f  (rho=%.1f d=%.1f)\n",
           name, x, y, z, b, r, c, sqrt(x * x + y * y), sqrt(x * x + y * y + (z - 38.0) * (z - 38.0)));
}

int main(void)
{
    printf("arm1 = { %.1f, %.1f, %.1f }   POS_HOME = (0.0, 80.0, 118.0)  /* static in .cpp */\n",
           arm1.armLength1, arm1.armLength2, arm1.armheight);
    printf("KIN: base %.0f~%.0f  rArm %.0f~%.0f  fArm %.0f~%.0f\n",
           KIN_BASE_MIN, KIN_BASE_MAX, KIN_RARM_MIN, KIN_RARM_MAX, KIN_FARM_MIN, KIN_FARM_MAX);
    printf("limit: x[%.0f,%.0f] y[%.0f,%.0f] z[%.0f,%.0f]\n\n",
           limit.minX, limit.maxX, limit.minY, limit.maxY, limit.minZ, limit.maxZ);

    /* [1] 每个 z 上的可达 rho 区间（沿 +y 方向，x=0，theta=0） */
    printf("[1] 可达 rho 区间（x=0 沿 +y，步长 1mm）\n");
    const double zs[] = { -100, -60, -42, -20, 0, 38, 60, 80, 100, 118, 140, 169, 198, 300 };
    for (unsigned k = 0; k < sizeof(zs) / sizeof(zs[0]); k++) {
        double z = zs[k];
        double lo = -1, hi = -1;
        for (double rho = 0; rho <= 170.0; rho += 1.0) {
            if (solv(0.0, rho, z, NULL, NULL, NULL)) { if (lo < 0) lo = rho; hi = rho; }
        }
        if (lo < 0) printf("  z=%7.1f  : 无解\n", z);
        else        printf("  z=%7.1f  : rho %.0f ~ %.0f\n", z, lo, hi);
    }
    printf("\n");

    /* [1b] 全程可达 z 带：用来对照用户要的高度窗口（-100 ~ +300） */
    printf("[1b] 全程可达 z 带（z 步长 0.5mm，rho 0~170 步长 1mm）\n");
    {
        double zLo = 0, zHi = 0, rhoLo = 0, rhoHi = 0;
        bool got = false;
        for (double z = -80.0; z <= 220.0; z += 0.5) {
            double firstRho = -1, lastRho = -1;
            for (double rho = 0; rho <= 170.0; rho += 1.0)
                if (solv(0.0, rho, z, NULL, NULL, NULL)) { if (firstRho < 0) firstRho = rho; lastRho = rho; }
            if (firstRho >= 0) {
                if (!got) { got = true; zLo = z; rhoLo = firstRho; }
                zHi = z; rhoHi = lastRho;
            }
        }
        if (got) {
            printf("  下限 z = %+7.1f mm (rho 从 %.0f 起)   上限 z = %+7.1f mm (rho 到 %.0f)\n",
                   zLo, rhoLo, zHi, rhoHi);
            printf("  => 末端实际能到的竖直范围约 [%+.1f, %+.1f] mm = [%+.1f, %+.1f] cm（自底盘顶面量）\n",
                   zLo, zHi, zLo / 10.0, zHi / 10.0);
            printf("  => 用户要的 -100 ~ +300 mm 里，可用的只有 [%+.1f, %+.1f]，两端各差 %.1f / %.1f mm\n\n",
                   zLo, zHi, zLo - (-100.0), 300.0 - zHi);
        }
    }

    /* [2] 候选点位：直接改这里再跑 */
    printf("[2] 候选点位校验\n");
    struct { const char *n; double x, y, z; } pts[] = {
        { "HOME",     0.0,   80.0, 118.0 },
        { "A-src",   69.0,   58.0,  20.0 },
        { "B-src",  -69.0,   58.0,  60.0 },
        { "C-src",    0.0,  105.0, 100.0 },
        { "A-dst",   36.0,   77.0,  20.0 },
        { "B-dst",  -36.0,   77.0,  60.0 },
        { "C-dst",    0.0,   75.0, 100.0 },
    };
    for (unsigned i = 0; i < sizeof(pts) / sizeof(pts[0]); i++)
        reportPoint(pts[i].n, pts[i].x, pts[i].y, pts[i].z);

    return 0;
}
