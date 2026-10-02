/*
 * probe_pick_place.cpp -- A/B/C 自动取放序列的自检探针
 *
 * 【为什么存在】pick_place.cpp 是一条非阻塞状态机，真机上"发一个字母看它动不动"
 * 很难覆盖拒绝分支、忙时让位、以及每一步的坐标自洽性。本探针在 PC 上把状态机
 * 完整跑三遍（A/B/C），逐轮断言不变量。
 *
 * 【约定（与其它探针一致）】
 *   - 直接链接固件，绝不自己重写运动学公式：反解一律调固件 getAngleEx()，
 *     正解一律调固件 recFromServo()；限位一律读固件全局 servoLimit/limit。
 *   - 必须先 posInit()：限位映射表与 Pos 都在它里面初始化。
 *   - 时间是 mock 的 g_mockMillis，靠它推进阶段。
 */

#include <cmath>
#include <cstdio>
#include <cstring>

#include "Arduino.h"
#include "constant_and_positions.h"
#include "move.h"
#include "joystick_control.h"
#include "serial_protocol.h"
#include "protocol_constants.h"
#include "pick_place.h"

static int failures = 0;

static void check(const char *name, bool ok, const char *detail) {
  printf("  %-52s %s  %s\n", name, ok ? "PASS" : "FAIL", detail);
  if (!ok) failures++;
}

/* 坐标与关节角自洽：recFromServo(Pos.ser) 必须等于 Pos.rec */
static bool selfConsistent(void) {
  REC chk;
  if (!recFromServo(&chk, &Pos.ser)) return false;
  return fabs(chk.x - Pos.rec.x) < 1e-9 &&
         fabs(chk.y - Pos.rec.y) < 1e-9 &&
         fabs(chk.z - Pos.rec.z) < 1e-9;
}

static bool recInLimit(const REC *r) {
  return r->x >= limit.minX && r->x <= limit.maxX &&
         r->y >= limit.minY && r->y <= limit.maxY &&
         r->z >= limit.minZ && r->z <= limit.maxZ;
}

static bool sameSer(const SER *a, const SER *b) {
  return a->angle1 == b->angle1 && a->angle2 == b->angle2 &&
         a->angle3 == b->angle3 && a->angle4 == b->angle4;
}

static bool sameRec(const REC *a, const REC *b) {
  return a->x == b->x && a->y == b->y && a->z == b->z;
}

/* 两点在 x/y 平面上的距离，用来判"是不是明显不同的位置" */
static double distXY(const REC *a, const REC *b) {
  double dx = a->x - b->x;
  double dy = a->y - b->y;
  return sqrt(dx * dx + dy * dy);
}

/* 调固件反解，要求成功且没被吸附 */
static bool ikOk(double x, double y, double z) {
  pos p;
  p.rec.x = x;
  p.rec.y = y;
  p.rec.z = z;
  p.ser = Pos.ser;            /* 必须带一个合法的 angle4，否则 clamped 恒真 */
  bool clamped = false;
  if (!getAngleEx(&p, &clamped)) return false;
  return !clamped;
}

/* 一个点能不能用：在 limit 内、在可达空间里、反解成功且没被吸附 */
static bool pointUsable(double x, double y, double z) {
  REC r;
  r.x = x;
  r.y = y;
  r.z = z;
  if (!recInLimit(&r)) return false;
  if (!isReachable(&r)) return false;
  return ikOk(x, y, z);
}

int main(void) {
  posInit();                  /* 必须先做：限位映射表与 Pos 都在这里初始化 */
  adjustSpeed(SPEED_NORMAL);  /* 固定初始档位，后面测 H/L 才有确定的期望 */
  g_mockMillis = 1000;

  printf("== pick_place 自检（A/B/C 自动取放序列）==\n");

  /* ---------------- 1) 位置表 ---------------- */
  printf("\n[1] 位置表\n");
  REC src[PICK_OBJECT_COUNT];
  REC dst[PICK_OBJECT_COUNT];
  bool tableOk = true;
  for (int i = 0; i < PICK_OBJECT_COUNT; i++) {
    if (!pickPlaceGetSource(i, &src[i].x, &src[i].y, &src[i].z)) tableOk = false;
    if (!pickPlaceGetTarget(i, &dst[i].x, &dst[i].y, &dst[i].z)) tableOk = false;
  }
  check("A/B/C 都能取到初始位置与放置位置", tableOk,
        tableOk ? "三个物体齐全" : "取表失败");

  char d[192];
  for (int i = 0; i < PICK_OBJECT_COUNT; i++) {
    snprintf(d, sizeof(d), "%c 初始(%.2f,%.2f,%.2f) -> 放置(%.2f,%.2f,%.2f)  dX=%+.2f dY=%+.2f",
             'A' + i, src[i].x, src[i].y, src[i].z, dst[i].x, dst[i].y, dst[i].z,
             dst[i].x - src[i].x, dst[i].y - src[i].y);
    bool ok = fabs(dst[i].x - src[i].x) >= 5.0 && fabs(dst[i].y - src[i].y) >= 5.0;
    check("该物体 x 与 y 方向都有明显位移（>=5）", ok, d);
  }

  for (int i = 0; i < PICK_OBJECT_COUNT; i++) {
    for (int k = i + 1; k < PICK_OBJECT_COUNT; k++) {
      double ds = distXY(&src[i], &src[k]);
      double dd = distXY(&dst[i], &dst[k]);
      snprintf(d, sizeof(d), "初始 %c-%c 相距 %.2f，放置 %c-%c 相距 %.2f",
               'A' + i, 'A' + k, ds, 'A' + i, 'A' + k, dd);
      check("两组位置都互不相同（相距 >=5）", ds >= 5.0 && dd >= 5.0, d);
    }
  }

  /* 放件时另一个物体可能还停在它的初始位置上：放置点离"别人的初始点"太近，
   * 夹爪垂直下降就会蹭到它。这个门限和上面一样取 5.0。 */
  double worstClear = 1e9;
  for (int i = 0; i < PICK_OBJECT_COUNT; i++) {
    for (int k = 0; k < PICK_OBJECT_COUNT; k++) {
      if (i == k) continue;
      double dc = distXY(&dst[i], &src[k]);
      if (dc < worstClear) worstClear = dc;
    }
  }
  snprintf(d, sizeof(d), "最紧的一对是 %.2f（要求 >=5）", worstClear);
  check("放置点不会蹭到别的物体（离别人的初始点 >=5）", worstClear >= 5.0, d);

  /* ---------------- 2) 取放点与接近点 ---------------- */
  printf("\n[2] 取放点与接近点可达性\n");
  for (int i = 0; i < PICK_OBJECT_COUNT; i++) {
    double rise = pickPlaceApproachDz();
    bool ok = pointUsable(src[i].x, src[i].y, src[i].z) &&
              pointUsable(dst[i].x, dst[i].y, dst[i].z) &&
              pointUsable(src[i].x, src[i].y, src[i].z + rise) &&
              pointUsable(dst[i].x, dst[i].y, dst[i].z + rise);
    snprintf(d, sizeof(d), "%c 与抬升 %.1f 的接近点：在 limit 内 + isReachable + 反解未被吸附",
             'A' + i, rise);
    check("取放点可用", ok, d);
  }

  /* ---------------- 3) 启动语义 ---------------- */
  printf("\n[3] 启动语义\n");
  int rc = pickPlaceStart(PICK_OBJECT_A);
  snprintf(d, sizeof(d), "rc=%d", rc);
  check("启动 A 返回 0", rc == 0, d);
  check("启动后 pickPlaceIsBusy 为真", pickPlaceIsBusy(), "忙");
  check("pickPlaceCurrentObject 是 A", pickPlaceCurrentObject() == PICK_OBJECT_A, "A");

  rc = pickPlaceStart(PICK_OBJECT_B);
  snprintf(d, sizeof(d), "rc=%d", rc);
  check("执行中再启动返回 -2（不打断当前序列）", rc == -2, d);
  check("仍然在执行 A", pickPlaceCurrentObject() == PICK_OBJECT_A, "A");

  rc = pickPlaceStart(-1);
  snprintf(d, sizeof(d), "rc=%d", rc);
  check("编号 -1 返回 -1", rc == -1, d);

  rc = pickPlaceStart(PICK_OBJECT_COUNT);
  snprintf(d, sizeof(d), "编号 3 rc=%d", rc);
  check("编号 3 返回 -1", rc == -1, d);

  /* ---------------- 4/5) 完整跑三遍 + 逐轮不变量 ---------------- */
  printf("\n[4] 完整跑 A/B/C 三轮（每轮逐点检查不变量）\n");
  int violations = 0;
  char firstViol[256];
  firstViol[0] = '\0';
  const char *trace[64];
  int traceN = 0;
  const char *lastName = "";

  for (int obj = 0; obj < PICK_OBJECT_COUNT; obj++) {
    rc = pickPlaceStart(obj);      /* 第一轮在 [3] 里已经启动过 A，重复启动返回 -2 */
    if (obj == 0 && rc == -2) rc = 0;
    if (rc != 0) {
      snprintf(d, sizeof(d), "obj=%c rc=%d", 'A' + obj, rc);
      check("启动该物体", false, d);
      continue;
    }

    long iter = 0;
    const long cap = 300000;
    while (pickPlaceIsBusy() && iter < cap) {
      pickPlaceLoop();
      g_mockMillis += 20;
      iter++;

      if (!selfConsistent() || !recInLimit(&Pos.rec) || !isServoInRange(&Pos.ser)) {
        if (violations == 0) {
          snprintf(firstViol, sizeof(firstViol),
                   "obj=%c iter=%ld self=%d inLimit=%d servo=%d pos=(%.4f,%.4f,%.4f) "
                   "ser=(%.4f,%.4f,%.4f,%.4f)",
                   'A' + obj, iter, (int)selfConsistent(), (int)recInLimit(&Pos.rec),
                   (int)isServoInRange(&Pos.ser), Pos.rec.x, Pos.rec.y, Pos.rec.z,
                   Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3, Pos.ser.angle4);
        }
        violations++;
      }

      const char *nm = pickPlaceStageName();
      if (strcmp(nm, lastName) != 0) {
        if (traceN < 64) trace[traceN++] = nm;
        lastName = nm;
      }
    }

    snprintf(d, sizeof(d), "%c 在 %ld 轮内跑完（上限 %ld）", 'A' + obj, iter, cap);
    check("序列跑完（没有卡住）", iter < cap && !pickPlaceIsBusy(), d);

    snprintf(d, sizeof(d), "结束阶段=%s", pickPlaceStageName());
    check("结束阶段回到 idle", strcmp(pickPlaceStageName(), "idle") == 0, d);

    snprintf(d, sizeof(d), "angle4=%.3f 期望 maxF=%.3f", Pos.ser.angle4, servoLimit.maxF);
    check("末端张开到位（与串口 O 同角度）", fabs(Pos.ser.angle4 - servoLimit.maxF) < 1e-9, d);

    snprintf(d, sizeof(d), "final=(%.4f,%.4f,%.4f) 期望=(%.4f,%.4f,%.4f)",
             Pos.rec.x, Pos.rec.y, Pos.rec.z,
             dst[obj].x, dst[obj].y, dst[obj].z + pickPlaceApproachDz());
    check("最终停在放置点正上方（撤离高度）",
          fabs(Pos.rec.x - dst[obj].x) < 1e-6 &&
          fabs(Pos.rec.y - dst[obj].y) < 1e-6 &&
          fabs(Pos.rec.z - (dst[obj].z + pickPlaceApproachDz())) < 1e-6, d);

    check("结束时坐标与关节角自洽", selfConsistent(), "recFromServo(Pos.ser) == Pos.rec");
  }

  snprintf(d, sizeof(d), "violations=%d%s%s", violations,
           violations ? " 首条: " : "", violations ? firstViol : "");
  check("逐轮不变量（自洽 / 在界内 / 关节角在行程内）", violations == 0, d);

  printf("      阶段轨迹：");
  for (int i = 0; i < traceN; i++) printf("%s%s", i ? " -> " : "", trace[i]);
  printf("\n");

  /* ---------------- 6) 忙时让位 ---------------- */
  printf("\n[6] 序列执行期间的命令让位\n");
  rc = pickPlaceStart(PICK_OBJECT_A);
  snprintf(d, sizeof(d), "rc=%d", rc);
  check("重新启动 A", rc == 0, d);

  SER serBefore = Pos.ser;
  REC recBefore = Pos.rec;
  int rBusy = protoHandleLine("O");
  snprintf(d, sizeof(d), "rc=%d", rBusy);
  check("忙时 O 返回 PROTO_RES_BUSY", rBusy == PROTO_RES_BUSY, d);
  rBusy = protoHandleLine("S");
  snprintf(d, sizeof(d), "rc=%d", rBusy);
  check("忙时 S 返回 PROTO_RES_BUSY", rBusy == PROTO_RES_BUSY, d);
  rBusy = protoHandleLine("x45");
  snprintf(d, sizeof(d), "rc=%d", rBusy);
  check("忙时 x45 返回 PROTO_RES_BUSY", rBusy == PROTO_RES_BUSY, d);
  rBusy = protoHandleLine("k");
  snprintf(d, sizeof(d), "rc=%d", rBusy);
  check("忙时 k 返回 PROTO_RES_BUSY", rBusy == PROTO_RES_BUSY, d);
  check("被拒绝的指令一个字节都没改",
        sameSer(&serBefore, &Pos.ser) && sameRec(&recBefore, &Pos.rec),
        "Pos.ser 与 Pos.rec 均未变");

  int lvl0 = speedGetLevel();
  int rH = protoHandleLine("H");
  int lvl1 = speedGetLevel();
  snprintf(d, sizeof(d), "rc=%d level %d -> %d", rH, lvl0, lvl1);
  check("忙时 H 仍然生效（提速）", rH == PROTO_RES_SPEED_UP && lvl1 != lvl0, d);
  int rL = protoHandleLine("L");
  int lvl2 = speedGetLevel();
  snprintf(d, sizeof(d), "rc=%d level %d -> %d", rL, lvl1, lvl2);
  check("忙时 L 仍然生效（降速回原档）", rL == PROTO_RES_SPEED_DOWN && lvl2 == lvl0, d);
  check("调速期间机械臂没动", sameSer(&serBefore, &Pos.ser), "Pos.ser 未变");

  long iter = 0;
  while (pickPlaceIsBusy() && iter < 300000) {
    pickPlaceLoop();
    g_mockMillis += 20;
    iter++;
  }
  check("让位测试后序列仍能正常跑完", !pickPlaceIsBusy(), "已结束");

  /* ---------------- 7) 空闲时反复调用 ---------------- */
  printf("\n[7] 空闲稳定性\n");
  serBefore = Pos.ser;
  recBefore = Pos.rec;
  for (int i = 0; i < 100; i++) pickPlaceLoop();
  check("空闲时 pickPlaceLoop 调 100 次不动任何状态",
        sameSer(&serBefore, &Pos.ser) && sameRec(&recBefore, &Pos.rec), "Pos 未变");
  check("空闲时阶段名是 idle", strcmp(pickPlaceStageName(), "idle") == 0, pickPlaceStageName());
  check("空闲时物体编号是 -1", pickPlaceCurrentObject() == -1, "-1");

  printf("\n");
  if (failures == 0) {
    printf(">>> ALL PASS (失败 0 项)\n");
    return 0;
  }
  printf(">>> FAILED (失败 %d 项)\n", failures);
  return 1;
}
