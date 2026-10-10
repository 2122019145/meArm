/* probe_protocol.cpp —— 串口协议模块端到端回归测试
 * 直接链接固件：serial_protocol.cpp + constant_and_positions.cpp + move.cpp + protocol_constants.cpp
 *
 * 仿真方式：用 mockSerialFeed() 喂串口命令，推进 g_mockMillis 走时间门控，
 *           用 serialProtocolLoop() 处理串口数据流，用 protoHandleLine() 直接处理单行。
 *
 * 本探针专门测试 serial_protocol.cpp 模块的 13 个核心功能：
 * 1) 固定指令 O/S
 * 2) 兼容命令 1/2/3
 * 3) H/L 档位升降
 * 4) k/K 步进与到限不越界
 * 5) x/y/z 三舵机同步角度（x->angle1(b)、y->angle2(r)、z->angle3(c)，一条指令一次写完）
 * 6) 只写一部分轴（其余关节保持在原位）
 * 7) 大小写与空白容忍
 * 8) 小数与可选等号
 * 9) 超出行程按 servoLimit 夹取（不拒绝整条指令，题目要的是"输出对应角度的 PWM"）
 * 10) 错误输入不改变状态
 * 11) 分片到达与无换行超时
 * 12) 串口应答与运行时功能开关（必须先于压力测试：压力测试会随机发出 A/B/C，
 *     启动取放序列后就一直是"忙"的了，而 R/P 只有在空闲时才谈得上回复）
 * 13) 随机压力测试
 */
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include "Arduino.h"
#include "constant_and_positions.h"
#include "draw_control.h"
#include "move.h"
#include "serial_protocol.h"
#include "protocol_constants.h"

/* Forward declaration since protoHandleLine is missing from serial_protocol.h */
int protoHandleLine(const char *line);

static int failures = 0;

static void check(const char *name, bool ok, const char *detail) {
  printf("  %-48s %s  %s\n", name, ok ? "PASS" : "FAIL", detail);
  if (!ok) failures++;
}

/* 把四路轴都回到中位，并清掉串口输入 */
static void resetInputs(void) {
  Pos.ser.angle1 = 90.0;
  Pos.ser.angle2 = 90.0;
  Pos.ser.angle3 = 90.0;
  Pos.ser.angle4 = 105.0;
  mockSerialClear();
}

/* 四个关节角组成的快照，便于比较 */
struct Joints { double b, r, c, f; };
static struct Joints snap(void) {
  struct Joints j;
  j.b = Pos.ser.angle1; j.r = Pos.ser.angle2;
  j.c = Pos.ser.angle3; j.f = Pos.ser.angle4;
  return j;
}

/* Pos.rec 的快照，便于比较 */
struct PosRecSnapshot { double x, y, z; };
static struct PosRecSnapshot snapRec(void) {
  struct PosRecSnapshot r;
  r.x = Pos.rec.x; r.y = Pos.rec.y; r.z = Pos.rec.z;
  return r;
}

/* 轴 a 的行程上下限（0=x->angle1/b，1=y->angle2/r，2=z->angle3/c）。
 * 期望值不写死，直接取固件自己的 servoLimit：标定改了探针跟着改。 */
static void probeTravel(int axis, double *lo, double *hi) {
  (void)axis;
  *lo = 0.0;
  *hi = 180.0;
}

/* 固件 protoApplyAngles() 的落地规则：写进去的度数按该关节的 servoLimit
 * 夹取（SERVO_LIMIT_CLAMP），所以探针的期望值也必须先夹一遍。 */
static double probeClampToTravel(int axis, double v) {
  double lo, hi;
  probeTravel(axis, &lo, &hi);
  if (v < lo) v = lo;
  if (v > hi) v = hi;
  return v;
}

/* 探针期望的落点与固件落点之间允许的坐标差。
 * 反解残差门限 0.05 是角度量，换算成 20mm 臂长的坐标误差远小于 1e-6，
 * 所以这个容差既容得下浮点误差，又不会把"明显没到位"放过。 */
static const double PROBE_POINT_TOL = 1e-6;

static double probePointErr(const REC *a, const REC *b) {
  return sqrt(pow(a->x - b->x, 2) + pow(a->y - b->y, 2) + pow(a->z - b->z, 2));
}

/* 压力测试用的角度区间（度）：故意跨到两端的行程之外，
 * 好让"超出行程按限位夹取、且未提到的轴不动"这条不变量真的被压到。 */
static const double PROBE_RAND_LO = -40.0;
static const double PROBE_RAND_HI = 220.0;

/* 压力测试的每条不变量各占一位，判定与日志都对着这些名字看。
 * 命名规则：INV1..INV5 是"角度有效性"，INV6/INV7 是"派生坐标自洽"，
 * INV8 是"角度指令逐轴落地正确 / 被拒的输入原封不动"。 */
#define INV_FINITE    0x01u  /* 四个角都是有限数 */
#define INV_A1_RANGE  0x02u  /* angle1(b) 在 servoLimit 行程内 */
#define INV_A2_RANGE  0x04u  /* angle2(r) 在 servoLimit 行程内 */
#define INV_A3_RANGE  0x08u  /* angle3(c) 在 servoLimit 行程内 */
#define INV_A4_RANGE  0x10u  /* angle4(f) 在 servoLimit 行程内 */
#define INV_REC_OK    0x20u  /* recFromServo() 返回 true */
#define INV_REC_MATCH 0x40u  /* Pos.rec 与 recFromServo() 结果逐分量一致 */
#define INV_ANGLE_LAND 0x80u /* 角度指令：逐轴落地=按行程夹取、未提到的关节不动 */
                                  /* 语法错的整行必须原封不动 */

int main(void) {

  resetInputs();
  g_mockMillis = 1000;
  serialProtocolBegin();
  posInit();

  char buf[260];
  Serial.clearOutput();
  check("feature help advertises runtime status command", protoHandleLine("!") == PROTO_RES_NONE && Serial.getOutput() == "P=1 B=1 D=1\n", Serial.getOutput().c_str());
  Serial.clearOutput();
  protoHandleLine("H");
  check("speed acknowledgement follows debug output", Serial.getOutput().find("OK\n") != std::string::npos, Serial.getOutput().c_str());
  Serial.clearOutput();

  printf("=== 1) 固定指令 O / S（绝对到位） ===\n");
  {
    resetInputs();
    Pos.ser.angle4 = (60.0 + 150.0) / 2;
    mockSerialFeed("O");
    serialProtocolLoop();
    snprintf(buf, sizeof(buf), "angle4=%.1f maxF=%.1f protoRes=%d",
             Pos.ser.angle4, 150.0, protoHandleLine("O"));
    check("mockSerialFeed(\"O\") -> angle4==150.0 且 protoHandleLine(\"O\")返回PROTO_RES_GRIPPER_OPEN",
          Pos.ser.angle4 == 150.0 && protoHandleLine("O") == PROTO_RES_GRIPPER_OPEN, buf);

    mockSerialFeed("S");
    serialProtocolLoop();
    snprintf(buf, sizeof(buf), "angle4=%.1f minF=%.1f protoRes=%d",
             Pos.ser.angle4, 60.0, protoHandleLine("S"));
    check("mockSerialFeed(\"S\") -> angle4==60.0 且 protoHandleLine(\"S\")返回PROTO_RES_GRIPPER_CLOSE",
          Pos.ser.angle4 == 60.0 && protoHandleLine("S") == PROTO_RES_GRIPPER_CLOSE, buf);
  }

  printf("=== 3) H / L 档位升降与端点行为 ===\n");
  {
    adjustSpeed(SPEED_NORMAL);  /* 回中档 */
    snprintf(buf, sizeof(buf), "初始档位=%d", speedGetLevel());
    check("adjustSpeed(SPEED_NORMAL) 回中档", speedGetLevel() == SPEED_NORMAL, buf);

    protoHandleLine("H");
    snprintf(buf, sizeof(buf), "第一次 H 后档位=%d", speedGetLevel());
    check("protoHandleLine(\"H\") -> speedGetLevel()==SPEED_FAST", speedGetLevel() == SPEED_FAST, buf);

    protoHandleLine("H");
    snprintf(buf, sizeof(buf), "第二次 H 后档位=%d", speedGetLevel());
    check("protoHandleLine(\"H\") 第二次仍==SPEED_FAST（到端点不变）", speedGetLevel() == SPEED_FAST, buf);

    protoHandleLine("L");
    snprintf(buf, sizeof(buf), "第一次 L 后档位=%d", speedGetLevel());
    check("protoHandleLine(\"L\") -> speedGetLevel()==SPEED_NORMAL", speedGetLevel() == SPEED_NORMAL, buf);

    protoHandleLine("L");
    snprintf(buf, sizeof(buf), "第二次 L 后档位=%d", speedGetLevel());
    check("protoHandleLine(\"L\") -> speedGetLevel()==SPEED_SLOW", speedGetLevel() == SPEED_SLOW, buf);

    protoHandleLine("L");
    snprintf(buf, sizeof(buf), "第三次 L 后档位=%d", speedGetLevel());
    check("protoHandleLine(\"L\") 第三次仍==SPEED_SLOW（到端点不变）", speedGetLevel() == SPEED_SLOW, buf);
  }

  printf("=== 4) O / S 到限不越界（重复执行幂等） ===\n");
  {
    resetInputs();
    Pos.ser.angle4 = (60.0 + 150.0) / 2;

    /* 连发 60 次 "O"（张开） */
    for (int i = 0; i < 60; i++) {
      protoHandleLine("O");
    }
    snprintf(buf, sizeof(buf), "60次\"O\"后 angle4=%.1f maxF=%.1f <= maxF",
             Pos.ser.angle4, 150.0);
    check("连发60次\"O\" -> angle4==150.0且<=maxF",
          Pos.ser.angle4 == 150.0 && Pos.ser.angle4 <= 150.0, buf);

    /* 连发 60 次 "S"（关闭） */
    for (int i = 0; i < 60; i++) {
      protoHandleLine("S");
    }
    snprintf(buf, sizeof(buf), "60次\"S\"后 angle4=%.1f minF=%.1f >= minF",
             Pos.ser.angle4, 60.0);
    check("连发60次\"S\" -> angle4==60.0且>=minF",
          Pos.ser.angle4 == 60.0 && Pos.ser.angle4 >= 60.0, buf);
  }

  printf("=== 5) x/y/z 三舵机同步角度（本需求的核心） ===\n");
  {
    resetInputs();
    /* 角度语义：x -> angle1(b)、y -> angle2(r)、z -> angle3(c)。
     * 一条指令把提到的轴一次写完，写完立刻做一次正解刷新，
     * 所以 Pos.rec 必须等于 recFromServo(Pos.ser)，而不是等于某个坐标目标。 */
    (void) recFromServo(&Pos.rec, &Pos.ser);
    Serial.clearOutput();
    mockSerialFeed("x20,y0,z40\n");
    serialProtocolLoop();

    double want1 = probeClampToTravel(0, 20.0);
    double want2 = probeClampToTravel(1,  0.0);
    double want3 = probeClampToTravel(2, 40.0);
    snprintf(buf, sizeof(buf),
             "angle1=%.6f(期望%.6f) angle2=%.6f(期望%.6f) angle3=%.6f(期望%.6f)",
             Pos.ser.angle1, want1, Pos.ser.angle2, want2, Pos.ser.angle3, want3);
    check("mockSerialFeed(\"x20,y0,z40\\n\") -> 三个关节角分别写成 20/0/40（按行程夹取）",
          fabs(Pos.ser.angle1 - want1) < 1e-9 &&
          fabs(Pos.ser.angle2 - want2) < 1e-9 &&
          fabs(Pos.ser.angle3 - want3) < 1e-9, buf);

    std::string out5 = Serial.getOutput();
    check("角度指令回 OK（题目要求的\"输入有响应\"）且不出现 REJECTED/ERR",
          out5.find("OK\n") != std::string::npos &&
          out5.find("REJECTED") == std::string::npos &&
          out5.find("ERR\n") == std::string::npos, out5.c_str());

    /* 写完立刻正解：Pos.rec 必须等于 recFromServo(Pos.ser)（坐标是角度的派生量） */
    REC fk;
    fk.x = 0.0; fk.y = 0.0; fk.z = 0.0;
    bool fkOk = recFromServo(&fk, &Pos.ser);
    snprintf(buf, sizeof(buf), "正解=(%.6f,%.6f,%.6f) Pos.rec=(%.6f,%.6f,%.6f) recFromServo=%d",
             fk.x, fk.y, fk.z, Pos.rec.x, Pos.rec.y, Pos.rec.z, (int)fkOk);
    check("写完角度后 Pos.rec == recFromServo(Pos.ser)（<1e-9，坐标永远等于角度的真实结果）",
          fkOk && probePointErr(&fk, &Pos.rec) < 1e-9, buf);

    /* 同步性：一条指令里的三个轴必须是同一次写入 —— 探针从"一个字节都没写坏的
     * 中间态"这一侧验证：拿一次只提到 x 的指令，y/z 必须保持上一次的值。 */
    double keepR = Pos.ser.angle2, keepC = Pos.ser.angle3;
    mockSerialFeed("x30\n");
    serialProtocolLoop();
    snprintf(buf, sizeof(buf), "angle1=%.6f(期望%.6f) angle2=%.6f(保持%.6f) angle3=%.6f(保持%.6f)",
             Pos.ser.angle1, probeClampToTravel(0, 30.0), Pos.ser.angle2, keepR, Pos.ser.angle3, keepC);
    check("只提到 x 时 y/z 保持（其余关节不被清零）",
          fabs(Pos.ser.angle1 - probeClampToTravel(0, 30.0)) < 1e-9 &&
          Pos.ser.angle2 == keepR && Pos.ser.angle3 == keepC, buf);
  }

  printf("=== 6) 只写一部分轴（其余关节留在原位） ===\n");
  {
    resetInputs();
    (void) recFromServo(&Pos.rec, &Pos.ser);

    mockSerialFeed("x20,y0,z40\n");        /* 先写一组已知角度 */
    serialProtocolLoop();
    double b0 = Pos.ser.angle1, c0 = Pos.ser.angle3;

    mockSerialFeed("y10\n");               /* 只给 y */
    serialProtocolLoop();

    snprintf(buf, sizeof(buf), "angle2=%.6f(期望%.6f) angle1=%.6f(基准%.6f) angle3=%.6f(基准%.6f)",
             Pos.ser.angle2, probeClampToTravel(1, 10.0), Pos.ser.angle1, b0, Pos.ser.angle3, c0);
    check("mockSerialFeed(\"y10\\n\") -> 只有 angle2 变成 10（按行程夹取），angle1/angle3 不动",
          fabs(Pos.ser.angle2 - probeClampToTravel(1, 10.0)) < 1e-9 &&
          Pos.ser.angle1 == b0 && Pos.ser.angle3 == c0, buf);

    /* 角度指令只动三个关节：末端夹具角必须原样保留 */
    Pos.ser.angle4 = 60.0;
    (void) recFromServo(&Pos.rec, &Pos.ser);
    mockSerialFeed("x20,y0,z40\n");
    serialProtocolLoop();

    snprintf(buf, sizeof(buf), "angle4=%.6f(指令前%.6f) 行程[%.1f,%.1f]",
             Pos.ser.angle4, 60.0, 60.0, 150.0);
    check("角度指令不碰末端夹具角（angle4 保持指令前的值）",
          Pos.ser.angle4 == 60.0, buf);
  }

  printf("=== 7) 大小写与空白容忍 ===\n");
  {
    resetInputs();
    (void) recFromServo(&Pos.rec, &Pos.ser);

    mockSerialFeed(" X20 , Y0 , Z40 \n");
    serialProtocolLoop();

    snprintf(buf, sizeof(buf), "angle1=%.6f angle2=%.6f angle3=%.6f",
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);
    check("mockSerialFeed(\" X20 , Y0 , Z40 \\n\") -> 与全小写写法落地完全一致",
          fabs(Pos.ser.angle1 - probeClampToTravel(0, 20.0)) < 1e-9 &&
          fabs(Pos.ser.angle2 - probeClampToTravel(1,  0.0)) < 1e-9 &&
          fabs(Pos.ser.angle3 - probeClampToTravel(2, 40.0)) < 1e-9, buf);
  }

  printf("=== 8) 小数与可选等号 ===\n");
  {
    resetInputs();
    (void) recFromServo(&Pos.rec, &Pos.ser);

    mockSerialFeed("x20,y0,z40\n");        /* 先写一组基准 */
    serialProtocolLoop();
    mockSerialFeed("x12.5\n");             /* 再只改 x，带小数 */
    serialProtocolLoop();

    snprintf(buf, sizeof(buf), "angle1=%.6f(期望12.5) angle2=%.6f(基准) angle3=%.6f(基准)",
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);
    check("mockSerialFeed(\"x12.5\\n\") -> angle1 变成 12.5，其余不动",
          fabs(Pos.ser.angle1 - probeClampToTravel(0, 12.5)) < 1e-9, buf);

    resetInputs();
    (void) recFromServo(&Pos.rec, &Pos.ser);
    mockSerialFeed("x=20,y=0,z=40\n");
    serialProtocolLoop();

    snprintf(buf, sizeof(buf), "angle1=%.6f angle2=%.6f angle3=%.6f",
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);
    check("mockSerialFeed(\"x=20,y=0,z=40\\n\") -> 等号形式与空格形式等价",
          fabs(Pos.ser.angle1 - probeClampToTravel(0, 20.0)) < 1e-9 &&
          fabs(Pos.ser.angle2 - probeClampToTravel(1,  0.0)) < 1e-9 &&
          fabs(Pos.ser.angle3 - probeClampToTravel(2, 40.0)) < 1e-9, buf);
  }

  printf("=== 9) 超出行程按 servoLimit 夹取（不拒绝整条指令） ===\n");
  {
    resetInputs();
    (void) recFromServo(&Pos.rec, &Pos.ser);
    mockSerialFeed("x20,y0,z40\n");        /* 建立已知基准点 */
    serialProtocolLoop();

    /* 题目要的是"控制对应编号的舵机输出对应角度的 PWM"：机械行程之外的角度
     * 不能把整条指令作废，而是吸附到该关节的行程端点（与摇杆/取放的夹取一致）。 */
    const char *over[]   = { "x200,y0,z40", "x20,y-90,z40", "x20,y0,z999", "x0,y200,z40" };
    const char *overWhy[] = { "x 远超上限", "y 远低于下限", "z 远超上限", "y 远超上限" };
    for (int i = 0; i < 4; i++) {
      resetInputs();
      (void) recFromServo(&Pos.rec, &Pos.ser);
      Serial.clearOutput();
      int rc = protoHandleLine(over[i]);
      std::string o = Serial.getOutput();

      /* 该用例只提到哪些轴就只检查哪些轴，其余轴按中位保持 */
      double mid1 = (0.0 + 180.0) / 2;
      double mid2 = (0.0 + 180.0) / 2;
      double mid3 = (0.0 + 180.0) / 2;
      double want1 = mid1, want2 = mid2, want3 = mid3;
      if (strcmp(over[i], "x200,y0,z40") == 0)  { want1 = probeClampToTravel(0, 200.0); want2 = probeClampToTravel(1, 0.0); want3 = probeClampToTravel(2, 40.0); }
      if (strcmp(over[i], "x20,y-90,z40") == 0) { want1 = probeClampToTravel(0,  20.0); want2 = probeClampToTravel(1, -90.0); want3 = probeClampToTravel(2, 40.0); }
      if (strcmp(over[i], "x20,y0,z999") == 0)  { want1 = probeClampToTravel(0,  20.0); want2 = probeClampToTravel(1, 0.0); want3 = probeClampToTravel(2, 999.0); }
      if (strcmp(over[i], "x0,y200,z40") == 0)  { want1 = probeClampToTravel(0,   0.0); want2 = probeClampToTravel(1, 200.0); want3 = probeClampToTravel(2, 40.0); }

      bool landed = fabs(Pos.ser.angle1 - want1) < 1e-9 &&
                    fabs(Pos.ser.angle2 - want2) < 1e-9 &&
                    fabs(Pos.ser.angle3 - want3) < 1e-9;
      bool okReply = o.find("OK\n") != std::string::npos && o.find("REJECTED") == std::string::npos;

      snprintf(buf, sizeof(buf), "\"%s\"(%s): 返回=%d angle=(%.1f,%.1f,%.1f) 期望=(%.1f,%.1f,%.1f) 回OK=%s",
               over[i], overWhy[i], rc, Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3,
               want1, want2, want3, okReply ? "true" : "false");
      check("超出行程：夹到行程端点 + 回 OK（不拒绝整条指令）",
            rc == PROTO_RES_ANGLES_SET && landed && okReply, buf);
    }

    /* 端点必须正好是 servoLimit 的限值，不是别的数（钉住"夹取"这件事本身） */
    resetInputs();
    (void) recFromServo(&Pos.rec, &Pos.ser);
    (void) protoHandleLine("x200");
    (void) protoHandleLine("y200");
    (void) protoHandleLine("z200");
    snprintf(buf, sizeof(buf), "b=%.6f(maxB=%.6f) r=%.6f(maxR=%.6f) c=%.6f(maxC=%.6f)",
             Pos.ser.angle1, 180.0, Pos.ser.angle2, 180.0,
             Pos.ser.angle3, 180.0);
    check("x200/y200/z200 后三个关节都停在自己的行程上限",
          Pos.ser.angle1 == 180.0 && Pos.ser.angle2 == 180.0 &&
          Pos.ser.angle3 == 180.0, buf);
  }

  printf("=== 10) 错误输入不改变状态（**这是最重要的一段**） ===\n");
  {
    const char* testCases[] = {"x10,,y20", "x10y20", "x", "y45,", ",x10", "x=", "x12.", "hello", "q", "", "   "};
    /* 每条用例允许的返回值集合；第二个元素 -1 = 只有第一个是合法的。
     * 说明: 用例 ",x10" 属于规格分歧 —— 首字符是逗号而不是轴字母，固件把它
     * 归为 UNKNOWN（"不认识的命令"），也可以论证它该是 BAD_SYNTAX
     *（"像角度指令但语法错"）。两种归类都合理，所以两者都接受；
     * 但"角度/坐标一个都不许动"这条断言不放宽。 */
    int allowedResults[11][2] = {
      { PROTO_RES_BAD_SYNTAX, -1 },               /* "x10,,y20" */
      { PROTO_RES_BAD_SYNTAX, -1 },               /* "x10y20"   */
      { PROTO_RES_BAD_SYNTAX, -1 },               /* "x"        */
      { PROTO_RES_BAD_SYNTAX, -1 },               /* "y45,"     */
      { PROTO_RES_UNKNOWN, PROTO_RES_BAD_SYNTAX },/* ",x10"     */
      { PROTO_RES_BAD_SYNTAX, -1 },               /* "x="       */
      { PROTO_RES_BAD_SYNTAX, -1 },               /* "x12."     */
      { PROTO_RES_UNKNOWN, -1 },                  /* "hello"    */
      { PROTO_RES_UNKNOWN, -1 },                  /* "q"        */
      { PROTO_RES_NONE, -1 },                     /* ""         */
      { PROTO_RES_NONE, -1 }                      /* "   "      */
    };

    for (int i = 0; i < 11; i++) {
      resetInputs();
      struct Joints j0 = snap();
      struct PosRecSnapshot r0 = snapRec();

      int result = protoHandleLine(testCases[i]);

      struct Joints j1 = snap();
      struct PosRecSnapshot r1 = snapRec();

      bool stateUnchanged = (j0.b == j1.b && j0.r == j1.r && j0.c == j1.c && j0.f == j1.f) &&
                           (r0.x == r1.x && r0.y == r1.y && r0.z == r1.z);

      bool resOk = (result == allowedResults[i][0]) ||
                   (allowedResults[i][1] >= 0 && result == allowedResults[i][1]);

      char allowTxt[24];
      if (allowedResults[i][1] >= 0) {
        snprintf(allowTxt, sizeof(allowTxt), "%d或%d", allowedResults[i][0], allowedResults[i][1]);
      } else {
        snprintf(allowTxt, sizeof(allowTxt), "%d", allowedResults[i][0]);
      }

      snprintf(buf, sizeof(buf), "用例\"%s\": 返回=%d(接受%s) 状态不变=%s",
               testCases[i], result, allowTxt, stateUnchanged ? "true" : "false");

      check(buf, resOk && stateUnchanged, buf);
    }
  }

  printf("=== 11) 分片到达与无换行超时（行缓冲行为） ===\n");
  {
    /* a) 分片到达
     * "x1" 单独一片不是合法指令（缺数字结尾与行尾），先缓冲；第二片 "0,y0,z40\n"
     * 接上后整行是 "x10,y0,z40" ⇒ 三个关节角应当写成 10/0/40，而不是 20/0/40：
     * 这里同时验证"分片拼接"，所以期望值按拼接后的 x=10 算。 */
    resetInputs();
    (void) recFromServo(&Pos.rec, &Pos.ser);
    mockSerialFeed("x1");
    serialProtocolLoop();  /* 'x' 不在快速派发表里，这一片只是进行缓冲，不该落地 */
    mockSerialFeed("0,y0,z40\n");
    serialProtocolLoop();

    snprintf(buf, sizeof(buf), "angle=(%.6f,%.6f,%.6f) 期望=(%.6f,%.6f,%.6f)",
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3,
             probeClampToTravel(0, 10.0), probeClampToTravel(1, 0.0), probeClampToTravel(2, 40.0));
    check("分片到达: \"x1\"+\"0,y0,z40\\n\" 拼成 x10 -> 三个角度写成 10/0/40",
          fabs(Pos.ser.angle1 - probeClampToTravel(0, 10.0)) < 1e-9 &&
          fabs(Pos.ser.angle2 - probeClampToTravel(1,  0.0)) < 1e-9 &&
          fabs(Pos.ser.angle3 - probeClampToTravel(2, 40.0)) < 1e-9, buf);

    /* b) 无换行 + 超时
     * 必须调用两次 serialProtocolLoop()：固件是在"读到字符的那一刻"记
     * s_lastCharMs = millis()，然后在本轮循环末尾才判超时。若把时间推进和
     * 第二次喂字符合并成一次调用，读字符时已经把 millis() 抬到了推进后的值，
     * 末尾相减恒为 0，超时永远不触发（探针自己写错，不是固件问题）。
     * 正确节奏：第 1 次调用把字符收进行缓冲（记下时间戳），再推进时钟，
     * 第 2 次调用没有新字符可读，末尾的 (millis()-s_lastCharMs) 才 >= 300。 */
    resetInputs();
    (void) recFromServo(&Pos.rec, &Pos.ser);
    mockSerialFeed("x20,y0,z40\n");        /* 先写一组基准 */
    serialProtocolLoop();

    mockSerialFeed("x12.5,y0,z40");        /* 不带换行 */
    serialProtocolLoop();                  /* 收字符：s_lastCharMs = 当前 g_mockMillis */
    g_mockMillis += 400;                   /* 超过 PROTO_LINE_TIMEOUT_MS = 300 */
    serialProtocolLoop();                  /* 无字符可读，末尾判断超时并派发整行 */

    snprintf(buf, sizeof(buf), "angle=(%.6f,%.6f,%.6f) 期望=(%.6f,%.6f,%.6f)",
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3,
             probeClampToTravel(0, 12.5), probeClampToTravel(1, 0.0), probeClampToTravel(2, 40.0));
    check("无换行+400ms超时(两次serialProtocolLoop): \"x12.5,y0,z40\" -> 角度写成 12.5/0/40",
          fabs(Pos.ser.angle1 - probeClampToTravel(0, 12.5)) < 1e-9 &&
          fabs(Pos.ser.angle2 - probeClampToTravel(1,   0.0)) < 1e-9 &&
          fabs(Pos.ser.angle3 - probeClampToTravel(2,  40.0)) < 1e-9, buf);

    /* c) 超长行丢弃
     * mock 的串口输入缓冲是固定 64 字节、只追加不回退（读空后 inLen 也不回退），
     * 所以先 mockSerialClear()，否则测的是"mock 缓冲写满"而不是固件的"超长行丢弃"。
     * 超长行用 39 个字节先填满行缓冲（PROTO_LINE_BUF_SIZE-1），前 10 个字节
     * 是一整条合法角度指令 "x20,y0,z40"，后面补空格 —— 空格在语法里会被跳过，
     * 所以只要固件把"填满缓冲的那一行"冲出去，就会真的把 angle1 写成 20。
     * 基准点故意取 x=12.5，与超长行前缀不同，误冲出去一定会被看出来。 */
    resetInputs();
    mockSerialClear();
    (void) recFromServo(&Pos.rec, &Pos.ser);
    mockSerialFeed("x12.5,y0,z40\n");      /* 基准点 */
    serialProtocolLoop();
    struct Joints jKeep = snap();

    char tooLong[64];
    strcpy(tooLong, "x20,y0,z40");                 /* 10 字节的合法前缀 */
    int tl = (int) strlen(tooLong);
    while (tl < PROTO_LINE_BUF_SIZE - 1) {         /* 补空格填满行缓冲 (39) */
      tooLong[tl++] = ' ';
    }
    tooLong[tl++] = 'B';                           /* 第 40 个字节：触发溢出丢行 */
    tooLong[tl] = '\0';
    printf("  （超长行长度 %d 字节：10 字节合法前缀 + %d 空格 + 'B'）\n",
           (int) strlen(tooLong), PROTO_LINE_BUF_SIZE - 12);

    mockSerialFeed(tooLong);
    serialProtocolLoop();   /* 触发超长：s_dropUntilEol = true */
    mockSerialFeed("\n");
    serialProtocolLoop();   /* 行尾把 s_dropUntilEol 清掉 */

    struct Joints jNow = snap();
    snprintf(buf, sizeof(buf), "超长行后 angle=(%.4f,%.4f,%.4f) 丢弃前=(%.4f,%.4f,%.4f)",
             jNow.b, jNow.r, jNow.c, jKeep.b, jKeep.r, jKeep.c);
    check("超长行丢弃: 40字节超长行(前缀是合法角度指令 x20,y0,z40)被整行丢掉，角度未动",
          jNow.b == jKeep.b && jNow.r == jKeep.r && jNow.c == jKeep.c && jNow.f == jKeep.f, buf);

    mockSerialClear();      /* 清空 mock 输入缓冲（固件的丢行状态已在上一步清掉） */
    mockSerialFeed("x20,y0,z40\n");
    serialProtocolLoop();

    snprintf(buf, sizeof(buf), "超长行丢弃后: angle=(%.4f,%.4f,%.4f) 期望=(%.4f,%.4f,%.4f)",
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3,
             probeClampToTravel(0, 20.0), probeClampToTravel(1, 0.0), probeClampToTravel(2, 40.0));
    check("超长行丢弃: 丢完并清空缓冲后，再\"x20,y0,z40\\n\" 正常落地",
          fabs(Pos.ser.angle1 - probeClampToTravel(0, 20.0)) < 1e-9 &&
          fabs(Pos.ser.angle2 - probeClampToTravel(1,  0.0)) < 1e-9 &&
          fabs(Pos.ser.angle3 - probeClampToTravel(2, 40.0)) < 1e-9, buf);
  }

  printf("=== 12) 串口应答与运行时功能开关 ===\n");
  {
    /* 这一段必须在压力测试之前跑：压力测试会随机发出 A/B/C 启动取放序列，
     * 之后 pickPlaceIsBusy() 一直是 true，而 R 只有在空闲时才可能回 OK。
     * 本段最后以"结束录制"收尾，不会把忙碌状态留给压力测试。 */
    resetInputs();
    mockSerialClear();
    serialProtocolBegin();
    Serial.clearOutput();
    check("运行时功能开关可查询", protoHandleLine("!") == PROTO_RES_NONE &&
          Serial.getOutput() == "P=1 B=1 D=1\n", Serial.getOutput().c_str());
    check("编译内功能可切换后恢复", protoHandleLine("!P") == PROTO_RES_FEATURE &&
          protoHandleLine("!P") == PROTO_RES_FEATURE, "!P toggles then toggles back");

    /* 运行时关掉按键模块：同一个 N 从含混的 REJECTED 变成明确的 OFF。 */
    Serial.clearOutput();
    (void)protoHandleLine("!B");
    int nOff = protoHandleLine("N");
    std::string offOut = Serial.getOutput();
    check("运行时关掉按键后 N 回 OFF（不再是含混的 REJECTED）",
          nOff == PROTO_RES_UNKNOWN && offOut.find("OFF\n") != std::string::npos &&
          offOut.find("REJECTED") == std::string::npos, offOut.c_str());
    check("compiled button feature toggles back on", protoHandleLine("!B") == PROTO_RES_FEATURE, "!B toggles back");
    check("compiled draw feature toggles off/on", protoHandleLine("!D") == PROTO_RES_FEATURE &&
          protoHandleLine("!D") == PROTO_RES_FEATURE, "!D toggles twice");
    Serial.clearOutput();
    check("speed command responds", protoHandleLine("H") == PROTO_RES_SPEED_UP &&
          Serial.getOutput().find("OK\n") != std::string::npos, Serial.getOutput().c_str());

    /* F 换任务必须回报"当前图形模式"（题目要求 5 个内置图形 / 2 个示教任务可循环）：
     * 回复形如 "OK F=TRI"，标签就是 drawTaskTag() 给出的 ASCII 短名。 */
    (void) drawSelectTask(DRAW_TASK_LINE);
    for (int t = 0; t < DRAW_TASK_COUNT; t++) {
      Serial.clearOutput();
      int fRes = protoHandleLine("F");
      std::string fOut = Serial.getOutput();
      int nowTask = drawGetTask();
      int wantTask = (t + 1) % DRAW_TASK_COUNT;
      char wantTag[32];
      snprintf(wantTag, sizeof(wantTag), "OK F=%s\n", drawTaskTag(wantTask));
      snprintf(buf, sizeof(buf), "\"F\" -> 任务 %d(%s) 回复=%s",
               nowTask, drawTaskTag(nowTask), fOut.c_str());
      check("F 换任务回 OK F=<模式>（依次覆盖 7 个任务）",
            fRes == PROTO_RES_DRAW_TASK_SELECTED && nowTask == wantTask &&
            fOut.find(wantTag) != std::string::npos, buf);
    }

    /* R 的失败原因不再被压成一个 REJECTED：开始录制回 OK；录制期间动作指令
     * （'O' 不在"忙碌时仍放行"的表里）回 BUSY；随后立刻结束录制，数据不合格
     * 回 DISCARD；还没有录制数据时 P 回 EMPTY。 */
    Serial.clearOutput();
    int r1 = protoHandleLine("R");
    std::string r1out = Serial.getOutput();
    Serial.clearOutput();
    int oBusy = protoHandleLine("O");
    std::string oBusyOut = Serial.getOutput();
    Serial.clearOutput();
    int r2 = protoHandleLine("R");
    std::string r2out = Serial.getOutput();
    Serial.clearOutput();
    int p1 = protoHandleLine("P");
    std::string p1out = Serial.getOutput();
    /* 注意：本探针用 WEARM_DEBUG_SERIAL=1 编译，mock 的输出流里既有协议回复
     * 也有中文调试日志，所以这里只能查"回复这一行在不在"，不能整串相等。 */
    check("R 开始录制回 OK", r1 == PROTO_RES_REC_STARTED &&
          r1out.find("OK\n") != std::string::npos, r1out.c_str());
    check("录制期间动作指令回 BUSY", oBusy == PROTO_RES_BUSY && oBusyOut == "BUSY\n",
          oBusyOut.c_str());
    check("R 立刻结束时数据不合格回 DISCARD", r2 == PROTO_RES_REC_REJECTED &&
          r2out.find("DISCARD\n") != std::string::npos, r2out.c_str());
    check("没有录制数据时 P 回 EMPTY", p1 == PROTO_RES_PLAY_NO_RECORD &&
          p1out.find("EMPTY\n") != std::string::npos, p1out.c_str());
  }

  printf("=== 13) 随机压力测试 ===\n");
  {
    resetInputs();
    adjustSpeed(SPEED_SLOW);
    /* 先建立自洽基线：resetInputs() 是探针直接改 Pos.ser 的，不走固件的写入路径，
     * 所以不会刷新派生坐标 Pos.rec。INV7 判的是 "Pos.rec == recFromServo(Pos.ser)"，
     * 不在循环前同步一次，前几轮"不写角度"的用例就会拿上一段留下的陈旧 Pos.rec 去比，
     * 凭空报 INV7 —— 这不是固件缺陷，是探针自己的基线没对齐。 */
    (void) recFromServo(&Pos.rec, &Pos.ser);

    /* 不变量清单：编号 -> 一句话判据。日志里的每条反例都按这些名字报出来。 */
    const unsigned invBits[8] = { INV_FINITE, INV_A1_RANGE, INV_A2_RANGE, INV_A3_RANGE,
                                  INV_A4_RANGE, INV_REC_OK, INV_REC_MATCH, INV_ANGLE_LAND };
    const char *invNames[8] = {
      "INV1 四个关节角都必须是有限数(非 NaN/Inf)",
      "INV2 angle1(b 基座) 必须落在 0.0..maxB 内",
      "INV3 angle2(r 上臂) 必须落在 0.0..maxR 内",
      "INV4 angle3(c 下臂) 必须落在 0.0..maxC 内",
      "INV5 angle4(f 末端) 必须落在 60.0..maxF 内",
      "INV6 recFromServo(&tmp,&Pos.ser) 必须成功返回 true",
      "INV7 Pos.rec 必须等于 recFromServo(&tmp,&Pos.ser)（逐分量差 < 1e-9）",
      "INV8 角度指令要么逐轴落地到（按 servoLimit 夹取）的结果、未提到的关节不动，要么被拒且原封不动"
    };

    /* 最多留 5 条反例的现场 */
    #define PROBE_MAX_EXAMPLES 5
    struct CounterExample {
      int    round;                 /* 第几轮（从 0 起） */
      int    rc;                    /* protoHandleLine() 的返回值 */
      char   input[40];             /* 这一轮喂进去的完整字符串 */
      struct Joints before, after;  /* 调用前后四个角度 b/r/c/f */
      struct PosRecSnapshot recBefore, recAfter;  /* 调用前后 Pos.rec x/y/z */
      unsigned badFlags;            /* 违反了哪几条不变量（位掩码） */
      double recErr;                /* INV7 实测差值；不涉及则为 -1 */
      bool   recOk;                 /* recFromServo() 的返回值 */
    };
    struct CounterExample examples[PROBE_MAX_EXAMPLES];
    int exampleCount = 0;
    int exampleSkipped = 0;         /* 超过 5 条之后只计数不再留现场 */

    /* 简单的线性同余伪随机数生成器 */
    unsigned long seed = 987654321UL;
    int violations = 0;

    for (int i = 0; i < 3000; i++) {
      seed = seed * 1103515245UL + 12345UL;

      /* 随机选择测试类型 */
      int testType = (int)((seed >> 16) % 4UL);

      /* 先把调用前的现场拍下来，并在下面各分支里填 input[] */
      struct Joints before = snap();
      struct PosRecSnapshot recBefore = snapRec();
      char input[40];
      input[0] = '\0';
      bool isAngle = false;
      double angWant[3] = { 0.0, 0.0, 0.0 };   /* 期望角度（只对提到过的轴有意义） */
      bool   angSaw[3]  = { false, false, false };
      int rc = 0;

      if (testType == 0) {
        /* 随机角度指令（题目语义：x/y/z 就是三个舵机的角度）：
         * 1~3 个轴，轴字母从 x/y/z 顺次取，值故意跨出行程（-40..220 覆盖 0..180 两侧），
         * 期望值就是固件自己那套夹取规则算出来的角度（probeClampToTravel）。 */
        /* 值取"整十分之一度"，保证 snprintf("%.1f") 打出来的数字与期望用的
         * double 是同一个数（随机浮点直接格式化会在第四位小数处四舍五入，
         * 造成"期望 48.14 / 指令 y48.1"的假失败）。 */
        const int randSteps = (int)((PROBE_RAND_HI - PROBE_RAND_LO) * 10.0 + 0.5);  /* 0.1 度一档 */
        double v[3];
        v[0] = PROBE_RAND_LO + (double)((seed >> 4) % (unsigned long)randSteps) / 10.0;
        v[1] = PROBE_RAND_LO + (double)((seed >> 6) % (unsigned long)randSteps) / 10.0;
        v[2] = PROBE_RAND_LO + (double)((seed >> 8) % (unsigned long)randSteps) / 10.0;
        int nAxes = 1 + (int)((seed >> 10) % 3UL);
        int first = (int)((seed >> 12) % 3UL);

        char *w = input;
        size_t left = sizeof(input);
        int used = 0;
        for (int k = 0; k < 3 && used < nAxes; k++) {
          int a = (first + k) % 3;
          angWant[a] = probeClampToTravel(a, v[a]);
          angSaw[a]  = true;
          int len = snprintf(w, left, "%s%c%.1f", (used == 0) ? "" : ",", protoAxisChar[a], v[a]);
          if (len <= 0 || (size_t)len >= left) break;
          w += len;
          left -= (size_t)len;
          used++;
        }
        isAngle = true;
        rc = protoHandleLine(input);
      } else if (testType == 1) {
        /* 随机语法错误 */
        int errType = (int)((seed >> 10) % 5UL);
        switch (errType) {
          case 0: snprintf(input, sizeof(input), "x10,,y20"); break;
          case 1: snprintf(input, sizeof(input), "x10y20"); break;
          case 2: snprintf(input, sizeof(input), "x"); break;
          case 3: snprintf(input, sizeof(input), "y45,"); break;
          case 4: snprintf(input, sizeof(input), ",x10"); break;
        }
        rc = protoHandleLine(input);
      } else if (testType == 2) {
        /* 随机单字符命令 */
        input[0] = (char)('A' + (int)((seed >> 8) % 26UL));
        input[1] = '\0';
        rc = protoHandleLine(input);
      } else {
        /* 随机垃圾字符串 */
        int len = 1 + (int)((seed >> 6) % 10UL);
        for (int j = 0; j < len && j < 31; j++) {
          input[j] = (char)('!' + (int)((seed >> (j + 4)) % 94UL));
        }
        input[len] = '\0';
        rc = protoHandleLine(input);
      }

      struct Joints after = snap();
      struct PosRecSnapshot recAfter = snapRec();

      /* ---- 逐条判定不变量，先只置位，最后按位统计（保持原来的计数口径：
       *      每条不变量失败各计 1，所以一轮可能 >1） ---- */
      unsigned badFlags = 0;
      if (!(after.b == after.b) || !(after.r == after.r) ||
          !(after.c == after.c) || !(after.f == after.f)) {
        badFlags |= INV_FINITE;                       /* INV1 */
      }
      if (after.b < 0.0 - 1e-6 || after.b > 180.0 + 1e-6) badFlags |= INV_A1_RANGE;  /* INV2 */
      if (after.r < 0.0 - 1e-6 || after.r > 180.0 + 1e-6) badFlags |= INV_A2_RANGE;  /* INV3 */
      if (after.c < 0.0 - 1e-6 || after.c > 180.0 + 1e-6) badFlags |= INV_A3_RANGE;  /* INV4 */
      if (after.f < 60.0 - 1e-6 || after.f > 150.0 + 1e-6) badFlags |= INV_A4_RANGE;  /* INV5 */

      REC tmp;
      double recErr = -1.0;
      bool ok = recFromServo(&tmp, &Pos.ser);
      if (!ok) {
        badFlags |= INV_REC_OK;                       /* INV6 */
      } else {
        recErr = sqrt(pow(tmp.x - Pos.rec.x, 2) + pow(tmp.y - Pos.rec.y, 2) +
                      pow(tmp.z - Pos.rec.z, 2));
        if (recErr >= 1e-9) badFlags |= INV_REC_MATCH;/* INV7 */
      }

      /* INV8：角度指令要么逐轴落地到"按 servoLimit 夹取"的值（未提到的关节一个字节都不动），
       * 要么被拒（忙守卫 BUSY 等）且整条一个字节都不动 —— 不存在第三种结果。 */
      if (isAngle) {
        bool okLand;
        if (rc == PROTO_RES_ANGLES_SET) {
          const double afterAng[3]  = { after.b, after.r, after.c };
          const double beforeAng[3] = { before.b, before.r, before.c };
          okLand = (after.f == before.f);
          for (int a = 0; a < 3; a++) {
            if (angSaw[a]) {
              if (fabs(afterAng[a] - angWant[a]) > 1e-9) okLand = false;
            } else if (afterAng[a] != beforeAng[a]) {
              okLand = false;
            }
          }
        } else {
          okLand = (before.b == after.b && before.r == after.r &&
                    before.c == after.c && before.f == after.f);
        }
        if (!okLand) badFlags |= INV_ANGLE_LAND;
      }

      for (int b = 0; b < 8; b++) {
        if (badFlags & invBits[b]) violations++;
      }

      /* ---- 留现场：前 5 条反例打印完整输入 + 前后角度/坐标 + 违反了哪条 ---- */
      if (badFlags != 0) {
        if (exampleCount < PROBE_MAX_EXAMPLES) {
          struct CounterExample *e = &examples[exampleCount];
          e->round = i;
          e->rc = rc;
          snprintf(e->input, sizeof(e->input), "%s", input);
          e->before = before;
          e->after = after;
          e->recBefore = recBefore;
          e->recAfter = recAfter;
          e->badFlags = badFlags;
          e->recErr = recErr;
          e->recOk = ok;
          exampleCount++;
        } else {
          exampleSkipped++;
        }
      }
    }

    for (int k = 0; k < exampleCount; k++) {
      const struct CounterExample *e = &examples[k];
      printf("  --- 反例 #%d（第 %d 轮，下标从 0 起）---\n", k + 1, e->round);
      printf("      这一轮输入串: \"%s\"（返回 %d）\n", e->input, e->rc);
      printf("      调用前 角度: b=%.10g r=%.10g c=%.10g f=%.10g | Pos.rec=(%.10g, %.10g, %.10g)\n",
             e->before.b, e->before.r, e->before.c, e->before.f,
             e->recBefore.x, e->recBefore.y, e->recBefore.z);
      printf("      调用后 角度: b=%.10g r=%.10g c=%.10g f=%.10g | Pos.rec=(%.10g, %.10g, %.10g)\n",
             e->after.b, e->after.r, e->after.c, e->after.f,
             e->recAfter.x, e->recAfter.y, e->recAfter.z);
      printf("      违反的不变量（%s）:\n", e->badFlags != 0 ? "逐条列出" : "无");
      for (int b = 0; b < 8; b++) {
        if (e->badFlags & invBits[b]) {
          printf("        - %s\n", invNames[b]);
        }
      }
      if (e->badFlags & INV_REC_OK) {
        printf("        (INV6 现场: recFromServo 返回 %s，本次没有可用的 rec 差值)\n",
               e->recOk ? "true" : "false");
      }
      if (e->badFlags & INV_REC_MATCH) {
        printf("        (INV7 现场: recFromServo 与 Pos.rec 的欧氏差 = %.17g)\n", e->recErr);
      }
    }
    printf("  反例现场已留 %d 条，超出上限未留现场的反例有 %d 条\n", exampleCount, exampleSkipped);
    #undef PROBE_MAX_EXAMPLES

    snprintf(buf, sizeof(buf), "violations=%d 反例已留%d条 末态 b=%.1f r=%.1f c=%.1f f=%.1f (%.1f,%.1f,%.1f)",
             violations, exampleCount, Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3, Pos.ser.angle4,
             Pos.rec.x, Pos.rec.y, Pos.rec.z);
    check("3000轮随机压力测试：零越界/零NaN/坐标与角度始终自洽", violations == 0, buf);
  }

  printf("\n>>> %s (失败 %d 项)\n", failures == 0 ? "ALL PASS" : "HAS FAILURES", failures);
  return failures == 0 ? 0 : 1;
}
