/* probe_protocol.cpp —— 串口协议模块端到端回归测试
 * 直接链接固件：serial_protocol.cpp + constant_and_positions.cpp + move.cpp + protocol_constants.cpp
 *
 * 仿真方式：用 mockSerialFeed() 喂串口命令，推进 g_mockMillis 走时间门控，
 *           用 serialProtocolLoop() 处理串口数据流，用 protoHandleLine() 直接处理单行。
 *
 * 本探针专门测试 serial_protocol.cpp 模块的 12 个核心功能：
 * 1) 固定指令 O/S
 * 2) 兼容命令 1/2/3
 * 3) H/L 档位升降
 * 4) k/K 步进与到限不越界
 * 5) x/y/z 三舵机同步
 * 6) 只写一部分轴
 * 7) 大小写与空白容忍
 * 8) 小数与可选等号
 * 9) 越界夹取
 * 10) 错误输入不改变状态
 * 11) 分片到达与无换行超时
 * 12) 随机压力测试
 */
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include "Arduino.h"
#include "constant_and_positions.h"
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
  for (int i = 0; i < 4; i++) {
    Pos.ser.angle1 = (servoLimit.minB + servoLimit.maxB) / 2;
    Pos.ser.angle2 = (servoLimit.minR + servoLimit.maxR) / 2;
    Pos.ser.angle3 = (servoLimit.minC + servoLimit.maxC) / 2;
    Pos.ser.angle4 = (servoLimit.minF + servoLimit.maxF) / 2;
  }
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

/* 现算某个轴（0=x/b, 1=y/r, 2=z/c）在 servoLimit 行程内的落点。
 * 探针不写死角度边界：行程真值只有 servoLimit 一处，改行程不该让探针失效。
 * 夹取规则与固件 protoApplyAngles() 一致（先取 protoAxisGetLimit 再夹）。 */
static double probeExpectAxis(int axis, double value) {
  double lo = 0.0, hi = 0.0;
  if (!protoAxisGetLimit(axis, &lo, &hi)) return value;
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

/* 压力测试用的"名义角度区间"：只用来生成随机指令。
 * 注意 protocol_constants.h 明确不定义 PROTO_ANGLE_MIN/MAX（角度合法性的真值
 * 只有 servoLimit 一处），所以这里用探针自己的局部常量。 */
static const double PROBE_RAND_ANGLE_MIN = 0.0;
static const double PROBE_RAND_ANGLE_MAX = 180.0;

/* 压力测试的每条不变量各占一位，判定与日志都对着这些名字看。
 * 命名规则：INV1..INV5 是"角度有效性"，INV6/INV7 是"派生坐标自洽"。 */
#define INV_FINITE    0x01u  /* 四个角都是有限数 */
#define INV_A1_RANGE  0x02u  /* angle1(b) 在 servoLimit 行程内 */
#define INV_A2_RANGE  0x04u  /* angle2(r) 在 servoLimit 行程内 */
#define INV_A3_RANGE  0x08u  /* angle3(c) 在 servoLimit 行程内 */
#define INV_A4_RANGE  0x10u  /* angle4(f) 在 servoLimit 行程内 */
#define INV_REC_OK    0x20u  /* recFromServo() 返回 true */
#define INV_REC_MATCH 0x40u  /* Pos.rec 与 recFromServo() 结果逐分量一致 */

int main(void) {
  (void) servoSelfCheck();
  (void) rangeClampConfig();
  resetInputs();
  g_mockMillis = 1000;
  serialProtocolBegin();
  posInit();

  char buf[260];

  printf("=== 1) 固定指令 O / S（绝对到位） ===\n");
  {
    resetInputs();
    Pos.ser.angle4 = (servoLimit.minF + servoLimit.maxF) / 2;
    mockSerialFeed("O");
    serialProtocolLoop();
    snprintf(buf, sizeof(buf), "angle4=%.1f maxF=%.1f protoRes=%d",
             Pos.ser.angle4, servoLimit.maxF, protoHandleLine("O"));
    check("mockSerialFeed(\"O\") -> angle4==servoLimit.maxF 且 protoHandleLine(\"O\")返回PROTO_RES_GRIPPER_OPEN",
          Pos.ser.angle4 == servoLimit.maxF && protoHandleLine("O") == PROTO_RES_GRIPPER_OPEN, buf);

    mockSerialFeed("S");
    serialProtocolLoop();
    snprintf(buf, sizeof(buf), "angle4=%.1f minF=%.1f protoRes=%d",
             Pos.ser.angle4, servoLimit.minF, protoHandleLine("S"));
    check("mockSerialFeed(\"S\") -> angle4==servoLimit.minF 且 protoHandleLine(\"S\")返回PROTO_RES_GRIPPER_CLOSE",
          Pos.ser.angle4 == servoLimit.minF && protoHandleLine("S") == PROTO_RES_GRIPPER_CLOSE, buf);
  }

  printf("=== 2) 兼容命令 1 / 2 / 3 ===\n");
  {
    mockSerialFeed("1");
    serialProtocolLoop();
    snprintf(buf, sizeof(buf), "stepSize=%.1f", speed.stepSize);
    check("串口 \"1\" -> speed.stepSize == 0.5", fabs(speed.stepSize - 0.5) < 1e-9, buf);

    mockSerialFeed("2");
    serialProtocolLoop();
    snprintf(buf, sizeof(buf), "stepSize=%.1f", speed.stepSize);
    check("串口 \"2\" -> speed.stepSize == 1.0", fabs(speed.stepSize - 1.0) < 1e-9, buf);

    mockSerialFeed("3");
    serialProtocolLoop();
    snprintf(buf, sizeof(buf), "stepSize=%.1f", speed.stepSize);
    check("串口 \"3\" -> speed.stepSize == 2.0", fabs(speed.stepSize - 2.0) < 1e-9, buf);
  }

  printf("=== 3) H / L 档位升降与端点行为 ===\n");
  {
    protoHandleLine("2");  /* 回中档 */
    snprintf(buf, sizeof(buf), "初始档位=%d", speedGetLevel());
    check("protoHandleLine(\"2\") 回中档", speedGetLevel() == SPEED_NORMAL, buf);

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

  printf("=== 4) k / K 步进与到限不越界 ===\n");
  {
    resetInputs();
    Pos.ser.angle4 = (servoLimit.minF + servoLimit.maxF) / 2;

    /* 连发 60 次 "k" */
    for (int i = 0; i < 60; i++) {
      protoHandleLine("k");
    }
    snprintf(buf, sizeof(buf), "60次\"k\"后 angle4=%.1f maxF=%.1f <= maxF",
             Pos.ser.angle4, servoLimit.maxF);
    check("连发60次\"k\" -> angle4==servoLimit.maxF且<=maxF",
          Pos.ser.angle4 == servoLimit.maxF && Pos.ser.angle4 <= servoLimit.maxF, buf);

    /* 连发 60 次 "K" */
    for (int i = 0; i < 60; i++) {
      protoHandleLine("K");
    }
    snprintf(buf, sizeof(buf), "60次\"K\"后 angle4=%.1f minF=%.1f >= minF",
             Pos.ser.angle4, servoLimit.minF);
    check("连发60次\"K\" -> angle4==servoLimit.minF且>=minF",
          Pos.ser.angle4 == servoLimit.minF && Pos.ser.angle4 >= servoLimit.minF, buf);
  }

  printf("=== 5) x/y/z 三舵机同步（本需求的核心） ===\n");
  {
    resetInputs();

    mockSerialFeed("x10,y30,z20\n");
    serialProtocolLoop();


    REC tmp;
    bool ok = recFromServo(&tmp, &Pos.ser);
    double recErr = ok ? sqrt(pow(tmp.x - Pos.rec.x, 2) + pow(tmp.y - Pos.rec.y, 2) +
                              pow(tmp.z - Pos.rec.z, 2)) : 1e9;

    snprintf(buf, sizeof(buf), "angle1=%.1f(期望10) angle2=%.1f(期望30) angle3=%.1f(期望20) angle4=%.1f",
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3, Pos.ser.angle4);
    check("mockSerialFeed(\"x10,y30,z20\\n\") + serialProtocolLoop() -> angle1==10且angle2==30且angle3==20",
          Pos.ser.angle1 == 10.0 && Pos.ser.angle2 == 30.0 && Pos.ser.angle3 == 20.0, buf);

    snprintf(buf, sizeof(buf), "Pos.rec与recFromServo差值=%.3g", recErr);
    check("调用完时 Pos.rec 与 recFromServo(&tmp,&Pos.ser)逐分量差 < 1e-9",
          recErr < 1e-9, buf);
  }

  printf("=== 6) 只写一部分轴 ===\n");
  {
    resetInputs();
    struct Joints j0 = snap();

    mockSerialFeed("y45\n");
    serialProtocolLoop();


    snprintf(buf, sizeof(buf), "angle1=%.1f(不变) angle2=%.1f(期望45) angle3=%.1f(不变) angle4=%.1f(不变)",
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3, Pos.ser.angle4);
    check("mockSerialFeed(\"y45\\n\") -> 只有angle2变为45，其他不变",
          Pos.ser.angle2 == 45.0 &&
          Pos.ser.angle1 == j0.b &&
          Pos.ser.angle3 == j0.c &&
          Pos.ser.angle4 == j0.f, buf);
  }

  printf("=== 7) 大小写与空白容忍 ===\n");
  {
    resetInputs();

    mockSerialFeed(" X10 , Y30 , Z20 \n");
    serialProtocolLoop();


    snprintf(buf, sizeof(buf), "angle1=%.1f angle2=%.1f angle3=%.1f",
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);
    check("mockSerialFeed(\" X10 , Y30 , Z20 \\n\") -> 得到10/30/20",
          Pos.ser.angle1 == 10.0 && Pos.ser.angle2 == 30.0 && Pos.ser.angle3 == 20.0, buf);
  }

  printf("=== 8) 小数与可选等号 ===\n");
  {
    resetInputs();

    mockSerialFeed("x12.5\n");
    serialProtocolLoop();

    snprintf(buf, sizeof(buf), "angle1=%.1f", Pos.ser.angle1);
    check("mockSerialFeed(\"x12.5\\n\") -> 得到12.5",
          Pos.ser.angle1 == 12.5, buf);

    resetInputs();
    mockSerialFeed("x=10,y=20,z=30\n");
    serialProtocolLoop();

    snprintf(buf, sizeof(buf), "angle1=%.1f angle2=%.1f angle3=%.1f",
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);
    check("mockSerialFeed(\"x=10,y=20,z=30\\n\") -> 得到10/20/30",
          Pos.ser.angle1 == 10.0 && Pos.ser.angle2 == 20.0 && Pos.ser.angle3 == 30.0, buf);
  }

  printf("=== 9) 越界夹取 ===\n");
  {
    resetInputs();

    mockSerialFeed("x200,y-30,z999\n");
    serialProtocolLoop();


    snprintf(buf, sizeof(buf), "angle1=%.1f(期望180) angle2=%.1f(期望minR=%.1f) angle3=%.1f(期望maxC=%.1f)",
             Pos.ser.angle1, Pos.ser.angle2, servoLimit.minR, Pos.ser.angle3, servoLimit.maxC);
    check("mockSerialFeed(\"x200,y-30,z999\\n\") -> 夹取到边界",
          Pos.ser.angle1 == 180.0 &&
          Pos.ser.angle2 == servoLimit.minR &&
          Pos.ser.angle3 == servoLimit.maxC, buf);

    /* 断言三个角都落在 servoLimit 各自的行程里 */
    bool inRange = (Pos.ser.angle1 >= servoLimit.minB && Pos.ser.angle1 <= servoLimit.maxB) &&
                   (Pos.ser.angle2 >= servoLimit.minR && Pos.ser.angle2 <= servoLimit.maxR) &&
                   (Pos.ser.angle3 >= servoLimit.minC && Pos.ser.angle3 <= servoLimit.maxC) &&
                   (Pos.ser.angle4 >= servoLimit.minF && Pos.ser.angle4 <= servoLimit.maxF);

    snprintf(buf, sizeof(buf), "所有角度都在servoLimit内: angle1[%.1f,%.1f] angle2[%.1f,%.1f] angle3[%.1f,%.1f] angle4[%.1f,%.1f]",
             servoLimit.minB, servoLimit.maxB, servoLimit.minR, servoLimit.maxR,
             servoLimit.minC, servoLimit.maxC, servoLimit.minF, servoLimit.maxF);
    check("越界夹取后所有角度仍在servoLimit行程内", inRange, buf);
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
    /* a) 分片到达 */
    resetInputs();
    mockSerialFeed("x1");
    serialProtocolLoop();  /* 此时不应落地，因为 "x1" 会被当单字符流程处理 */
    mockSerialFeed("0,y20,z30\n");
    serialProtocolLoop();

    snprintf(buf, sizeof(buf), "分片到达: angle1=%.1f angle2=%.1f angle3=%.1f",
             Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);
    check("分片到达: \"x1\"+\"0,y20,z30\\n\" -> 最终angle1==10 angle2==20 angle3==30",
          Pos.ser.angle1 == 10.0 && Pos.ser.angle2 == 20.0 && Pos.ser.angle3 == 30.0, buf);

    /* b) 无换行 + 超时
     * 必须调用两次 serialProtocolLoop()：固件是在"读到字符的那一刻"记
     * s_lastCharMs = millis()，然后在本轮循环末尾才判超时。若把时间推进和
     * 第二次喂字符合并成一次调用，读字符时已经把 millis() 抬到了推进后的值，
     * 末尾相减恒为 0，超时永远不触发（探针自己写错，不是固件问题）。
     * 正确节奏：第 1 次调用把字符收进行缓冲（记下时间戳），再推进时钟，
     * 第 2 次调用没有新字符可读，末尾的 (millis()-s_lastCharMs) 才 >= 300。 */
    resetInputs();
    double x33 = probeExpectAxis(0, 33.0);
    double y44 = probeExpectAxis(1, 44.0);
    mockSerialFeed("x33,y44");  /* 不带换行 */
    serialProtocolLoop();       /* 收字符：s_lastCharMs = 当前 g_mockMillis */
    g_mockMillis += 400;        /* 超过 PROTO_LINE_TIMEOUT_MS = 300 */
    serialProtocolLoop();       /* 无字符可读，末尾判断超时并派发整行 */

    snprintf(buf, sizeof(buf), "无换行超时: angle1=%.1f(期望%.1f) angle2=%.1f(期望%.1f)",
             Pos.ser.angle1, x33, Pos.ser.angle2, y44);
    check("无换行+400ms超时(两次serialProtocolLoop): \"x33,y44\" -> angle1/angle2已按行程夹取落地",
          Pos.ser.angle1 == x33 && Pos.ser.angle2 == y44, buf);

    /* c) 超长行丢弃
     * mock 的串口输入缓冲是固定 64 字节、只追加不回退（读空后 inLen 也不回退）。
     * 第一批 60 个 'A' 加换行已经占到 61 字节，如果不清空，后面的 "y55\n"
     * 会塞满 64 字节把结尾的换行丢掉 —— 那样测的是"mock 缓冲写满"，不是固件
     * 的"超长行丢弃"。所以这里先清空 mock 缓冲，并把 angle2 设回一个已知值，
     * 保证下面测的是固件丢行状态机之后的正常解析。 */
    resetInputs();
    mockSerialClear();
    /* 原用例想喂 60 个 'A'，但字面量实际是 70 个：一喂就把 mock 的 64 字节输入缓冲
     * 塞满（只追加、读空后也不回退），紧跟的那个 '\n' 根本进不去缓冲区，固件的
     * s_dropUntilEol 永远等不到行尾。这里改成程序拼一个长度精确的 40 字节超长行：
     *   - 40 > PROTO_LINE_BUF_SIZE-1(=39)，必定触发固件的丢行状态；
     *   - 40 < 64，行尾 '\n' 一定进得去，丢行状态能被正常收掉；
     *   - 前缀 "x10" 本身是合法指令，固件若把溢出前的残行冲出去 angle1 会变成 10，
     *     所以除了"下一行 y55 能落地"，还额外断言 angle1 没被动过。 */
    double bKeep = Pos.ser.angle1;
    char tooLong[64];
    memcpy(tooLong, "x10", 3);
    for (int k = 3; k < 40; k++) tooLong[k] = 'A';
    tooLong[40] = '\0';
    mockSerialFeed(tooLong);
    serialProtocolLoop();   /* 触发超长：s_dropUntilEol = true */
    mockSerialFeed("\n");
    serialProtocolLoop();   /* 行尾把 s_dropUntilEol 清掉 */

    snprintf(buf, sizeof(buf), "超长行丢弃: 40字节超长行后 angle1=%.1f (丢弃前%.1f)", Pos.ser.angle1, bKeep);
    check("超长行丢弃: 40字节超长行(前缀是合法的 x10)被整行丢掉, angle1不变", Pos.ser.angle1 == bKeep, buf);

    mockSerialClear();      /* 清空 mock 输入缓冲（固件的丢行状态已在上一步清掉） */
    double y55 = probeExpectAxis(1, 55.0);
    /* 把 angle2 设成一个与期望值不同的已知起点，避免"起点正好等于期望"的假通过 */
    double yStart = (fabs(servoLimit.minR - y55) > 1e-9) ? servoLimit.minR : servoLimit.maxR;
    Pos.ser.angle2 = yStart;

    mockSerialFeed("y55\n");
    serialProtocolLoop();

    snprintf(buf, sizeof(buf), "超长行丢弃后: angle2=%.1f (起点%.1f 期望%.1f)",
             Pos.ser.angle2, yStart, y55);
    check("超长行丢弃: 超长行丢完并清空缓冲，再\"y55\\n\" -> angle2==按行程夹取的55",
          Pos.ser.angle2 == y55, buf);
  }

  printf("=== 12) 随机压力测试 ===\n");
  {
    resetInputs();
    adjustSpeed(SPEED_SLOW);
    /* 先建立自洽基线：resetInputs() 是探针直接改 Pos.ser 的，不走固件的写入路径，
     * 所以不会刷新派生坐标 Pos.rec。INV7 判的是 "Pos.rec == recFromServo(Pos.ser)"，
     * 不在循环前同步一次，前几轮"不写角度"的用例就会拿上一段留下的陈旧 Pos.rec 去比，
     * 凭空报 INV7 —— 这不是固件缺陷，是探针自己的基线没对齐。 */
    (void) recFromServo(&Pos.rec, &Pos.ser);

    /* 不变量清单：编号 -> 一句话判据。日志里的每条反例都按这些名字报出来。 */
    const unsigned invBits[7] = { INV_FINITE, INV_A1_RANGE, INV_A2_RANGE, INV_A3_RANGE,
                                  INV_A4_RANGE, INV_REC_OK, INV_REC_MATCH };
    const char *invNames[7] = {
      "INV1 四个关节角都必须是有限数(非 NaN/Inf)",
      "INV2 angle1(b 基座) 必须落在 servoLimit.minB..maxB 内",
      "INV3 angle2(r 上臂) 必须落在 servoLimit.minR..maxR 内",
      "INV4 angle3(c 下臂) 必须落在 servoLimit.minC..maxC 内",
      "INV5 angle4(f 末端) 必须落在 servoLimit.minF..maxF 内",
      "INV6 recFromServo(&tmp,&Pos.ser) 必须成功返回 true",
      "INV7 Pos.rec 必须等于 recFromServo(&tmp,&Pos.ser)（逐分量差 < 1e-9）"
    };

    /* 最多留 5 条反例的现场 */
    #define PROBE_MAX_EXAMPLES 5
    struct CounterExample {
      int    round;                 /* 第几轮（从 0 起） */
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

      if (testType == 0) {
        /* 随机合法角度指令 */
        int axis1 = (int)((seed >> 8) % 3UL);
        double angle1 = PROBE_RAND_ANGLE_MIN + (PROBE_RAND_ANGLE_MAX - PROBE_RAND_ANGLE_MIN) * ((seed >> 4) % 1000UL) / 1000.0;
        int axis2 = (int)((seed >> 12) % 3UL);
        double angle2 = PROBE_RAND_ANGLE_MIN + (PROBE_RAND_ANGLE_MAX - PROBE_RAND_ANGLE_MIN) * ((seed >> 6) % 1000UL) / 1000.0;

        if (axis1 == axis2) {
          snprintf(input, sizeof(input), "%c%.1f", protoAxisChar[axis1], angle1);
        } else {
          snprintf(input, sizeof(input), "%c%.1f,%c%.1f", protoAxisChar[axis1], angle1, protoAxisChar[axis2], angle2);
        }

        protoHandleLine(input);
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
        protoHandleLine(input);
      } else if (testType == 2) {
        /* 随机单字符命令 */
        input[0] = (char)('A' + (int)((seed >> 8) % 26UL));
        input[1] = '\0';
        protoHandleLine(input);
      } else {
        /* 随机垃圾字符串 */
        int len = 1 + (int)((seed >> 6) % 10UL);
        for (int j = 0; j < len && j < 31; j++) {
          input[j] = (char)('!' + (int)((seed >> (j + 4)) % 94UL));
        }
        input[len] = '\0';
        protoHandleLine(input);
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
      if (after.b < servoLimit.minB - 1e-6 || after.b > servoLimit.maxB + 1e-6) badFlags |= INV_A1_RANGE;  /* INV2 */
      if (after.r < servoLimit.minR - 1e-6 || after.r > servoLimit.maxR + 1e-6) badFlags |= INV_A2_RANGE;  /* INV3 */
      if (after.c < servoLimit.minC - 1e-6 || after.c > servoLimit.maxC + 1e-6) badFlags |= INV_A3_RANGE;  /* INV4 */
      if (after.f < servoLimit.minF - 1e-6 || after.f > servoLimit.maxF + 1e-6) badFlags |= INV_A4_RANGE;  /* INV5 */

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

      for (int b = 0; b < 7; b++) {
        if (badFlags & invBits[b]) violations++;
      }

      /* ---- 留现场：前 5 条反例打印完整输入 + 前后角度/坐标 + 违反了哪条 ---- */
      if (badFlags != 0) {
        if (exampleCount < PROBE_MAX_EXAMPLES) {
          struct CounterExample *e = &examples[exampleCount];
          e->round = i;
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
      printf("      这一轮输入串: \"%s\"\n", e->input);
      printf("      调用前 角度: b=%.10g r=%.10g c=%.10g f=%.10g | Pos.rec=(%.10g, %.10g, %.10g)\n",
             e->before.b, e->before.r, e->before.c, e->before.f,
             e->recBefore.x, e->recBefore.y, e->recBefore.z);
      printf("      调用后 角度: b=%.10g r=%.10g c=%.10g f=%.10g | Pos.rec=(%.10g, %.10g, %.10g)\n",
             e->after.b, e->after.r, e->after.c, e->after.f,
             e->recAfter.x, e->recAfter.y, e->recAfter.z);
      printf("      违反的不变量（%s）:\n", e->badFlags != 0 ? "逐条列出" : "无");
      for (int b = 0; b < 7; b++) {
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