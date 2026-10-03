/*
 * probe_pins_off.cpp -- 默认配置（WEARM_BUTTON_PINS=0）的回归探针
 *
 * 【测什么】"物理按键完全不读、四个功能全部改由串口触发"这条实机修复。
 *   实机 bug：右摇杆推到最前会机械压合摇杆自带的按压开关（KY-023 的 SW 脚），
 *   那根线接在 D2 = 按键1 = 循环取放，于是"推下臂"变成"触发自动取放"，
 *   摇杆随即被 locked 锁住。修法是编译期开关 WEARM_BUTTON_PINS 默认 0：
 *   buttonSetup() 不做 pinMode、buttonLoop() 不扫描、btnEdge()/BTN_PIN[]/
 *   三个消抖数组整段裁掉；四个功能全部走串口 N / R / P / M（或 0）。
 *
 * 【为什么单独一个探针】这是编译期开关，一个可执行文件只能验一条路径：
 *   probe_button / probe_draw 用 -DWEARM_BUTTON_PINS=1 测物理按键逻辑，
 *   本探针刻意用与出厂固件一致的 0 编译（run_all.ps1 的 $pinOff），
 *   直接验证"按住 D2~D5 什么都不会发生"以及"串口那四条命令照样好用"。
 *
 * 【约定（与其它探针一致）】
 *   - 直接链接固件，绝不自己重写运动学/限位。
 *   - 必须先 posInit()：限位映射表与 Pos 都在它里面初始化。
 *   - 时间用 g_mockMillis；按键脚电平写 g_mockDigital[2..5]（LOW = 按下）；
 *     摇杆写 g_mockAnalog[0..3]（0=A0 … 3=A3，A3 = 右摇杆 Y = 下臂 c）。
 *   - 串口用 mockSerialFeed() + serialProtocolLoop()，回复读 Serial.getOutput()。
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
#include "draw_control.h"
#include "button_control.h"
#include "weArm_config.h"

static int failures = 0;

static void check(const char *name, bool ok, const char *detail) {
  printf("  %-52s %s  %s\n", name, ok ? "PASS" : "FAIL", detail);
  if (!ok) failures++;
}

/* 四个按键脚（与 button_control.cpp 的 BTN_PIN_* 一致；本探针只把它们拉到低电平） */
static const int KEY_PIN[4] = { 2, 3, 4, 5 };

static void centerSticks(void) {
  for (int i = 0; i < 4; i++) g_mockAnalog[i] = 512;
}

/* 一次"真实 loop 周期"：串口 -> 按键 -> 摇杆 -> 取放 -> 绘图 */
static void loopOnce(unsigned long dtMs) {
  serialProtocolLoop();
  buttonLoop();
  joystickLoop();
  pickPlaceLoop();
  drawLoop();
  g_mockMillis += dtMs;
}

static void runLoops(unsigned long ms) {
  unsigned long target = g_mockMillis + ms;
  while (g_mockMillis < target) loopOnce(5UL);
}

static long runUntilIdle(long cap) {
  long n = 0;
  while ((pickPlaceIsBusy() || buttonControlBusy() || drawControlBusy()) && n < cap) {
    loopOnce(20UL);
    n++;
  }
  return n;
}

/* 串口发一条命令并回读回复。
 * 【必须带 \n】N/R/P/M/0 都不在固件的"单字符快速派发表"里（s_fastBits 只有
 *   1/2/3、B/C/D、H/K/L/O、S、k 这些立即派发），所以它们要等到行尾或 400ms
 *   行超时才落地 —— 只喂一个字符又不推进时间，命令会一直躺在缓冲里。 */
static void feed(const char *cmd) {
  char buf[8];
  snprintf(buf, sizeof(buf), "%s\n", cmd);
  Serial.clearOutput();
  mockSerialFeed(buf);
  serialProtocolLoop();
}

/* 回复原文（换行换成空格，方便塞进 check 的 detail 里打印） */
static std::string reply(void) {
  std::string s = Serial.getOutput();
  for (size_t i = 0; i < s.size(); i++) {
    if (s[i] == '\n' || s[i] == '\r') s[i] = ' ';
  }
  return s;
}

static bool replyHas(const char *text) {
  return Serial.getOutput().find(text) != std::string::npos;
}

/* 四个脚一起按住/松开 */
static void holdKeys(int level) {
  for (int k = 0; k < 4; k++) g_mockDigital[KEY_PIN[k]] = level;
}

/* ================================================================= */

int main(void) {
  char d[192];

  /* ---------------- 0) 编译开关自检 ---------------- */
  printf("\n[0] 编译开关\n");
  check("本探针用 WEARM_BUTTON_PINS=0 编译（否则本探针没在测该路径）",
        WEARM_BUTTON_PINS == 0, "WEARM_BUTTON_PINS=0");

  posInit();          /* 必须先做：限位映射表与 Pos 都在里面初始化 */
  joystickSetup();
  buttonSetup();
  drawSetup();
  centerSticks();
  serialProtocolBegin();

  /* ---------------- 1) 初始化完全不碰 D2~D5 ---------------- */
  printf("\n[1] 初始化不碰按键引脚\n");
  {
    bool untouched = true;
    for (int k = 0; k < 4; k++) {
      if (g_mockPinMode[KEY_PIN[k]] != INPUT) untouched = false;
    }
    snprintf(d, sizeof(d), "D2~D5 = %d/%d/%d/%d（INPUT=%d，未被 pinMode 改动）",
             g_mockPinMode[2], g_mockPinMode[3], g_mockPinMode[4], g_mockPinMode[5],
             INPUT);
    check("buttonSetup() 之后 D2~D5 仍是 INPUT（没做 pinMode）", untouched, d);
    check("启动日志说明不读 D2~D5", replyHas("不读 D2~D5"),
          "Serial 含 \"不读 D2~D5\"");
    check("初始空闲、不录制、不回放",
          !buttonControlBusy() && !buttonIsRecording() && !buttonPlaybackActive() &&
          strcmp(buttonStateName(), "空闲") == 0, buttonStateName());
  }

  /* ---------------- 2) 四个脚全按住 1 秒：必须什么都不发生 ---------------- */
  printf("\n[2] 把 D2~D5 全部按住（模拟摇杆 SW 被机械压合）\n");
  {
    Serial.clearOutput();
    holdKeys(LOW);
    runLoops(1000);
    check("按住 1 秒：没有启动取放序列", !pickPlaceIsBusy(), pickPlaceStageName());
    check("按住 1 秒：没有开始录制", !buttonIsRecording(), "recording=false");
    check("按住 1 秒：模块仍空闲、循环序号没推进",
          !buttonControlBusy() && buttonPickNext() == PICK_OBJECT_A, "next=A");
    check("按住 1 秒：串口没有任何回复（没有 OK）", !replyHas("OK"),
          "Serial 不含 \"OK\"");

    holdKeys(HIGH);
    runLoops(200);
    check("松开后也不会补一次动作（不认边沿）",
          !pickPlaceIsBusy() && !buttonControlBusy() &&
          buttonPickNext() == PICK_OBJECT_A, "next=A busy=false");
  }

  /* ---------------- 3) 右摇杆推到最前：下臂照常动，不触发取放 ---------------- */
  printf("\n[3] 右摇杆推到最前（用户报的那个操作）\n");
  {
    centerSticks();
    runLoops(100);
    double c0 = Pos.ser.angle3;             /* A3 -> 下臂 c (angle3) */
    Serial.clearOutput();
    g_mockAnalog[3] = 1023;                 /* 右摇杆 Y 推到底 */
    runLoops(900);
    double dc = Pos.ser.angle3 - c0;
    snprintf(d, sizeof(d), "angle3 %.2f -> %.2f (Δ%.2f)", c0, Pos.ser.angle3, dc);
    check("下臂照常动起来", fabs(dc) > 0.5, d);
    check("没有顺手触发取放", !pickPlaceIsBusy(), pickPlaceStageName());
    check("没有串口回复（既没回 OK 也没回别的）", !replyHas("OK"),
          "Serial 不含 \"OK\"");
    centerSticks();
    runLoops(100);
  }

  /* ---------------- 4) 四个功能全部由串口触发 ---------------- */
  printf("\n[4] 功能改由串口字符触发\n");
  {
    feed("N");
    check("串口 N -> 开始夹 A", pickPlaceIsBusy() && pickPlaceCurrentObject() == PICK_OBJECT_A,
          pickPlaceStageName());
    check("串口 N -> 回 OK", replyHas("OK"), reply().c_str());
    snprintf(d, sizeof(d), "已跑 %ld 轮", runUntilIdle(300000));
    check("序列跑完回到空闲", !pickPlaceIsBusy(), d);

    feed("R");
    check("串口 R -> 进入录制", buttonIsRecording(), buttonStateName());
    check("串口 R -> 回 OK", replyHas("OK"), reply().c_str());
    runLoops(300);                          /* 完全没动：位移 0，判废（时长不再是门槛） */
    feed("R");
    check("串口 R 再发一次 -> 结束录制，没有位移回 DISCARD",
          !buttonIsRecording() && replyHas("DISCARD"), reply().c_str());
    check("被丢弃后没有留下录制数据", !buttonHasRecording(), "hasRecording=false");

    feed("P");
    check("没有录制时串口 P -> 回 EMPTY",
          !buttonPlaybackActive() && replyHas("EMPTY"), reply().c_str());

    feed("M");
    check("串口 M -> 开始回中", buttonControlBusy(), buttonStateName());
    check("串口 M -> 回 OK", replyHas("OK"), reply().c_str());
    snprintf(d, sizeof(d), "已跑 %ld 轮", runUntilIdle(300000));
    check("回中结束回到空闲", !buttonControlBusy(), d);
    SER home;
    bool haveHome = posGetHomeAngles(&home);
    check("回到开机初始位姿（b/r/c 与 posGetHomeAngles() 一致）",
          haveHome &&
          fabs(Pos.ser.angle1 - home.angle1) < 0.01 &&
          fabs(Pos.ser.angle2 - home.angle2) < 0.01 &&
          fabs(Pos.ser.angle3 - home.angle3) < 0.01, "Δ<0.01");

    feed("0");
    check("串口 0（按键4 的别名）-> 同样开始回中", buttonControlBusy(), buttonStateName());
    check("串口 0 -> 回 OK", replyHas("OK"), reply().c_str());
    (void) runUntilIdle(300000);
  }

  printf("\n");
  if (failures == 0) {
    printf("ALL PASS\n");
    return 0;
  }
  printf("%d 项失败\n", failures);
  return 1;
}
