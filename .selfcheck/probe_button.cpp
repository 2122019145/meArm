/*
 * probe_button.cpp -- 四个按键（button_control.cpp）的自检探针
 *
 * 【为什么存在】按键模块里最容易出错、又最难在真机上复现的几件事：
 *   - 消抖与"只认按下沿"（真机上双击、长按都会踩）
 *   - 按键1 的循环顺序与"忙时不推进"
 *   - 录制的保存门槛（末端位移 >=10、缓冲不溢出；**最小时长限制已删除**，
 *     所以短录像只要动了就保存，见 [3]）
 *   - 录制期间摇杆必须可用、播放/回中期间摇杆必须让位
 *   - 串口忙守卫必须放行 N/R/P/M（否则录制中发 R 结束不了录制）
 * 这些都在 PC 上用 mock 时钟跑完整流程来验证。
 *
 * 【约定（与其它探针一致）】
 *   - 直接链接固件，绝不自己重写运动学/限位：反解与正解都调固件函数，
 *     限位读固件全局 servoLimit/limit。
 *   - 必须先 posInit()：限位映射表与 Pos 都在它里面初始化。
 *   - 时间是 mock 的 g_mockMillis；按键脚电平写在 g_mockDigital[2..5]。
 *   - 摇杆输入写在 g_mockAnalog[0..3]（0=A0 … 3=A3），驱动真实的 joystickLoop()。
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
#include "button_control.h"

static int failures = 0;

static void check(const char *name, bool ok, const char *detail) {
  printf("  %-52s %s  %s\n", name, ok ? "PASS" : "FAIL", detail);
  if (!ok) failures++;
}

/* ---------- 仿真工具 ---------- */

/* 按键 1~4 接 D2~D5，与 button_control.cpp 的 BTN_PIN_* 一致 */
static const int KEY_PIN[4] = { 2, 3, 4, 5 };

/* 推进 mock 时间 5ms 一步，每步都跑按键模块与摇杆模块（录制期间摇杆要真的能动） */
static void advance(unsigned long ms) {
  unsigned long target = g_mockMillis + ms;
  while (g_mockMillis < target) {
    buttonLoop();
    joystickLoop();
    g_mockMillis += 5UL;
  }
}

/* 推进时间直到"取放序列 + 按键模块"都不忙（每轮 20ms，模拟真实 loop 周期） */
static long runUntilIdle(long cap) {
  long n = 0;
  while ((pickPlaceIsBusy() || buttonControlBusy()) && n < cap) {
    pickPlaceLoop();
    buttonLoop();
    joystickLoop();
    g_mockMillis += 20UL;
    n++;
  }
  return n;
}

/* 完整按一下某个按键：按下 -> 等过消抖 -> 松手 -> 等过消抖 */
static void clickKey(int key) {
  g_mockDigital[KEY_PIN[key]] = LOW;
  advance(60);
  g_mockDigital[KEY_PIN[key]] = HIGH;
  advance(60);
}

static void centerSticks(void) {
  for (int i = 0; i < 4; i++) g_mockAnalog[i] = 512;
}

static bool sameSer(const SER *a, const SER *b) {
  return a->angle1 == b->angle1 && a->angle2 == b->angle2 &&
         a->angle3 == b->angle3 && a->angle4 == b->angle4;
}

/* ================================================================= */

int main(void) {
  char d[192];

  posInit();        /* 必须先做：限位映射表与 Pos 都在里面初始化 */
  joystickSetup();  /* 摇杆模块的引脚与内部状态；录制用例要真的推摇杆 */
  buttonSetup();    /* 按键模块 */
  centerSticks();

  /* ---------------- 1) 初始化 ---------------- */
  printf("\n[1] 按键初始化\n");
  {
    bool pinsOk = true;
    for (int k = 0; k < 4; k++) {
      if (g_mockPinMode[KEY_PIN[k]] != INPUT_PULLUP) pinsOk = false;
    }
    snprintf(d, sizeof(d), "D2~D5 = %d/%d/%d/%d (INPUT_PULLUP=%d)",
             g_mockPinMode[2], g_mockPinMode[3], g_mockPinMode[4], g_mockPinMode[5],
             INPUT_PULLUP);
    check("按键脚 D2~D5 都被设成 INPUT_PULLUP", pinsOk, d);
  }
  check("初始没有录制数据", !buttonHasRecording(), "hasRecording=false");
  check("初始不在录制", !buttonIsRecording(), "recording=false");
  check("初始不忙、不让位", !buttonControlBusy() && !buttonControlLocked(), "busy=false locked=false");
  check("初始状态名是空闲", strcmp(buttonStateName(), "空闲") == 0, buttonStateName());
  check("按键1 第一次会夹 A", buttonPickNext() == PICK_OBJECT_A, "next=A");
  check("按键编号表正确", strcmp(buttonName(BTN_KEY_CYCLE), "按键1 循环执行") == 0 &&
        strcmp(buttonName(BTN_KEY_HOME), "按键4 回中") == 0, buttonName(BTN_KEY_RECORD));
  check("N/R/P/M/0 是按键命令，O 不是",
        buttonIsCommandChar('N') && buttonIsCommandChar('R') && buttonIsCommandChar('P') &&
        buttonIsCommandChar('M') && buttonIsCommandChar('0') && !buttonIsCommandChar('O'),
        "N/R/P/M/0=true O=false");
  check("未知字符返回 PROTO_RES_UNKNOWN",
        buttonHandleCommand('Z') == PROTO_RES_UNKNOWN, "rc=8");

  /* ---------------- 2) 按键1 循环执行 A->B->C->A ---------------- */
  printf("\n[2] 按键1 循环执行（物理按键路径）\n");
  {
    clickKey(BTN_KEY_CYCLE);
    check("第 1 次按下：开始夹 A",
          pickPlaceIsBusy() && pickPlaceCurrentObject() == PICK_OBJECT_A,
          pickPlaceStageName());
    check("按下后下次序号变成 B", buttonPickNext() == PICK_OBJECT_B, "next=B");

    /* 忙的时候再按：不能插队，也不能把序号往前推 */
    clickKey(BTN_KEY_CYCLE);
    check("序列执行中再按按键1 被拒（序号不推进）",
          pickPlaceIsBusy() && buttonPickNext() == PICK_OBJECT_B, "next=B");
    snprintf(d, sizeof(d), "已跑 %ld 轮", runUntilIdle(300000));
    check("序列 1 跑完回到空闲", !pickPlaceIsBusy(), pickPlaceStageName());

    clickKey(BTN_KEY_CYCLE);
    check("第 2 次按下：开始夹 B",
          pickPlaceIsBusy() && pickPlaceCurrentObject() == PICK_OBJECT_B, pickPlaceStageName());
    check("按下后下次序号变成 C", buttonPickNext() == PICK_OBJECT_C, "next=C");
    (void) runUntilIdle(300000);

    clickKey(BTN_KEY_CYCLE);
    check("第 3 次按下：开始夹 C",
          pickPlaceIsBusy() && pickPlaceCurrentObject() == PICK_OBJECT_C, pickPlaceStageName());
    (void) runUntilIdle(300000);

    clickKey(BTN_KEY_CYCLE);
    check("第 4 次按下：序号回到 A（循环）",
          pickPlaceIsBusy() && pickPlaceCurrentObject() == PICK_OBJECT_A, pickPlaceStageName());
    (void) runUntilIdle(300000);
    check("循环结束仍空闲", !pickPlaceIsBusy() && buttonPickNext() == PICK_OBJECT_B, "next=B");
  }

  /* ---------------- 3) 录制：不达标要被拒 ---------------- */
  printf("\n[3] 录制门槛（只有位移，没有最小时长）\n");
  {
    int rc = buttonHandleCommand(PROTO_CMD_BTN_PLAY);
    check("还没有录制时按播放：回 PLAY_NO_RECORD",
          rc == PROTO_RES_PLAY_NO_RECORD && !buttonPlaybackActive(), "rc=16");

    rc = buttonHandleCommand(PROTO_CMD_BTN_RECORD);
    check("按 R 开始录制", rc == PROTO_RES_REC_STARTED && buttonIsRecording(), "rc=12");
    check("录制中 buttonControlBusy=true", buttonControlBusy(), "busy=true");
    check("录制中 buttonControlLocked=false（摇杆必须还能用）",
          !buttonControlLocked(), "locked=false");
    check("录制中状态名是录制中", strcmp(buttonStateName(), "录制中") == 0, buttonStateName());

    /* 录制中发 O：忙守卫要挡下（录制独占动作指令） */
    check("录制中发 O 被挡下（回 BUSY）",
          protoHandleLine("O") == PROTO_RES_BUSY, "rc=11");
    /* 录制中发 H：调速必须仍然有效 */
    check("录制中发 H 仍然生效（调速放行）",
          protoHandleLine("H") == PROTO_RES_SPEED_UP, "rc=3");
    check("录制中发 L 仍然生效",
          protoHandleLine("L") == PROTO_RES_SPEED_DOWN, "rc=4");

    advance(3000);
    rc = buttonHandleCommand(PROTO_CMD_BTN_RECORD);
    check("只录 3 秒但完全没动：回 REC_REJECTED（现在是位移不够，不是时长）",
          rc == PROTO_RES_REC_REJECTED && !buttonIsRecording(), "rc=14");
    check("不达标的录制没有被保存", !buttonHasRecording(), "hasRecording=false");

    /* 时长限制已删除的回归：短录像（<10 秒）只要末端真的走了 >=10 就保存。
     * 旧的"时长必须 >10 秒"规则会把这一条整段判废。 */
    rc = buttonHandleCommand(PROTO_CMD_BTN_RECORD);
    check("重新开始录制（准备一段短录像）", rc == PROTO_RES_REC_STARTED, "rc=12");
    g_mockAnalog[0] = 512 + 500;      /* 基座推到底 */
    advance(5000);
    g_mockAnalog[0] = 512 - 500;      /* 再反推回来：包围盒跨度更大，稳稳超过 10 */
    advance(3000);
    centerSticks();
    advance(200);
    unsigned long shortMs = buttonRecordingMs();
    rc = buttonHandleCommand(PROTO_CMD_BTN_RECORD);
    snprintf(d, sizeof(d), "rc=%d 时长=%lu ms 条目=%d 位移=%.2f",
             rc, shortMs, buttonRecordingEntries(), buttonRecordingTravel());
    check("短录像（不到 10 秒）只要位移够就保存：回 REC_SAVED",
          rc == PROTO_RES_REC_SAVED && buttonHasRecording(), d);
    check("这段录像的时长确实小于 10 秒（旧规则一定判废）",
          shortMs < 10000UL, d);

    /* 位移门槛还在：录得再久，机械臂完全没动也要判废 */
    rc = buttonHandleCommand(PROTO_CMD_BTN_RECORD);
    check("重新开始录制", rc == PROTO_RES_REC_STARTED, "rc=12");
    advance(11500);
    rc = buttonHandleCommand(PROTO_CMD_BTN_RECORD);
    snprintf(d, sizeof(d), "rc=%d 时长=%lu 位移=%.2f", rc, buttonRecordingMs(), buttonRecordingTravel());
    check("只有时长、没有位移：回 REC_REJECTED",
          rc == PROTO_RES_REC_REJECTED && !buttonHasRecording(), d);
  }

  /* ---------------- 4) 录制：用摇杆真的走一段 ---------------- */
  printf("\n[4] 录制一段真实摇杆动作（长录像 11 秒 + 有明显位移）\n");
  {
    SER before = Pos.ser;
    int rc = buttonHandleCommand(PROTO_CMD_BTN_RECORD);
    check("开始录制", rc == PROTO_RES_REC_STARTED && buttonIsRecording(), "rc=12");

    /* 推基座轴 A0：角度 +（左/右推），走满约 4 秒 */
    g_mockAnalog[0] = 512 + 500;
    advance(4000);
    SER afterFwd = Pos.ser;
    snprintf(d, sizeof(d), "angle1 %.2f -> %.2f", before.angle1, afterFwd.angle1);
    check("录制期间摇杆可用：基座角被推动", fabs(afterFwd.angle1 - before.angle1) > 5.0, d);

    /* 停 3 秒（这段会变成 WAIT 条目，回放时节奏要对） */
    centerSticks();
    advance(3000);

    /* 反方向推 4 秒 */
    g_mockAnalog[0] = 512 - 500;
    advance(4000);
    centerSticks();
    advance(600);

    unsigned long recMs = buttonRecordingMs();
    rc = buttonHandleCommand(PROTO_CMD_BTN_RECORD);
    snprintf(d, sizeof(d), "rc=%d 时长=%lu ms 条目=%d 位移=%.2f",
             rc, recMs, buttonRecordingEntries(), buttonRecordingTravel());
    check("结束录制并保存成功", rc == PROTO_RES_REC_SAVED && buttonHasRecording(), d);
    check("长录像（11.6 秒，缓冲上限 12.8 秒内）照样保存", recMs > 10000UL, d);
    check("末端位移达到明显位移门槛(>=10)", buttonRecordingTravel() >= 10.0, d);
    check("条目数大于 0", buttonRecordingEntries() > 0, d);
    check("录制结束后状态回到空闲", strcmp(buttonStateName(), "空闲") == 0, buttonStateName());

    /* ---------------- 5) 播放 ---------------- */
    printf("\n[5] 播放（按键3）\n");
    SER recEnd = Pos.ser;           /* 录制结束时的真实位姿，回放结束应该回到这里 */
    unsigned long t0 = g_mockMillis;
    rc = buttonHandleCommand(PROTO_CMD_BTN_PLAY);
    check("按 P 开始播放", rc == PROTO_RES_PLAY_STARTED && buttonPlaybackActive(), "rc=15");
    check("播放期间 buttonControlLocked=true（摇杆让位）", buttonControlLocked(), "locked=true");
    check("播放期间状态名不是空闲", strcmp(buttonStateName(), "空闲") != 0, buttonStateName());
    /* 播放中按按键4 / 发 M 必须被拒：否则 btnStartRamp 会覆盖播放的时间轴，
     * 静默劫持中止播放（审计发现 1）。 */
    int rcHome = buttonHandleCommand(PROTO_CMD_BTN_HOME);
    check("播放中按键4 被拒（回 BUSY，播放不被劫持）",
          rcHome == PROTO_RES_BUSY && buttonPlaybackActive(), "rc=11 且仍在播放");
    int rcHomeAlt = buttonHandleCommand(PROTO_CMD_BTN_HOME_ALT);
    check("播放中别名 '0' 同样被拒", rcHomeAlt == PROTO_RES_BUSY, "rc=11");

    /* 播放/预摆期间推上臂轴 A1（录制里完全没有动过 angle2）：
     * 摇杆若没让位，angle2 会立刻变化，这里能一眼看出来。
     * 注意只在"仍被独占"时推摇杆 —— 播放一结束摇杆就该立刻恢复可用，
     * 那一瞬间的步进是正常行为，不属于"让位失效"。 */
    double rBefore = Pos.ser.angle2;
    g_mockAnalog[1] = 512 + 500;
    long lockIter = 0;
    while (buttonControlLocked() && lockIter < 300000) {
      buttonLoop();
      if (buttonControlLocked()) joystickLoop();
      g_mockMillis += 20UL;
      lockIter++;
    }
    snprintf(d, sizeof(d), "angle2 %.4f -> %.4f（独占期间推了 %ld 轮）",
             rBefore, Pos.ser.angle2, lockIter);
    check("播放期间摇杆确实被让位（angle2 未被摇杆改动）",
          fabs(Pos.ser.angle2 - rBefore) < 1e-9, d);
    centerSticks();
    (void) runUntilIdle(300000);

    unsigned long elapsed = g_mockMillis - t0;
    check("播放结束回到空闲", !buttonPlaybackActive() && strcmp(buttonStateName(), "空闲") == 0,
          buttonStateName());
    snprintf(d, sizeof(d), "angle1 %.2f vs 录制末态 %.2f", Pos.ser.angle1, recEnd.angle1);
    check("回放终态与录制末态一致（误差 <= 0.5 度量化步长）",
          fabs(Pos.ser.angle1 - recEnd.angle1) <= 0.51, d);
    snprintf(d, sizeof(d), "耗时 %lu ms（录制 %lu ms + 预摆 1500 ms）", elapsed, recMs);
    check("回放耗时与录制时长同量级（说明 WAIT 条目被正确还原）",
          elapsed > 10000UL && elapsed < recMs + 1500UL + 200UL, d);

    /* ---------------- 6) 回中（按键4） ---------------- */
    printf("\n[6] 回中（按键4）\n");
    SER home;
    home.angle1 = 0.0; home.angle2 = 0.0; home.angle3 = 0.0; home.angle4 = Pos.ser.angle4;
    bool homeOk = posGetHomeAngles(&home);
    double a4Before = Pos.ser.angle4;

    rc = buttonHandleCommand(PROTO_CMD_BTN_HOME);
    check("按 M 开始回中", rc == PROTO_RES_HOME_STARTED, "rc=17");
    check("回中期间摇杆让位", buttonControlLocked(), "locked=true");
    check("回中期间状态名正确", strcmp(buttonStateName(), "回中中") == 0, buttonStateName());
    (void) runUntilIdle(300000);

    snprintf(d, sizeof(d), "b/r/c = %.6f/%.6f/%.6f", Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3);
    check("回中后三个关节角 == POS_HOME 反解值（1e-9）",
          homeOk && fabs(Pos.ser.angle1 - home.angle1) < 1e-9 &&
          fabs(Pos.ser.angle2 - home.angle2) < 1e-9 &&
          fabs(Pos.ser.angle3 - home.angle3) < 1e-9, d);
    snprintf(d, sizeof(d), "angle4 %.6f -> %.6f", a4Before, Pos.ser.angle4);
    check("回中不动末端开合角 angle4", fabs(Pos.ser.angle4 - a4Before) < 1e-9, d);
    check("回中结束回到空闲", strcmp(buttonStateName(), "空闲") == 0, buttonStateName());

    /* 别名 '0' 与 'M' 等价 */
    check("回中别名 '0' 同样有效",
          buttonHandleCommand(PROTO_CMD_BTN_HOME_ALT) == PROTO_RES_HOME_STARTED, "rc=17");
    (void) runUntilIdle(300000);
  }

  /* ---------------- 7) 串口路径与忙守卫 ---------------- */
  printf("\n[7] 串口路径（N/R/P/M）\n");
  {
    /* 取放序列执行期间的忙守卫 */
    int rc = protoHandleLine("N");
    check("串口 N 启动按键1 循环执行", rc == PROTO_RES_PICK_STARTED && pickPlaceIsBusy(), "rc=10");
    check("序列忙时 O 被挡下", protoHandleLine("O") == PROTO_RES_BUSY, "rc=11");
    check("序列忙时 y45 角度指令被挡下", protoHandleLine("y45") == PROTO_RES_BUSY, "rc=11");
    check("序列忙时 H 仍然生效", protoHandleLine("H") == PROTO_RES_SPEED_UP, "rc=3");
    check("序列忙时 N 被放行到按键模块并被拒（回 BUSY）",
          protoHandleLine("N") == PROTO_RES_BUSY, "rc=11");
    (void) runUntilIdle(300000);
    check("忙守卫测试后序列跑完", !pickPlaceIsBusy(), pickPlaceStageName());

    /* 录制中发 R：必须被放行到按键模块（这正是豁免 N/R/P/M 的理由） */
    rc = protoHandleLine("R");
    check("串口 R 开始录制", rc == PROTO_RES_REC_STARTED && buttonIsRecording(), "rc=12");
    advance(200);
    rc = protoHandleLine("R");
    check("录制中串口 R 能结束录制（没有被忙守卫吞掉）",
          rc == PROTO_RES_REC_REJECTED && !buttonIsRecording(), "rc=14");

    /* mock 喂进真实串口主循环，走完整路径。
     * 注意这里不能断言"夹的是 A"：循环序号在前面的用例里已经推进过，
     * 正确的期望值是"按下之前 buttonPickNext() 报的那个物体"。 */
    int nextBefore = buttonPickNext();
    mockSerialClear();
    mockSerialFeed("N\n");
    serialProtocolLoop();
    snprintf(d, sizeof(d), "当前 %d / 预期 %d / 阶段 %s",
             pickPlaceCurrentObject(), nextBefore, pickPlaceStageName());
    check("mock 喂 N\\n 走 serialProtocolLoop 也能启动取放",
          pickPlaceIsBusy() && pickPlaceCurrentObject() == nextBefore, d);
    (void) runUntilIdle(300000);

    mockSerialClear();
    mockSerialFeed("M\n");
    serialProtocolLoop();
    check("mock 喂 M\\n 触发回中", buttonControlLocked(), buttonStateName());
    (void) runUntilIdle(300000);
    check("回中后回到空闲", strcmp(buttonStateName(), "空闲") == 0, buttonStateName());
  }

  /* ---------------- 8) 空闲稳定性 ---------------- */
  printf("\n[8] 空闲稳定性\n");
  {
    SER s0 = Pos.ser;
    REC r0 = Pos.rec;
    int entries0 = buttonRecordingEntries();
    for (int i = 0; i < 100; i++) {
      buttonLoop();
      g_mockMillis += 3UL;
    }
    check("空闲时 buttonLoop 调 100 次不改动 Pos",
          sameSer(&s0, &Pos.ser) &&
          fabs(r0.x - Pos.rec.x) < 1e-12 && fabs(r0.y - Pos.rec.y) < 1e-12 &&
          fabs(r0.z - Pos.rec.z) < 1e-12, "Pos 未变");
    check("空闲时条目数不变", buttonRecordingEntries() == entries0, "entries 未变");
    check("空闲时仍不忙", !buttonControlBusy() && !buttonControlLocked(), "busy=false");
  }

  /* ---------------- 9) 最坏情况录制：四路摇杆同时连续动 ---------------- */
  printf("\n[9] 最坏情况录制：四路摇杆连续推动 11 秒（每周期都在动）\n");
  {
    /* 历史教训：早先"每个采样周期都写一条 WAIT(1)"的格式下，四关节同时动
     * 每周期要花 5 条，512 条只够 103 个周期（100ms tick 下 10.3 秒，
     * 40ms tick 下只有 4.1 秒）→ 必然写满 → 时长 <10 秒 → 判废。
     * 改用"新周期标志 0x80"后每周期只花"动了几个关节"条，本用例应能通过。 */
    int rc = buttonHandleCommand(PROTO_CMD_BTN_RECORD);
    check("开始录制（最坏情况）", rc == PROTO_RES_REC_STARTED && buttonIsRecording(), "rc=12");

    /* 四路都推满，每 700ms 翻一次方向：既让四个关节每个周期都在动，
     * 又不让它们一直朝一个方向撞到硬限位（撞死就没动作、条目反而变少）。 */
    unsigned long t0 = g_mockMillis;
    for (int k = 0; k < 16; k++) {
      int dir = ((k % 2) == 0) ? +500 : -500;
      for (int a = 0; a < 4; a++) g_mockAnalog[a] = 512 + dir;
      advance(700);
    }
    unsigned long recMs = g_mockMillis - t0;
    int    entries = buttonRecordingEntries();
    SER    recEnd  = Pos.ser;        /* 按下结束键瞬间的真实位姿（回放应回到这里） */

    rc = buttonHandleCommand(PROTO_CMD_BTN_RECORD);
    /* 注意：位移与条目数是在"结束录制"里才结算的，录制过程中读只能读到 0 */
    double travel = buttonRecordingTravel();
    snprintf(d, sizeof(d), "rc=%d 时长=%lu ms 条目=%d/128 位移=%.2f",
             rc, recMs, entries, travel);
    check("四路连续动 11 秒仍能保存成功（没被 128 条写满）",
          rc == PROTO_RES_REC_SAVED && buttonHasRecording(), d);
    check("最坏情况录制时长确实大于 10 秒（仍在 12.8 秒窗口内）", recMs > 10000UL, d);
    check("条目数没有溢出缓冲", entries > 0 && entries <= 128, d);
    check("四路动作产生了明显位移（>=10）", travel >= 10.0, d);

    /* 立刻回放这种"每个周期都有动作"的录制，确认新周期标志被正确还原 */
    centerSticks();
    rc = buttonHandleCommand(PROTO_CMD_BTN_PLAY);
    check("最坏情况录制也能播放", rc == PROTO_RES_PLAY_STARTED && buttonPlaybackActive(), "rc=15");
    (void) runUntilIdle(600000);
    double d1 = fabs(Pos.ser.angle1 - recEnd.angle1);
    double d2 = fabs(Pos.ser.angle2 - recEnd.angle2);
    double d3 = fabs(Pos.ser.angle3 - recEnd.angle3);
    double d4 = fabs(Pos.ser.angle4 - recEnd.angle4);
    snprintf(d, sizeof(d), "末态差 b=%.2f r=%.2f c=%.2f f=%.2f 度",
             d1, d2, d3, d4);
    check("回放终态与录制末态一致（每轴 <= 1.5 度：0.5 度量化漂移）",
          d1 <= 1.5 && d2 <= 1.5 && d3 <= 1.5 && d4 <= 1.5, d);
    check("回放结束回到空闲", !buttonPlaybackActive() && strcmp(buttonStateName(), "空闲") == 0,
          buttonStateName());
  }

  /* ---------------- 11) 录制中直接按 P ---------------- */
  /* 现场问题："录完动作后再输入 P，就会触发 busy，录的动作执行不了"。
   * 以前 btnActionPlay 第一句就查 s_recording -> BUSY；而"手动完了"并不等于
   * "录制停了"，于是用户觉得录完了却永远收到 BUSY。
   * 现在改成：录制中按 P 先调用 btnStopRecording() 收尾 —— 合格就接着播放，
   * 不合格（没位移/缓冲溢出）就回 REC_REJECTED 让用户重录，不再是 BUSY。 */
  printf("\n[11] 录制中直接按 P：先收尾再播放\n");
  {
    /* 11a) 不合格的短录制：按 P 应当收尾 + 回 DISCARD，且不开始播放。
     * 注意：原来这条是靠"2 秒 < 10 秒门槛"判废的；时长门槛删除后，改成录一段
     * **没有位移**的动作来走同一条 DISCARD 路径（合格的短录像见 [3]）。 */
    centerSticks();
    int rc = buttonHandleCommand(PROTO_CMD_BTN_RECORD);
    check("开始录制（短）", rc == PROTO_RES_REC_STARTED && buttonIsRecording(), "rc=12");
    advance(2000);                       /* 只录 2 秒且四路摇杆居中：位移 0，判废 */
    centerSticks();
    rc = buttonHandleCommand(PROTO_CMD_BTN_PLAY);
    snprintf(d, sizeof(d), "rc=%d 仍在录制=%d 播放中=%d", rc, (int)buttonIsRecording(),
             (int)buttonPlaybackActive());
    check("录制中按 P（没有位移）：回 DISCARD 且没有开始播放",
          rc == PROTO_RES_REC_REJECTED && !buttonIsRecording() && !buttonPlaybackActive(), d);

    /* 11b) 合格录制（11.7 秒、有明显位移）：按 P 立刻收尾并接着播放 */
    centerSticks();
    rc = buttonHandleCommand(PROTO_CMD_BTN_RECORD);
    check("开始录制（合格）", rc == PROTO_RES_REC_STARTED && buttonIsRecording(), "rc=12");
    for (int a = 0; a < 4; a++) g_mockAnalog[a] = 512 + 500;
    advance(5000);
    for (int a = 0; a < 4; a++) g_mockAnalog[a] = 512 - 500;
    advance(5000);
    for (int a = 0; a < 4; a++) g_mockAnalog[a] = 512 + 500;
    advance(1500);                       /* 合计 11.5 秒（上限 12.8 秒内） */
    centerSticks();
    advance(200);
    SER recEnd = Pos.ser;                /* 按 P 这一刻的位姿 = 录制末态 */
    rc = buttonHandleCommand(PROTO_CMD_BTN_PLAY);
    snprintf(d, sizeof(d), "rc=%d 仍在录制=%d 播放中=%d", rc, (int)buttonIsRecording(),
             (int)buttonPlaybackActive());
    check("录制中按 P（合格）：先收尾保存再立刻播放，不再回 BUSY",
          rc == PROTO_RES_PLAY_STARTED && !buttonIsRecording() && buttonPlaybackActive(), d);
    check("收尾后确实有录制数据", buttonHasRecording(), "hasRecording=true");

    (void) runUntilIdle(600000);
    double e1 = fabs(Pos.ser.angle1 - recEnd.angle1);
    double e2 = fabs(Pos.ser.angle2 - recEnd.angle2);
    double e3 = fabs(Pos.ser.angle3 - recEnd.angle3);
    double e4 = fabs(Pos.ser.angle4 - recEnd.angle4);
    snprintf(d, sizeof(d), "末态差 b=%.2f r=%.2f c=%.2f f=%.2f 度", e1, e2, e3, e4);
    check("回放终态 == 按 P 那一刻的位姿（每轴 <= 1.5 度）",
          e1 <= 1.5 && e2 <= 1.5 && e3 <= 1.5 && e4 <= 1.5, d);
    check("回放结束回到空闲", !buttonPlaybackActive() && strcmp(buttonStateName(), "空闲") == 0,
          buttonStateName());
  }

  printf("\n");
  if (failures == 0) {
    printf(">>> ALL PASS (失败 0 项)\n");
    return 0;
  }
  printf(">>> FAILED (失败 %d 项)\n", failures);
  return 1;
}
