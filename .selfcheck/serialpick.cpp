/* serialpick.cpp -- 把串口"一行命令"喂进固件本体，看它到底回什么、以及为什么。
 *
 * 为什么需要它：真机上 `A`/`B`/`C` 回的是 `BUSY`，而 `picktable_run.cpp` 直接调
 * pickPlaceStart() 在 PC 上是 0。两者走的是不同的链路：
 *     serialProtocolLoop -> protoHandleLine -> 忙判定(drawControlBusy()
 *     || buttonControlBusy() || pickPlaceIsBusy()) -> pickPlaceStart()
 * 本工具按 meArm.ino 的 setup() 逐条初始化，再逐条喂命令，打印
 *   ① 固件真正吐给串口的那一行文本（模拟 Arduino 串口监视器）
 *   ② 内部状态：Pos.rec / Pos.ser / 三个 busy 标志
 * 用来区分 "真忙"、"-3 当前位姿分支不符"、"-4 高度/飞行高度"、"没编进该模块"。
 *
 * 编译（链接固件本体，不碰 avr）：
 *   g++ -std=gnu++17 -O2 -I. -I.\.selfcheck\mock `
 *       -DWEARM_ENABLE_PICK_PLACE=1 -DWEARM_ENABLE_BUTTONS=1 `
 *       .\.selfcheck\serialpick.cpp .\serial_protocol.cpp .\pick_place.cpp `
 *       .\constant_and_positions.cpp .\move.cpp .\servo_drive.cpp `
 *       .\button_control.cpp .\joystick_control.cpp .\draw_control.cpp `
 *       .\protocol_constants.cpp .\.selfcheck\mock\Arduino.cpp `
 *       -o .\.selfcheck\out\serialpick.exe
 *
 * 用法：serialpick.exe [场景]   场景 = boot | idle | jog | refuse | nobtn | draw | all（默认 all）
 */

#include <stdio.h>
#include <string.h>
#include <string>

#include <Arduino.h>
#include "constant_and_positions.h"
#include "serial_protocol.h"
#include "pick_place.h"
#include "button_control.h"
#include "draw_control.h"
#include "joystick_control.h"
#include "servo_drive.h"

extern unsigned long g_mockMillis;
int protoHandleLine(const char *line);

static void pump(int ms)
{
  for (int t = 0; t < ms; t += 10) {
    serialProtocolLoop();
    pickPlaceLoop();
    buttonLoop();
    drawLoop();
    joystickLoop();
    g_mockMillis += 10UL;
  }
}

static void state(const char *tag)
{
  printf("      [%s] rec(%7.2f,%7.2f,%7.2f)  ser(b%7.2f r%7.2f c%7.2f t%6.1f)"
         "  busy(pick=%d btn=%d draw=%d)\n",
         tag, Pos.rec.x, Pos.rec.y, Pos.rec.z,
         Pos.ser.angle1, Pos.ser.angle2, Pos.ser.angle3, Pos.ser.angle4,
         (int) pickPlaceIsBusy(), (int) buttonControlBusy(), (int) drawControlBusy());
}

static void feed(const char *line)
{
  Serial.clearOutput();
  int rc = protoHandleLine(line);
  std::string o = Serial.getOutput();
  printf("   > %-6s  rc=%-4d 固件回复: %s", line, rc, o.c_str());
  if (o.empty() || o[o.size() - 1] != '\n') printf("\n");
}

static void boot(void)
{
  /* 场景之间要把上一段序列排空：pick_place 没有公开的 cancel，
   * 而 posInit() 不会清它的状态机，残留的 busy 会让下一场景全被忙守卫挡下。 */
  if (pickPlaceIsBusy()) {
    printf("   （先排空上一段取放序列…）\n");
    for (int i = 0; i < 20000 && pickPlaceIsBusy(); i++) {
      pickPlaceLoop();
      g_mockMillis += 10UL;
    }
  }
  Serial.clearOutput();
  posInit();
  serialProtocolBegin();
  std::string b = Serial.getOutput();
  Serial.clearOutput();
  printf("\n=== 开机横幅（真机串口监视器上看到的就是这一行）===\n%s", b.c_str());
  joystickSetup();
  buttonSetup();
  drawSetup();
  servoDriveBegin();
  servoDriveAttach(0, 9);
  servoDriveAttach(1, 8);
  servoDriveAttach(2, 7);
  servoDriveAttach(3, 6);
  printf("\n=== 初始化完成（同 meArm.ino setup()）===\n");
  state("boot");
  feed("!");
  state("after !");
}

int main(int argc, char **argv)
{
  const char *scene = (argc > 1) ? argv[1] : "all";

  if (!strcmp(scene, "boot") || !strcmp(scene, "all")) {
    printf("\n########## 场景 boot：上电后立刻发 A / B / C ##########\n");
    boot();
    feed("A");
    state("after A");
    feed("B");
    state("after B");
    pump(40000);
    state("drained 40s");
    feed("C");
    state("after C");
    pump(40000);
    state("drained 40s");
  }

  if (!strcmp(scene, "idle") || !strcmp(scene, "all")) {
    printf("\n########## 场景 idle：上电后空转 30 秒（摇杆噪声会动到臂）再发 A ##########\n");
    boot();
    pump(30000);
    state("idle 30s");
    feed("A");
    state("after A");
  }

  if (!strcmp(scene, "jog") || !strcmp(scene, "all")) {
    printf("\n########## 场景 jog：先用 x/y/z 走开，再发 A ##########\n");
    boot();
    feed("x100");
    pump(5000);
    state("after x100");
    feed("y100");
    pump(5000);
    state("after y100");
    feed("A");
    state("after A");
  }

  if (!strcmp(scene, "refuse") || !strcmp(scene, "all")) {
    printf("\n########## 场景 refuse：先用角度指令摆到极端位姿，再发 A ##########\n");
    static const char *poses[] = { "y45", "y160", "z30", "z110", "y45,z30", "y160,z110" };
    for (unsigned i = 0; i < sizeof(poses) / sizeof(poses[0]); i++) {
      printf("\n--- 位姿 %s ---\n", poses[i]);
      boot();
      feed(poses[i]);
      state("after pose");
      feed("A");
      state("after A");
      pump(2000);
    }
  }

  if (!strcmp(scene, "nobtn") || !strcmp(scene, "all")) {
    printf("\n########## 场景 nobtn：按键模块被裁掉时 M/0 与 x90,y90,z90 ##########\n");
    boot();
    feed("y120"); /* 先走开（上臂舵机角 120），便于看"回中"是不是真的回去了 */
    pump(2000);
    state("after y120");
    feed("M");
    state("after M");
    feed("0");
    state("after 0");
    feed("x90,y90,z90"); /* 与 M 等价的家位角 */
    state("after x90,y90,z90");
  }

  if (!strcmp(scene, "draw") || !strcmp(scene, "all")) {
    printf("\n########## 场景 draw：发绘图命令（默认分支里绘图没编进来）##########\n");
    boot();
    feed("F");
    feed("D");
    feed("p-6");
    feed("!D");
  }

  printf("\n");
  return 0;
}
