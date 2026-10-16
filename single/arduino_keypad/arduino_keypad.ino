// ============================================================================
// meArm 按键板 —— 单文件交付稿（第二块 Arduino：UNO / Nano）
// ----------------------------------------------------------------------------
// 这是什么：一块独立的"按键遥控板"。4 个按键 + 1 个 LED，通过软串口把命令字符
//   发给跑 meArm 主控固件的那块 Arduino，代替它板上那 4 个物理按键。
//   本文件只依赖 Arduino 核心自带的 <SoftwareSerial.h>，不依赖本仓库其它文件。
//
// 从哪来：由仓库里的 .selfcheck/make_keypad_single.ps1 生成 ——
//   arduino_keypad/arduino_keypad.ino 逐字拷贝 + 本说明头。
//   要改功能请回去改源文件、再重新生成，别只改这一份。
//
// 怎么上传：Arduino IDE 要求 **.ino 文件名与所在文件夹同名**。
//   本文件叫 arduino_keypad.ino，所以它必须在名为 arduino_keypad 的文件夹里
//   （仓库里就是 single/arduino_keypad/arduino_keypad.ino）。双击打开 →
//   选开发板 Arduino Uno（Nano 也一样）→ 选端口 → 上传。不需要装任何库。
//
// 配套文件（都不影响上传，只是说明与辅助）：
//   arduino_keypad/README.md    接线、两套预设、按键映射的详细说明
//   arduino_keypad/keysim.js    把按键状态机搬到 PC 上用 node 跑的波形复核
//                               （纯离线模拟，不连板子、与上传无关）
//
// 实测占用（avr-gcc 7.3.0 + Arduino AVR core 1.8.8，atmega328p）：
//   Program 6170 字节（18.8%） / Data 975 字节（47.6%）—— UNO 上很宽裕。
//
// 主控固件要求：本板发出的 N/R/P/M/0 属于主控的"按键模块"。本仓库 v1.6.8 的
//   默认功能组合是「取放 + 绘图，按键关」，那 5 个槽位会被主控回 REJECTED；
//   要配合本板，请把主控改成「取放 + 按键（绘图关）」那套开关组合。
//   详细对照见下面源码开头的注释（"和主控固件的配套关系"一节）。
// ============================================================================

/*
 * ============================================================================
 *  文件：arduino_keypad.ino
 *  工程：机械臂按键板（第二块 Arduino UNO / Nano）—— 纯有线按键控制
 * ----------------------------------------------------------------------------
 *  功能：4 个按键（默认 D8/D9/D10/D11，按下=低电平），识别单击/双击/三击/长按，
 *        把对应指令发给机械臂主控板。
 *
 *  【两套预设，编译期切换】
 *    PRESET_MEARM （默认）：配本仓库 meArm 固件（v1.6.8 主线）。
 *        波特率 115200，命令见 protocol_constants.h：单字符 + '\n'。
 *        单击 = 主控板上那四个物理按键的串口孪生命令（N/R/P/M 那一族）,
 *        其余槽位给取放对象/夹爪/调速/绘图/流程。
 *        【注意】N/R/P/M/0 属于主控的"按键模块"，而 v1.6.8 的默认功能组合
 *        把它裁掉了 —— 上主控前先看下面「和主控固件的配套关系」一节。
 *    PRESET_TASK5        ：配 others/f/MeArm 的 Task5 系列固件
 *        （波特率 9600、单字符 '1'..'8'）。用法见同目录 README。
 *        切换：配置区改 PRESET，或命令行 -DPRESET=2。
 *
 *  和原来那块 KeypadBoard_Task5（others/f/MeArm/KeypadBoard_Task5）的区别：
 *    1) 默认用软串口 D2(RX)/D3(TX) 跟机械臂通话，**USB 串口留着当调试口**：
 *       既能随时上传程序，又能看到"按了什么、发了什么、机械臂回了什么"。
 *       原来的版本用 Serial(D0/D1) 直连机械臂，插着线就没法上传、也看不到回包。
 *       （要完全照旧：把 ARM_LINK_SOFTWARE 改成 0。）
 *    2) 板载 LED(D13) 给出本地反馈：命令发出闪 2 下、无此映射闪 1 下、长按闪 4 下。
 *    3) 多一个长按（原来的板子只有单击/双击/三击）。
 *    4) 按键用表驱动，加键/改映射只改两张表，不用动逻辑。
 *    5) 开机不给机械臂发任何字符（原来会发一句横幅，Task5 会回一条 "! 指令：…"）。
 *
 *  【PRESET_MEARM 按键映射】
 *    键1 D8  取放/夹爪：单击 N 循环取放(A→B→C)  双击 O 爪张开  三击 S 爪合拢  长按 M 回开机位姿
 *    键2 D9  录制/速度：单击 R 录制开始/结束    双击 P 播放     三击 H 升一档  长按 L 降一档
 *    键3 D10 指定物体：单击 A 夹物体A          双击 B 夹物体B  三击 C 夹物体C 长按 ! 查模块开关
 *    键4 D11 调试/方向：单击 m 查换向状态      双击 mS 上臂换向 三击 mE 下臂换向 长按 0 回开机位姿
 *
 *    【和主控固件的配套关系（v1.6.8 起）】：主控的默认功能组合是「取放 + 绘图，
 *    按键关」（PICK 1 / BUTTONS 0 / DRAW 1，31740 B），**按键模块被裁掉了**，
 *    所以上表里属于按键模块的 5 个槽位 —— 键1 单击 N、键1 长按 M、键2 单击 R、
 *    键2 双击 P、键4 长按 0 —— 在主控上只会回 "REJECTED"。
 *    要用这块按键板的全部功能，把主控改成「取放 + 按键（绘图关）」
 *    （PICK 1 / BUTTONS 1 / DRAW 0，22770 B，本板最初就是照它设计的），
 *    或者「绘图 + 按键」（PICK 0 / BUTTONS 1 / DRAW 1，29356 B）。
 *    三个功能全开装不进 Uno（35162 B > 可用 32256 B），必须裁一个。
 *    默认组合下其余键位都好用：O/S（夹爪张开/合拢）、H/L（调速）、A/B/C（取放）、
 *    长按 !（查模块开关）、m/mS/mE（查/切换向）都不属于按键模块。
 *
 *    【为什么键3/键4 空着绘图位】：本板最初配的是「绘图关」那套主控固件，所以
 *    键3/键4 换成了那一分支一定可用的命令。想用绘图按键就把这两行换回去
 *    （绘图模块在默认组合里是开着的）：
 *      键3：单击 F 换图形      双击 D 开始绘制  三击 G 记示教点 长按 E 撤示教点
 *      键4：单击 Q 暂停        双击 U 继续      三击 W 取消     长按 m 查换向状态
 *    主控回复的三种"没反应"要分清："REJECTED" = 该模块没编进固件（改开关重编）；
 *    "OFF" = 编进来了但被 !B/!D 运行时关掉了（发一次 !B/!D 打开）；"BUSY" = 主控
 *    正在取放/录制/绘图，等一下再按。
 *
 *  【PRESET_TASK5 按键映射】（与原 KeypadBoard_Task5 完全一致）
 *    键1 D8 ：单击 '1' 示教/记录点           长按 '?' 状态查询
 *    键2 D9 ：单击 '2' 暂停                  双击 '3' 继续
 *    键3 D10：单击 '4' 取消                  双击 '5' 折线   三击 '6' 曲线
 *    键4 D11：单击 '7' 直线演示              双击 '8' 字母 N
 *
 *  硬件：按键一端接引脚，另一端接 GND（靠芯片内部上拉 INPUT_PULLUP，不用外接电阻）。
 *        Arduino 只有一路硬件串口，所以默认用软串口跟机械臂对话，接线：
 *          Uno D2(RX) ← 机械臂 D1(TX)      Uno D3(TX) → 机械臂 D0(RX)      两块板 GND 必须相连
 *        注意：软串口在 16MHz AVR 上 115200 能发，但"收"容易丢字符（回包转发可能缺字）。
 *        要 100% 可靠就用 ARM_LINK_SOFTWARE 0（D0/D1 硬件串口直连），代价是上传时要拔线。
 *        也可以把两边都降到 38400（固件 protocol_constants.h 的 PROTO_BAUD + 这里 ARM_BAUD），
 *        软串口在 38400 是完全可靠的。
 * ============================================================================
 */

/* ============================== 1. 配置区 ============================== */

#define PRESET_MEARM      1       // 本仓库 meArm 固件（115200，见 protocol_constants.h）
#define PRESET_TASK5      2       // others/f/MeArm 的 Task5 固件（9600，'1'..'8'）
#ifndef PRESET                   // 命令行 -DPRESET=2 可覆盖
#define PRESET            PRESET_MEARM
#endif

#if PRESET == PRESET_MEARM
  #define PRESET_NAME     "meArm 主线固件"
  #define ARM_BAUD        115200  // = protocol_constants.h 的 PROTO_BAUD（改固件要同步改这里）
#else
  #define PRESET_NAME     "Task5 固件"
  #define ARM_BAUD        9600    // Task5 系列固定在 9600
#endif

#ifndef ARM_LINK_SOFTWARE         // 也可用 -DARM_LINK_SOFTWARE=0 在命令行切换
#define ARM_LINK_SOFTWARE 1       // 1=软串口 D2(RX)/D3(TX) 连机械臂，USB 留作调试（推荐）
#endif                            // 0=直接用 Serial(D0/D1) 连机械臂（最稳，但 USB 不可用）

#define PIN_ARM_RX        2       // 软串口 RX ← 机械臂 TX（D1）
#define PIN_ARM_TX        3       // 软串口 TX → 机械臂 RX（D0）

#ifndef ECHO_ARM
#define ECHO_ARM          1       // 1=把机械臂的回包转发到 USB 串口（只在软串口模式下有效）
#endif
#define DBG_BAUD          115200  // USB 调试口波特率

#define PIN_K1            8       // 键1
#define PIN_K2            9       // 键2
#define PIN_K3            10      // 键3
#define PIN_K4            11      // 键4
#define PIN_LED           13      // 板载 LED（D13），本地反馈

#define DEBOUNCE_MS       30      // 消抖时间：电平稳定这么久才算数
#define CLICK_WINDOW      350     // 连击窗口：松开后这么久内没有新按下就结算发送
#define LONG_PRESS_MS     700     // 长按判定（0 = 关闭长按功能）
#define LED_STEP_MS       60      // LED 闪烁半周期

/* 软串口模式下才有调试口；用 Serial 直连机械臂时不能再打印任何调试信息 */
#if ARM_LINK_SOFTWARE
  #define HAVE_DEBUG 1
  #include <SoftwareSerial.h>
  SoftwareSerial armSerial(PIN_ARM_RX, PIN_ARM_TX);
  #define ARM_BEGIN(b)  armSerial.begin(b)
  #define ARM_AVAIL()   armSerial.available()
  #define ARM_READ()    armSerial.read()
  #define ARM_WRITE(c)  armSerial.write(c)
#else
  #define HAVE_DEBUG 0
  #undef  ECHO_ARM
  #define ECHO_ARM 0
  #define ARM_BEGIN(b)  Serial.begin(b)
  #define ARM_AVAIL()   Serial.available()
  #define ARM_READ()    Serial.read()
  #define ARM_WRITE(c)  Serial.write(c)
#endif

/* ============================== 2. 按键表 ============================== */

/* 要接的按键引脚（改成别的引脚只改这一行；加第 5 个键就往后加一个） */
static const uint8_t PINS[] = { PIN_K1, PIN_K2, PIN_K3, PIN_K4 };
#define NPIN ((uint8_t)(sizeof(PINS) / sizeof(PINS[0])))

/* 单击/双击/三击 → 指令（指令是字符串，所以 "mS"/"!P" 这类多字符命令也能放进来） */
struct KeyMap { uint8_t pin; uint8_t clicks; const char *cmd; const char *label; };
/* 长按 → 指令（不想用长按就把 LONG_PRESS_MS 设成 0） */
struct LongMap { uint8_t pin; const char *cmd; const char *label; };

#if PRESET == PRESET_MEARM
static const KeyMap KEYS[] = {
  { PIN_K1, 1, "N", "键1 单击 循环取放(A/B/C)" },
  { PIN_K1, 2, "O", "键1 双击 爪张开" },
  { PIN_K1, 3, "S", "键1 三击 爪合拢" },
  { PIN_K2, 1, "R", "键2 单击 录制开始/结束" },
  { PIN_K2, 2, "P", "键2 双击 播放上次录制" },
  { PIN_K2, 3, "H", "键2 三击 速度升一档" },
  { PIN_K3, 1, "A", "键3 单击 夹取物体A" },
  { PIN_K3, 2, "B", "键3 双击 夹取物体B" },
  { PIN_K3, 3, "C", "键3 三击 夹取物体C" },
  { PIN_K4, 1, "m", "键4 单击 查换向状态(S=/E=)" },
  { PIN_K4, 2, "mS", "键4 双击 上臂换向" },
  { PIN_K4, 3, "mE", "键4 三击 下臂换向" },
};
static const LongMap LONGS[] = {
  { PIN_K1, "M", "键1 长按 回开机位姿" },
  { PIN_K2, "L", "键2 长按 速度降一档" },
  { PIN_K3, "!", "键3 长按 查模块开关(P=/B=/D=)" },
  { PIN_K4, "0", "键4 长按 回开机位姿(M 的别名)" },
};
#else /* PRESET_TASK5：与原 KeypadBoard_Task5 一致 */
static const KeyMap KEYS[] = {
  { PIN_K1, 1, "1", "键1 单击 示教/记录点" },
  { PIN_K2, 1, "2", "键2 单击 暂停" },
  { PIN_K2, 2, "3", "键2 双击 继续" },
  { PIN_K3, 1, "4", "键3 单击 取消" },
  { PIN_K3, 2, "5", "键3 双击 折线绘制" },
  { PIN_K3, 3, "6", "键3 三击 曲线绘制" },
  { PIN_K4, 1, "7", "键4 单击 直线演示" },
  { PIN_K4, 2, "8", "键4 双击 基础图形(字母N)" },
};
static const LongMap LONGS[] = {
  { PIN_K1, "?", "键1 长按 状态查询" },
};
#endif

#define NKEYS ((uint8_t)(sizeof(KEYS) / sizeof(KEYS[0])))
#define NLONG ((uint8_t)(sizeof(LONGS) / sizeof(LONGS[0])))

/* ============================== 3. 状态 ============================== */

struct KeyState {
  bool     stable;      // 消抖确认后的电平（true = 按下）
  bool     prev;        // 上一次读到的原始电平
  bool     longFired;   // 本次按住是否已经发过长按指令
  uint8_t  clicks;      // 本次连击次数
  uint32_t debounceAt;  // 电平变化时刻（消抖计时起点）
  uint32_t releaseAt;   // 确认松开时刻（连击窗口起点）
  uint32_t pressAt;     // 确认按下时刻（长按计时起点）
};
static KeyState ks[NPIN];       // 全局变量默认全 0：stable=false 正好等于"松开"

static uint8_t  ledSteps = 0;   // 剩余闪烁步数（一步 = 半周期）
static uint32_t ledAt    = 0;
static bool     ledOn    = false;
#if ECHO_ARM
static bool     lineStart = true;   // 回包转发：是否在行首
#endif

/* ============================== 4. 收发 ============================== */

/* 本地 LED 反馈：闪 n 下（非阻塞） */
static void ledBlink(uint8_t n) { ledSteps = (uint8_t)(n * 2); }

static void ledTick() {
  if (ledSteps == 0) {
    if (ledOn) { ledOn = false; digitalWrite(PIN_LED, LOW); }
    return;
  }
  const uint32_t now = millis();
  if (now - ledAt < LED_STEP_MS) return;
  ledAt = now;
  ledOn = !ledOn;
  digitalWrite(PIN_LED, ledOn ? HIGH : LOW);
  ledSteps--;
}

/* 发一条指令给机械臂：命令字符串 + 换行（固件都按 '\n' 断行） */
static void sendCmd(const char *cmd, const char *what) {
  for (const char *p = cmd; *p != '\0'; p++) ARM_WRITE((uint8_t)*p);
  ARM_WRITE('\n');
#if HAVE_DEBUG
  Serial.print(F("[按键] "));
  Serial.print(what);
  Serial.print(F("  → 发送 \""));
  Serial.print(cmd);
  Serial.println(F("\""));
#else
  (void)what;
#endif
}

/* 查表：某引脚连击 n 次对应的指令（找不到返回 NULL） */
static const char *clickCmd(uint8_t pin, uint8_t n) {
  for (uint8_t i = 0; i < NKEYS; i++) {
    if (KEYS[i].pin == pin && KEYS[i].clicks == n) return KEYS[i].cmd;
  }
  return NULL;
}

/* 查表：某引脚长按对应的指令（找不到返回 NULL） */
static const char *longCmd(uint8_t pin) {
  for (uint8_t i = 0; i < NLONG; i++) {
    if (LONGS[i].pin == pin) return LONGS[i].cmd;
  }
  return NULL;
}

/* 把按键动作翻译成可读名字，用于调试输出 */
static const char *clickName(uint8_t pin, uint8_t n) {
  for (uint8_t i = 0; i < NKEYS; i++) {
    if (KEYS[i].pin == pin && KEYS[i].clicks == n) return KEYS[i].label;
  }
  return "无此映射";
}

static const char *longName(uint8_t pin) {
  for (uint8_t i = 0; i < NLONG; i++) {
    if (LONGS[i].pin == pin) return LONGS[i].label;
  }
  return "无此映射";
}

/* ============================== 5. 按键扫描 ============================== */

/* 消抖 → 计击数 → 长按 → 松开后超窗结算发送 */
static void keysTick() {
  const uint32_t now = millis();

  for (uint8_t i = 0; i < NPIN; i++) {
    KeyState &k = ks[i];
    const uint8_t pin = PINS[i];
    const bool raw = (digitalRead(pin) == LOW);      // true = 按下（内部上拉，按下接地）

    if (raw != k.prev) {
      k.prev = raw;                                  // 电平刚变化：重新开始消抖计时
      k.debounceAt = now;
    } else if (now - k.debounceAt >= DEBOUNCE_MS && raw != k.stable) {
      k.stable = raw;                                // 电平稳定够久：确认变化
      if (raw) {                                     // 确认按下
        if (k.clicks < 9) k.clicks++;                // 计一击（上限防抖动跑飞）
        k.pressAt   = now;
        k.longFired = false;
      } else {                                       // 确认松开
        k.releaseAt = now;
      }
    }

    /* 长按：按住超过阈值且有映射 → 立刻发长按指令，并取消本次的点击结算 */
    if (LONG_PRESS_MS > 0 && k.stable && !k.longFired && k.clicks > 0 &&
        now - k.pressAt >= LONG_PRESS_MS) {
      k.longFired = true;
      k.clicks = 0;                                  // 长按不再触发单击指令
      const char *cmd = longCmd(pin);
      if (cmd != NULL) {
        sendCmd(cmd, longName(pin));
        ledBlink(4);
      } else {
        ledBlink(1);                                 // 该键没有长按映射：只闪一下提示
      }
    }

    /* 结算：已松开，且连击窗口内没有新的按下 */
    if (k.clicks > 0 && !k.stable && now - k.releaseAt >= CLICK_WINDOW) {
      const uint8_t n = k.clicks;
      k.clicks = 0;
      const char *cmd = clickCmd(pin, n);
      if (cmd != NULL) {
        sendCmd(cmd, clickName(pin, n));
        ledBlink(2);
      } else {
        ledBlink(1);                                 // 比如键4 四击：没有定义，闪一下告诉你按到了
#if HAVE_DEBUG
        Serial.print(F("[按键] "));
        Serial.print(clickName(pin, n));
        Serial.print(F("（D"));
        Serial.print(pin);
        Serial.print(F(" 键 ×"));
        Serial.print(n);
        Serial.println(F("）无对应指令，忽略"));
#endif
      }
    }
  }
}

/* ============================== 6. 机械臂回包 ============================== */

/* 一直读、及时清空软串口缓冲；ECHO_ARM=1 时按行转发到 USB 调试口 */
static void armReadTick() {
  while (ARM_AVAIL() > 0) {
    const int c = ARM_READ();
    if (c < 0) break;
#if ECHO_ARM
    if (c == '\n') {
      Serial.println();
      lineStart = true;
    } else if (c != '\r') {
      if (lineStart) { Serial.print(F("< ")); lineStart = false; }
      Serial.write((uint8_t)c);
    }
#endif
  }
}

/* ============================== 7. setup / loop ============================== */

void setup() {
#if HAVE_DEBUG
  Serial.begin(DBG_BAUD);
#endif
  ARM_BEGIN(ARM_BAUD);                               // 先连上机械臂，再报自己的状态

  for (uint8_t i = 0; i < NPIN; i++) pinMode(PINS[i], INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

#if HAVE_DEBUG
  Serial.println();
  Serial.println(F("=== 机械臂按键板（第二块 Arduino）==="));
  Serial.print(F("预设："));
  Serial.print(F(PRESET_NAME));
  Serial.print(F("，机械臂串口 "));
  Serial.print(ARM_BAUD);
  Serial.println(F(" 8N1"));
#if ARM_LINK_SOFTWARE
  Serial.print(F("机械臂串口：软串口 D"));
  Serial.print(PIN_ARM_RX);
  Serial.print(F("(RX) / D"));
  Serial.print(PIN_ARM_TX);
  Serial.println(F("(TX)；USB 调试口见上"));
#else
  Serial.println(F("机械臂串口：Serial(D0/D1) 直连（无调试输出）"));
#endif
  Serial.print(F("消抖 "));
  Serial.print(DEBOUNCE_MS);
  Serial.print(F("ms，连击窗口 "));
  Serial.print(CLICK_WINDOW);
  Serial.print(F("ms，长按 "));
  Serial.print(LONG_PRESS_MS);
  Serial.println(F("ms"));
  Serial.print(F("按键引脚："));
  for (uint8_t i = 0; i < NPIN; i++) {
    Serial.print(F("D"));
    Serial.print(PINS[i]);
    Serial.print(' ');
  }
  Serial.println(F("（另一端接 GND，内部上拉）"));

  Serial.print(F("映射（"));
  Serial.print((uint8_t)(NKEYS + NLONG));
  Serial.println(F(" 条）："));
  for (uint8_t i = 0; i < NKEYS; i++) {
    Serial.print(F("  "));
    Serial.print(KEYS[i].label);
    Serial.print(F("  → \""));
    Serial.print(KEYS[i].cmd);
    Serial.println(F("\""));
  }
  for (uint8_t i = 0; i < NLONG; i++) {
    Serial.print(F("  "));
    Serial.print(LONGS[i].label);
    Serial.print(F("  → \""));
    Serial.print(LONGS[i].cmd);
    Serial.println(F("\""));
  }
  Serial.println(F("就绪（LED 闪 3 下）。LED：命令发出闪 2 下 / 无此映射闪 1 下 / 长按闪 4 下。"));
#endif

  ledBlink(3);                                       // 开机自检：闪 3 下表示活着
}

void loop() {
  keysTick();       // 按键
  armReadTick();    // 机械臂回包
  ledTick();        // LED
}
