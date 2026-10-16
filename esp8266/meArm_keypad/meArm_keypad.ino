/* ============================================================================
 * meArm 按键板（ESP8266）—— 纯有线按键控制，没有 WiFi、没有网页
 * ----------------------------------------------------------------------------
 * 作用：替代 others/f/MeArm/KeypadBoard_Task5 那块"第二块 UNO + 4 个按键"，
 *       用一块 ESP8266 把 4 个按键变成机械臂的串口指令：
 *
 *   ┌─ 按键 ────────────────────────────────────────────────────────────────┐
 *   │  K1 单击            → '1'  示教/记录（没在示教就先进入示教模式）      │
 *   │  K2 单击 / 双击      → '2' 暂停 / '3' 继续                            │
 *   │  K3 单击/双击/三击   → '4' 取消 / '5' 折线 / '6' 曲线                 │
 *   │  K4 单击 / 双击      → '7' 直线演示 / '8' 基础图形（字母 N）          │
 *   └──────────────────────────────────────────────────────────────────────┘
 *   与原按键板完全一致（DEBOUNCE 30ms、连击窗口 350ms），所以原来的
 *   `任务五_四按键使用说明.txt` 照旧能用。
 *
 * 接线（机械臂 = Arduino Uno/ATmega328P，5V 逻辑；ESP8266 = 3.3V 逻辑）
 *   ┌──────────────┬──────────────────────┬────────────────────────────────┐
 *   │ 机械臂 Uno   │ ESP8266 (NodeMCU)    │ 说明                            │
 *   ├──────────────┼──────────────────────┼────────────────────────────────┤
 *   │ D0 (RX)      │ D6 / GPIO12  (TX)    │ 3.3V 直连可用（>=3.0V 即高）    │
 *   │ D1 (TX, 5V)  │ D5 / GPIO14  (RX)    │ 1kΩ 串联 + 2kΩ 到 GND 再进 ESP  │
 *   │ GND          │ GND                  │ 必须共地                        │
 *   └──────────────┴──────────────────────┴────────────────────────────────┘
 *   ┌──────────────┬──────────────────────┬────────────────────────────────┐
 *   │ 按键         │ ESP8266 引脚         │ 说明                            │
 *   ├──────────────┼──────────────────────┼────────────────────────────────┤
 *   │ K1 示教      │ D1 / GPIO5           │ 按键一端接引脚，另一端接 GND    │
 *   │ K2 暂停/继续 │ D2 / GPIO4           │ 用芯片内部上拉，不用外接电阻    │
 *   │ K3 取消/折线 │ D7 / GPIO13          │ 按下 = 低电平                    │
 *   │ K4 直线/图形 │ D3 / GPIO0           │ = 板上 FLASH 键那个脚；**上电那  │
 *   │              │                      │ 一下别按它**（GPIO0 拉低会进下载 │
 *   │              │                      │ 模式，机械臂会没反应）           │
 *   └──────────────┴──────────────────────┴────────────────────────────────┘
 *   板载 LED（D4 / GPIO2）发命令时闪一下，作为"发出去了"的提示。
 *
 * 注意 1：给机械臂（Uno）烧录程序时，把连到它 D0 的那根线（ESP 的 D6）拔掉，
 *         否则 USB 转串口芯片和 ESP 会同时驱动 D0 → avrdude 会一直
 *         "not in sync: resp=0x00"。
 * 注意 2：ESP8266 峰值电流 300mA+，请用独立 AMS1117-3.3（前面并 470uF），
 *         不要用 Uno 的 3.3V 脚（只有约 50mA）。
 * 注意 3：这里用 D5/D6 软串口跟机械臂通话，所以 ESP 的 USB 串口（115200）
 *         还能当调试口看日志。想改用硬件串口（GPIO3/GPIO1）就把下面的
 *         ARM_LINK_SOFTWARE 改成 0——代价是 USB 串口监视器被机械臂占用。
 *
 * 编译：开发板选 "NodeMCU 1.0 (ESP-12E Module)"，直接上传即可。
 *
 * 串口调试输出（115200）：按下的键、发出去的命令、以及机械臂的每一行回包
 * （">>> …" 进度、"! …" 拒绝、开机的 "=== Task5 轨迹绘制 ==="），
 * 都带前缀打在 USB 串口上，接线/调试时看一眼就知道通没通。
 * ==========================================================================*/

#include <SoftwareSerial.h>

/* ====================== 1. 用户配置 ====================== */

#define ARM_BAUD           9600      /* 机械臂波特率（Task5 / V-test 都是 9600） */
#ifndef ARM_LINK_SOFTWARE
#define ARM_LINK_SOFTWARE  1         /* 1 = D5/D6 软串口（USB 留作调试，推荐）
                                      * 0 = GPIO3(RX)/GPIO1(TX) 硬件串口（最稳，
                                      *     但 USB 串口监视器就被机械臂占用了） */
#endif
#define PIN_ARM_RX         14        /* D5 <- 机械臂 TX（要分压） */
#define PIN_ARM_TX         12        /* D6 -> 机械臂 RX */

#define PIN_LED            2         /* D4 板载 LED（低电平点亮），发命令时闪一下 */
#define PIN_K1             5         /* D1 */
#define PIN_K2             4         /* D2 */
#define PIN_K3             13        /* D7 */
#define PIN_K4             0         /* D3（GPIO0：上电瞬间别按，见文件头注意） */

#define DEBOUNCE_MS        30        /* 消抖时间（与原按键板一致） */
#define CLICK_WINDOW_MS    350       /* 连击窗口：最后一次按下后静这么久就发命令 */
#define ECHO_ARM           1         /* 把机械臂回包转发到 USB 串口（调试用） */
#define DEBUG_BAUD         115200
#define LED_MS             40        /* 发命令时 LED 亮多久 */

/* ====================== 2. 按键表 ======================
 * 一行一个"第几次连击 → 哪个字符"。想加按键，就在下面表格里加行，
 * 并在上面的用户配置里补一个 PIN_Kx；想在某个键上加第 4 种连击，
 * 直接加一行 { PIN_Kx, 4, '9', "……" } 即可（连击数最多数到 4）。 */
struct KeyMap { uint8_t pin; uint8_t clicks; char cmd; const char *label; };

static const KeyMap KEYS[] = {
  { PIN_K1, 1, '1', "示教/记录" },
  { PIN_K2, 1, '2', "暂停" },
  { PIN_K2, 2, '3', "继续" },
  { PIN_K3, 1, '4', "取消" },
  { PIN_K3, 2, '5', "折线" },
  { PIN_K3, 3, '6', "曲线" },
  { PIN_K4, 1, '7', "直线演示" },
  { PIN_K4, 2, '8', "基础图形(字母 N)" },
};

#define NKEY (sizeof(KEYS) / sizeof(KEYS[0]))

/* ====================== 3. 全局状态 ====================== */

#if ARM_LINK_SOFTWARE
SoftwareSerial armSerial(PIN_ARM_RX, PIN_ARM_TX);   /* D5=RX, D6=TX */
  #define ARM_BEGIN()   armSerial.begin(ARM_BAUD)
  #define ARM_AVAIL()   armSerial.available()
  #define ARM_READ()    armSerial.read()
  #define ARM_WRITE(b)  armSerial.write((uint8_t)(b))
#else
  #define ARM_BEGIN()   Serial.begin(ARM_BAUD)
  #define ARM_AVAIL()   Serial.available()
  #define ARM_READ()    Serial.read()
  #define ARM_WRITE(b)  Serial.write((uint8_t)(b))
#endif

#define MAX_BTNS 6           /* 最多同时用几个按键引脚 */

static uint8_t  btnPin[MAX_BTNS];      /* 实际要轮询的引脚（从按键表里去重） */
static bool     btnStable[MAX_BTNS];   /* 消抖后的电平 */
static bool     btnRaw[MAX_BTNS];      /* 上一次读到的原始电平 */
static uint8_t  btnClicks[MAX_BTNS];   /* 累计连击数 */
static uint32_t btnChangeMs[MAX_BTNS]; /* 电平最后一次变化的时间 */
static uint32_t btnLastClickMs[MAX_BTNS];
static uint8_t  nBtn = 0;

static uint32_t ledUntil = 0;

/* ====================== 4. 收发 ====================== */

static void armSend(char c) {
  ARM_WRITE(c);
  ARM_WRITE('\n');
  digitalWrite(PIN_LED, LOW);          /* 板上 LED 低电平点亮 */
  ledUntil = millis() + LED_MS;
  Serial.print(F("> "));
  Serial.println(c);
}

static void armReadTick() {
  static char buf[96];
  static uint8_t len = 0;
  uint8_t budget = 48;
  while (budget-- && ARM_AVAIL()) {
    char c = (char)ARM_READ();
    if (c == '\n' || c == '\r') {
      if (len) {
        buf[len] = 0;
#if ECHO_ARM
        Serial.print(F("< "));
        Serial.println(buf);
#endif
        len = 0;
      }
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;                          /* 行太长就丢掉，重新计数 */
    }
  }
}

/* ====================== 5. 按键 ====================== */

static void addBtn(uint8_t pin) {
  for (uint8_t i = 0; i < nBtn; i++) if (btnPin[i] == pin) return;   /* 去重 */
  if (nBtn >= MAX_BTNS) return;
  btnPin[nBtn] = pin;
  nBtn++;
}

static void sendForClicks(uint8_t idx, uint8_t clicks) {
  for (uint8_t k = 0; k < NKEY; k++) {
    if (KEYS[k].pin == btnPin[idx] && KEYS[k].clicks == clicks) {
      Serial.print(F("[键] "));
      Serial.print(clicks);
      Serial.print(clicks == 1 ? F(" 单击") : clicks == 2 ? F(" 双击") : F(" 三击"));
      Serial.print(F(" → "));
      Serial.println(KEYS[k].label);
      armSend(KEYS[k].cmd);
      return;
    }
  }
  Serial.print(F("[键] "));
  Serial.print(clicks);
  Serial.println(F(" 连击 —— 这张表里没有定义，什么都没发"));
}

static void keysTick() {
  uint32_t now = millis();
  for (uint8_t i = 0; i < nBtn; i++) {
    bool raw = (digitalRead(btnPin[i]) == LOW);        /* 按下 = 低电平 */
    if (raw != btnRaw[i]) {                            /* 有变化 → 重新计时 */
      btnRaw[i] = raw;
      btnChangeMs[i] = now;
    } else if (now - btnChangeMs[i] >= DEBOUNCE_MS && raw != btnStable[i]) {
      btnStable[i] = raw;                              /* 消抖后确认 */
      if (raw) {                                       /* 按下的一刻算一次点击 */
        if (btnClicks[i] < 4) btnClicks[i]++;
        btnLastClickMs[i] = now;
      }
    }
    /* 连击窗口结束 → 按累计次数发命令 */
    if (btnClicks[i] && now - btnLastClickMs[i] >= CLICK_WINDOW_MS) {
      uint8_t n = btnClicks[i];
      btnClicks[i] = 0;
      sendForClicks(i, n);
    }
  }
}

/* ====================== 6. setup / loop ====================== */

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);                 /* 熄灭（低电平点亮） */

  for (uint8_t i = 0; i < NKEY; i++) addBtn(KEYS[i].pin);
  for (uint8_t i = 0; i < nBtn; i++) {
    pinMode(btnPin[i], INPUT_PULLUP);          /* 按键另一端接 GND */
    btnRaw[i] = btnStable[i] = false;
  }

  Serial.begin(DEBUG_BAUD);
  Serial.println();
  Serial.println(F("=== meArm 按键板（ESP8266）==="));
  Serial.print(F("机械臂串口："));
#if ARM_LINK_SOFTWARE
  Serial.print(F("软串口 D5(RX)/D6(TX)，"));
#else
  Serial.print(F("硬件串口 GPIO3(RX)/GPIO1(TX)，"));
#endif
  Serial.print(ARM_BAUD);
  Serial.println(F(" 8N1"));
  Serial.print(F("按键："));
  for (uint8_t i = 0; i < nBtn; i++) {
    Serial.print(F(" D"));
    Serial.print(btnPin[i]);
  }
  Serial.println(F("  （另一端接 GND，内部上拉）"));
  Serial.println(F("映射：K1×1=1 示教 | K2×1=2 暂停 K2×2=3 继续 | K3×1=4 取消 K3×2=5 折线 "
                   "K3×3=6 曲线 | K4×1=7 直线 K4×2=8 字母N"));

  ARM_BEGIN();
}

void loop() {
  keysTick();
  armReadTick();
  if (ledUntil && millis() >= ledUntil) { digitalWrite(PIN_LED, HIGH); ledUntil = 0; }
}
