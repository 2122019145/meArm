/* ============================================================================
 * meArm WiFi 遥控桥 —— ESP8266 (NodeMCU 1.0 / ESP-12E、Wemos D1 mini)
 * ----------------------------------------------------------------------------
 * 给 others/f/MeArm 的机械臂固件配一块"无线遥控板"（替代原来那块插满按键的 UNO）：
 *   - 把机械臂的串口接到 ESP8266，用手机/电脑浏览器控制（不用申请 App、不用联网）；
 *   - 网页上的按钮全部由本文件的按钮表生成，改一行就能改命令；
 *   - 自带串口控制台（telnet/网页）、状态监视、日志和"按固件要求排队发送"的机制；
 *   - 掉电不丢 WiFi 配置（存 EEPROM），连不上时自己开热点 + 强制门户配网。
 *
 * 默认识别的固件协议 = others/f/MeArm 的 **Task5 系列**（目录里最新的一版
 * Task5_LineN2_TeachFix.ino，轨迹绘制 + 摇杆示教）：
 *   串口 9600 8N1，一条命令一行（\n 结束），**单个 ASCII 字符**，大小写不敏感；
 *   1 示教/记录 | 2 暂停 | 3 继续 | 4 取消 | 5 折线 | 6 曲线 | 7 直线 | 8 基础图形(字母 N)
 *   调试同义键：t=示教 s=折线 c=曲线 g=直线 p=暂停/继续切换 x=取消 ?=状态 其它=指令表
 *   原按键板（KeypadBoard_Task5）的映射：键1×1=1 键2×1=2 键2×2=3 键3×1=4
 *                                       键3×2=5 键3×3=6 键4×1=7 键4×2=8
 *   回包：开机横幅 "=== Task5 轨迹绘制 ==="、进度行 ">>> …"、拒绝/错误行 "! …"；
 *   **没有 DONE/OK 完成行**，也没有角度指令 → 本预设不用发送队列、不显示关节滑条。
 *   注意：绘制/示教中途发 7/8 会被固件忽略并回 "! 绘制/示教中，忽略"。
 *
 * 另外内置两套固件的命令表，改一行 PRESET 就能换：
 *   PRESET_VTEST（others/RC-robot-hand/V-test v3.3 摇杆连续示教版，9600）：
 *   X 停止/保持 | D 舵机脱力 | STOP 结束录制 | ? T STATUS 状态 | P 暂停/继续回放
 *   H 摇杆加速 | L 摇杆减速 | K 摇杆停 | JOY 摇杆开 | CJ 爪随摇杆 | CL 锁爪
 *   E 使能并回 B90R90F90C120 + 开摇杆 | H0 G 缓慢回记录起点 | V PLAY 回放
 *   REC R 录制 | SAVE 存 EEPROM | LOAD 读 EEPROM | TRIM5 裁掉末 5 秒 | DUMP 导出
 *   CAL 摇杆校准 | J 原始 ADC | HELP 帮助
 *   B<R> / R<R> / F<R> / C<R> 绝对角度，一条一行（一次只接受一条：正在 MOVING 时回 BUSY）
 *   回包：STATE:... / B=.. R=.. F=.. C=.. / JOY:.. CLAW_LOCK:.. / RAM:.. / DONE:... / ERR:...
 *   （关节行程 B 0~180、R 10~170、F 20~150、C 50~130，见 V-test 的 Q_MIN/Q_MAX）
 *   PRESET_MEARM165（meArm 主固件 v1.6.5，115200）：
 *   O/S 开合爪、H/L 调速、x<d>,y<d>,z<d> 写角度、A/B/C 取放、N/R/P/M/0 录制回放、
 *   F/D/G/E/Q/U/W 绘图、p<z> 纸面高度、!/!P/!B/!D 查询
 *
 * 接线（机械臂 = Arduino Uno/ATmega328P，5V 逻辑；ESP8266 = 3.3V 逻辑）
 *   ┌──────────────┬─────────────────────────┬──────────────────────────────┐
 *   │ 机械臂 Uno   │ ESP8266                 │ 说明                          │
 *   ├──────────────┼─────────────────────────┼──────────────────────────────┤
 *   │ D1 (TX, 5V)  │ D5 / GPIO14  (分压后!)  │ 1kΩ 串联 + 2kΩ 到 GND 再进 ESP│
 *   │ D0 (RX)      │ D6 / GPIO12             │ 3.3V 直连可用（>=3.0V 即高）  │
 *   │ GND          │ GND                     │ 必须共地                      │
 *   │ 5V           │ VIN (经 AMS1117-3.3)    │ 不要用 Uno 的 3.3V 脚(≈50mA) │
 *   └──────────────┴─────────────────────────┴──────────────────────────────┘
 *   注意 1：Uno 的 D0/D1 同时接 USB 串口。用 USB 给 Uno 烧录时，USB 转串口和
 *          ESP 会同时驱动 D0 —— 烧录前请拔掉 ESP 的 D6 杜邦线（或加开关）。
 *   注意 2：ESP8266 峰值电流 300mA 以上，AMS1117 前最好并 470uF，否则会复位。
 *   注意 3：想彻底避开分压/共地麻烦，也可以在 Uno 和 ESP 之间放一块
 *          3.3V 双向电平转换模块，接线照上表即可。
 *
 * 编译/烧录
 *   1) Arduino IDE → 开发板管理器 安装 "esp8266 by ESP8266 Community"（本机已装 3.1.1）
 *   2) 打开本目录下的 meArm_wifi.ino（文件夹名 = 文件名，IDE 要求）
 *   3) 开发板选 "NodeMCU 1.0 (ESP-12E Module)"，端口选 ESP 的 USB，点上传
 *   4) 首次上电：15 秒连不上 WiFi 就自己开热点 meArm-XXXXXX（密码见 AP_PASSWORD），
 *      手机连上后会弹出配网页（或手动打开 http://192.168.4.1）
 *
 * 网页地址：http://mearm.local 或串口打印的 IP；telnet <IP> 23 是串口控制台
 * ==========================================================================*/

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <ArduinoOTA.h>

/* ====================== 1. 用户配置 ====================== */

#define PRESET_TASK5      3   /* others/f/MeArm 的 Task5 系列（9600，单字符指令 1~8 + g/t/s/c/p/x/?） */
#define PRESET_VTEST      1   /* V-test v3.3（RC-robot-hand 家族，9600） */
#define PRESET_MEARM165   2   /* meArm 主固件 v1.6.5（115200，x/y/z 方言） */
#ifndef PRESET
#define PRESET            PRESET_TASK5   /* 默认目标：others/f/MeArm 的 Task5 固件；
                                          * 编译时可用 -DPRESET=1 / -DPRESET=2 临时换表 */
#endif

#define ARM_BAUD          9600      /* 机械臂波特率：Task5=9600；V-test=9600；meArm v1.6.5=115200 */
#ifndef ARM_LINK_SOFTWARE
#define ARM_LINK_SOFTWARE 1         /* 1 = 用 D5/D6 软串口（USB 串口留给调试，推荐）
                                     * 0 = 用 GPIO3(RX)/GPIO1(TX) 硬件串口（最稳，
                                     *     但 USB 串口监视器就被机械臂占用了） */
#endif
#define PIN_ARM_RX        14        /* D5 <- 机械臂 TX（要分压） */
#define PIN_ARM_TX        12        /* D6 -> 机械臂 RX */

#define WIFI_SSID         ""        /* 留空 = 直接开热点；也可以在网页 /wifi 里填 */
#define WIFI_PASS         ""
#define AP_PASSWORD       "mearm1234"  /* 热点密码，至少 8 位；想开放热点改成 "" */
#define WIFI_CONNECT_MS   15000UL   /* STA 连接超时，超时后开热点 */
#define HOST_NAME         "mearm"   /* http://mearm.local */

#define ENABLE_TELNET     1         /* telnet <IP> 23 串口控制台 */
#define TELNET_PORT       23
#define ENABLE_OTA        1         /* Over-The-Air 空中升级 */
#define OTA_PASSWORD      "mearm1234"

#define STATUS_POLL_MS    3000UL    /* 网页自动发 "?" 问状态的周期；0 = 不自动问 */
#define LOG_LINES         28        /* 内存里保留多少行回包给网页看 */
#define LOG_LEN           72
#define QUEUE_MAX         12        /* 顺序发送队列长度（关节滑条"依次发送"用） */
#define QUEUE_REPLY_MS    3500UL    /* 队列里等 DONE/ERR/BUSY 的最长时间 */
#define QUEUE_SETTLE_MS   80UL      /* 两条命令之间的间隔 */

/* ====================== 2. 按钮表（网页上的按钮就是这张表） ======================
 * flags: 1 = 会动机器人（网页弹确认框）  2 = 红色强调
 * 想加/改按钮，只改这里；网页用 /buttons 读这张表，不用动 HTML。 */
struct Btn { const char *group; const char *label; const char *cmd; uint8_t flags; };

#if PRESET == PRESET_VTEST
static const Btn BTNS[] = {
  { "状态/安全", "查询状态",   "?",   0 },
  { "状态/安全", "急停·保持",  "X",   2 },
  { "状态/安全", "舵机脱力",   "D",   1 | 2 },
  { "状态/安全", "帮助",       "HELP", 0 },

  { "使能与位姿", "使能+回初始位姿", "E",  1 | 2 },
  { "使能与位姿", "回记录起点(H0)",  "H0", 1 },
  { "使能与位姿", "爪子 +1°",   "C+",  1 },
  { "使能与位姿", "爪子 −1°",   "C-",  1 },

  { "录制/回放", "开始录制",   "REC",   0 },
  { "录制/回放", "结束录制",   "STOP",  0 },
  { "录制/回放", "存 EEPROM",  "SAVE",  0 },
  { "录制/回放", "裁掉末 5 秒", "TRIM5", 0 },
  { "录制/回放", "读 EEPROM",  "LOAD",  0 },
  { "录制/回放", "回放",       "V",     1 | 2 },
  { "录制/回放", "暂停/继续",  "P",     0 },
  { "录制/回放", "导出记录",   "DUMP",  0 },

  { "摇杆/调速", "摇杆开",     "JOY",  0 },
  { "摇杆/调速", "摇杆停",     "K",    0 },
  { "摇杆/调速", "爪子随摇杆", "CJ",   0 },
  { "摇杆/调速", "锁住爪子",   "CL",   0 },
  { "摇杆/调速", "摇杆校准",   "CAL",  0 },
  { "摇杆/调速", "调速 +",     "H",    0 },
  { "摇杆/调速", "调速 −",     "L",    0 },
  { "摇杆/调速", "读 ADC",     "J",    0 },
};
struct JointDef { char name; int lo, hi, def; };
static const JointDef JOINTS[] = {
  { 'B',   0, 180,  90 },   /* 底座 */
  { 'R',  10, 170,  90 },   /* 大臂 */
  { 'F',  20, 150,  90 },   /* 小臂 */
  { 'C',  50, 130, 120 },   /* 爪子 */
};
#define HAS_JOINTS         1
#define JOINT_ONE_PER_LINE 1   /* V-test：一条命令只能带一个关节角，队列逐条发 */
#define POLL_CMD           "?" /* 自动查状态用的字符 */
#elif PRESET == PRESET_MEARM165
static const Btn BTNS[] = {
  { "状态/安全", "查询开关状态", "!",  0 },
  { "状态/安全", "查询位姿",     "!P", 0 },
  { "状态/安全", "查询按键",     "!B", 0 },
  { "状态/安全", "查询绘图",     "!D", 0 },
  { "状态/安全", "加速",         "H",  0 },
  { "状态/安全", "降速",         "L",  0 },
  { "状态/安全", "开爪",         "O",  0 },
  { "状态/安全", "合爪",         "S",  0 },

  { "取放(自动)", "取放 A", "A", 1 | 2 },
  { "取放(自动)", "取放 B", "B", 1 | 2 },
  { "取放(自动)", "取放 C", "C", 1 | 2 },
  { "取放(自动)", "循环取放", "M", 1 | 2 },
  { "取放(自动)", "停止", "0", 2 },

  { "录制/回放", "开始录制", "N", 1 },
  { "录制/回放", "回放",     "R", 1 },
  { "录制/回放", "回初始位姿", "P", 1 },

  { "绘图", "切换绘图任务", "F", 1 },
  { "绘图", "绘图开始", "G", 1 },
  { "绘图", "绘图取消", "E", 2 },
  { "绘图", "抬笔",     "Q", 0 },
  { "绘图", "落笔",     "U", 0 },
  { "绘图", "示教点",   "W", 0 },
  { "绘图", "纸面 z=-20", "p-20", 1 },
};
struct JointDef { char name; int lo, hi, def; };
static const JointDef JOINTS[] = {
  { 'x', 0, 180, 90 },
  { 'y', 0, 180, 90 },
  { 'z', 0, 180, 90 },
};
#define HAS_JOINTS         1
#define JOINT_ONE_PER_LINE 0   /* meArm v1.6.5：x,y,z 一条命令写完 */
#define POLL_CMD           "!" /* meArm v1.6.5：查询开关状态 */
#else   /* ===== PRESET_TASK5：others/f/MeArm 的 Task5 轨迹绘制固件 =====
         * 协议：9600 8N1，单个 ASCII 字符 + 换行；没有角度指令 → 没有关节滑条。
         * 原按键板 `KeypadBoard_Task5` 的映射写在标签里：
         *   键1×1=1 示教/记录   键2×1=2 暂停   键2×2=3 继续   键3×1=4 取消
         *   键3×2=5 折线       键3×3=6 曲线   键4×1=7 直线   键4×2=8 基础图形(字母N)
         * 调试同义键：t=示教 s=折线 c=曲线 g=直线 p=暂停/继续切换 x=取消 ?=状态 */
static const Btn BTNS[] = {
  { "绘制", "直线演示 (键4×1)",   "7", 1 | 2 },
  { "绘制", "字母 N (键4×2)",     "8", 1 | 2 },
  { "绘制", "折线 (键3×2)",       "5", 1 },
  { "绘制", "曲线 (键3×3)",       "6", 1 },

  { "示教", "进入示教/记录点 (键1)", "1", 1 },

  { "控制", "暂停 (键2×1)",       "2", 0 },
  { "控制", "继续 (键2×2)",       "3", 0 },
  { "控制", "取消·回待机 (键3×1)", "4", 2 },

  { "状态", "状态查询",           "?", 0 },
  { "状态", "指令表/帮助",        "h", 0 },

  { "调试", "直线 (调试 g)",      "g", 1 },
  { "调试", "折线 (调试 s)",      "s", 1 },
  { "调试", "曲线 (调试 c)",      "c", 1 },
  { "调试", "暂停/继续切换 (调试 p)", "p", 0 },
  { "调试", "取消 (调试 x)",      "x", 2 },
  { "调试", "示教 (调试 t)",      "t", 1 },
};
#define HAS_JOINTS         0   /* Task5 没有角度指令 → 网页隐藏关节滑条区 */
#define JOINT_ONE_PER_LINE 0
#define POLL_CMD           "?" /* 自动查状态 */
#endif

#define NBTN   (sizeof(BTNS) / sizeof(BTNS[0]))
#if HAS_JOINTS
#define NJOINT (sizeof(JOINTS) / sizeof(JOINTS[0]))
#else
#define NJOINT 0
#endif

/* ====================== 3. 串口 & 全局状态 ====================== */

#if ARM_LINK_SOFTWARE
SoftwareSerial armSerial(PIN_ARM_RX, PIN_ARM_TX);   /* D5=RX, D6=TX */
  #define ARM_BEGIN()     armSerial.begin(ARM_BAUD)
  #define ARM_AVAIL()     armSerial.available()
  #define ARM_READ()      armSerial.read()
  #define ARM_WRITE(b)    armSerial.write((uint8_t)(b))
static void dbgBegin() { Serial.begin(115200); Serial.println(); }
static void dbg(const char *s) { Serial.print(F("[esp] ")); Serial.println(s); }
#else
  #define ARM_BEGIN()     Serial.begin(ARM_BAUD)
  #define ARM_AVAIL()     Serial.available()
  #define ARM_READ()      Serial.read()
  #define ARM_WRITE(b)    Serial.write((uint8_t)(b))
static void dbgBegin() {}
static void dbg(const char *) {}
#endif

ESP8266WebServer server(80);
DNSServer        dns;
#if ENABLE_TELNET
WiFiServer       telnet(TELNET_PORT);
WiFiClient       tnClients[4];
#endif

static char logRing[LOG_LINES][LOG_LEN];
static uint8_t logCount = 0, logHead = 0;
static uint32_t logSeq = 0;

static char armState[24]  = "-";      /* STATE: 后面的内容 */
static char armPose[48]   = "-";      /* "B=90.00 R=90.00 F=90.00 C=120.00" */
static char armJoy[24]    = "-";      /* "JOY:0 CLAW_LOCK:1" 的原始行 */
static char armRam[40]    = "-";      /* "RAM:1 samples=300 ms=30000" */
static char armMsg[96]    = "-";      /* 最近一条 ">>> ..." / "! ..." 消息（Task5 预设用） */
#if PRESET == PRESET_TASK5
static char penPos[48]    = "-";      /* Task5：从 "?" 回包里解析出的笔尖坐标 */
static int  teachCount    = 0;        /* Task5：已记录示教点数 */
#endif
static char armIp[20]     = "-";
static bool apMode        = false;
static bool autoPoll      = true;      /* 网页自动查状态开关，/auto 可改 */
static unsigned long lastPoll = 0;

static void pushLog(const char *s);
static void broadcastTelnet(const char *s);

#if PRESET == PRESET_TASK5
/* 按 UTF-8 边界安全截断地拷贝（避免把一个汉字切成半个，网页显示成乱码方块） */
static void copyUtf8Safe(char *dst, size_t cap, const char *src) {
  size_t n = strlen(src);
  if (n > cap - 1) {
    n = cap - 1;
    while (n && ((unsigned char)src[n] & 0xC0) == 0x80) n--;   /* 退到字符开头 */
  }
  memcpy(dst, src, n);
  dst[n] = 0;
}

static void setArmState(const char *s) { copyUtf8Safe(armState, sizeof(armState), s); }
#endif

/* ====================== 4. 日志环 ====================== */

static void pushLog(const char *s) {
  char *dst = logRing[logHead];
  strncpy(dst, s, LOG_LEN - 1);
  dst[LOG_LEN - 1] = 0;
  logHead = (uint8_t)((logHead + 1) % LOG_LINES);
  if (logCount < LOG_LINES) logCount++;
  logSeq++;
  broadcastTelnet(s);
}

static void espLog(const char *fmt, ...) {
  char buf[LOG_LEN];
  va_list ap; va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  pushLog(buf);
}

/* ====================== 5. 机械臂串口 ====================== */

static char rxBuf[96];
static uint8_t rxLen = 0;
static bool rxDrop = false;

/* 顺序发送队列：关节滑条"依次发送"、以及任何需要"发一条等一条"的场合 */
static char    qText[QUEUE_MAX][24];
static uint8_t qHead = 0, qCount = 0;
static bool    qBusy = false;
static unsigned long qDeadline = 0, qNext = 0;

static void armSendRaw(const char *cmd) {
  while (*cmd) ARM_WRITE(*cmd++);
  ARM_WRITE('\n');                     /* V-test 接受 \n 或 \r；只发一个，避免空行 */
  espLog("> %s", cmd);
}

static bool queueFull() { return qCount >= QUEUE_MAX; }

static void queuePush(const char *cmd) {
  if (queueFull()) { pushLog("[esp] 队列已满，丢弃："); return; }
  char *dst = qText[(qHead + qCount) % QUEUE_MAX];
  strncpy(dst, cmd, sizeof(qText[0]) - 1);
  dst[sizeof(qText[0]) - 1] = 0;
  qCount++;
}

static void queueClear() { qCount = 0; qBusy = false; }

static void queueTick() {
  unsigned long now = millis();
  if (qBusy) {
    if (now >= qDeadline) { qBusy = false; pushLog("[esp] 等回包超时，继续下一条"); qNext = now + QUEUE_SETTLE_MS; }
    return;
  }
  if (!qCount) return;
  if (now < qNext) return;
  char *line = qText[qHead];
  qHead = (uint8_t)((qHead + 1) % QUEUE_MAX);
  qCount--;
  armSendRaw(line);
  qBusy = true;
  qDeadline = now + QUEUE_REPLY_MS;
}

/* 收到一行机械臂回包 */
static void handleArmLine(char *line) {
  pushLog(line);

  if (!strncmp(line, "STATE:", 6)) { strncpy(armState, line + 6, sizeof(armState) - 1); armState[sizeof(armState)-1]=0; }
  else if (!strncmp(line, "JOY:", 4)) { strncpy(armJoy, line, sizeof(armJoy) - 1); armJoy[sizeof(armJoy)-1]=0; }
  else if (!strncmp(line, "RAM:", 4)) { strncpy(armRam, line, sizeof(armRam) - 1); armRam[sizeof(armRam)-1]=0; }
  else if (line[0] && line[1] == '=') { strncpy(armPose, line, sizeof(armPose) - 1); armPose[sizeof(armPose)-1]=0; }

#if PRESET == PRESET_TASK5
  /* Task5 固件不打印 STATE:/B=…，只用 ">>> xxx"（进行中）和 "! xxx"（拒绝/错误）
   * 说话。这里把它的中文进度行翻译成状态，并把最后一条挂到 armMsg 上给网页显示。 */
  if (!strncmp(line, ">>> ", 4) || line[0] == '!') copyUtf8Safe(armMsg, sizeof(armMsg), line);
  if      (strstr(line, "示教模式"))                                                     setArmState("示教中");
  else if (strstr(line, "绘制开始") || strstr(line, "直线绘制演示") ||
           strstr(line, "基础图形绘制"))                                                 setArmState("绘制中");
  else if (strstr(line, "已暂停"))                                                       setArmState("已暂停");
  else if (strstr(line, "已继续"))                                                       setArmState("绘制中");
  else if (strstr(line, "已取消") || strstr(line, "绘制结束") || strstr(line, "绘制完成")) setArmState("待机");

  /* "?" 查询的回包（一行）：状态:0 示教点数:2  笔尖(x,y,z): 12.3, 45.6, 7.8
   * 这是最权威的状态来源（0待机 1示教 2绘制 3暂停），顺便把笔尖坐标也抓出来。 */
  const char *ps = strstr(line, "状态:");
  if (ps) {
    int n = atoi(ps + strlen("状态:"));   /* 0待机 1示教 2绘制 3暂停 */
    setArmState(n == 1 ? "示教中" : n == 2 ? "绘制中" : n == 3 ? "已暂停" : "待机");
  }
  const char *pt = strstr(line, "示教点数:");
  if (pt) teachCount = atoi(pt + strlen("示教点数:"));
  const char *pp = strstr(line, "笔尖(x,y,z):");
  if (pp) snprintf(penPos, sizeof(penPos), "笔尖%.*s",
                   (int)sizeof(penPos) - 8, pp + strlen("笔尖(x,y,z):"));
#endif

  if (!strncmp(line, "DONE", 4) || !strncmp(line, "ERR", 3) || !strncmp(line, "BUSY", 4) ||
      !strncmp(line, "OK", 2)    || !strncmp(line, "REJECTED", 8) ||
      !strncmp(line, "PAUSED", 6) || !strncmp(line, "RESUMED", 7)) {
    if (qBusy) { qBusy = false; qNext = millis() + QUEUE_SETTLE_MS; }
  }
}

static void armReadTick() {
  uint8_t budget = 48;                       /* 一帧最多处理这么多字节，别饿死 WiFi */
  while (budget-- && ARM_AVAIL()) {
    char c = (char)ARM_READ();
    if (c == '\n' || c == '\r') {
      if (rxDrop) espLog("[esp] 回包过长，已丢弃");
      else if (rxLen) { rxBuf[rxLen] = 0; handleArmLine(rxBuf); }
      rxLen = 0; rxDrop = false;
    } else if (!rxDrop) {
      if (rxLen < sizeof(rxBuf) - 1) rxBuf[rxLen++] = c;
      else rxDrop = true;
    }
  }
}

/* ====================== 6. telnet 控制台 ====================== */

static void broadcastTelnet(const char *s) {
#if ENABLE_TELNET
  for (uint8_t i = 0; i < 4; i++) if (tnClients[i] && tnClients[i].connected()) tnClients[i].println(s);
#endif
}

static void telnetTick() {
#if ENABLE_TELNET
  if (telnet.hasClient()) {
    WiFiClient c = telnet.accept();
    int8_t slot = -1;
    for (uint8_t i = 0; i < 4; i++) if (!tnClients[i] || !tnClients[i].connected()) { slot = i; break; }
    if (slot >= 0) { tnClients[slot] = c; tnClients[slot].println(F("meArm telnet console. 直接输命令，回车发送。")); }
    else c.stop();
  }
  static char tb[64]; static uint8_t tl = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (!tnClients[i] || !tnClients[i].connected()) continue;
    while (tnClients[i].available()) {
      char c = (char)tnClients[i].read();
      if (c == '\n' || c == '\r') { if (tl) { tb[tl] = 0; armSendRaw(tb); tl = 0; } }
      else if (tl < sizeof(tb) - 1) tb[tl++] = c;
    }
  }
#endif
}

/* ====================== 7. WiFi 配置（EEPROM） ====================== */

struct WifiCfg { uint32_t magic; char ssid[33]; char pass[65]; };
static const uint32_t CFG_MAGIC = 0x4D524D31UL;   /* "MRM1" */
static WifiCfg cfg;
static char apSsid[24];

static void cfgLoad() {
  EEPROM.begin(sizeof(WifiCfg) + 8);
  EEPROM.get(0, cfg);
  if (cfg.magic != CFG_MAGIC) { cfg.magic = CFG_MAGIC; cfg.ssid[0] = 0; cfg.pass[0] = 0; }
  cfg.ssid[32] = 0; cfg.pass[64] = 0;
}
static void cfgSave() { EEPROM.put(0, cfg); EEPROM.commit(); }

static void wifiStart() {
  snprintf(apSsid, sizeof(apSsid), "meArm-%06X", ESP.getChipId());
  WiFi.persistent(false);
  WiFi.hostname(HOST_NAME);

  const char *ssid = cfg.ssid[0] ? cfg.ssid : WIFI_SSID;
  const char *pass = cfg.pass[0] ? cfg.pass : WIFI_PASS;

  if (ssid[0]) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, pass);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_CONNECT_MS) { delay(200); }
  }
  if (WiFi.status() == WL_CONNECTED) {
    apMode = false;
    snprintf(armIp, sizeof(armIp), "%s", WiFi.localIP().toString().c_str());
    dbg("STA ok");
    pushLog("[esp] WiFi 已连接");
  } else {
    apMode = true;
    WiFi.mode(WIFI_AP_STA);
    if (AP_PASSWORD[0]) WiFi.softAP(apSsid, AP_PASSWORD);
    else                WiFi.softAP(apSsid);
    dns.start(53, "*", WiFi.softAPIP());
    snprintf(armIp, sizeof(armIp), "%s", WiFi.softAPIP().toString().c_str());
    dbg("AP mode");
    pushLog("[esp] 已开热点");
  }
  MDNS.begin(HOST_NAME);
  MDNS.addService("http", "tcp", 80);
#if ENABLE_TELNET
  telnet.begin();
  telnet.setNoDelay(true);
#endif
  espLog("[esp] 网页 http://%s/", armIp);
}

/* ====================== 8. 网页 ====================== */

static const char PAGE[] PROGMEM = R"HTML(<!doctype html><html lang="zh"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>meArm WiFi 遥控</title><style>
*{box-sizing:border-box}body{margin:0;font:15px/1.45 system-ui,"Microsoft YaHei",sans-serif;background:#12151a;color:#e8eaf0}
header{padding:10px 14px;background:#1b2027;display:flex;flex-wrap:wrap;gap:10px;align-items:center;position:sticky;top:0;z-index:9}
h1{font-size:17px;margin:0}.pill{font:12px/1 monospace;background:#232a33;padding:5px 8px;border-radius:99px}
main{padding:12px;max-width:960px;margin:0 auto}h2{font-size:14px;color:#8fa4c0;margin:16px 0 6px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(120px,1fr));gap:8px}
button{font:14px system-ui;padding:11px 6px;border:0;border-radius:9px;background:#2a3340;color:#e8eaf0;cursor:pointer}
button:active{transform:scale(.97)}button.safe:hover{background:#33415a}button.danger{background:#5a2a2f}
button.danger:hover{background:#7a3239}button.stop{background:#8c2020;font-weight:700}
#status{display:grid;grid-template-columns:repeat(auto-fill,minmax(160px,1fr));gap:6px;font:13px/1.4 monospace}
#status div{background:#1b2027;padding:7px 9px;border-radius:8px;overflow:hidden}
.j{margin:9px 0;background:#1b2027;padding:9px 11px;border-radius:9px}
.j b{display:inline-block;width:1.4em}.j input[type=range]{width:min(70%,520px);vertical-align:middle}
.j span{font:13px monospace;display:inline-block;min-width:3.4em;text-align:right;color:#7fd1a0}
#log{background:#0c0e12;border:1px solid #232a33;border-radius:9px;padding:8px;height:230px;overflow:auto;
     font:12px/1.5 monospace;white-space:pre-wrap;word-break:break-all}
.row{display:flex;gap:8px;margin:8px 0;flex-wrap:wrap}.row input[type=text]{flex:1;min-width:180px;padding:10px;
     border-radius:9px;border:1px solid #2a3340;background:#0c0e12;color:#e8eaf0;font:14px monospace}
label{font-size:13px;color:#9fb0c8}a{color:#7fb0ff}
</style></head><body>
<header><h1>meArm WiFi 遥控</h1><span class="pill" id="pState">STATE —</span>
<span class="pill" id="pPose">B — R — F — C —</span><span class="pill" id="pNet">…</span>
<span class="pill" id="pQ">队列 0</span></header>
<main>
<div id="status"></div>
<div id="btns"></div>
<div id="jsec"><h2 id="jtitle">四关节（拖动后点“发送”或“依次发送”）</h2>
<div id="joints"></div></div>
<div class="row"><button class="safe" id="bAll" onclick="sendAll()">依次发送全部关节</button>
<button class="safe" onclick="fetch('/queue?clear=1')">清空队列</button>
<label><input type="checkbox" id="auto" checked onchange="setAuto()"> 自动查状态（每 3 秒）</label></div>
<h2>手动命令 / 串口控制台</h2>
<div class="row"><input type="text" id="cmd" placeholder="例如 B90 或 REC 或 DUMP" onkeydown="if(event.key==='Enter')sendBox()">
<button class="safe" onclick="sendBox()">发送</button><button class="safe" onclick="document.getElementById('log').textContent=''">清屏</button>
<a href="/log.txt" download="mearm_log.txt"><button class="safe">下载日志</button></a>
<a href="/wifi"><button class="safe">WiFi 设置</button></a></div>
<div id="log"></div>
</main><script>
const $=id=>document.getElementById(id);
let joints=[],cur=[],one=1;
function esc(s){return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));}
function line(s){const l=$('log');const at=l.scrollTop+l.clientHeight>=l.scrollHeight-24;
  l.textContent+=s+'\n';if(l.textContent.length>20000)l.textContent=l.textContent.slice(-16000);if(at)l.scrollTop=l.scrollHeight;}
async function send(c,danger,quiet){
  if(danger&&!confirm('确认发送「'+c+'」？机械臂会动作！'))return false;
  line('> '+c);await fetch('/cmd?c='+encodeURIComponent(c));return true;}
function sendBox(){const v=$('cmd').value.trim();if(v){send(v,false);$('cmd').value='';}}
async function loadButtons(){const a=await(await fetch('/buttons')).json();const box=$('btns');box.innerHTML='';
  const g={};for(const b of a)(g[b.g]=g[b.g]||[]).push(b);
  for(const k in g){const h=document.createElement('h2');h.textContent=k;box.appendChild(h);
    const d=document.createElement('div');d.className='grid';
    for(const b of g[k]){const el=document.createElement('button');
      el.textContent=b.l;el.className=(b.f&2)?'stop':(b.d?'danger':'safe');
      el.onclick=()=>send(b.c,!!b.d);d.appendChild(el);}
    box.appendChild(d);}}
async function loadJoints(){const d=await(await fetch('/joints')).json();one=d.one;joints=d.list;
  const box=$('joints');box.innerHTML='';cur=joints.map(j=>j.v);
  if(!joints.length){$('jsec').style.display='none';$('bAll').style.display='none';return;}
  $('jtitle').textContent=one?'四关节（拖动后点“发送”，或“依次发送”排队）':'关节（拖动后点“发送”，一次发全部）';
  joints.forEach((j,i)=>{const e=document.createElement('div');e.className='j';
    e.innerHTML='<b>'+j.n+'</b><input type="range" min="'+j.lo+'" max="'+j.hi+'" value="'+j.v+'" id="r'+i+'">'+
      '<span id="v'+i+'">'+j.v+'°</span> <button class="safe" onclick="sendOne('+i+')">发送</button>';
    box.appendChild(e);
    e.querySelector('input').addEventListener('input',ev=>{$('v'+i).textContent=ev.target.value+'°';cur[i]=+ev.target.value;});});}
function vals(){return joints.map((j,i)=>+$('r'+i).value);}
async function sendOne(i){const v=+$('r'+i).value;line('> '+joints[i].n+v);
  if(one)await fetch('/joint?i='+i+'&v='+v);
  else await fetch('/joint?i='+i+'&v='+v+'&all='+vals().join(','));}
async function sendAll(){
  const v=vals();
  if(one){
    if(!confirm('依次发送全部关节角？机械臂会动作！'))return;
    for(let i=0;i<joints.length;i++)await fetch('/joint?i='+i+'&v='+v[i]+'&q=1');
    line('> 已排入队列：'+joints.map((j,i)=>j.n+v[i]).join(' '));
  }else{
    if(!confirm('发送全部关节角？机械臂会动作！'))return;
    await fetch('/joint?i=0&v='+v[0]+'&all='+v.join(','));
    line('> '+joints.map((j,i)=>j.n+v[i]).join(','));}}
function setAuto(){fetch('/auto?on='+($('auto').checked?1:0));}
async function poll(){
  try{const s=await(await fetch('/state')).json();
    const short=t=>t&&t.length>30?t.slice(0,30)+'…':t;
    $('pState').textContent=s.state;$('pPose').textContent=short(s.pose);
    $('pNet').textContent=s.net;$('pQ').textContent='队列 '+s.q;
    const c=s.cards||[['状态',s.state],['关节角',s.pose],['摇杆/爪',s.joy],['记录',s.ram],
      ['WiFi',s.net],['堆内存',s.heap+' B'],['运行',s.up+' s'],['队列',s.q+' 条']];
    $('status').innerHTML=c.map(x=>'<div>'+esc(x[0])+'<br><b>'+esc(x[1])+'</b></div>').join('');
    if(s.new&&s.new.length)s.new.forEach(l=>line(l));
  }catch(e){$('pNet').textContent='连接断开…';}}
loadButtons();loadJoints();poll();
setInterval(poll,1200);setAuto();
</script></body></html>)HTML";

static String jsonEscape(const char *s) {
  String out; out.reserve(strlen(s) + 8);
  for (const char *p = s; *p; p++) {
    char c = *p;
    if (c == '"' || c == '\\') { out += '\\'; out += c; }
    else if ((uint8_t)c < 0x20) { out += ' '; }
    else out += c;
  }
  return out;
}

static void handleRoot()   { server.send_P(200, "text/html; charset=utf-8", PAGE); }
static void handleFavicon(){ server.send(204, "text/plain", ""); }

static void handleButtons() {
  String j; j.reserve(2200); j += '[';
  for (size_t i = 0; i < NBTN; i++) {
    if (i) j += ',';
    j += "{\"g\":\""; j += jsonEscape(BTNS[i].group);
    j += "\",\"l\":\""; j += jsonEscape(BTNS[i].label);
    j += "\",\"c\":\""; j += jsonEscape(BTNS[i].cmd);
    j += "\",\"f\":"; j += BTNS[i].flags;
    j += ",\"d\":"; j += (BTNS[i].flags & 1) ? 1 : 0; j += '}';
  }
  j += ']';
  server.send(200, "application/json", j);
}

static void handleJoints() {
  /* one=1：固件一次只接受一个关节角（V-test），网页需要逐条排队发送；
   * one=0：x,y,z 可一条命令写完（meArm v1.6.5），网页再一次发全部。
   * Task5 固件（HAS_JOINTS=0）根本没有角度指令 → 回空表，网页自动隐藏滑条区。 */
#if HAS_JOINTS
  String j; j.reserve(420); j += F("{\"one\":"); j += JOINT_ONE_PER_LINE; j += F(",\"list\":[");
  for (size_t i = 0; i < NJOINT; i++) {
    if (i) j += ',';
    j += "{\"n\":\""; j += JOINTS[i].name; j += "\",\"lo\":";
    j += JOINTS[i].lo; j += ",\"hi\":"; j += JOINTS[i].hi;
    j += ",\"v\":"; j += JOINTS[i].def; j += '}';
  }
  j += "]}";
  server.send(200, "application/json", j);
#else
  server.send(200, "application/json", "{\"one\":0,\"list\":[]}");
#endif
}

static uint32_t lastSentSeq = 0;      /* 网页上次已经取到第几行日志 */

static void handleState() {
  /* 网络那格先拼出来 */
  String net;
  if (apMode) { net += F("AP "); net += apSsid; }
  else        { net += F("STA "); net += WiFi.SSID(); net += ' '; net += WiFi.RSSI(); net += F("dBm"); }
  net += ' '; net += armIp;

  String j; j.reserve(1900);
  j += F("{\"state\":\""); j += jsonEscape(armState);
  j += F("\",\"pose\":\"");
#if PRESET == PRESET_TASK5
  j += jsonEscape(strcmp(penPos, "-") ? penPos : armMsg);   /* Task5：优先显示笔尖坐标 */
#else
  j += jsonEscape(armPose);
#endif
  j += F("\",\"msg\":\"");  j += jsonEscape(armMsg);
  j += F("\",\"joy\":\"");  j += jsonEscape(armJoy);
  j += F("\",\"ram\":\"");  j += jsonEscape(armRam);
  j += F("\",\"q\":");      j += (int)qCount;
  j += F(",\"heap\":");     j += ESP.getFreeHeap();
  j += F(",\"up\":");       j += (uint32_t)(millis() / 1000UL);
  j += F(",\"net\":\"");    j += jsonEscape(net.c_str()); j += '"';

  /* 状态卡片：内容随预设不同，网页只管按这个数组渲染 */
  j += F(",\"cards\":[[\"状态\",\""); j += jsonEscape(armState); j += '"';
#if PRESET == PRESET_TASK5
  j += F("],[\"当前动作\",\""); j += jsonEscape(armMsg); j += '"';
  j += F("],[\"笔尖坐标\",\""); j += jsonEscape(penPos); j += '"';
  j += F("],[\"示教点\",\"");   j += teachCount; j += F(" 个\"");
#else
  j += F("],[\"关节角\",\""); j += jsonEscape(armPose); j += '"';
  j += F("],[\"摇杆/爪\",\""); j += jsonEscape(armJoy); j += '"';
  j += F("],[\"记录\",\"");    j += jsonEscape(armRam); j += '"';
#endif
  j += F("],[\"WiFi\",\"");    j += jsonEscape(net.c_str()); j += '"';
  j += F("],[\"堆内存\",\" "); j += ESP.getFreeHeap(); j += F(" B\"");
  j += F("],[\"运行\",\" ");   j += (uint32_t)(millis() / 1000UL); j += F(" s\"");
  j += F("],[\"队列\",\" ");   j += (int)qCount; j += F(" 条\"]]");

  /* 只把"网页没看过的新行"发过去，省流量也省解析 */
  uint32_t from = lastSentSeq;
  if (logSeq - from > LOG_LINES) from = logSeq - LOG_LINES;
  j += F(",\"new\":[");
  bool first = true;
  for (uint32_t s = from; s < logSeq; s++) {
    uint8_t idx = (uint8_t)(s % LOG_LINES);
    if (!first) j += ',';
    first = false;
    j += '"'; j += jsonEscape(logRing[idx]); j += '"';
  }
  j += F("]}");
  lastSentSeq = logSeq;
  server.send(200, "application/json", j);
}

static void handleCmd() {
  if (!server.hasArg("c")) { server.send(400, "text/plain", "need c"); return; }
  if (server.hasArg("q")) queuePush(server.arg("c").c_str());
  else                    armSendRaw(server.arg("c").c_str());
  server.send(200, "application/json", "{\"ok\":1}");
}

/* 关节：i=<序号>&v=<角度>[&all=v0,v1,v2][&q=1 只排队不立刻发]
 * one=1 的固件（V-test）一次只接受一个关节角；one=0（meArm v1.6.5）用网页带来的
 * 全部滑条值拼成一条 "x<d>,y<d>,z<d>" 发出去。 */
static void handleJoint() {
#if !HAS_JOINTS
  server.send(400, "text/plain", "this preset has no joint command");
#else
  if (!server.hasArg("i") || !server.hasArg("v")) { server.send(400, "text/plain", "need i,v"); return; }
  int i = server.arg("i").toInt();
  int v = server.arg("v").toInt();
  if (i < 0 || i >= (int)NJOINT) { server.send(400, "text/plain", "bad i"); return; }

  const JointDef &jd = JOINTS[i];
  if (v < jd.lo) v = jd.lo;
  if (v > jd.hi) v = jd.hi;

  char line[24];
#if JOINT_ONE_PER_LINE
  snprintf(line, sizeof(line), "%c%d", jd.name, v);
  if (server.hasArg("q")) queuePush(line);
  else                    armSendRaw(line);
#else
  int vals[NJOINT];
  for (size_t k = 0; k < NJOINT; k++) vals[k] = JOINTS[k].def;
  if (server.hasArg("all")) {
    String a = server.arg("all");
    int start = 0;
    for (size_t k = 0; k < NJOINT; k++) {
      int comma = a.indexOf(',', start);
      String piece = (comma < 0) ? a.substring(start) : a.substring(start, comma);
      if (piece.length()) vals[k] = piece.toInt();
      if (comma < 0) break;
      start = comma + 1;
    }
  }
  vals[i] = v;
  snprintf(line, sizeof(line), "%c%d,%c%d,%c%d",
           JOINTS[0].name, vals[0], JOINTS[1].name, vals[1], JOINTS[2].name, vals[2]);
  armSendRaw(line);
#endif
  server.send(200, "application/json", "{\"ok\":1}");
#endif   /* HAS_JOINTS */
}

static void handleQueue() {
  if (server.hasArg("clear")) queueClear();
  String j = "{\"q\":"; j += (int)qCount; j += '}';
  server.send(200, "application/json", j);
}

static void handleAuto() {
  /* 网页上的"自动问状态"开关。V-test 的 ? 和 meArm 的 ! 都只是查询，不会打断动作 */
  if (server.hasArg("on")) autoPoll = (server.arg("on").toInt() != 0);
  String j = "{\"auto\":"; j += autoPoll ? 1 : 0; j += '}';
  server.send(200, "application/json", j);
}

static void handleLogTxt() {
  String t; t.reserve(LOG_LINES * (LOG_LEN + 2));
  uint32_t from = (logSeq > LOG_LINES) ? logSeq - LOG_LINES : 0;
  for (uint32_t s = from; s < logSeq; s++) {
    uint8_t idx = (uint8_t)(s % LOG_LINES);
    t += logRing[idx]; t += "\r\n";
  }
  server.send(200, "text/plain; charset=utf-8", t);
}

static void handleWifiPage() {
  String h = F("<!doctype html><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
               "<style>body{font:15px system-ui;background:#12151a;color:#e8eaf0;padding:18px;max-width:520px;margin:auto}"
               "input{width:100%;padding:10px;margin:6px 0 14px;border-radius:8px;border:1px solid #2a3340;background:#0c0e12;color:#e8eaf0}"
               "button{padding:11px 16px;border:0;border-radius:9px;background:#2a5a3a;color:#fff;font-size:15px}a{color:#7fb0ff}</style>"
               "<h2>WiFi 设置</h2><p>当前：");
  h += apMode ? apSsid : WiFi.SSID().c_str();
  h += F("<br>留空保存 = 清除已存配置，下次上电直接开热点。</p><form method=post action=/wifi>"
         "<label>SSID</label><input name=ssid value='");
  h += cfg.ssid;
  h += F("'><label>密码</label><input name=pass type=password value='");
  h += cfg.pass;
  h += F("'><button>保存并重启</button></form><p><a href=/>返回遥控页</a></p>");
  server.send(200, "text/html; charset=utf-8", h);
}

static void handleWifiSave() {
  strncpy(cfg.ssid, server.arg("ssid").c_str(), sizeof(cfg.ssid) - 1); cfg.ssid[sizeof(cfg.ssid) - 1] = 0;
  strncpy(cfg.pass, server.arg("pass").c_str(), sizeof(cfg.pass) - 1); cfg.pass[sizeof(cfg.pass) - 1] = 0;
  cfg.magic = CFG_MAGIC;
  cfgSave();
  server.send(200, "text/html; charset=utf-8", F("<meta charset=utf-8>已保存，正在重启…<script>setTimeout(()=>location='/',4000)</script>"));
  delay(600);
  ESP.restart();
}

static void handleReboot() {
  server.send(200, "text/plain; charset=utf-8", "rebooting");
  delay(300);
  ESP.restart();
}

static void handleNotFound() {
  if (apMode) { server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true); server.send(302, "text/plain", ""); }
  else        { server.send(404, "text/plain", "not found"); }
}

/* ====================== 9. setup / loop ====================== */

void setup() {
  dbgBegin();
  cfgLoad();
  logCount = 0; logHead = 0; logSeq = 0;

  ARM_BEGIN();
  wifiStart();

  server.on("/",        HTTP_GET,  handleRoot);
  server.on("/favicon.ico", HTTP_GET, handleFavicon);
  server.on("/buttons", HTTP_GET,  handleButtons);
  server.on("/joints",  HTTP_GET,  handleJoints);
  server.on("/state",   HTTP_GET,  handleState);
  server.on("/cmd",     HTTP_GET,  handleCmd);
  server.on("/joint",   HTTP_GET,  handleJoint);
  server.on("/queue",   HTTP_GET,  handleQueue);
  server.on("/auto",    HTTP_GET,  handleAuto);
  server.on("/log.txt", HTTP_GET,  handleLogTxt);
  server.on("/wifi",    HTTP_GET,  handleWifiPage);
  server.on("/wifi",    HTTP_POST, handleWifiSave);
  server.on("/reboot",  HTTP_GET,  handleReboot);
  server.onNotFound(handleNotFound);

  server.begin();
  server.enableCORS(true);

#if ENABLE_OTA
  ArduinoOTA.setHostname(HOST_NAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.onStart([]() { pushLog("[esp] OTA 开始"); });
  ArduinoOTA.onEnd([]()   { pushLog("[esp] OTA 完成"); });
  ArduinoOTA.begin();
#endif

  pushLog("[esp] 就绪：机械臂串口 ");
  espLog("      %d 8N1（%s）", ARM_BAUD, ARM_LINK_SOFTWARE ? "软串口 D5/D6" : "硬件串口 GPIO3/GPIO1");
}

void loop() {
  server.handleClient();
  armReadTick();
  queueTick();
#if ENABLE_TELNET
  telnetTick();
#endif
  if (apMode) dns.processNextRequest();
  MDNS.update();
#if ENABLE_OTA
  ArduinoOTA.handle();
#endif

  /* 网页可以关掉自动查状态；默认每 STATUS_POLL_MS 发一个查询字符（只查询，不会打断动作） */
#if STATUS_POLL_MS > 0
  if (autoPoll && millis() - lastPoll >= STATUS_POLL_MS) { lastPoll = millis(); armSendRaw(POLL_CMD); }
#endif

  delay(2);
}
