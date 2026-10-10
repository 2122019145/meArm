/*
// serial_protocol.cpp
// 串口命令协议：固定指令 + 三舵机同步角度指令（x/y/z，单位度：
// x = 基座、y = 上臂、z = 下臂，详见 serial_protocol.h）。
// 同时托管 A/B/C 取放序列的启动入口（序列运行期间本层挡住所有其它运动指令）、
// 四个物理按键的串口孪生命令 N/R/P/M（实现位于 button_control.cpp）
// 以及绘图命令 F/D/G/E/Q/U/W 和纸面标定命令 p/n/o（实现位于 draw_control.cpp）。
// 负责字符输入、行缓冲、命令解析与关节写入。
*/

#include "Arduino.h"
#include "constant_and_positions.h"
#include "pick_place.h"
#include "protocol_constants.h"
#include "serial_protocol.h"
#include "button_control.h"
#include "draw_control.h"
#include "weArm_config.h"

/* ---------- 行缓冲与状态 ---------- */
static char s_line[PROTO_LINE_BUF_SIZE];
static int  s_len = 0;
static bool s_pending = false;          /* 有已缓冲但还没派发的字符 */
static bool s_dropUntilEol = false;     /* 本行太长，一直丢弃到行尾 */
static unsigned long s_lastCharMs = 0;

/* ---------- 辅助函数前置声明 ---------- */
/* 刻意不加 noinline：在真实 AVR 构建上实测这个属性是无效的。
 * serialProtocolLoop() 从三处到达这里，但 gcc 仍只会保留一份 out-of-line 副本；
 * 整个派发器只会存在一份，因为 protoHandleLine() 在本文件外没有调用者。 */
static void protoFlushLine(void);
static bool protoParseAxisLine(const char *s, double angles[3], uint8_t *seen);
static bool protoParseNumber(const char **pp, double *out);
static void protoApplyAngles(const double angles[3], uint8_t seen);
static int protoSpeedStep(int delta);
static int protoHandleDrawCalib(const char *line, char cmd);
#define PROTO_FEATURE_PICK   0x01u
#define PROTO_FEATURE_BUTTON 0x02u
#define PROTO_FEATURE_DRAW   0x04u

static bool protoRuntimeEnabled(uint8_t feature);
static uint8_t protoCompiledFeatures(void);
static uint8_t s_runtimeFeatures =
    (WEARM_ENABLE_PICK_PLACE ? PROTO_FEATURE_PICK : 0u) |
    (WEARM_ENABLE_BUTTONS ? PROTO_FEATURE_BUTTON : 0u) |
    (WEARM_ENABLE_DRAW ? PROTO_FEATURE_DRAW : 0u);

/* 跳过空白（空格 / 制表符）并返回第一个有效字符。
 * 刻意 out-of-line：十几处调用点共用这一份代码。实测强制 always_inline
 * 要多花 40 字节，一次函数调用比展开的循环更便宜。 */
static const char *protoSkipBlanks(const char *p)
{
  while (*p == ' ' || *p == '\t') {
    p++;
  }
  return p;
}

/* 原地跳过空白 */
#define PROTO_SKIP_BLANKS(p) do { (p) = protoSkipBlanks(p); } while (0)

/* PC 端自检 mock 没有 pgmspace.h；AVR 核心两个宏都有。 */
#ifndef PSTR
#define PSTR(s) (s)
#endif
#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
#ifndef PROGMEM
#define PROGMEM
#endif

/* ========== 串口后端 ========== */
/* 真实 Uno 上的 release 构建直接驱动 ATmega328P 的 USART 寄存器，
 * 而不是走 Arduino 的 Serial 对象。
 *
 * 原因：协议只需要"发一个字节"和"有没有字节可读"。只要还链接 *任何* 一处
 * HardwareSerial，它的 vtable 就会保持存活，而 vtable 会把所有虚成员
 * （write / flush / available / read / peek / availableForWrite）
 * 以及两个 USART 中断向量和 64+64 字节环形缓冲一起拖进来 ——
 * 在全功能构建上实测约 1.2 KB flash 和 600 字节 SRAM，
 * 比整个剩余预算还多。
 *
 * -DWEARM_SERIAL_ARDUINO=1（逃生开关）会保留 Arduino Serial 对象做双向收发。
 * PC 端自检 mock 没有 UCSR0A/UDR0，所以所有 host 构建（以及所有探针）
 * 都走同一套 Serial 分支。 */
#ifndef WEARM_SERIAL_ARDUINO
#define WEARM_SERIAL_ARDUINO 0
#endif

#if defined(__AVR__) && !WEARM_SERIAL_ARDUINO
#define WEARM_UART_RAW 1
#else
#define WEARM_UART_RAW 0
#endif

#if WEARM_UART_RAW
#include <avr/interrupt.h>

/* 接收环形缓冲，形状与 Arduino 核心里的那个一致：掩码能工作是因为
 * 缓冲区大小是 2 的幂；缓冲满时丢弃最新字节，而不是破坏还没读走的数据。 */
#define PROTO_RX_BUF_MASK 63u
static volatile uint8_t s_rxBuf[PROTO_RX_BUF_MASK + 1u];
static volatile uint8_t s_rxHead = 0;
static volatile uint8_t s_rxTail = 0;

/* 刻意写成普通函数："缓冲满就丢字节"这条规则因此只存在于一句语句里，
 * 而不是塞进中断例程内部。 */
static void protoRxStore(uint8_t b)
{
  uint8_t next = (uint8_t)((s_rxHead + 1u) & PROTO_RX_BUF_MASK);

  if (next != s_rxTail) {
    s_rxBuf[s_rxHead] = b;
    s_rxHead = next;
  }
}

ISR(USART_RX_vect)
{
  protoRxStore(UDR0);
}

/* 输出一个字节。115200 波特率下等 UDRE0 最多 87 µs，
 * 而回复最长也就十几个字节。 */
static void protoPut(char c)
{
  while ((UCSR0A & (1u << UDRE0)) == 0u) {
  }
  UDR0 = (uint8_t)c;
}

/* 读一个已收到的字节；缓冲区空时返回 -1。 */
static int protoRxTake(void)
{
  uint8_t b;

  if (s_rxHead == s_rxTail) {
    return -1;
  }
  b = s_rxBuf[s_rxTail];
  s_rxTail = (uint8_t)((s_rxTail + 1u) & PROTO_RX_BUF_MASK);
  return (int)b;
}

/* UART 初始化，照 Arduino 核心抄：双倍速（U2X0）、相同的 UBRR、8N1、
 * 收发器 + 接收中断。F_CPU 与 PROTO_BAUD 都是编译期常量，所以 UBRR
 * 会在这里被折叠成常数。 */
static void protoSerialBegin(void)
{
  uint16_t ubrr = (uint16_t)((F_CPU / 4UL / (unsigned long)PROTO_BAUD - 1UL) / 2UL);

  UCSR0A = (uint8_t)(1u << U2X0);
  UBRR0H = (uint8_t)(ubrr >> 8);
  UBRR0L = (uint8_t)ubrr;
  UCSR0C = (uint8_t)((1u << UCSZ01) | (1u << UCSZ00));   /* 8 数据位、无校验、1 停止位 */
  UCSR0B = (uint8_t)((1u << RXEN0) | (1u << TXEN0) | (1u << RXCIE0));
}
#else
/* Arduino Serial：强制回退或 PC 端自检 mock */
static void protoPut(char c)
{
  Serial.print(c);
}

static int protoRxTake(void)
{
  if (Serial.available() <= 0) {
    return -1;
  }
  return Serial.read();
}

static void protoSerialBegin(void)
{
  Serial.begin(PROTO_BAUD);
}
#endif

/* ========== 响应层（WEARM_SERIAL_RESPONSES） ========== */
#if WEARM_SERIAL_RESPONSES

/* 夹爪回显：angle4 被夹在 60..150，所以普通的三位十进制打印机就够用。 */
static void protoWriteInt(int v)
{
  if (v >= 100) {
    protoPut((char)('0' + v / 100));
  }
  if (v >= 10) {
    protoPut((char)('0' + (v / 10) % 10));
  }
  protoPut((char)('0' + v % 10));
}

/* 从 flash 里逐字节走出 <text> 并发送；每条回复以换行结尾，
 * 而 '#' 会被替换成夹爪角度的十进制值。刻意 out-of-line：
 * 在 -flto 下 gcc 会把这个循环克隆到每个回复点，而每个点的参数准备
 * 比一次调用还要贵。 */
__attribute__((noinline, noclone))
static void protoReply(const char *text)
{
  char c;

  while ((c = (char)pgm_read_byte(text++)) != '\0') {
    if (c == '#') {
      protoWriteInt((int)Pos.ser.angle4);
    } else {
      protoPut(c);
    }
  }
  protoPut('\n');
}

/* 每条回复文本一份 flash 副本。给数组起名是为了让 gcc 不要为每个调用点
 * 各生成一份单独的副本（外加对齐填充）。 */
static const char s_rOk[] PROGMEM = "OK";
static const char s_rErr[] PROGMEM = "ERR";
static const char s_rRej[] PROGMEM = "REJECTED";
static const char s_rOkN[] PROGMEM = "OK #";
/* 三种"看懂了但没执行"的回复，按键族以前都合并成 REJECTED。
 * 把它们区分开是让纯串口主机能做出反应的关键：BUSY = 机械臂在动，
 * DISCARD = 录制被丢弃（位移不够 / 没有动作 / 缓冲满），
 * EMPTY = P 时没有任何录制数据。
 * OFF = 固件里有该模块，但用户用 !P/!B/!D 关掉了它；
 * 而 REJECTED 则表示该模块从未被编译进来。 */
static const char s_rBusy[] PROGMEM = "BUSY";
static const char s_rDiscard[] PROGMEM = "DISCARD";
static const char s_rEmpty[] PROGMEM = "EMPTY";
static const char s_rOff[] PROGMEM = "OFF";
static const char s_rBoot[] PROGMEM = "CMD OSHL 123 kK ABC NRPM0 FDGEQUW pno ANG x,y,z ! !P !B !D";

/* 'F'（切换绘图任务）回复 "OK F=<tag>"：tag 是简短的 ASCII 名称
 * （"LINE" / "N" / "TRI" / "Z" / "V" / "POLY" / "CURVE"），
 * 让纯串口主机（PC 终端或 ESP8266 板子）知道当前选中的是哪种绘图模式。
 * tag 本身由 draw_control.cpp 提供（drawTaskTag）。 */
__attribute__((noinline))
static void protoReplyTaskTag(const char *tag)
{
  protoPut('O');
  protoPut('K');
  protoPut(' ');
  protoPut('F');
  protoPut('=');
  while (*tag != '\0') {
    protoPut(*tag++);
  }
  protoPut('\n');
}

#define R_OK()   protoReply(s_rOk)
#define R_ERR()  protoReply(s_rErr)
#define R_REJ()  protoReply(s_rRej)
#define R_OKN()  protoReply(s_rOkN)
#define R_BUSY() protoReply(s_rBusy)
#define R_DISCARD() protoReply(s_rDiscard)
#define R_EMPTY() protoReply(s_rEmpty)
#define R_OFF()  protoReply(s_rOff)
#define R_BOOT() protoReply(s_rBoot)
#define R_FEATURE() do { protoPut('P'); protoPut('='); protoPut((s_runtimeFeatures & PROTO_FEATURE_PICK) ? '1' : '0'); protoPut(' '); protoPut('B'); protoPut('='); protoPut((s_runtimeFeatures & PROTO_FEATURE_BUTTON) ? '1' : '0'); protoPut(' '); protoPut('D'); protoPut('='); protoPut((s_runtimeFeatures & PROTO_FEATURE_DRAW) ? '1' : '0'); protoPut(10); } while (0)
#else
#define R_OK()   do { } while (0)
#define R_ERR()  do { } while (0)
#define R_REJ()  do { } while (0)
#define R_OKN()  do { } while (0)
#define R_BUSY() do { } while (0)
#define R_DISCARD() do { } while (0)
#define R_EMPTY() do { } while (0)
#define R_OFF()  do { } while (0)
#define R_BOOT() do { } while (0)
#define R_FEATURE() do { } while (0)
#define protoReplyTaskTag(tag) do { (void)(tag); } while (0)
#endif

/* ========== 命令分类位图 ========== */
/* 下标 i = cmd - '0' 覆盖 '0'(0x30) .. 'p'(0x70)，包括所有命令字符；
 * 其它字符会越出表尾并被拒绝。
 * s_liveBits：模块忙时该命令仍然被接受。
 * s_fastBits：命令只要单字符一到就立即执行，不必等行尾。 */
static const uint8_t s_liveBits[9] PROGMEM = { 0x0F, 0x00, 0xF0, 0x71, 0xA7, 0x00, 0x00, 0xC0, 0x01 };
static const uint8_t s_fastBits[9] PROGMEM = { 0x0E, 0x00, 0x0E, 0x99, 0x08, 0x00, 0x00, 0x08, 0x00 };

__attribute__((noinline)) static bool protoCmdBit(const uint8_t *bits, char cmd)
{
  uint8_t i = (uint8_t)cmd - (uint8_t)'0';

  if (i > 64u) {
    return false;
  }
  return (pgm_read_byte(bits + (i >> 3)) & (uint8_t)(1u << (i & 7u))) != 0;
}

/* 一个命令字符需要哪个功能模块：每个字符 2 位，
 * 下标与上面的位图完全一样（i = cmd - '0'）。
 * 0 = 命令不受模块门控，1 = 取放，2 = 按键，3 = 绘图。
 *
 * 这张表替代了派发器里的三条 if 链（3 + 5 + 10 = 18 次字符比较，
 * 每次还要带自己的 WEARM_ENABLE_* 与运行时检查）。
 *
 * 真实 Uno 构建实测（全功能）：派发器本身从 2192 字节减到 2134 字节，
 * 但加上表与查表开销，整镜像反而大了 10 字节（32710 -> 32720）。
 * 保留它是因为"哪个模块拥有哪个命令"这样表达更清晰，
 * 且全功能构建仍有富余空间 —— 不是因为它省了什么。
 * 每个字节都要省的话就退回显式 if 链。
 *
 * 字节 k 的位布局：bit 1:0 = 字符(4k)，3:2 = 字符(4k+1)，
 *                   5:4 = 字符(4k+2)，7:6 = 字符(4k+3)。 */
static const uint8_t s_featureBits[17] PROGMEM = {
  0x02,                                     /* '0' -> 按键（回中，M 的别名） */
  0x00, 0x00, 0x00,
  0x54,                                     /* A B C -> 取放，D -> 绘图      */
  0xFF,                                     /* D E F G 全部 -> 绘图          */
  0x00,
  0x28,                                     /* M N -> 按键                    */
  0x2E,                                     /* P -> 按键，Q -> 绘图，R -> 按键 */
  0xCC,                                     /* U -> 绘图，W -> 绘图           */
  0x00, 0x00, 0x00, 0x00, 0x00,
  0xF0,                                     /* n o -> 绘图                    */
  0x03                                      /* p -> 绘图                      */
};

__attribute__((noinline)) static uint8_t protoCmdFeature(char cmd)
{
  uint8_t i = (uint8_t)cmd - (uint8_t)'0';

  if (i > 64u) {
    return 0u;
  }
  return (uint8_t)((pgm_read_byte(s_featureBits + (i >> 2)) >> (uint8_t)((i & 3u) * 2u)) & 0x03u);
}

/* ========== 主循环接口 ========== */
/* 每轮 loop() 调用一次：把已到达的字符攒成一行并执行 */
void serialProtocolLoop(void)
{
  int c;

  while ((c = protoRxTake()) >= 0) {
    /* 记住最后一个字符到达的时刻 */
    s_lastCharMs = millis();

    /* 超长行：保持在丢弃状态直到行尾 */
    if (c == '\n' || c == '\r') {
      if (s_dropUntilEol) {
        s_dropUntilEol = false;
        s_len = 0;
        s_pending = false;
      } else {
        protoFlushLine();
      }
      continue;
    }
    if (s_dropUntilEol) {
      continue;  /* 丢弃到行尾之前的字符 */
    }

    /* 行缓冲溢出 */
    if (s_len >= PROTO_LINE_BUF_SIZE - 1) {
      s_dropUntilEol = true;
      s_len = 0;
      s_pending = false;
      continue;
    }

    /* 存入字符 */
    s_line[s_len++] = (char)c;
    s_pending = true;

    /* 单字符固定命令立即执行 */
    if (s_len == 1 && protoCmdBit(s_fastBits, s_line[0])) {
      protoFlushLine();  /* 立即派发单字符命令 */
    }
  }

  /* 行超时：缓冲数据静默了一段时间就按整行已到处理 */
  if (s_pending && (millis() - s_lastCharMs) >= PROTO_LINE_TIMEOUT_MS) {
    protoFlushLine();
  }

  /* 超长行丢弃超时 */
  if (s_dropUntilEol && (millis() - s_lastCharMs) >= PROTO_LINE_TIMEOUT_MS) {
    s_dropUntilEol = false;
    s_line[0] = '\0';
    s_len = 0;
    s_pending = false;
  }
}

/* ========== 派发 ========== */
/* 把缓冲好的行派发给命令处理器 */
static bool protoRuntimeEnabled(uint8_t feature)
{
  return (s_runtimeFeatures & feature) != 0;
}

static uint8_t protoCompiledFeatures(void)
{
  return (WEARM_ENABLE_PICK_PLACE ? PROTO_FEATURE_PICK : 0u) |
         (WEARM_ENABLE_BUTTONS ? PROTO_FEATURE_BUTTON : 0u) |
         (WEARM_ENABLE_DRAW ? PROTO_FEATURE_DRAW : 0u);
}

/* 按键族结果（N/R/P/M/0）的统一回复，让这几条命令都用同一种方式回答：
 * OK / BUSY / DISCARD / EMPTY / ERR。 */
static void protoReplyButtonResult(int result)
{
  (void)result; /* 关闭响应层时用来压掉 -Wunused-parameter 警告 */
  if (result == PROTO_RES_BUSY) {
    R_BUSY();
  } else if (result == PROTO_RES_REC_REJECTED) {
    R_DISCARD();
  } else if (result == PROTO_RES_PLAY_NO_RECORD) {
    R_EMPTY();
  } else if (result < 0) {
    R_ERR();
  } else {
    R_OK();
  }
}

/* 回答一个其所属模块现在不肯接受的命令：固件里确实有该模块但被用户用
 * !P/!B/!D 关掉了就回 "OFF"，被编译掉了就回 "REJECTED"。
 * 所有拒绝点共用这一份，防止两种情况漂移。 */
static void protoRefuseModule(uint8_t feature)
{
  if ((protoCompiledFeatures() & feature) != 0u) {
    R_OFF();
  } else {
    R_REJ();
  }
}

static void protoFlushLine(void)
{
  s_line[s_len] = '\0';
  if (s_len > 0) {
    protoHandleLine(s_line);
  }
  s_len = 0;
  s_pending = false;
}

/* ========== 命令处理 ========== */
/* 处理一整行（换行符已被移除） */
int protoHandleLine(const char *line)
{
  if (line == NULL) {
    return PROTO_RES_NONE;
  }

  /* 直接在调用者的字符串上工作。下面每个解析器都会自己跳过空白，
   * 所以旧版"把空白压进本地副本"那一遍（以及它需要的 40 字节栈帧）已经没了。 */
  const char *p = line;
  PROTO_SKIP_BLANKS(p);

  const char cmd = *p;
  if (cmd == '\0') {
    return PROTO_RES_NONE;
  }

  /* "single" 就是旧版的"归一化长度 == 1"：命令字符之后只有空白。
   * 旧副本的 buf[0] 恰好就是这个 cmd。 */
  const char *tail = p + 1;
  PROTO_SKIP_BLANKS(tail);
  const bool single = (*tail == '\0');

  if (cmd == '!' && !single) {
    const char *q = tail;
    uint8_t feature;
    if (q[1] != '\0' || (q[0] != 'P' && q[0] != 'B' && q[0] != 'D')) {
      R_REJ();
      return PROTO_RES_UNKNOWN;
    }
    feature = q[0] == 'P' ? PROTO_FEATURE_PICK :
              q[0] == 'B' ? PROTO_FEATURE_BUTTON : PROTO_FEATURE_DRAW;
    if (!(protoCompiledFeatures() & feature)) {
      R_REJ();
      return PROTO_RES_UNKNOWN;
    }
    if (!protoRuntimeEnabled(feature)) s_runtimeFeatures |= feature;
    else s_runtimeFeatures &= (uint8_t)~feature;
    R_FEATURE();
    return PROTO_RES_FEATURE;
  }
  if (cmd == '!' && single) {
    R_FEATURE();
    return PROTO_RES_NONE;
  }

  if (!single && (cmd == PROTO_CMD_BTN_CYCLE || cmd == PROTO_CMD_BTN_RECORD ||
      cmd == PROTO_CMD_BTN_PLAY || cmd == PROTO_CMD_BTN_HOME ||
      cmd == PROTO_CMD_BTN_HOME_ALT)) {
    R_ERR();
    return PROTO_RES_BAD_SYNTAX;
  }
  if (single) {
    /* 功能门控：属于某个模块（取放、按键、绘图）的命令，只有该模块既被
     * 编译进来、又处于运行时开启状态才被接受。
     * s_runtimeFeatures 初始就是编译期的集合，只有 '!' 命令会改它，
     * 所以 protoRuntimeEnabled() 一票就能判定：被编译掉的模块永远打不开。 */
    uint8_t module = protoCmdFeature(cmd);

    if (module != 0u) {
      uint8_t feature = (module == 1u) ? PROTO_FEATURE_PICK :
                        (module == 2u) ? PROTO_FEATURE_BUTTON : PROTO_FEATURE_DRAW;

      if (!protoRuntimeEnabled(feature)) {
        protoRefuseModule(feature);
        return PROTO_RES_UNKNOWN;
      }
    }
  }

  /* 忙判定：取放序列运行中、按键模块录制/播放/回中中、或绘图任务运行中，
   * 其它所有串口运动指令一律让位。
   *
   * 有两类例外必须放行（真正是否执行由被调方再次判定，
   * 不同意时会回 PROTO_RES_BUSY）：
   *   1) 调速命令（H、L、1、2、3）—— 你可能想在取放序列或录制中途改速度；
   *   2) 按键命令（N、R、P、M）—— 用 R 结束录制不能被这里挡住，
   *      否则录制永远停不下来。
   *   3) 绘图命令（F、D、G、E、Q、U、W 以及 p、n、o）——
   *      暂停/继续/取消/示教/标定正是绘制时该发的命令，
   *      挡住它们等于三项控制功能全废。
   *
   * 三个判据都是纯查询，|| 顺序不会改变结果；
   * 这里按"最便宜的先算"来写只是为了代码短。
   * 实测：与"pickPlaceIsBusy() 放最前"相比省 8 字节。 */
  if (drawControlBusy() || buttonControlBusy() || pickPlaceIsBusy()) {
    if (!protoCmdBit(s_liveBits, cmd)) {
      R_BUSY();
      return PROTO_RES_BUSY;
    }
  }

  /* 单字符命令 */
  if (single) {
    /* '0' 是唯一落在 'O'..'W' 范围以下的命令，所以从 if 链里单拎出来：
     * 剩下的链只需要为自己用到的比较买单。 */
    if (cmd == PROTO_CMD_BTN_HOME_ALT) {
      int result = buttonHandleCommand(cmd);
      protoReplyButtonResult(result);
      return result < 0 ? PROTO_RES_UNKNOWN : result;
    }

    /* 刻意用 if 链而不是 switch：case 标签跨 'O'..'W'，
     * switch 会生成一张 64 项的跳转表，而 if 链只为自己用到的比较买单。
     * 真实 AVR 构建实测：整个派发器省 78 字节。 */
    if (cmd == PROTO_CMD_GRIPPER_OPEN) {
      posSetAngle4(150.0);  // 硬编码：夹爪张开（与 f 行程上限一致）
      R_OKN();
      return PROTO_RES_GRIPPER_OPEN;

    } else if (cmd == PROTO_CMD_GRIPPER_CLOSE) {
      posSetAngle4(60.0);   // 硬编码：夹爪合拢（与 f 行程下限一致）
      R_OKN();
      return PROTO_RES_GRIPPER_CLOSE;

    } else if (cmd == PROTO_CMD_SPEED_UP || cmd == PROTO_CMD_SPEED_DOWN) {
        protoSpeedStep((cmd == PROTO_CMD_SPEED_UP) ? +1 : -1);
        R_OK();
        return (cmd == PROTO_CMD_SPEED_UP) ? PROTO_RES_SPEED_UP : PROTO_RES_SPEED_DOWN;

    } else if (cmd == PROTO_CMD_PICK_A || cmd == PROTO_CMD_PICK_B ||
               cmd == PROTO_CMD_PICK_C) {
        int object = PICK_OBJECT_A + (cmd - PROTO_CMD_PICK_A);
        int rc = pickPlaceStart(object);
        /* 0 = 已启动；-2 = 忙；-1/-3 = 这一次启动不了（编号非法 /
         * 路径校验失败）。这两种都回 BUSY，让主机知道命令被看懂了，
         * 但序列没有跑起来。 */
        if (rc == 0) {
          R_OK();
          return PROTO_RES_PICK_STARTED;
        }
        R_BUSY();
        return PROTO_RES_BUSY;

    /* N/R/P/M 与 '0'：四个物理按键的串口孪生。
     * 能不能立刻执行（录制/播放/取放中）由 button_control.cpp 内部判定，
     * 不行时它回 PROTO_RES_BUSY —— 上面的忙判定刻意放行这几个字符，
     * 否则"用 R 结束录制"就会被挡住。 */
    } else if (cmd == PROTO_CMD_BTN_CYCLE || cmd == PROTO_CMD_BTN_RECORD ||
               cmd == PROTO_CMD_BTN_PLAY || cmd == PROTO_CMD_BTN_HOME) {
      int result = buttonHandleCommand(cmd);
      protoReplyButtonResult(result);
      return result < 0 ? PROTO_RES_UNKNOWN : result;

    /* F/D/G/E/Q/U/W：绘图命令（选图形 / 开始 / 记示教点 / 撤销 /
     * 暂停 / 继续 / 取消）。能不能执行（已在绘制中、示教点不够、
     * 路径校验不通过）由 draw_control.cpp 内部判定，
     * 它会回 PROTO_RES_BUSY 或 PROTO_RES_DRAW_REJECTED ——
     * 上面的忙判定刻意放行这几个字符，
     * 否则绘制中的暂停/取消就会被挡住。 */
    } else if (cmd == PROTO_CMD_DRAW_TASK || cmd == PROTO_CMD_DRAW_START ||
               cmd == PROTO_CMD_DRAW_RECORD || cmd == PROTO_CMD_DRAW_UNDO ||
               cmd == PROTO_CMD_DRAW_PAUSE || cmd == PROTO_CMD_DRAW_RESUME ||
               cmd == PROTO_CMD_DRAW_CANCEL) {
      int result = drawHandleCommand(cmd);
      /* 'F' 用"当前选中的图形"回复（"OK F=V"），而不是干巴巴的 "OK"：
       * 操作者（以及 ESP8266 板子）没有别的办法知道 7 种绘图任务里
       * 究竟选中了哪一种。 */
      if (result == PROTO_RES_DRAW_TASK_SELECTED) {
        protoReplyTaskTag(drawTaskTag(drawGetTask()));
      } else if (result == PROTO_RES_BUSY) R_BUSY();
      else if (result == PROTO_RES_DRAW_REJECTED) R_REJ();
      else if (result < 0) R_ERR();
      else R_OK();
      return result;
    }
    /* 未知单字符命令：落到下面的角度解析 */
  }

  /* 纸面标定 p/n/o：多字符（p12.5 / n6 / o20,0）。
   * 放在角度解析之前处理，因为 'o' 行里带逗号，
   * 否则会被当成"语法错误的角度命令"。 */
  if ((cmd == PROTO_CMD_DRAW_PAPER_Z ||
       cmd == PROTO_CMD_DRAW_HALF ||
       cmd == PROTO_CMD_DRAW_CENTER) && !single) {
    if (!protoRuntimeEnabled(PROTO_FEATURE_DRAW)) {
      protoRefuseModule(PROTO_FEATURE_DRAW);
      return PROTO_RES_UNKNOWN;
    }
    int result = protoHandleDrawCalib(p, cmd);
    if (result == PROTO_RES_DRAW_REJECTED) R_REJ();
    return result;
  }

  /* x/y/z：三舵机同步角度指令（考题 1.3）。
   * x -> 基座舵机（angle1 = b），y -> 上臂舵机（angle2 = r），
   * z -> 下臂舵机（angle3 = c）。一行里出现的每个轴都在同一次
   * 正解刷新之前一次写完，所以一行就是一次同步姿态改变 ——
   * 与固件其它部分同一约定（Pos.ser 是真值，Pos.rec 由它派生）。 */
  if (protoAxisIndexFromChar(cmd) >= 0) {
    double angles[3];
    uint8_t seen = 0;

    if (!protoParseAxisLine(p, angles, &seen)) {
      R_ERR();
      return PROTO_RES_BAD_SYNTAX;
    }

    /* 超出机械行程是夹取而不是拒绝：b/r/c 三轴的上下限已硬编码为 0..180，
     * 所以 "x200" 会落到该关节行程的上限，
     * 而不是把整条命令变成什么都不做的空操作。
     * 一个轴都没提到的行根本走不到这里 —— protoParseAxisLine 会拒掉它。 */
    protoApplyAngles(angles, seen);

    R_OK();
    return PROTO_RES_ANGLES_SET;
  }

  /* 不是轴字母，但里面带逗号或开头是 '='：看起来像写错的角度命令。
   * 其它一切视为未知。 */
  {
    const char *scan = p;
    while (*scan != '\0' && *scan != ',') {
      scan++;
    }
    if (*scan != ',' && cmd != '=') {
      R_ERR();
      return PROTO_RES_UNKNOWN;
    }
  }

  R_ERR();
  return PROTO_RES_BAD_SYNTAX;
}

/* ========== 角度解析 ========== */
/* 解析一条角度命令：group (',' group)*，group = [空白] 轴字母 [空白]
 * [可选 '='] [空白] 数值。解析出的数值是舵机角度（度），
 * x/y/z 按 protoAxisIndexFromChar() 返回的下标顺序排列。
 * 整行必须能解析；同一个轴给两次以最后一次为准。
 * 语法不匹配就返回 false，除了 angles/seen 之外什么都不碰。 */
static bool protoParseAxisLine(const char *s, double angles[3], uint8_t *seen)
{
  const char *p = s;
  int groups = 0;

  for (;;) {
    /* 1) 组之前的空白 */
    PROTO_SKIP_BLANKS(p);

    /* 2) 轴字母（大小写均可） */
    int axis = protoAxisIndexFromChar(*p);
    if (axis < 0) return false;
    p++;

    /* 3) 字母与 '=' 之间的空白 */
    PROTO_SKIP_BLANKS(p);

    /* 4) 可选 '=' */
    if (*p == '=') {
      p++;
      PROTO_SKIP_BLANKS(p);
    }

    /* 5) 数值（必须真的以数字开头） */
    if (!protoParseNumber(&p, &angles[axis])) return false;

    /* 6) 记录下来（同一个轴给两次以最后一次为准） */
    *seen |= (uint8_t)(1u << axis);
    groups++;

    /* 7) 后面只能是行尾或逗号 */
    PROTO_SKIP_BLANKS(p);
    if (*p == '\0') break;
    if (*p != ',') return false;
    p++;   /* 吃掉逗号，处理下一组 */
  }

  return (groups > 0);
}

/* 手写数字解析器：避免引入 strtod/atof */
static bool protoParseNumber(const char **pp, double *out)
{
  const char *p = *pp;
  bool neg = false;
  unsigned int ip = 0;
  double value;

  /* 可选符号 */
  if (*p == '+') {
    p++;
  } else if (*p == '-') {
    neg = true;
    p++;
  }

  /* 整数部分：至少一位数字 */
  if (*p < '0' || *p > '9') {
    return false;
  }
  /* 数字先攒在整数里、最后一次转换。饱和在 1000000 之后夹取结果
   * 与老版本逐位累积结果一致：本固件接受为角度的值都远低于饱和上限，
   * 所以两种写法都落到同一个极限。 */
  do {
    if (ip < 9999u) {
      ip = ip * 10u + (unsigned int)(*p - '0');
    }
    p++;
  } while (*p >= '0' && *p <= '9');

  value = (double)ip;

  /* 小数部分：'.' 后面必须跟数字，数字仍是按十进制位逐个加上去，
   * 所以舍入结果不变 */
  if (*p == '.') {
    double decimal_place = 0.1;
    p++;
    if (*p < '0' || *p > '9') {
      return false;  /* "12." 不合法 */
    }
    do {
      value += (*p - '0') * decimal_place;
      decimal_place *= 0.1;
      p++;
    } while (*p >= '0' && *p <= '9');
  }

  *out = neg ? -value : value;
  *pp = p;
  return true;
}

/* ========== 角度应用 ========== */
/* 把这一行提到的轴写进三个关节，并只刷新一次正运动学。
 * 轴 a 就是关节 a+1：angle1 = b（基座），angle2 = r（上臂），
 * angle3 = c（下臂）—— 正是考题要求的 x/y/z 映射，
 * 也是为什么三个关节能用一次指针遍历写完，而不必 switch 展开。
 *
 * 角度在这里硬夹到 0..180（取代了原来的 clampServoAngles() 调用）。
 * 同样的上限在舵机驱动层也会执行，但在这里夹一次能让 Pos.ser 自身
 * 保持在合法范围内。 */
static void protoApplyAngles(const double angles[3], uint8_t seen)
{
  for (int a = 0; a < PROTO_AXIS_COUNT; a++) {
    if ((seen & (uint8_t)(1u << a)) == 0u) continue;
    double v = angles[a];
    if (v < 0.0)   v = 0.0;
    if (v > 180.0) v = 180.0;
    (&Pos.ser.angle1)[a] = v;
  }

  (void) recFromServo(&Pos.rec, &Pos.ser);
}

/* ========== 绘图参数标定 ========== */
/* p<纸面 z> / n<半宽> / o<中心 x>,<中心 y>
 * 语法错误或值越界会回 PROTO_RES_DRAW_REJECTED，并且不改动任何参数。 */
static int protoHandleDrawCalib(const char *line, char cmd)
{
  const char *p = line + 1;   /* 跳过命令字母 */
  double v1;
  double v2;

  PROTO_SKIP_BLANKS(p);
  if (!protoParseNumber(&p, &v1)) {
    R_ERR();
    return PROTO_RES_DRAW_REJECTED;
  }
  PROTO_SKIP_BLANKS(p);

  bool ok = false;
  if (cmd == PROTO_CMD_DRAW_PAPER_Z) {
    if (*p == '\0') ok = drawSetPaperZ(v1);
  } else if (cmd == PROTO_CMD_DRAW_HALF) {
    if (*p == '\0') ok = drawSetHalfSize(v1);
  } else if (*p == ',') {
    p++;
    PROTO_SKIP_BLANKS(p);
    if (protoParseNumber(&p, &v2) && *p == '\0') {
      ok = drawSetCenter(v1, v2);
    }
  }

  if (ok) {
    R_OK();
    return PROTO_RES_DRAW_CALIBRATED;
  }

  R_ERR();
  return PROTO_RES_DRAW_REJECTED;
}

/* ========== 调速控制 ========== */
/* 步进一档速度 */
static int protoSpeedStep(int delta)
{
  int current_level = speedGetLevel();

  /* 自定义档位回落到默认档 */
  if (current_level < PROTO_SPEED_LEVEL_MIN || current_level > PROTO_SPEED_LEVEL_MAX) {
    current_level = PROTO_SPEED_LEVEL_DEF;
  }

  int want_level = current_level + delta;

  /* 夹到档位范围 */
  if (want_level < PROTO_SPEED_LEVEL_MIN) {
    want_level = PROTO_SPEED_LEVEL_MIN;
  } else if (want_level > PROTO_SPEED_LEVEL_MAX) {
    want_level = PROTO_SPEED_LEVEL_MAX;
  }

  /* 已经在端点：不再调用 adjustSpeed */
  if (want_level == current_level) {
    return current_level;
  }

  /* 切换速度档位 */
  want_level = adjustSpeed(want_level);
  return want_level;
}

/* ========== 初始化 ========== */
/* UART 初始化（release AVR 构建走裸寄存器，其它情况走 Serial）
 * 以及命令表输出 */
void serialProtocolBegin(void)
{
  protoSerialBegin();
  s_runtimeFeatures = protoCompiledFeatures();

  /* 命令表：放在 WEARM_SERIAL_RESPONSES 里，
   * 这样主机能拿到命令列表。刻意写得简洁：这里每个字符都是 flash，
   * 所以命令列表与角度语法共挤一行。 */
  R_BOOT();
}