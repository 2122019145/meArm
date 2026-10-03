/*
 * servo_drive.cpp —— 4 路舵机驱动实现（见 servo_drive.h 的完整说明）
 *
 * 结构：
 *   servoDriveBegin()          初始化 Timer1（预分频 8、普通模式、比较中断）
 *   servoDriveAttach()         绑定引脚
 *   servoDriveWrite()          写目标角度（换算成 tick，临界区里存入 s_ticks）
 *   servoDriveTicksForDeg()    纯函数：角度 -> tick（复刻 Servo 库映射）
 *   servoDriveStep()           纯函数：脉冲状态机（PC 端可步进验证）
 *   ISR(TIMER1_COMPA_vect)     硬件壳子：调用状态机 + 写 OCR1A
 */
#include "servo_drive.h"

/* ---- 与 Arduino Servo 库逐位一致的脉宽映射参数 ---- */
#define SERVO_MIN_US      544     /* MIN_PULSE_WIDTH  */
#define SERVO_MAX_US      2400    /* MAX_PULSE_WIDTH  */
#define SERVO_TRIM_US     2       /* TRIM_DURATION：补偿 ISR 开销，Servo 库同值 */
#define SERVO_TICKS_PER_US 2u     /* 预分频 8 => 0.5us/tick => 1us = 2 tick */
#define SERVO_FRAME_TICKS 40000u  /* REFRESH_INTERVAL 20ms */
#define SERVO_FIRST_TICKS 1000u   /* 上电后 0.5ms 发第一个事件（等寄存器稳定） */
#define SERVO_DEFAULT_TICKS 3000u /* usToTicks(1500us)：与 Servo 库 attach 的默认脉宽一致 */
#define SERVO_PIN_NONE    0xFFu   /* 未 attach 的通道哨兵（引脚号 0~19 不会撞上） */

static uint8_t  s_pin[SERVO_CH_COUNT];          /* 通道 -> 引脚号，0xFF = 未绑定 */
static volatile uint16_t s_ticks[SERVO_CH_COUNT]; /* 通道 -> 脉冲宽度（tick） */
/* 事件游标：0..3 表示"该通道的脉冲正在输出，下一次事件是把它拉低"；
 * == SERVO_CH_COUNT 表示"4 路都发完了，下一次事件就是下一帧的帧边界"。 */
static uint8_t  s_ch;

uint16_t servoDriveTicksForDeg(double deg) {
  int a = (int)deg;                     /* 与原代码 (int) angle 的截断一致 */
  if (a < 0) { a = 0; } else if (a > 180) { a = 180; }
  /* map(a, 0, 180, 544, 2400) = a * (2400-544) / 180 + 544（Servo 库用 long 运算） */
  uint16_t us = (uint16_t)(SERVO_MIN_US + (int)(((long)a * (SERVO_MAX_US - SERVO_MIN_US)) / 180L));
  us = (uint16_t)(us - SERVO_TRIM_US);
  return (uint16_t)(us * SERVO_TICKS_PER_US);
}

void servoDriveBegin(void) {
  for (uint8_t i = 0; i < SERVO_CH_COUNT; i++) {
    s_pin[i]   = SERVO_PIN_NONE;
    s_ticks[i] = SERVO_DEFAULT_TICKS;   /* 未写过角度时保持 1500us 中位（Servo 库 attach 同值） */
  }
  s_ch    = SERVO_CH_COUNT;   /* 第一次中断就当成帧边界：清零 TCNT1 并起第 0 路脉冲 */
#if defined(__AVR__)
  TCCR1A = 0;
  /* 普通模式（WGM12 = 0，比较匹配**不**清零 TCNT1）+ 预分频 8：16MHz/8 = 2MHz => 0.5us/tick。
   * 普通模式正是这里需要的：状态机排的是"绝对比较值"（当前计数 + 脉宽，以及帧长 40000），
   * 计数器只在帧边界被软件清零，所以每个比较值都落在未来的确定时刻上，不会漂移。 */
  TCCR1B = _BV(CS11);
  TCNT1  = 0;
  TIFR1 |= _BV(OCF1A);        /* 清掉可能挂起的比较中断标志 */
  OCR1A  = SERVO_FIRST_TICKS;
  TIMSK1 |= _BV(OCIE1A);      /* 打开 Timer1 比较中断 */
#endif
}

void servoDriveAttach(uint8_t ch, uint8_t pin) {
  if (ch >= SERVO_CH_COUNT) { return; }
  s_pin[ch] = pin;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void servoDriveWrite(uint8_t ch, double deg) {
  if (ch >= SERVO_CH_COUNT) { return; }
  uint16_t t = servoDriveTicksForDeg(deg);
#if defined(__AVR__)
  uint8_t s = SREG;           /* 16 位变量被中断读到"半新半旧"会输出一个错误脉冲，故进临界区 */
  cli();
  s_ticks[ch] = t;
  SREG = s;
#else
  s_ticks[ch] = t;
#endif
}

/* 脉冲状态机（纯函数，PC 端可逐步验证时序） */
uint8_t servoDriveStep(uint16_t nowTicks, uint16_t *nextDelayTicks) {
  if (s_ch >= SERVO_CH_COUNT) {
    /* 帧边界：上一次中断已经排好的 20ms 比较值到点了。
     * 这里做两件事：把 TCNT1 清零（由调用者写寄存器），并**在同一次中断里**开始本帧
     * 第 0 路的脉冲 —— 也就是"帧边界 = 第 0 路上升沿"。
     *
     * 【为什么帧尾的比较值必须在上一次中断里就算好】中断入口读到的 TCNT1 已经
     * 越过比较值几个 tick。本分支真正被执行时计数器刚刚到达帧尾（≈40000），
     * 若在这里现算"还差多少"：remain = (uint16_t)(SERVO_FRAME_TICKS - nowTicks)
     * 会下溢成 65530 附近（uint16），于是 OCR1A 被排到一个**计数器刚刚过去的
     * 值**上，要等 16 位计数器绕整整一圈（约 32.8ms）才会再次匹配 —— 帧长会变成
     * 约 52.8ms（19Hz）、舵机刷新忽快忽慢。
     * 所以帧尾比较值在"4 路都发完"那一次中断里就算成绝对计数 SERVO_FRAME_TICKS
     * （见下面最后一段，那时 nowTicks 只有一万多，一定还在未来），
     * 帧边界分支只负责清零 TCNT1 并起下一帧。 */
    s_ch = 0u;
    if (s_pin[0] != SERVO_PIN_NONE) { digitalWrite(s_pin[0], HIGH); }
    *nextDelayTicks = s_ticks[0];
    return 1u;
  }

  /* 结束当前通道的脉冲 */
  if (s_pin[s_ch] != SERVO_PIN_NONE) { digitalWrite(s_pin[s_ch], LOW); }
  s_ch++;
  if (s_ch < SERVO_CH_COUNT) {
    /* 紧接着输出下一路（和 Servo 库一样：脉冲之间不留大空隙，只隔一次 ISR 的开销） */
    if (s_pin[s_ch] != SERVO_PIN_NONE) { digitalWrite(s_pin[s_ch], HIGH); }
    *nextDelayTicks = (uint16_t)(nowTicks + s_ticks[s_ch]);
    return 0u;
  }

  /* 4 路都发完：把帧长本身写成比较值（相对本帧 TCNT1 清零点，即绝对计数 40000）。
   * 此刻 nowTicks 只是四路脉宽之和（默认 4×3000 = 12000，最宽也不到 19184），
   * 离 40000 还很远，所以这次比较一定命中、不会绕回；到点后由上面的帧边界分支
   * 接着发下一帧，帧长因此严格是 40000 + 一次中断响应开销 ≈ 20ms。
   * 注意这里**不能**写 nowTicks + remain 这种"现算剩余量"的等价形式：
   * 一旦哪天这个分支被移动到帧尾中断里执行（那时 nowTicks ≈ 40006），
   * uint16 减法下溢就会把下一帧推到 52.8ms 之后（见上面的长注释）。 */
  *nextDelayTicks = SERVO_FRAME_TICKS;
  return 0u;
}

#if defined(__AVR__)
ISR(TIMER1_COMPA_vect) {
  uint16_t next;
  if (servoDriveStep((uint16_t)TCNT1, &next) != 0u) {
    TCNT1 = 0;            /* 帧起点：计数器清零，20ms 帧从 0 起算 */
  }
  OCR1A = next;
}
#endif
