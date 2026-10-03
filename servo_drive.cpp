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
static uint8_t  s_ch;                           /* 当前正在输出的通道 */
static uint8_t  s_phase;                        /* 0 = 待发帧首脉冲，1 = 正在输出脉冲 */

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
  s_ch    = 0u;
  s_phase = 0u;
#if defined(__AVR__)
  TCCR1A = 0;                 /* 普通模式，不用硬件 PWM 输出脚 */
  TCCR1B = _BV(CS11);         /* 预分频 8：16MHz/8 = 2MHz => 0.5us/tick */
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
  if (s_phase == 0u) {
    /* 帧起点：开始第一个通道的脉冲，并要求调用者把 TCNT1 清零（帧长因此恒为 20ms） */
    s_ch    = 0u;
    s_phase = 1u;
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

  /* 4 路都发完：等 20ms 帧边界（绝对比较值，与 Servo 库 usToTicks(REFRESH_INTERVAL) 一致） */
  uint16_t remain = (uint16_t)(SERVO_FRAME_TICKS - nowTicks);
  if (remain < 40u) { remain = 40u; }   /* 极端情况下也留出足够时间，别丢比较中断 */
  *nextDelayTicks = (uint16_t)(nowTicks + remain);
  s_phase = 0u;
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
