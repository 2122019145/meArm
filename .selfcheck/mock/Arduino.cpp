//
// mock Arduino 实现 —— 仅用于 PC 端语法自检
// 说明: Serial 的过滤逻辑在 mock/Arduino.h 的 MockSerial 内联实现里，
//       默认静默，设 WEARM_MOCK_SERIAL=1 可打印固件日志。
//
#include "Arduino.h"

MockSerial Serial;

/* AVR IO 寄存器的宿主替身（说明见 Arduino.h）：初值与复位后一致，全 0 */
volatile uint8_t DDRB = 0, PORTB = 0, DDRC = 0, PORTC = 0;

/* 可由测试脚本改写的仿真时钟与引脚状态 */
unsigned long g_mockMillis = 0;
int g_mockAnalog[8] = { 512, 512, 512, 512, 512, 512, 512, 512 };
int g_mockDigital[20] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH,
                          HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };
/* 记录 pinMode() 设过的模式，初值 INPUT，供探针断言"按键脚被设成 INPUT_PULLUP"。
 * 真实硬件上 pinMode 是有效果的，这里只是把效果记下来，别的探针都不读它。 */
int g_mockPinMode[20] = { INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT,
                          INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT, INPUT };

/* digitalWrite() 的仿真记录（v1.3.0 起）：除了每个引脚的当前电平，
 * 还按调用顺序记录最近 MOCK_WRITE_LOG_MAX 次"写引脚"事件。
 * probe_servo_drive 靠这个日志验证 servo_drive.cpp 的脉冲顺序/极性；
 * 其它探针仍然只读 g_mockPinMode，不受影响。 */
int g_mockPinLevel[20] = { LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW,
                           LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW };
int g_mockWriteLogPin[MOCK_WRITE_LOG_MAX];
int g_mockWriteLogLevel[MOCK_WRITE_LOG_MAX];
int g_mockWriteLogCount = 0;
int g_mockWriteTotal = 0;   /* 累计 digitalWrite 次数（日志满了也继续计数，便于发现"还有脉冲"） */

void mockDigitalWriteReset(void) {
  for (int i = 0; i < 20; i++) g_mockPinLevel[i] = LOW;
  for (int i = 0; i < MOCK_WRITE_LOG_MAX; i++) { g_mockWriteLogPin[i] = -1; g_mockWriteLogLevel[i] = -1; }
  g_mockWriteLogCount = 0;
  g_mockWriteTotal = 0;
}

unsigned long millis(void) { return g_mockMillis; }
unsigned long micros(void) { return g_mockMillis * 1000UL; }
void delay(unsigned long ms) { g_mockMillis += ms; }
void delayMicroseconds(unsigned int us) { (void) us; }
void pinMode(uint8_t pin, uint8_t mode) {
  if (pin < 20) g_mockPinMode[pin] = (int) mode;
}
void digitalWrite(uint8_t pin, uint8_t value) {
  if (pin < 20) g_mockPinLevel[pin] = (int) value;
  if (g_mockWriteLogCount < MOCK_WRITE_LOG_MAX) {
    g_mockWriteLogPin[g_mockWriteLogCount]   = (int) pin;
    g_mockWriteLogLevel[g_mockWriteLogCount] = (int) value;
    g_mockWriteLogCount++;
  }
  g_mockWriteTotal++;
}
// 模拟真实按钮的电气行为：如果测试脚本把引脚电平设成 LOW，
// 就让它保持 LOW 一段时间（由 g_mockButtonHoldMs 控制），
// 这样固件里"读两次确认"的消抖逻辑也能被正确模拟。
// 默认 0 = 立即跟随脚本设置的电平。
int g_mockButtonHoldMs = 0;
static int holdRaw[20] = { 0 };
static unsigned long holdStart[20] = { 0 };

int  digitalRead(uint8_t pin) {
  if (pin >= 20) return HIGH;
  int target = g_mockDigital[pin];
  if (g_mockButtonHoldMs <= 0) return target;
  if (holdRaw[pin] != target) {       /* 脚本刚改了电平：记录切换时刻 */
    holdRaw[pin] = target;
    holdStart[pin] = g_mockMillis;
  }
  if (g_mockMillis - holdStart[pin] < (unsigned long) g_mockButtonHoldMs) {
    return LOW;                        /* 回弹窗口内仍报告按下 */
  }
  return target;
}

int  analogRead(uint8_t pin) {
  /* A0..A7 展开后是 14..21，所以要减掉 A0 再索引 */
  int idx = (int) pin - 14;
  if (idx < 0 || idx > 7) return 512;
  return g_mockAnalog[idx];
}

/* ---------- 串口输入仿真 ---------- */
static char inBuf[64];
static int  inLen = 0;
static int  inPos = 0;

void mockSerialFeed(const char *chars) {
  if (chars == NULL) return;
  for (int i = 0; chars[i] != '\0' && inLen < (int)sizeof(inBuf); i++) {
    inBuf[inLen++] = chars[i];
  }
}

void mockSerialClear(void) {
  inLen = 0;
  inPos = 0;
}

int MockSerial::available() { return inLen - inPos; }

int MockSerial::read() {
  if (inPos >= inLen) return -1;
  return (unsigned char) inBuf[inPos++];
}