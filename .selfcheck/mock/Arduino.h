//
// mock Arduino.h —— 仅用于在 PC 上用 g++ 做语法/类型自检，不参与烧录
//
#ifndef MOCK_ARDUINO_H
#define MOCK_ARDUINO_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <string>

/* 字符串 / 串口 */
class __FlashStringHelper;
typedef const __FlashStringHelper *PGM_P;
#ifndef PROGMEM
#define PROGMEM
#endif
#define F(str) (reinterpret_cast<const __FlashStringHelper *>(str))
typedef std::string String;

/* 默认把 Serial 输出丢弃，避免固件里的调试日志把探针结果淹没。
 * 设置环境变量 WEARM_MOCK_SERIAL=1 可恢复固件日志输出。 */
inline bool mockSerialOn(void) {
  static int enabled = -1;
  if (enabled < 0) {
    const char *v = getenv("WEARM_MOCK_SERIAL");
    enabled = (v != NULL && v[0] == '1') ? 1 : 0;
  }
  return enabled == 1;
}

struct MockSerial {
  void begin(unsigned long) {}
  int  available();
  int  read();
  void print(const char *s)                { if (mockSerialOn()) fputs(s, stdout); }
  void print(const __FlashStringHelper *s) { if (mockSerialOn()) fputs(reinterpret_cast<const char *>(s), stdout); }
  void print(int v)                        { if (mockSerialOn()) printf("%d", v); }
  void print(unsigned int v)               { if (mockSerialOn()) printf("%u", v); }
  void print(long v)                       { if (mockSerialOn()) printf("%ld", v); }
  void print(unsigned long v)              { if (mockSerialOn()) printf("%lu", v); }
  void print(double v)                     { if (mockSerialOn()) printf("%f", v); }
  void print(double v, int digits)         { if (mockSerialOn()) printf("%.*f", digits, v); }
  void println()                           { if (mockSerialOn()) fputc('\n', stdout); }
  void println(const char *s)              { if (mockSerialOn()) { fputs(s, stdout); fputc('\n', stdout); } }
  void println(const __FlashStringHelper *s) { if (mockSerialOn()) { fputs(reinterpret_cast<const char *>(s), stdout); fputc('\n', stdout); } }
  void println(int v)                      { if (mockSerialOn()) printf("%d\n", v); }
  void println(unsigned int v)             { if (mockSerialOn()) printf("%u\n", v); }
  void println(long v)                     { if (mockSerialOn()) printf("%ld\n", v); }
  void println(unsigned long v)            { if (mockSerialOn()) printf("%lu\n", v); }
  void println(double v)                   { if (mockSerialOn()) printf("%f\n", v); }
};
extern MockSerial Serial;

/* 数字引脚别名，与 Uno 一致 */
#define A0 14
#define A1 15
#define A2 16
#define A3 17

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

unsigned long millis(void);
unsigned long micros(void);
void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
int  digitalRead(uint8_t pin);
int  analogRead(uint8_t pin);

/* 仿真输入：测试脚本可改写这些变量来模拟时钟推进与引脚电平。
 * g_mockAnalog 按引脚号索引(A0=14 之外单独用 0..7 模拟)；g_mockDigital 范围 0..19。 */
extern unsigned long g_mockMillis;
extern int g_mockAnalog[8];
extern int g_mockDigital[20];

/* 摇杆模块接入的引脚索引。
 * 注意: g_mockAnalog 用"A0 起算的下标"索引（0=A0, 1=A1, 2=A2, 3=A3），
 *       而 g_mockDigital 用"展开后的数字引脚号"索引（A0=14 ... A3=17）。 */
#define MOCK_AX 0
#define MOCK_AY 1
#define MOCK_TX 2   /* A2：右手柄 X（末端） */
#define MOCK_TY 3   /* A3：右手柄 Y（升降） */
#define MOCK_SW1 16 /* 旧版单摇杆按键，双摇杆板上无按键，保留兼容 */
#define MOCK_SW2 17

/* 串口输入仿真：测试可以用 mockSerialFeed() 喂字符，模拟你在串口助手敲的
 * '1'/'2'/'3'（调速）与 'k'/'K'（末端开合）。 */
void mockSerialFeed(const char *chars);
void mockSerialClear(void);

#endif
