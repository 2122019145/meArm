//
// joystick_control.cpp
// 手柄控制实现 —— MeArm 套件自带手柄（两根双轴摇杆，共占 A0~A3）
//
// 【控制方式：直接控制关节角】摇杆推动 = 某个舵机角度增大/减小，
// 不再通过末端坐标反解。被控量是 Pos.ser.angle1..angle4；
// Pos.rec.x/y/z 由正运动学 recFromServo() 实时算出，只作显示。
//
// 一根摇杆出 2 路模拟电压（X/Y），两根共 4 路，恰好对应 4 个关节：
//   左手柄 X (A0) -> b = angle1 基座回转
//   左手柄 Y (A1) -> r = angle2 上臂俯仰
//   右手柄 X (A2) -> f = angle4 末端夹具
//   右手柄 Y (A3) -> c = angle3 下臂俯仰
// 因此不需要"模式切换"，4 个关节可以同时操作。
//
// 接线（Arduino Uno）：
//   左手柄 X -> A0     左手柄 Y -> A1
//   右手柄 X -> A2     右手柄 Y -> A3
//   手柄 VCC -> 5V,  GND -> GND
//   LED      -> D13（板载 LED，可改 PIN_LED_MODE）
//   ★ 如果发现左右手反了，把 JOY_LX_PIN/JOY_LY_PIN 与
//     JOY_RX_PIN/JOY_RY_PIN 两组引脚对调即可，逻辑不用改。
//
// 完整操作表见 joystick_control.h 顶部注释。
//
#include "Arduino.h"
#include "constant_and_positions.h"
#include "move.h"
#include "joystick_control.h"
#include "pick_place.h"

/* ---------- 编译开关 ---------- */
/* 置 1: 打开调试串口输出（波特率由 serial_protocol 模块初始化）。
 * 需与 constant_and_positions.cpp 中的同名开关保持一致。 */
#define WEARM_DEBUG_SERIAL 1
#define WEARM_JOY_DEBUG    0   /* 置 1 时每次移动都打印各轴与角度，调试用 */
#define WEARM_HAVE_BUTTONS 0   /* 置 1 时启用板载摇杆按键（本套件板上无按键） */

#if WEARM_DEBUG_SERIAL
  #define JLOG(msg)   Serial.println(F(msg))
#else
  #define JLOG(msg)   ((void)0)
#endif

/* ---------- 配置：按硬件修改引脚 ---------- */
/* 左手柄 */
#define JOY_LX_PIN     A0   /* -> 基座 b (angle1)  */
#define JOY_LY_PIN     A1   /* -> 上臂 r (angle2)  */
/* 右手柄 */
#define JOY_RX_PIN     A2   /* -> 末端 f (angle4)  */
#define JOY_RY_PIN     A3   /* -> 下臂 c (angle3)  */

#define PIN_LED_MODE   13   /* 板载 LED 作状态指示，接别的灯改这里 */

/* ---------- 配置：摇杆刻度 ---------- */
#define JOY_CENTER     512  /* ADC 中位（10 位 ADC 的一半） */
#define JOY_DEADZONE   15   /* 中性区：偏转小于此值视为没推杆 */
#define JOY_FULL_SCALE 500  /* 满偏参考幅度，用于把偏转量归一到 0~1 */

/* ---------- 配置：时序 ---------- */
#define BTN_DEBOUNCE_MS 30  /* 按键消抖窗口（本套件默认无按键，保留供扩展） */
#define LED_FAST_MS    100  /* 运动中闪烁半周期 */

/* 关节编号（用于 lastStepTime[] 与调试打印，顺序与 JOY_ACT_* 位图一致） */
enum JointIdx { JIDX_BASE = 0, JIDX_SHOULDER, JIDX_ELBOW, JIDX_TOOL, JIDX_COUNT };

/* ---------- 内部状态 ---------- */
/* 每个关节各自记录上次步进时刻：一个关节被推住不放，不会拖慢其它关节。 */
static unsigned long lastStepTime[JIDX_COUNT] = { 0, 0, 0, 0 };
static unsigned long lastBlockLogTime = 0;  /* 上次打印被挡提示的时刻（限流） */

#if WEARM_HAVE_BUTTONS
static bool sw1Prev = false;
static bool sw2Prev = false;
static bool sw2LongFired = false;
static unsigned long sw2DownTime = 0;
#endif

/* 被行程限位挡住时的串口提示（限流：同一类提示每 500ms 最多一条，
 * 否则持续推杆会把串口刷爆，反而拖慢 loop）。 */
static void logBlocked(const __FlashStringHelper *msg) {
#if WEARM_DEBUG_SERIAL
  unsigned long now = millis();
  if (now - lastBlockLogTime < 500) return;
  lastBlockLogTime = now;
  Serial.println(msg);
#else
  (void) msg;
#endif
}

/* ---------- 按键读取（时间消抖，本套件默认不用） ---------- */
#if WEARM_HAVE_BUTTONS
/* 内部上拉 + 外部按下拉低，因此"按下"= LOW。
 * 消抖策略：引脚电平必须连续保持 BTN_DEBOUNCE_MS 不变，才认可为新的稳定状态。
 * 这样抖动只会推迟状态翻转，不会产生多次边沿。
 * 注意: 这里不能用 delay()，否则会阻塞串口命令的响应。 */
static bool buttonDown(uint8_t pin) {
  static bool stable[20] = { false };      /* 已确认的稳定状态（true = 按下） */
  static unsigned long changeAt[20] = { 0 };
  if (pin >= 20) return false;

  bool raw = (digitalRead(pin) == LOW);
  if (raw == stable[pin]) return stable[pin];   /* 与稳定态一致，直接返回 */

  /* 电平与稳定态不同：开始/继续计时，满 BTN_DEBOUNCE_MS 才翻转 */
  unsigned long now = millis();
  if (changeAt[pin] == 0) {
    changeAt[pin] = now;
    return stable[pin];                    /* 仍在消抖窗口内，维持旧状态 */
  }
  if (now - changeAt[pin] >= BTN_DEBOUNCE_MS) {
    stable[pin] = raw;
    changeAt[pin] = 0;
  }
  return stable[pin];
}
#endif /* WEARM_HAVE_BUTTONS */

/* ---------- 单轴采样 ---------- */

/* 读取一根摇杆的一路轴：原始 ADC 值写入 *raw，带死区的偏转量返回。
 * 偏转量只表示"离中位多远"（恒为非负），方向要看原始值是大于还是小于中位。
 * 之所以扣掉死区，是为了让"离中位越远转得越快"的调速曲线从 0 平滑起步，
 * 而不是刚出中位就直接按死区边界算满速。 */
static int readAxisAmp(uint8_t pin, int *raw) {
  int v = analogRead(pin);
  if (raw != NULL) *raw = v;
  int d = v - JOY_CENTER;
  int a = (d >= 0) ? d : -d;
  a -= JOY_DEADZONE;
  return (a > 0) ? a : 0;
}

/* ---------- 摇杆采样 ---------- */

void joystickReadState(struct joyState *st) {
  if (st == NULL) return;

  /* 1) 四路轴各自采样（两根摇杆互不影响），每路对应一个关节 */
  st->base     = readAxisAmp(JOY_LX_PIN, &st->sx);
  st->shoulder = readAxisAmp(JOY_LY_PIN, &st->sy);
  st->tool     = readAxisAmp(JOY_RX_PIN, &st->tx);
  st->elbow    = readAxisAmp(JOY_RY_PIN, &st->ty);

  /* 2) 统计本轮有动作的关节位图，并把幅度最大的那一路编码进 dir/mag。
   *    dir/mag 只是"最活跃的那一路"的摘要，供 joystickRead() 与调试使用；
   *    实际控制逐关节独立进行（见 joystickLoop），斜推可以同时动多个关节。
   *    方向码 JOY_DIR_* 与"角度增大/减小"的对应见 joystick_control.h。 */
  int mask  = 0;
  int mag   = 0;
  int best  = JOY_DIR_NONE;

  if (st->base > 0) {
    mask |= JOY_ACT_BASE;
    if (st->base > mag) {
      mag = st->base;
      best = (st->sx > JOY_CENTER) ? JOY_DIR_RIGHT : JOY_DIR_LEFT;
    }
  }
  if (st->shoulder > 0) {
    mask |= JOY_ACT_SHOULDER;
    if (st->shoulder > mag) {
      mag = st->shoulder;
      best = (st->sy > JOY_CENTER) ? JOY_DIR_BWD : JOY_DIR_FWD;
    }
  }
  if (st->elbow > 0) {
    mask |= JOY_ACT_ELBOW;
    if (st->elbow > mag) {
      mag = st->elbow;
      best = (st->ty > JOY_CENTER) ? JOY_DIR_DOWN : JOY_DIR_UP;
    }
  }
  if (st->tool > 0) {
    mask |= JOY_ACT_TOOL;
    /* 末端没有进"方向码 -> 关节"的映射表，幅度比较时保留 best 不变；
     * 但它仍在 mask 里，方便调试与"有动作就点灯"。 */
    if (st->tool > mag) mag = st->tool;
  }

  st->mask = mask;
  st->dir  = best;
  st->mag  = mag;
}

int joystickRead(void) {
  struct joyState st;
  joystickReadState(&st);
  return st.dir;
}

/* 本硬件 4 个关节同时可用，不做模式切换，恒返回 PLANE。 */
int joystickGetMode(void) {
  return JOY_MODE_PLANE;
}

/* ---------- 速度时间门控 ---------- */

/* 计算本次允许移动的最小间隔（ms）。
 * 映射规则：偏转幅度越小越接近 minDelayMs（连续快走），
 * 满偏时接近 fullDelayMs（慢而稳，最安全）。 */
static int speedIntervalMs(int mag) {
  int minDelay = speed.minDelayMs;
  int fullDelay = speed.fullDelayMs;
  if (fullDelay <= minDelay) return minDelay;

  double ratio = (double)mag / (double)JOY_FULL_SCALE;
  if (ratio < 0.0) ratio = 0.0;
  if (ratio > 1.0) ratio = 1.0;
  return minDelay + (int)(ratio * (fullDelay - minDelay));
}

/* ---------- 末端舵机 (angle4 / f) ---------- */

/* 右手柄左右推：按偏转比例步进末端角度，到机械限位就停住。
 * sign = +1 角度增大（张开），-1 角度减小（收回）。
 * 返回本次是否真的改动了角度。 */
static bool toolStep(int amp, int sign) {
  if (amp <= 0) return false;

  double ratio = (double)amp / (double)JOY_FULL_SCALE;
  if (ratio > 1.0) ratio = 1.0;
  double step = speed.stepSize * ratio * (double)sign;
  if (step == 0.0) return false;

  return posSetAngle4(Pos.ser.angle4 + step);
}

/* ---------- 指示灯 ---------- */

/* 板载 LED：任一关节在动就按快闪，全部静止则灭。
 * 旧版的 PLANE/VERT 模式指示已随模式切换一并去掉。 */
static void updateLed(bool moving) {
  if (!moving) {
    digitalWrite(PIN_LED_MODE, LOW);
    return;
  }
  bool on = ((millis() / (unsigned long)LED_FAST_MS) % 2UL) == 0UL;
  digitalWrite(PIN_LED_MODE, on ? HIGH : LOW);
}

/* ---------- 对外主循环 ---------- */

void joystickSetup(void) {
  pinMode(JOY_LX_PIN, INPUT);
  pinMode(JOY_LY_PIN, INPUT);
  pinMode(JOY_RX_PIN, INPUT);
  pinMode(JOY_RY_PIN, INPUT);
  pinMode(PIN_LED_MODE, OUTPUT);
  digitalWrite(PIN_LED_MODE, LOW);

#if WEARM_DEBUG_SERIAL
  JLOG("[joy] MeArm dual-stick JOINT control ready (A0-A3)");
  JLOG("[joy] A0 -> b angle1 base   A1 -> r angle2 shoulder");
  JLOG("[joy] A3 -> c angle3 elbow  A2 -> f angle4 tool");
#endif

  for (int i = 0; i < JIDX_COUNT; i++) lastStepTime[i] = millis();
  lastBlockLogTime = 0;
#if WEARM_HAVE_BUTTONS
  sw1Prev = false;
  sw2Prev = false;
  sw2LongFired = false;
#endif
}

void joystickLoop(void) {
  /* 1) 摇杆采样（4 路轴一次读完，同时给出方向、幅度与活跃位图） */
  struct joyState st;
  joystickReadState(&st);

  unsigned long now = millis();
  bool moved = false;

  /* 2) 按"关节 | 偏转量 | 原始轴值 | 该关节的计时槽"逐轴步进。
   *    每个关节自己一套时间门控：一个关节推到头或在慢速档，
   *    不会把另一个关节也拖慢。偏转越大步越慢（安全）。 */
  /* 【方向约定】上臂 r 与下臂 c 这两路推杆方向是上机实测后调转过的：
   *   上臂 A1 前推 = r 减小、后拉 = r 增大；
   *   下臂 A3 前推 = c 减小、后拉 = c 增大。
   *   改动方式是把这两路的 dirPos/dirNeg 对调；JOY_DIR_FWD/UP 等方向码
   *   本身的含义（"该关节角度增大"）没变，只是哪一侧推杆对应哪个码换了。
   *   基座 A0 与末端 A2 两路的方向没有改动。 */
  const struct {
    int amp;        /* 该轴扣死区后的偏转量，0 = 没推 */
    int raw;        /* 该轴原始 ADC 值，用来判方向 */
    int dirPos;     /* 角度增大方向编码 */
    int dirNeg;     /* 角度减小方向编码 */
    int idx;        /* lastStepTime 下标 */
  } axes[JIDX_COUNT] = {
    { st.base,     st.sx, JOINT_B_RIGHT, JOINT_B_LEFT,  JIDX_BASE     },
    { st.shoulder, st.sy, JOINT_R_BWD,   JOINT_R_FWD,   JIDX_SHOULDER },
    { st.elbow,    st.ty, JOINT_C_DOWN,  JOINT_C_UP,    JIDX_ELBOW    },
    { st.tool,     st.tx, 0,             0,             JIDX_TOOL     }
  };

  for (int i = 0; i < JIDX_COUNT; i++) {
    /* 取放序列执行期间让位：序列独占 b/r/c 三个关节角与末端角，摇杆一律不步进 */
    if (pickPlaceIsBusy()) break;

    if (axes[i].amp <= 0) continue;

    int interval = speedIntervalMs(axes[i].amp);
    if (now - lastStepTime[axes[i].idx] < (unsigned long)interval) continue;

    if (i == JIDX_TOOL) {
      /* 末端夹具：角度增大 = 张开，减小 = 收回。
       * 方向约定与备份里的旧手柄版一致（D:\wearm-backup-cartesian-20261002-205737\
       * joystick_control.cpp 里写作 tx > JOY_CENTER ? -1 : 1，注释为"右推关闭、左推张开"）：
       * 右推 = 收回（角度减小），左推 = 张开（角度增大）。
       * 上机若觉得反了，把这里的三元式对调即可。 */
      int sign = (axes[i].raw > JOY_CENTER) ? -1 : 1;
      if (toolStep(axes[i].amp, sign)) {
        moved = true;
      } else {
        logBlocked(F("[joy] blocked: tool at servo limit"));
      }
    } else {
      /* 关节角步进：moveJointStep 自带 servoLimit 夹取与正解坐标刷新 */
      int dir = (axes[i].raw > JOY_CENTER) ? axes[i].dirPos : axes[i].dirNeg;
      int res = moveJointStep(dir, speed.stepSize);
      if (res == MOVE_OK) {
        moved = true;
      } else if (res == MOVE_AT_LIMIT) {
        logBlocked(F("[joy] blocked: joint at travel limit"));
      } else if (res == MOVE_UNREACHABLE) {
        logBlocked(F("[joy] blocked: position out of workspace"));
      }
    }
    /* 无论成功还是被限位都重置计时，避免顶住不放时刷串口 */
    lastStepTime[axes[i].idx] = now;
  }

#if WEARM_JOY_DEBUG
  if (moved) {
    Serial.print(F("[joy] raw A0..A3="));
    Serial.print(st.sx); Serial.print(',');
    Serial.print(st.sy); Serial.print(',');
    Serial.print(st.tx); Serial.print(',');
    Serial.print(st.ty);
    Serial.print(F("  b=")); Serial.print(Pos.ser.angle1);
    Serial.print(F(" r="));  Serial.print(Pos.ser.angle2);
    Serial.print(F(" c="));  Serial.print(Pos.ser.angle3);
    Serial.print(F(" f="));  Serial.print(Pos.ser.angle4);
    Serial.print(F("  rec x=")); Serial.print(Pos.rec.x);
    Serial.print(F(" y="));      Serial.print(Pos.rec.y);
    Serial.print(F(" z="));      Serial.println(Pos.rec.z);
  }
#endif

  /* 3) 指示灯：本轮有任何关节在动 -> 快闪；否则灭 */
  updateLed(moved);
}
