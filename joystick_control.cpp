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
#include "button_control.h"
#include "draw_control.h"

/* ---------- 编译开关 ---------- */
/* 置 1: 打开调试串口输出（波特率由 serial_protocol 模块初始化）。
 * 需与 constant_and_positions.cpp 中的同名开关保持一致。 */
#include "weArm_config.h"
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

/* ---------- 配置：中位自标定 ---------- */
/* 真实手柄的机械中位很少正好是 ADC 的 512：实测常见偏 20~60 个计数。
 * 偏 20 就已经超过死区（15），于是"手不碰摇杆"时每一路都被当成轻微推杆，
 * 机械臂会持续缓慢地自己乱走 —— 这正是"无故乱动"的软件侧根因。
 * 所以开机时（joystickSetup）把四路各平均若干次，把静态偏差记下来，
 * 之后所有读数都先扣掉这个偏差，中位判据仍然只需要和 512 比。
 * 上限 JOY_CAL_MAX_OFF 用来兜底：开机时若有人手压着摇杆，偏差会远超此值，
 * 那就判定"这次标定不可信"，退回到标准中位 512（偏差记 0）。 */
#define JOY_CAL_SAMPLES  8  /* 每路标定取多少次 ADC 求平均 */
#define JOY_CAL_MAX_OFF 64  /* 允许的静态偏差上限（计数） */

/* ---------- 配置：时序 ---------- */
#define JOY_BTN_DEBOUNCE_MS 30  /* 按键消抖窗口（本套件默认无按键，保留供扩展） */
#define LED_FAST_MS    100  /* 运动中闪烁半周期 */

/* 关节编号（用于 lastStepTime[] 与调试打印，顺序与 JOY_ACT_* 位图一致） */
enum JointIdx { JIDX_BASE = 0, JIDX_SHOULDER, JIDX_ELBOW, JIDX_TOOL, JIDX_COUNT };

/* "Angle decreases" direction code of each joint axis, indexed like JointIdx (the
 * tool axis never looks this table up).  The "angle increases" code of an axis is
 * exactly this value + 1, because move.h declares MoveDir / JointDir in adjacent
 * pairs (JOINT_B_LEFT/RIGHT = 3/4, JOINT_R_FWD/BWD = 5/6, JOINT_C_UP/DOWN = 1/2).
 * Storing only the "decreases" side halves the table; JOY_DIR_* codes in the
 * header keep their published values.  PROGMEM because AVR RAM is precious. */
static const unsigned char JOY_DIR_NEG[JIDX_COUNT] PROGMEM = {
  (unsigned char)JOINT_B_LEFT,
  (unsigned char)JOINT_R_FWD,
  (unsigned char)JOINT_C_UP,
  0
};

/* English note: on AVR the table above lives in flash, so it is fetched with
 * pgm_read_byte().  The PC self-check harness defines PROGMEM as nothing and
 * has no pgmspace helpers, so there it is an ordinary RAM array read directly.
 * Both builds read the same four bytes. */
#ifdef __AVR__
#define JOY_DIRNEG(i) ((int)pgm_read_byte(&JOY_DIR_NEG[(i)]))
#else
#define JOY_DIRNEG(i) ((int)JOY_DIR_NEG[(i)])
#endif

/* joyState 里四路原始 ADC 值按 sx, sy, tx, ty 声明，而关节顺序是
 * base, shoulder, elbow, tool —— 后两个正好对调。这张 4 字节的表把"关节下标"
 * 换算成"原始值下标"，于是幅度与原始值两组字段都能跟着同一个 i 顺序走，
 * 循环里不必再往栈上抄一份 raw[] 副本（每个轴一对 ldd/std，很不划算）。 */
static const unsigned char JOY_RAW_SLOT[JIDX_COUNT] PROGMEM = { 0, 1, 3, 2 };

#ifdef __AVR__
#define JOY_RAWSLOT(i) ((int)pgm_read_byte(&JOY_RAW_SLOT[(i)]))
#else
#define JOY_RAWSLOT(i) ((int)JOY_RAW_SLOT[(i)])
#endif

/* ---------- 内部状态 ---------- */
/* 每个关节各自记录上次步进时刻：一个关节被推住不放，不会拖慢其它关节。 */
static unsigned long lastStepTime[JIDX_COUNT] = { 0, 0, 0, 0 };

/* 四路 ADC 的中位偏差（下标 = pin - A0，顺序 A0,A1,A2,A3），开机自标定得到。
 * 只在这里保存，扣减发生在 readAxisAmp() 内部，所以下面所有
 * "raw > JOY_CENTER" 的方向判断、以及死区计算都无需改动。 */
static int s_centerOff[4] = { 0, 0, 0, 0 };

#if WEARM_DEBUG_SERIAL
static unsigned long lastBlockLogTime = 0;  /* 上次打印被挡提示的时刻（限流） */
#endif

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
 * 消抖策略：引脚电平必须连续保持 JOY_BTN_DEBOUNCE_MS 不变，才认可为新的稳定状态。
 * 这样抖动只会推迟状态翻转，不会产生多次边沿。
 * 注意: 这里不能用 delay()，否则会阻塞串口命令的响应。 */
static bool buttonDown(uint8_t pin) {
  static bool stable[20] = { false };      /* 已确认的稳定状态（true = 按下） */
  static unsigned long changeAt[20] = { 0 };
  if (pin >= 20) return false;

  bool raw = (digitalRead(pin) == LOW);
  if (raw == stable[pin]) return stable[pin];   /* 与稳定态一致，直接返回 */

  /* 电平与稳定态不同：开始/继续计时，满 JOY_BTN_DEBOUNCE_MS 才翻转 */
  unsigned long now = millis();
  if (changeAt[pin] == 0) {
    changeAt[pin] = now;
    return stable[pin];                    /* 仍在消抖窗口内，维持旧状态 */
  }
  if (now - changeAt[pin] >= JOY_BTN_DEBOUNCE_MS) {
    stable[pin] = raw;
    changeAt[pin] = 0;
  }
  return stable[pin];
}
#endif /* WEARM_HAVE_BUTTONS */

/* ---------- 单轴采样 ---------- */

/* 开机自标定一路轴：平均若干次读数，把相对 512 的静态偏差记进 s_centerOff。
 * 偏差超过 JOY_CAL_MAX_OFF 就判为"标定时手正压着摇杆"（真中位不可能偏这么多），
 * 记 0 —— 退回到标准中位，宁可回到老行为也不要把一路轴标歪。 */
static void joyCalibrateAxis(uint8_t pin) {
  long sum = 0;
  for (uint8_t i = 0; i < JOY_CAL_SAMPLES; i++) { sum += analogRead(pin); }
  int off = (int)(sum / (long)JOY_CAL_SAMPLES) - JOY_CENTER;
  if (off > JOY_CAL_MAX_OFF || off < -JOY_CAL_MAX_OFF) { off = 0; }
  s_centerOff[pin - A0] = off;
}

/* 读取一根摇杆的一路轴：原始 ADC 值写入 *raw，带死区的偏转量返回。
 * 偏转量只表示"离中位多远"（恒为非负），方向要看原始值是大于还是小于中位。
 * 之所以扣掉死区，是为了让"离中位越远转得越快"的调速曲线从 0 平滑起步，
 * 而不是刚出中位就直接按死区边界算满速。
 * 读数先扣掉开机自标定的静态偏差：*raw 交出去的也是扣过的值，
 * 于是下游 "raw > JOY_CENTER" 的方向判断照旧成立（校正后的中位就是 512）。 */
static int readAxisAmp(uint8_t pin, int *raw) {
  int v = analogRead(pin) - s_centerOff[pin - A0];
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

  /* 原来是把 ratio 夹到 [0,1]（两次浮点比较 + __cmpsf2/__gesf2 调用），
   * 现在改成夹 mag 这个整数：ratio = mag / JOY_FULL_SCALE，
   * ratio < 0 <=> mag < 0，ratio > 1 <=> mag > JOY_FULL_SCALE —— 逐条等价，
   * 对整数 mag 来说浮点除法的舍入不会改变这两个判断的结果。 */
  if (mag < 0) mag = 0;
  else if (mag > JOY_FULL_SCALE) mag = JOY_FULL_SCALE;

  return minDelay + (int)((double)mag / (double)JOY_FULL_SCALE * (double)(fullDelay - minDelay));
}

/* ---------- 末端舵机 (angle4 / f) ---------- */

/* 右手柄左右推：按偏转比例步进末端角度，到机械限位就停住。
 * sign = +1 角度增大（张开），-1 角度减小（收回）。
 * 返回本次是否真的改动了角度。 */
static bool toolStep(int amp, int sign) {
  if (amp <= 0) return false;

  /* 与上面同理：把 ratio 夹到 <= 1 换成"把 amp 夹到 <= JOY_FULL_SCALE"，
     整数比较省掉一次 __gesf2 调用。 */
  if (amp > JOY_FULL_SCALE) amp = JOY_FULL_SCALE;

  double step = speed.stepSize * ((double)amp / (double)JOY_FULL_SCALE);
  /* sign 只会是 +1 / -1：乘 ±1.0f 与"按符号取负"在 IEEE 下逐位等价
     （x * -1 == -x，x * 1 == x），省掉一次 __floatsisf + __mulsf3。
     step 为 0 时 posSetAngle4() 会因为"值没变"自己返回 false，
     非有限值也由它挡掉，所以这里不再单独判 step == 0.0。 */
  if (sign < 0) step = -step;

  return posSetAngle4(Pos.ser.angle4 + step);
}

/* ---------- 指示灯 ---------- */

/* 板载 LED：任一关节在动就按快闪，全部静止则灭。
 * 旧版的 PLANE/VERT 模式指示已随模式切换一并去掉。
 *
 * 这里直接写 PORTB5（D13）而不是 digitalWrite()：理由与 joystickSetup() 里相同，
 * digitalWrite() 在 Uno 上是真实函数调用，sbi/cbi 两条指令与它语义一致。 */
static void updateLed(bool moving) {
  if (!moving) {
    PORTB &= (unsigned char)~_BV(PB5);
    return;
  }
  bool on = ((millis() / (unsigned long)LED_FAST_MS) % 2UL) == 0UL;
  if (on) PORTB |= (unsigned char)_BV(PB5);
  else    PORTB &= (unsigned char)~_BV(PB5);
}

/* ---------- 对外主循环 ---------- */

void joystickSetup(void) {
  /* 四路 ADC 引脚设为输入（无上拉）、板载 LED 设为输出并灭灯。
   * 这里直接写 AVR 寄存器：pinMode()/digitalWrite() 在 Uno 上都是非内联库函数，
   * 五次调用光压参 + call 就要三十多字节，而下面三句寄存器操作是等价的
   * （A0..A3 = PC0..PC3，D13 = PB5，这是 ATmega328P 上的固定映射）。
   * PORTC 复位后为 0，pinMode(INPUT) 顺手清掉的上拉位本来就是 0，故不再重复清。 */
  DDRC &= (unsigned char)~(_BV(PC0) | _BV(PC1) | _BV(PC2) | _BV(PC3));
  DDRB |= (unsigned char)_BV(PB5);
  PORTB &= (unsigned char)~_BV(PB5);

#if WEARM_DEBUG_SERIAL
  JLOG("[joy] MeArm dual-stick JOINT control ready (A0-A3)");
  JLOG("[joy] A0 -> b angle1 base   A1 -> r angle2 shoulder");
  JLOG("[joy] A3 -> c angle3 elbow  A2 -> f angle4 tool");
#endif

  /* 摇杆中位自标定：四路各平均 JOY_CAL_SAMPLES 次，把静态偏差存起来。
   * 必须在这里、且在机械臂开始动作之前做完 —— 此刻没人碰摇杆，读到的就是真中位。
   * 若某路偏差大得离谱（开机就被压住），joyCalibrateAxis() 会自己退回 0。 */
  joyCalibrateAxis(JOY_LX_PIN);
  joyCalibrateAxis(JOY_LY_PIN);
  joyCalibrateAxis(JOY_RX_PIN);
  joyCalibrateAxis(JOY_RY_PIN);

  /* 四个关节的"上次步进时刻"清零到当前时刻。
   * 注意：不要把 millis() 提到循环外只取一次 —— 实测那样做 GCC 会把这 4 次
   * 迭代完全展开成 16 条直写（sts），从 58 字节涨到 100 字节。
   * 原来的写法（循环体里调用）反而让编译器保留指针自增写法，更省。 */
  for (int i = 0; i < JIDX_COUNT; i++) lastStepTime[i] = millis();
#if WEARM_DEBUG_SERIAL
  lastBlockLogTime = 0;
#endif
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

    /* 2) Evaluate the exclusivity test once: while a pick/place sequence, a button
   *    playback/homing run or a draw task is active, the stick never steps any
   *    axis (all three own b/r/c and the tool angle).  The old code repeated
   *    this test inside the loop body (once per axis) and again in the closing
   *    expression (three more times); now it is computed once.
   *    buttonControlLocked() deliberately excludes "recording": recording is
   *    meant to capture your stick motion, so the stick must stay live.
   *    The five-point teach of a draw task does not come through here: while
   *    teaching, drawControlLocked() is true as well, so this test blocks the
   *    stick and draw_control.cpp reads it itself for cartesian jogging
   *    (see drawLoop). */
  /* 三个判据都是"只读一个静态标志"的纯函数，没有副作用，所以用按位或
   * 把三个结果一次算完，省掉 || 的短路分支（结果完全相同）。 */
  bool locked = pickPlaceIsBusy() | buttonControlLocked() | drawControlLocked();
  bool moved = false;

  /* English note: the four joint axes are visited in JointIdx order (base,
   * shoulder, elbow, tool).  joyState keeps the four amplitudes at consecutive
   * offsets in exactly that order, so a single pointer walks them with no stack
   * copy; the four raw ADC values (declared sx, sy, tx, ty) are reached through
   * the same index via JOY_RAW_SLOT, which fixes up the last two.  Each axis
   * keeps its own time gate, so one axis held against a limit or running in the
   * slow band never slows the others down.  The order itself is load bearing:
   * when two axes pass their gate in the same round, the first move shifts the
   * pose the second one is checked against, so it must not be reordered. */
  const int *amp  = &st.base;
  const int *rawp = &st.sx;

  if (!locked) {
    unsigned long now = millis();

    for (int i = 0; i < JIDX_COUNT; i++) {
      if (amp[i] <= 0) continue;

      int interval = speedIntervalMs(amp[i]);
      if (now - lastStepTime[i] < (unsigned long)interval) continue;

      int raw = rawp[JOY_RAWSLOT(i)];

            /* Direction convention: the two stick channels of the upper arm r and the
       * lower arm c were swapped after testing on the real machine:
       *   upper arm A1 pushed forward = r decreases, pulled back = r increases;
       *   lower arm A3 pushed forward = c decreases, pulled back = c increases.
       * So the base/shoulder/elbow slots of JOY_DIR_NEG[] hold
       * JOINT_B_LEFT / JOINT_R_FWD / JOINT_C_UP, and the "angle increases" side
       * is that value + 1.  The meaning of the JOY_DIR_FWD/UP codes themselves
       * ("this joint angle increases") is unchanged; only which side of the
       * stick maps to which code.  Base A0 and tool A2 are untouched. */
      if (i == JIDX_TOOL) {
                /* Tool gripper: a larger angle opens it, a smaller one closes it.  The
         * convention matches the old handset version in the backup
         * (D:\wearm-backup-cartesian-20261002-205737\joystick_control.cpp writes
         * tx > JOY_CENTER ? -1 : 1, commented "push right closes, push left
         * opens"): push right = close (angle decreases), push left = open
         * (angle increases).  If it feels inverted on the machine, swap the two
         * branches of this ternary. */
        int sign = (raw > JOY_CENTER) ? -1 : 1;
        if (toolStep(amp[i], sign)) {
          moved = true;
        } else {
          logBlocked(F("[joy] blocked: tool at servo limit"));
        }
      } else {
                /* Joint step: moveJointStep clamps to servoLimit and refreshes Pos.rec */
        int dirNeg = JOY_DIRNEG(i);
        int dir = (raw > JOY_CENTER) ? (dirNeg + 1) : dirNeg;
        int res = moveJointStep(dir, speed.stepSize);
        if (res == MOVE_OK) {
          moved = true;
        } else if (res == MOVE_AT_LIMIT) {
          logBlocked(F("[joy] blocked: joint at travel limit"));
        } else if (res == MOVE_UNREACHABLE) {
          logBlocked(F("[joy] blocked: position out of workspace"));
        }
      }
            /* Reset the time gate either way, so holding a stick still never floods. */
      lastStepTime[i] = now;
    }
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

    /* 3) Indicator LED: fast blink while any joint moves this round, off when all
   *    are still.  While a pick/place sequence, a button playback/homing run or a
   *    draw task owns the arm the stick is locked (moved stays false) but the
   *    joints really do move, so those "someone else drives the arm" states count
   *    as moving too; otherwise the LED would be dark for those tens of seconds
   *    and look like a hang.  locked is the result computed above, reused here
   *    instead of calling the three predicates again. */
  updateLed(moved || locked);
}
