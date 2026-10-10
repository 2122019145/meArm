/*
 * button_control.cpp -- implementation of the four on-board buttons.
 *
 * The public interface is documented in button_control.h.  Implementation notes:
 *
 *   1) Hardware: buttons 1~4 are wired to D2~D5, INPUT_PULLUP, a press reads LOW.
 *      Software debounce is BTN_DEBOUNCE_MS and only the press edge acts.
 *      【本机默认不走这条路】WEARM_BUTTON_PINS = 0（见 weArm_config.h 第 4 节）
 *      时整段引脚扫描都被裁掉，四个功能全部改由串口 N/R/P/M/0 触发；
 *      原因见 weArm_config.h：右摇杆推到最前会压到 SW，假触发按键1 抢走操作。
 *
 *   2) Recording format (fixed static buffer, no dynamic allocation):
 *      one packed record per BTN_TICK_MS (= WEARM_REC_TICK_MS, default 200 ms)
 *      sampling tick holds the angle delta of all four joints, 8 signed bits
 *      each (unit = 0.5 deg):
 *         byte 0  joint 0 (base)          delta in recording units
 *         byte 1  joint 1 (upper arm)
 *         byte 2  joint 2 (forearm)
 *         byte 3  joint 3 (end effector)
 *      BTN_REC_ENTRIES = 192 records x 4 bytes = 768 bytes = 192 ticks = 38.4 s
 *      (see weArm_config.h section 5 for the SRAM / length trade-off).
 *      Every tick gets exactly one record, moving or not, so the playback time
 *      axis is the exact recording timeline; an all-zero record simply means
 *      "no motion".
 *      A joint delta wider than the 8-bit field (+-128 units = +-64 deg per tick)
 *      is clamped, but s_snap[] follows the *emitted* position rather than the
 *      real one, so the remainder is carried into the next ticks: the recorded
 *      total displacement stays exact and only a very fast move arrives a tick
 *      or two later during playback.  (At 200 ms per tick the field limit is
 *      317 deg/s per joint, so the slow and normal presets are covered exactly;
 *      only the top of the fast preset - 400 deg/s - relies on the carry, and
 *      btnStopRecording() flushes the leftover carry at the end.)
 *      Playback spreads every record over BTN_PLAY_SUBSTEPS equal sub-steps so
 *      a coarse sampling rate does not turn the replay into 200 ms jumps; the
 *      last sub-step lands exactly on the recorded delta.
 *
 *   3) Save validation (the end-effector must have moved, otherwise the
 *      recording is dropped):
 *         end-effector travel on x/y/z >= BTN_REC_MIN_TRAVEL (10.0)
 *      【没有最小时长门槛】原来还要求"录制时长必须大于 10 秒"，已按要求删除：
 *      短动作（哪怕 1 秒）只要末端走够了 10 个单位就照样保存，最长受
 *      BTN_REC_ENTRIES = 192 条 = 38.4 秒的缓冲上限约束（写满即溢出判废）。
 *      A new recording reuses the same buffer from its first tick on, so a
 *      rejected recording also invalidates the previous one.  This keeps the
 *      longest possible recording inside the 2 KB SRAM of the Uno; the serial
 *      port says so when a recording starts.
 *
 *   4) Both playback and homing first ramp (BTN_RAMP_MS) smoothly to the target
 *      pose, in joint space (no inverse kinematics, so it can never be
 *      unreachable on the way).
 *
 *   5) Arbitration (see the end of button_control.h):
 *      the joystick must stay live while recording; playback and homing take it
 *      over; serial motion commands are rejected by buttonHandleCommand() while
 *      recording / playing / homing, but N/R/P/M themselves stay exempt in the
 *      busy guard of serial_protocol.cpp so that R can still end a recording.
 */
#include "Arduino.h"
#include "constant_and_positions.h"
#include "pick_place.h"
#include "protocol_constants.h"
#include "button_control.h"
#include "draw_control.h"

#if WEARM_ENABLE_BUTTONS

/* ---------------- tunables ---------------- */

/* Button pins: D2/D3/D4/D5 are the only four spare easy-to-wire digital pins
 * on this board (D0/D1 serial, D6~D9 servos, D13 led, A0~A3 joystick).
 *
 * 【默认不接物理按键】本机没接独立按键，只有摇杆自带的 SW 脚；而右摇杆推到
 * 最前时机械上会压到那颗轻触开关，接到 D2 就会假触发"按键1 循环取放"、
 * 把正在手动推杆的操作抢走。所以下面这一整段"读引脚"的代码默认被
 * WEARM_BUTTON_PINS=0 裁掉（见 weArm_config.h 第 4 节），按键功能全部改走
 * 串口 N / R / P / M（或 0）。把 WEARM_BUTTON_PINS 改回 1 即可恢复物理按键。 */
#if WEARM_BUTTON_PINS
#define BTN_PIN_CYCLE   2
#define BTN_PIN_RECORD  3
#define BTN_PIN_PLAY    4
#define BTN_PIN_HOME    5
#endif

#define BTN_DEBOUNCE_MS     25UL      /* button debounce time（仅物理按键用） */
/* 采样周期与缓冲条数是 weArm_config.h 第 5 节的编译期开关：改短了回放更顺但同样
 * SRAM 录得更短，改长了相反。默认 192 条 × 200 ms = 38.4 秒、768 B SRAM。 */
#define BTN_TICK_MS         ((unsigned long) WEARM_REC_TICK_MS)  /* sampling period */
#define BTN_RAMP_MS         1500UL    /* smooth ramp before playback / homing */
#define BTN_REC_MIN_TRAVEL  10.0      /* minimum end-effector travel */
#define BTN_REC_ENTRIES     WEARM_REC_ENTRIES  /* 192 * 4 = 768 bytes SRAM */
#define BTN_REC_BYTES       4         /* packed size of one record: 4 x 8 bit */
#define BTN_REC_LIMIT       128       /* 8-bit signed field range: -128..+127 units */
#define BTN_ANGLE_UNIT      2.0       /* 1 deg = 2 recording units (unit = 0.5 deg) */
/* 回放时把一条记录拆成几个子步（1 = 整条一次跳完，退回老行为）。200 ms 采样在
 * 4 个子步下就是每 50 ms 走一小段，动作不会一顿一顿。必须能整除 BTN_TICK_MS。 */
#define BTN_PLAY_SUBSTEPS   4

#if WEARM_BUTTON_PINS
static const uint8_t BTN_PIN[BTN_COUNT] = {
  BTN_PIN_CYCLE, BTN_PIN_RECORD, BTN_PIN_PLAY, BTN_PIN_HOME
};
#endif

/* ---------------- recording buffer ---------------- */

/* One packed record per sampling tick: four 8-bit signed joint deltas.
 * 192 条 × 4 字节 = 768 B —— 这是整个固件最大的一块 SRAM。 */
static uint8_t s_buf[BTN_REC_ENTRIES * BTN_REC_BYTES];

/* ---------------- runtime state ---------------- */

enum { BS_IDLE = 0, BS_RAMP, BS_PLAY };

static uint8_t s_state       = BS_IDLE;  /* playback / homing state machine */
static uint8_t s_cycleIdx    = 0;        /* next object picked by button 1 */
static bool    s_recording   = false;    /* recording in progress */
static bool    s_rampThenPlay = false;   /* after the ramp: play (true) or stop (false) */

/* recording */
static uint16_t      s_recCount     = 0;     /* records written by the current recording */
static uint16_t      s_recLen       = 0;     /* records of the saved recording */
static bool          s_hasRec       = false; /* a playable recording exists */
static bool          s_recOverflow  = false; /* the buffer ran full */
static unsigned long s_recStartMs    = 0;
static unsigned long s_recDurationMs = 0;
static unsigned long s_tickNow       = 0;    /* last tick index written */
static double        s_snap[4]      = {0.0, 0.0, 0.0, 0.0}; /* last *emitted* joint angles (deg) */
static double        s_recStart[4]  = {0.0, 0.0, 0.0, 0.0}; /* joint angles when recording began */
static double        s_recTravel    = 0.0;                  /* end-effector travel */
static double        s_bbMin[3]     = {0.0, 0.0, 0.0};      /* x/y/z box of the recording */
static double        s_bbMax[3]     = {0.0, 0.0, 0.0};

/* ramping */
static double        s_rampFrom[4] = {0.0, 0.0, 0.0, 0.0};
static double        s_rampTo[4]   = {0.0, 0.0, 0.0, 0.0};
static unsigned long s_rampStartMs = 0;

/* playback */
static uint16_t      s_playIdx   = 0;     /* uint16_t: 缓冲可达 256 条以上，uint8_t 会装不下 */
static unsigned long s_playDueMs = 0;
static uint8_t       s_subIdx    = 0;     /* 当前记录已插值走的子步数（0 = 还没装载） */
static double        s_subDelta[4]   = {0.0, 0.0, 0.0, 0.0};  /* 整条记录的关节增量（度） */
static double        s_subApplied[4] = {0.0, 0.0, 0.0, 0.0};  /* 子步已经贴上去的部分（度） */

/* button debounce（只有物理按键在用；WEARM_BUTTON_PINS=0 时整个裁掉，
 * 省下 4+4+16 字节 SRAM 和 4 路 digitalRead） */
#if WEARM_BUTTON_PINS
static bool          s_btnStable[BTN_COUNT] = {false, false, false, false};
static bool          s_btnArmed[BTN_COUNT]  = {false, false, false, false};
static unsigned long s_btnChangeAt[BTN_COUNT] = {0UL, 0UL, 0UL, 0UL};
#endif

/* ---------------- small helpers ---------------- */

/* SER keeps angle1..angle4 as four consecutive doubles, so one pointer does the
 * job of the two four-way switches this used to need. */
static double *btnAngle(int joint) {
  return &Pos.ser.angle1 + joint;
}

/* Apply the joint angles, then re-clamp and re-solve once so that Pos.ser and
 * Pos.rec always stay consistent.  Recording only moves inside the configured
 * travel, so clampServoAngles() normally changes nothing; this is just a guard
 * against a stale out-of-range angle sticking in Pos. */
static void btnCommitAngles(void) {
  SER tmp = Pos.ser;
  /* b/r/c 三轴硬编码 0~180、f 硬编码 60~150（原 clampServoAngles 已随
   * servoLimit 结构体一起删除）。录制/回放/回中的角度理论上都落在合法区间
   * 内，此夹取只是一道保险。 */
  if (tmp.angle1 < 0.0)   tmp.angle1 = 0.0;
  if (tmp.angle1 > 180.0) tmp.angle1 = 180.0;
  if (tmp.angle2 < 0.0)   tmp.angle2 = 0.0;
  if (tmp.angle2 > 180.0) tmp.angle2 = 180.0;
  if (tmp.angle3 < 0.0)   tmp.angle3 = 0.0;
  if (tmp.angle3 > 180.0) tmp.angle3 = 180.0;
  if (tmp.angle4 < 60.0)  tmp.angle4 = 60.0;
  if (tmp.angle4 > 150.0) tmp.angle4 = 150.0;
  Pos.ser = tmp;
  (void) recFromServo(&Pos.rec, &Pos.ser);
}

/* Angle delta -> recording units (0.5 deg, rounded to nearest).  No clamping:
 * whatever does not fit into one record is carried into the following ones. */
static int btnToUnits(double deltaDeg) {
  double u = deltaDeg * BTN_ANGLE_UNIT;
  return (int) ((u >= 0.0) ? (u + 0.5) : (u - 0.5));
}

static double btnFromUnits(int units) {
  return (double) units / BTN_ANGLE_UNIT;
}

/* Recorded travel statistics: only the end-effector range on x/y/z matters.
 * REC keeps x/y/z as three consecutive doubles.  Indices are read from the
 * object, so the loop body stays a single shared copy. */
static void btnUpdateTravel(const REC *rec) {
  const double *v = &rec->x;
  for (int a = 0; a < 3; a++) {
    double x = v[a];
    if (x < s_bbMin[a]) s_bbMin[a] = x;
    if (x > s_bbMax[a]) s_bbMax[a] = x;
  }
}

static const char *btnObjectName(int object) {
  static const char *const name[PICK_OBJECT_COUNT] = { "A", "B", "C" };
  if (object < 0 || object >= PICK_OBJECT_COUNT) return "?";
  return name[object];
}

/* The end-effector travel of the recording: the widest of the three x/y/z
 * min/max spans.  A local temp keeps s_recTravel out of the loop body. */
static double btnTravelSpan(void) {
  double t = s_bbMax[0] - s_bbMin[0];
  for (int a = 1; a < 3; a++) {
    double d = s_bbMax[a] - s_bbMin[a];
    if (d > t) t = d;
  }
  return t;
}

/* ---------------- recording ---------------- */

/* Pack one record: four 8-bit signed joint deltas into BTN_REC_BYTES bytes. */
static void btnRecPut(const int *u) {
  if (s_recCount >= BTN_REC_ENTRIES) {
    s_recOverflow = true;
    return;
  }
  uint8_t *p = &s_buf[s_recCount * BTN_REC_BYTES];
  uint32_t v = 0;
  for (int j = 0; j < 4; j++) {
    v |= ((uint32_t) (u[j] & 0xFF)) << (8 * j);
  }
  p[0] = (uint8_t) v;
  p[1] = (uint8_t) (v >> 8);
  p[2] = (uint8_t) (v >> 16);
  p[3] = (uint8_t) (v >> 24);
  s_recCount++;
}

/* Collect what moved during one tick: the delta of every joint since the angle
 * the recording emitted last, clamped to the 8-bit field range (-128..+127, so
 * the positive limit is BTN_REC_LIMIT - 1: +128 would alias to -128).  s_snap[]
 * is advanced by the clamped amount (not by the real angle), so anything that
 * does not fit into this record is carried into the next one.  Returns true if
 * anything moved. */
static bool btnRecCollect(int *u) {
  bool moved = false;
  for (int j = 0; j < 4; j++) {
    int d = btnToUnits(*btnAngle(j) - s_snap[j]);
    if (d >  BTN_REC_LIMIT - 1) d =  BTN_REC_LIMIT - 1;
    if (d < -BTN_REC_LIMIT)     d = -BTN_REC_LIMIT;
    u[j] = d;
    if (d != 0) {
      moved = true;
      s_snap[j] += btnFromUnits(d);
    }
  }
  return moved;
}

static int  btnStopRecording(void);   /* forward declaration: the buffer may run full */

/* noinline: the body is reached from two places (按键2 的 btnAction 入口 和
 * btnRecTick 的缓冲写满自动收尾)，内联会把这一整段复制两份。 */
static void btnStartRecording(void) __attribute__((noinline));

static void btnStartRecording(void) {
  s_recCount      = 0;
  s_recOverflow   = false;
  s_recording     = true;
  s_recStartMs    = millis();
  s_tickNow       = 0;
  s_recDurationMs = 0;
  s_recTravel     = 0.0;
  for (int j = 0; j < 4; j++) {
    s_snap[j]     = *btnAngle(j);
    s_recStart[j] = s_snap[j];
  }
  s_bbMin[0] = s_bbMax[0] = Pos.rec.x;
  s_bbMin[1] = s_bbMax[1] = Pos.rec.y;
  s_bbMin[2] = s_bbMax[2] = Pos.rec.z;

#if WEARM_DEBUG_SERIAL
  Serial.print(F("[btn] 开始录制：请用摇杆操控机械臂（要有明显位移，最长 "));
  Serial.print((unsigned long) BTN_REC_ENTRIES * BTN_TICK_MS / 1000UL);
  Serial.println(F(" 秒）"));
  Serial.println(F("[btn] 提示：本次录制会覆盖上一次的录制数据"));
#endif
}

/* Runs once per loop() while recording: every sampling tick that has elapsed gets
 * exactly one record, so the time axis is the real recording timeline even when
 * one loop iteration spans several ticks. */
static void btnRecTick(unsigned long now) {
  unsigned long tick = (now - s_recStartMs) / BTN_TICK_MS;

  while (s_tickNow < tick && !s_recOverflow) {
    int u[4];
    s_tickNow++;
    (void) btnRecCollect(u);
    btnUpdateTravel(&Pos.rec);
    btnRecPut(u);
  }

  if (s_recOverflow) {
#if WEARM_DEBUG_SERIAL
    Serial.print(F("[btn] 录制缓冲已满（"));
    Serial.print((unsigned int) BTN_REC_ENTRIES);
    Serial.println(F(" 条），自动结束录制"));
#endif
    (void) btnStopRecording();
  }
}

/* End the recording and decide whether it is worth keeping; the return value is
 * the PROTO_RES_* reply of the serial protocol.  A recording that fails the
 * checks is dropped, and since it shares the buffer with the previous one, that
 * one is gone as well. */
static int btnStopRecording(void) {
  unsigned long now = millis();

  if (!s_recOverflow) {
    /* Flush the motion of the unfinished tick.  One record normally holds all of
     * it, but a joint that outran the field range can still have a carry backlog
     * left (the carry is drained one record per call), so keep flushing until
     * nothing is left.  Without this the tail of a fast move would be dropped and
     * the playback would stop short of where the recording ended. */
    int u[4];
    for (int i = 0; i < 8 && !s_recOverflow; i++) {
      if (!btnRecCollect(u)) break;
      btnUpdateTravel(&Pos.rec);
      btnRecPut(u);
    }
  }

  s_recording     = false;
  s_recDurationMs = now - s_recStartMs;

  s_recTravel = btnTravelSpan();

#if WEARM_DEBUG_SERIAL
  Serial.print(F("[btn] 结束录制：时长 "));
  Serial.print(s_recDurationMs);
  Serial.print(F(" ms，条目 "));
  Serial.print(s_recCount);
  Serial.print(F(" 条，末端位移 "));
  Serial.print(s_recTravel, 2);
  Serial.println(F("（坐标单位）"));
#endif

  bool ok = true;
  if (s_recOverflow) {
    ok = false;
#if WEARM_DEBUG_SERIAL
    Serial.print(F("[btn] 不合格：动作太长把 "));
    Serial.print((unsigned int) BTN_REC_ENTRIES);
    Serial.println(F(" 条缓冲写满了，尾部动作丢失"));
#endif
  } else if (s_recCount <= 0) {
    ok = false;
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 不合格：缓冲里没有任何动作"));
#endif
  } else if (s_recTravel < BTN_REC_MIN_TRAVEL) {
    ok = false;
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 不合格：位移太小，看不出明显移动"));
#endif
  }

  if (!ok) {
    s_recLen = 0;
    s_recCount = 0;
    s_hasRec = false;
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 本次录制已丢弃，上一次的录制数据也已被覆盖，请重录"));
#endif
    return PROTO_RES_REC_REJECTED;
  }

  s_recLen = s_recCount;
  s_hasRec = true;
#if WEARM_DEBUG_SERIAL
  Serial.print(F("[btn] 录制已保存："));
  Serial.print(s_recLen);
  Serial.println(F(" 条，按按键3 可播放"));
#endif
  return PROTO_RES_REC_SAVED;
}

/* ---------------- smooth ramp (pose before playback / homing) ---------------- */

static void btnStartRamp(const double *target, bool thenPlay) {
  for (int j = 0; j < 4; j++) {
    s_rampFrom[j] = *btnAngle(j);
    s_rampTo[j]   = target[j];
  }
  s_rampStartMs  = millis();
  s_rampThenPlay = thenPlay;
  s_state        = BS_RAMP;
}

static void btnRampTick(unsigned long now) {
  double t = (double) (now - s_rampStartMs) / (double) BTN_RAMP_MS;
  if (t > 1.0) t = 1.0;

  for (int j = 0; j < 4; j++) {
    *btnAngle(j) = s_rampFrom[j] + (s_rampTo[j] - s_rampFrom[j]) * t;
  }
  btnCommitAngles();

  if (t < 1.0) return;

  /* Land exactly on the target so that the interpolation cannot leave a 1e-12
   * sized residue behind. */
  for (int j = 0; j < 4; j++) {
    *btnAngle(j) = s_rampTo[j];
  }
  btnCommitAngles();

  if (s_rampThenPlay) {
    s_rampThenPlay = false;
    s_state        = BS_PLAY;
    s_playIdx      = 0;
    s_playDueMs    = millis();
    s_subIdx       = 0;
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 已摆到录制起点，开始播放"));
#endif
  } else {
    s_state = BS_IDLE;
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 已回到初始位置"));
#endif
  }
}

/* ---------------- playback ---------------- */

/* Load the record that is due now: sign-extend its four 8-bit fields into
 * s_subDelta[] (degrees) and clear the sub-step progress. */
static void btnPlayLoad(void) {
  const uint8_t *p = &s_buf[s_playIdx * BTN_REC_BYTES];
  uint32_t v = (uint32_t) p[0] | ((uint32_t) p[1] << 8) |
               ((uint32_t) p[2] << 16) | ((uint32_t) p[3] << 24);
  for (int j = 0; j < 4; j++) {
    int u = (int) ((v >> (8 * j)) & 0xFF);
    u = (u ^ BTN_REC_LIMIT) - BTN_REC_LIMIT;   /* sign-extend the 8-bit field */
    s_subDelta[j]   = btnFromUnits(u);
    s_subApplied[j] = 0.0;
  }
  s_subIdx = 0;
}

/* Move to the next sub-step of the current record.  All four joints of one
 * record belong together, so they move in the same sub-step and the pose is
 * re-clamped / re-solved once.  The target is an exact fraction k/N of the
 * recorded delta and the last sub-step (k == N) therefore lands exactly on it,
 * no matter how the fractions round in between. */
static void btnPlaySub(void) {
  s_subIdx++;
  for (int j = 0; j < 4; j++) {
    double target = s_subDelta[j] * (double) s_subIdx / (double) BTN_PLAY_SUBSTEPS;
    *btnAngle(j) += target - s_subApplied[j];
    s_subApplied[j] = target;
  }
  btnCommitAngles();
}

static void btnPlayTick(unsigned long now) {
  /* The sub-steps of all records fall on one uniform time base: a record of
   * BTN_TICK_MS is spread over BTN_PLAY_SUBSTEPS pieces of BTN_TICK_MS /
   * BTN_PLAY_SUBSTEPS, so the whole playback is just "apply the next sub-step
   * every subMs".  That keeps the total playback length exactly
   * s_recLen * BTN_TICK_MS, like the plain per-record version did.
   * btnPlayLoad() is idempotent (it re-reads the same record and clears the
   * progress), so calling it on every pass while waiting is harmless. */
  const unsigned long subMs = BTN_TICK_MS / (unsigned long) BTN_PLAY_SUBSTEPS;

  while (s_playIdx < s_recLen) {
    if (s_subIdx == 0) btnPlayLoad();

    unsigned long due = s_playDueMs + subMs;   /* the first sub-step is due one in */
    if ((long) (now - due) < 0L) break;        /* not yet: keep it for later */
    s_playDueMs = due;
    btnPlaySub();
    if (s_subIdx >= BTN_PLAY_SUBSTEPS) {
      s_subIdx = 0;
      s_playIdx++;
    }
  }

  if (s_playIdx >= s_recLen) {
    s_state = BS_IDLE;
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 播放结束"));
#endif
  }
}

/* ---------------- the four button actions ---------------- */

static int btnActionCycle(void) {
  if (s_recording) {
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 正在录制，按键1 忽略（先按按键2 结束录制）"));
#endif
    return PROTO_RES_BUSY;
  }
  if (s_state != BS_IDLE) {
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 正在播放/回中，按键1 忽略"));
#endif
    return PROTO_RES_BUSY;
  }

  int object = s_cycleIdx;
  int rc = pickPlaceStart(object);
  if (rc != 0) {
#if WEARM_DEBUG_SERIAL
    Serial.print(F("[btn] 取放启动失败 rc="));
    Serial.println(rc);
#endif
    return PROTO_RES_BUSY;
  }

#if WEARM_DEBUG_SERIAL
  Serial.print(F("[btn] 按键1 循环执行：夹取物体 "));
  Serial.print(btnObjectName(object));
  Serial.print(F("，下一次是 "));
  Serial.println(btnObjectName((object + 1) % PICK_OBJECT_COUNT));
#endif
  s_cycleIdx = (uint8_t)((object + 1) % PICK_OBJECT_COUNT);
  return PROTO_RES_PICK_STARTED;
}

static int btnActionRecord(void) {
  if (!s_recording) {
    if (pickPlaceIsBusy() || s_state != BS_IDLE) {
#if WEARM_DEBUG_SERIAL
      Serial.println(F("[btn] 机械臂正忙，现在不能开始录制"));
#endif
      return PROTO_RES_BUSY;
    }
    btnStartRecording();
    return PROTO_RES_REC_STARTED;
  }
  return btnStopRecording();
}

static int btnActionPlay(void) {
  /* 录制还开着就直接播放。旧行为是回 BUSY，实机上表现为"录完动作再输入 P 就
   * 一直 busy、录的动作执行不了"：录制只能由"结束录制"（按键2 / 串口 R）停掉，
   * 而用户按播放键 / 输入 P 时，他心里已经是"我录完了"。所以这里改成先把录制
   * 收尾（等价于按一次结束录制），合格就接着播放；不合格（没动 / 太长溢出）
   * 就如实返回 DISCARD，让他重录 —— 而不是含糊地回 BUSY。 */
  if (s_recording) {
    int stopped = btnStopRecording();
    if (stopped != PROTO_RES_REC_SAVED) {
      return stopped;   /* PROTO_RES_REC_REJECTED -> 串口回 DISCARD */
    }
  }
  if (pickPlaceIsBusy() || s_state != BS_IDLE) {
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 机械臂正忙，按键3 忽略"));
#endif
    return PROTO_RES_BUSY;
  }
  if (!s_hasRec) {
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 还没有录制数据，先按按键2 录一段"));
#endif
    return PROTO_RES_PLAY_NO_RECORD;
  }

  btnStartRamp(s_recStart, true);
#if WEARM_DEBUG_SERIAL
  Serial.print(F("[btn] 按键3 播放：先平滑摆到录制起点，再复现 "));
  Serial.print(s_recLen);
  Serial.println(F(" 条动作"));
#endif
  return PROTO_RES_PLAY_STARTED;
}

static int btnActionHome(void) {
  if (s_recording) {
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 正在录制，按键4 忽略（先按按键2 结束录制）"));
#endif
    return PROTO_RES_BUSY;
  }
  if (pickPlaceIsBusy() || s_state != BS_IDLE) {
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 正在播放/回中或取放中，按键4 忽略"));
#endif
    return PROTO_RES_BUSY;
  }

  /* angle4 先读出来：posGetHomeAngles() 只写 angle1..3，读回来的还是调用前的值。 */
  SER home;
  home.angle4 = Pos.ser.angle4;
  if (!posGetHomeAngles(&home)) {
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 回中失败：初始位姿反解不成功"));
#endif
    return PROTO_RES_BUSY;
  }

  /* SER 的 angle1..angle4 是连续的四个 double，&home.angle1 就是那四个目标角，
   * 直接交给 btnStartRamp()，省掉一次目标数组拷贝。 */
  btnStartRamp(&home.angle1, false);

#if WEARM_DEBUG_SERIAL
  Serial.println(F("[btn] 按键4 回中：平滑回到开机初始位姿"));
#endif
  return PROTO_RES_HOME_STARTED;
}

static int btnAction(int key) {
  /* While a drawing job runs (or is being taught) the four keys belong to
   * draw_control.cpp: 1=record 2=undo 3=cancel 4=start while teaching and
   * 1=pause 2=resume 3=cancel 4=nothing while drawing.  When idle
   * drawAcceptButton() returns false and the keys keep their own meaning. */
  if (drawAcceptButton(key)) {
    return drawHandleButton(key);
  }

  switch (key) {
    case BTN_KEY_CYCLE:  return btnActionCycle();
    case BTN_KEY_RECORD: return btnActionRecord();
    case BTN_KEY_PLAY:   return btnActionPlay();
    case BTN_KEY_HOME:   return btnActionHome();
    default:             return PROTO_RES_UNKNOWN;
  }
}

/* ---------------- key scanning ---------------- */

#if WEARM_BUTTON_PINS
/* Returns true when this scan confirmed one press edge (release never does). */
static bool btnEdge(int key) {
  bool raw = (digitalRead(BTN_PIN[key]) == LOW);
  unsigned long now = millis();

  if (raw == s_btnStable[key]) {
    s_btnArmed[key] = false;
    return false;
  }
  if (!s_btnArmed[key]) {
    s_btnArmed[key]    = true;
    s_btnChangeAt[key] = now;
    return false;
  }
  if ((now - s_btnChangeAt[key]) < BTN_DEBOUNCE_MS) return false;

  s_btnStable[key] = raw;
  s_btnArmed[key]  = false;
  return raw;
}
#endif /* WEARM_BUTTON_PINS */

/* ---------------- public interface ---------------- */

void buttonSetup(void) {
#if WEARM_BUTTON_PINS
  for (int k = 0; k < BTN_COUNT; k++) {
    pinMode(BTN_PIN[k], INPUT_PULLUP);
    s_btnStable[k]   = (digitalRead(BTN_PIN[k]) == LOW);
    s_btnArmed[k]    = false;
    s_btnChangeAt[k] = 0UL;
  }
#endif

  s_state        = BS_IDLE;
  s_recording    = false;
  s_hasRec       = false;
  s_recLen       = 0;
  s_recCount     = 0;
  s_recOverflow  = false;
  s_cycleIdx     = 0;
  s_rampThenPlay = false;

#if WEARM_DEBUG_SERIAL
  Serial.println(F("[btn] 按键初始化："));
#if WEARM_BUTTON_PINS
  Serial.println(F("[btn]   按键1 D2  循环执行（每次按顺序夹 A -> B -> C，串口 N）"));
  Serial.println(F("[btn]   按键2 D3  录制 开/关（有明显位移即可，串口 R）"));
  Serial.println(F("[btn]   按键3 D4  播放上一次录制的内容（串口 P）"));
  Serial.println(F("[btn]   按键4 D5  回中：回到开机初始位姿（串口 M 或 0）"));
#else
  Serial.println(F("[btn]   不读 D2~D5 物理按键（WEARM_BUTTON_PINS=0）："));
  Serial.println(F("[btn]     N 循环执行（夹 A -> B -> C）  R 录制开/关"));
  Serial.println(F("[btn]     P 播放上一次录制            M 或 0 回中"));
#endif
#endif
}

void buttonLoop(void) {
  unsigned long now = millis();

  if (s_recording)            btnRecTick(now);
  if (s_state == BS_RAMP)     btnRampTick(now);
  else if (s_state == BS_PLAY) btnPlayTick(now);

#if WEARM_BUTTON_PINS
  for (int k = 0; k < BTN_COUNT; k++) {
    if (!btnEdge(k)) continue;
    (void) btnAction(k);   /* the action itself reports and answers the serial port */
  }
#endif
}

int buttonHandleCommand(char c) {
  /* 【v1.1.0 容量压缩】这一处**不要**再改成"位折叠 + u <= 3"之类的紧凑写法。
   * 命令字符的实际差值是 M/N/P/R = 0/1/3/5（不是 0/1/2/3），折叠会把 'R' 整个丢掉、
   * 并把 'N'/'P'/'M' 各自错位到别的按键上（probe_button 会挂 32 项）。
   * 这 20 字节的收益不值得，保持显式 switch。 */
  switch (c) {
    case PROTO_CMD_BTN_CYCLE:    return btnAction(BTN_KEY_CYCLE);
    case PROTO_CMD_BTN_RECORD:   return btnAction(BTN_KEY_RECORD);
    case PROTO_CMD_BTN_PLAY:     return btnAction(BTN_KEY_PLAY);
    case PROTO_CMD_BTN_HOME:
    case PROTO_CMD_BTN_HOME_ALT: return btnAction(BTN_KEY_HOME);
    default:                     return PROTO_RES_UNKNOWN;
  }
}

bool buttonIsCommandChar(char c) {
  return (c == PROTO_CMD_BTN_CYCLE)
      || (c == PROTO_CMD_BTN_RECORD)
      || (c == PROTO_CMD_BTN_PLAY)
      || (c == PROTO_CMD_BTN_HOME)
      || (c == PROTO_CMD_BTN_HOME_ALT);
}

int buttonPickNext(void) {
  return s_cycleIdx;
}

bool buttonIsRecording(void) {
  return s_recording;
}

bool buttonPlaybackActive(void) {
  return (s_state == BS_PLAY) || (s_state == BS_RAMP && s_rampThenPlay);
}

int buttonRecordingEntries(void) {
  return s_recording ? s_recCount : s_recLen;
}

unsigned long buttonRecordingMs(void) {
  if (s_recording) return millis() - s_recStartMs;
  return s_recDurationMs;
}

double buttonRecordingTravel(void) {
  return s_recTravel;
}

bool buttonHasRecording(void) {
  return s_hasRec;
}

bool buttonControlBusy(void) {
  return s_recording || (s_state != BS_IDLE);
}

bool buttonControlLocked(void) {
  return (s_state != BS_IDLE);
}

const char *buttonName(int key) {
  switch (key) {
    case BTN_KEY_CYCLE:  return "按键1 循环执行";
    case BTN_KEY_RECORD: return "按键2 录制";
    case BTN_KEY_PLAY:   return "按键3 播放";
    case BTN_KEY_HOME:   return "按键4 回中";
    default:             return "未知按键";
  }
}

const char *buttonStateName(void) {
  if (s_recording)          return "录制中";
  if (s_state == BS_PLAY)   return "播放中";
  if (s_state == BS_RAMP)   return s_rampThenPlay ? "预摆中" : "回中中";
  return "空闲";
}

#endif /* WEARM_ENABLE_BUTTONS */
