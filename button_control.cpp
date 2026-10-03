/*
 * button_control.cpp -- implementation of the four on-board buttons.
 *
 * The public interface is documented in button_control.h.  Implementation notes:
 *
 *   1) Hardware: buttons 1~4 are wired to D2~D5, INPUT_PULLUP, a press reads LOW.
 *      Software debounce is BTN_DEBOUNCE_MS and only the press edge acts.
 *
 *   2) Recording format (fixed 384-byte buffer, no dynamic allocation):
 *      one packed record per BTN_TICK_MS (100 ms) sampling tick holds the angle
 *      delta of all four joints, 6 signed bits each (unit = 0.5 deg):
 *         bits  0.. 5  joint 0 (base)      delta in recording units
 *         bits  6..11  joint 1 (upper arm)
 *         bits 12..17  joint 2 (forearm)
 *         bits 18..23  joint 3 (end effector)
 *      BTN_REC_ENTRIES = 128 records x 3 bytes = 384 bytes = 128 ticks = 12.8 s,
 *      which is still more than BTN_REC_MIN_MS (10 s).  Every tick gets exactly
 *      one record, moving or not, so the playback time axis is the exact
 *      recording timeline; an all-zero record simply means "no motion".
 *      A joint delta wider than the 6-bit field (+-32 units = +-16 deg per tick)
 *      is clamped, but s_snap[] follows the *emitted* position rather than the
 *      real one, so the remainder is carried into the next ticks: the recorded
 *      total displacement stays exact and only a very fast move arrives a tick
 *      or two later during playback.
 *
 *   3) Save validation (both must hold, otherwise the recording is dropped):
 *         duration > BTN_REC_MIN_MS (10 s, strictly greater)
 *         end-effector travel on x/y/z >= BTN_REC_MIN_TRAVEL (10.0)
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
 * on this board (D0/D1 serial, D6~D9 servos, D13 led, A0~A3 joystick). */
#define BTN_PIN_CYCLE   2
#define BTN_PIN_RECORD  3
#define BTN_PIN_PLAY    4
#define BTN_PIN_HOME    5

#define BTN_DEBOUNCE_MS     25UL      /* button debounce time */
#define BTN_TICK_MS         100UL     /* recording sampling period */
#define BTN_RAMP_MS         1500UL    /* smooth ramp before playback / homing */
#define BTN_REC_MIN_MS      10000UL   /* minimum recording time (must be > 10 s) */
#define BTN_REC_MIN_TRAVEL  10.0      /* minimum end-effector travel */
#define BTN_REC_ENTRIES     128       /* record limit (128 * 3 = 384 bytes SRAM) */
#define BTN_REC_BYTES       3         /* packed size of one 100 ms record */
#define BTN_REC_LIMIT       32        /* 6-bit signed field range: -32..+31 units */
#define BTN_ANGLE_UNIT      2.0       /* 1 deg = 2 recording units (unit = 0.5 deg) */

static const uint8_t BTN_PIN[BTN_COUNT] = {
  BTN_PIN_CYCLE, BTN_PIN_RECORD, BTN_PIN_PLAY, BTN_PIN_HOME
};

/* ---------------- recording buffer ---------------- */

/* One packed record per 100 ms tick: four 6-bit signed joint deltas. */
static uint8_t s_buf[BTN_REC_ENTRIES * BTN_REC_BYTES];

/* ---------------- runtime state ---------------- */

enum { BS_IDLE = 0, BS_RAMP, BS_PLAY };

static uint8_t s_state       = BS_IDLE;  /* playback / homing state machine */
static uint8_t s_cycleIdx    = 0;        /* next object picked by button 1 */
static bool    s_recording   = false;    /* recording in progress */
static bool    s_rampThenPlay = false;   /* after the ramp: play (true) or stop (false) */

/* recording */
static uint8_t       s_recCount     = 0;     /* records written by the current recording */
static uint8_t       s_recLen       = 0;     /* records of the saved recording */
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
static uint8_t       s_playIdx   = 0;
static unsigned long s_playDueMs = 0;

/* button debounce */
static bool          s_btnStable[BTN_COUNT] = {false, false, false, false};
static bool          s_btnArmed[BTN_COUNT]  = {false, false, false, false};
static unsigned long s_btnChangeAt[BTN_COUNT] = {0UL, 0UL, 0UL, 0UL};

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
  (void) clampServoAngles(&tmp);
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

/* Pack one record: four 6-bit signed joint deltas into BTN_REC_BYTES bytes. */
static void btnRecPut(const int *u) {
  if (s_recCount >= BTN_REC_ENTRIES) {
    s_recOverflow = true;
    return;
  }
  uint8_t *p = &s_buf[s_recCount * BTN_REC_BYTES];
  uint32_t v = 0;
  for (int j = 0; j < 4; j++) {
    v |= ((uint32_t) (u[j] & 0x3F)) << (6 * j);
  }
  p[0] = (uint8_t) v;
  p[1] = (uint8_t) (v >> 8);
  p[2] = (uint8_t) (v >> 16);
  s_recCount++;
}

/* Collect what moved during one tick: the delta of every joint since the angle
 * the recording emitted last, clamped to the 6-bit field range (-32..+31, so the
 * positive limit is BTN_REC_LIMIT - 1: +32 would alias to -32).  s_snap[] is
 * advanced by the clamped amount (not by the real angle), so anything that does
 * not fit into this record is carried into the next one.  Returns true if
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
  Serial.println(F("[btn] 开始录制：请用摇杆操控机械臂（时长需 >10 秒，且要有明显位移）"));
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
    Serial.println(F("[btn] 录制缓冲已满（128 条），自动结束录制"));
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
    /* Flush the motion of the unfinished tick as one last record.  Without it the
     * playback would stop one tick early, which is clearly visible in the fast
     * speed preset (a whole tick of travel at the end). */
    int u[4];
    if (btnRecCollect(u)) {
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
    Serial.println(F("[btn] 不合格：动作太长把 128 条缓冲写满了，尾部动作丢失"));
#endif
  } else if (s_recCount <= 0) {
    ok = false;
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 不合格：缓冲里没有任何动作"));
#endif
  } else if (s_recDurationMs <= BTN_REC_MIN_MS) {
    ok = false;
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 不合格：录制时长必须大于 10 秒"));
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

/* Apply one whole 100 ms record: all four joint deltas belong to the same tick,
 * so they land together and the pose is re-clamped / re-solved once. */
static void btnPlayRecord(void) {
  const uint8_t *p = &s_buf[s_playIdx * BTN_REC_BYTES];
  uint32_t v = (uint32_t) p[0] | ((uint32_t) p[1] << 8) | ((uint32_t) p[2] << 16);
  for (int j = 0; j < 4; j++) {
    /* Sign-extend the 6-bit field, then move that joint. */
    int u = (int) ((v >> (6 * j)) & 0x3F);
    u = (u ^ BTN_REC_LIMIT) - BTN_REC_LIMIT;
    if (u != 0) *btnAngle(j) += btnFromUnits(u);
  }
  btnCommitAngles();
}

static void btnPlayTick(unsigned long now) {
  while (s_playIdx < s_recLen) {
    unsigned long due = s_playDueMs + BTN_TICK_MS;   /* record 0 is due one tick in */
    if ((long) (now - due) < 0L) break;              /* not yet: keep it for later */
    s_playDueMs = due;
    btnPlayRecord();
    s_playIdx++;
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
  if (s_recording) {
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 正在录制，按键3 忽略（先按按键2 结束录制）"));
#endif
    return PROTO_RES_BUSY;
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

/* ---------------- public interface ---------------- */

void buttonSetup(void) {
  for (int k = 0; k < BTN_COUNT; k++) {
    pinMode(BTN_PIN[k], INPUT_PULLUP);
    s_btnStable[k]   = (digitalRead(BTN_PIN[k]) == LOW);
    s_btnArmed[k]    = false;
    s_btnChangeAt[k] = 0UL;
  }

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
  Serial.println(F("[btn]   按键1 D2  循环执行（每次按顺序夹 A -> B -> C，串口 N）"));
  Serial.println(F("[btn]   按键2 D3  录制 开/关（时长需 >10 秒且有明显位移，串口 R）"));
  Serial.println(F("[btn]   按键3 D4  播放上一次录制的内容（串口 P）"));
  Serial.println(F("[btn]   按键4 D5  回中：回到开机初始位姿（串口 M 或 0）"));
#endif
}

void buttonLoop(void) {
  unsigned long now = millis();

  if (s_recording)            btnRecTick(now);
  if (s_state == BS_RAMP)     btnRampTick(now);
  else if (s_state == BS_PLAY) btnPlayTick(now);

  for (int k = 0; k < BTN_COUNT; k++) {
    if (!btnEdge(k)) continue;
    (void) btnAction(k);   /* the action itself reports and answers the serial port */
  }
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
