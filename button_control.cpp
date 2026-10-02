/*
 * button_control.cpp -- 四个按键功能的全部实现（这是本版唯一新增的"按键"实现文件）
 *
 * 接口与使用说明见 button_control.h。这里只补充实现层面的约定：
 *
 *   1) 硬件：按键 1~4 分别接 D2~D5，一律 INPUT_PULLUP，按下读到 LOW。
 *      软件消抖 BTN_DEBOUNCE_MS，并且只认"按下沿"，松手不触发。
 *
 *   2) 录制格式（1024 字节的定长缓冲，不动态分配）：
 *        struct btnRecEntry { uint8_t code; int8_t arg; };
 *        code  0x00~0x03 -> 对应关节(基座/上臂/下臂/末端)，arg 是"相对上一条事件的
 *                          角度增量"，单位 0.5 度（BTN_ANGLE_UNIT = 2.0 单位/度）
 *        code  0xFF      -> 等待，arg 是"等待多少个 tick"（1~127），这段时间没动
 *        code  bit 0x80  -> 本条是"新一个采样周期"的第一条条目：回放时先把时间轴
 *                          推进 1 个 tick 再应用它。
 *      "新周期标志"是为了省缓冲：一个周期内有动作时，时间推进被塞进该周期第一条
 *      增量条目里，而不是每个周期都写一条 WAIT(1)。于是每个周期最多只花
 *      "动了几个关节"条，而不是"1 + 动了几个关节"条。
 *      容量（BTN_TICK_MS = 100ms，512 条缓冲）：
 *        单关节连续推   -> 1 条/周期 -> 512 周期 -> 51.2 秒
 *        双关节连续推   -> 2 条/周期 -> 256 周期 -> 25.6 秒
 *        三关节连续推   -> 3 条/周期 -> 170 周期 -> 17.0 秒
 *        四关节连续推   -> 4 条/周期 -> 128 周期 -> 12.8 秒（最坏情况，仍 > 10 秒）
 *        几乎不动       -> 每条 WAIT 最多 127 周期 = 12.7 秒，最省
 *      （历史教训：早先每周期都写一条 WAIT(1)，四关节连续推只能录 4.1 秒，
 *        达不到"大于 10 秒"的要求就必然被判废。）
 *
 *   3) 保存校验（两条缺一不可，都不满足就丢弃并串口报明原因）：
 *        时长 > BTN_REC_MIN_MS（10 秒，严格大于）
 *        末端在 x/y/z 上的最大位移 >= BTN_REC_MIN_TRAVEL（10.0）
 *      注意：新录制一开始就复用了同一块缓冲，所以"录了一半觉得不好、又结束了
 *      一次不达标的录制"会把上一次的录制数据一起冲掉。这是为了在 Uno 的 2KB
 *      SRAM 里做出尽可能长的录制时间而做的取舍，串口在开始录制时会明确提示。
 *
 *   4) 播放/回中前都先做一次线性插值（BTN_RAMP_MS）把机械臂平滑摆到目标位姿，
 *      避免从当前姿态"跳"过去。插值在关节空间做，只改角度、不做反解，
 *      因此不会出现中途不可达的问题。
 *
 *   5) 想让位关系（详见 button_control.h 末尾）：
 *      录制期间摇杆必须可用；播放/回中期间摇杆必须让位；
 *      串口动作指令在"录制/播放/回中"期间一律被 buttonHandleCommand() 拒掉，
 *      但 N/R/P/M 这四个字符本身要在 serial_protocol.cpp 的忙守卫里豁免，
 *      否则录制中想发 R 结束录制都不会被受理。
 */
#include "Arduino.h"
#include "constant_and_positions.h"
#include "pick_place.h"
#include "protocol_constants.h"
#include "button_control.h"

/* 与 constant_and_positions.cpp / serial_protocol.cpp 保持一致：1 = 打调试日志 */
#define WEARM_DEBUG_SERIAL 1

/* ---------------- 可自定义的参数（改这里就行，不用动下面的逻辑） ---------------- */

/* 按键引脚：D2/D3/D4/D5 是这块板子上唯一空闲好接的四个数字脚
 * （D0/D1 串口、D6~D9 舵机、D13 指示灯、A0~A3 摇杆）。 */
#define BTN_PIN_CYCLE   2
#define BTN_PIN_RECORD  3
#define BTN_PIN_PLAY    4
#define BTN_PIN_HOME    5

#define BTN_DEBOUNCE_MS     25UL      /* 按键消抖时间 */
#define BTN_TICK_MS         100UL     /* 录制采样周期：100ms 是"最坏情况仍能录满 10 秒"的取值
                                       * （512 条 / 最多 4 条每周期 = 128 周期 = 12.8 秒） */
#define BTN_RAMP_MS         1500UL    /* 播放/回中前的平滑插值时长 */
#define BTN_REC_MIN_MS      10000UL   /* 录制时长下限（要求"大于"10 秒） */
#define BTN_REC_MIN_TRAVEL  10.0      /* 末端最小位移，小于它算"没有明显位移" */
#define BTN_REC_ENTRIES     512       /* 录制条目上限（512 * 2 = 1024 字节 SRAM） */
#define BTN_ANGLE_UNIT      2.0       /* 1 度 = 2 个记录单位（即单位 = 0.5 度） */
#define BTN_WAIT_CODE       0xFF      /* 条目 type：等待（这段时间没有动作） */
#define BTN_WAIT_MAX        127       /* 单条等待最多表达 127 个 tick */
#define BTN_TICK_FLAG       0x80      /* 条目 code 的 bit7：本周期的时间推进放在这条里 */

static const uint8_t BTN_PIN[BTN_COUNT] = {
  BTN_PIN_CYCLE, BTN_PIN_RECORD, BTN_PIN_PLAY, BTN_PIN_HOME
};

/* ---------------- 录制缓冲 ---------------- */

struct btnRecEntry {
  uint8_t code;   /* 0~3 = 关节下标，0xFF = 等待 */
  int8_t  arg;    /* 关节：角度增量（0.5 度为单位）；等待：tick 数 */
};

static struct btnRecEntry s_buf[BTN_REC_ENTRIES];

/* ---------------- 运行状态 ---------------- */

enum { BS_IDLE = 0, BS_RAMP, BS_PLAY };

static int  s_state        = BS_IDLE;   /* 播放/回中的状态机 */
static int  s_cycleIdx     = 0;         /* 按键1 下一次夹哪个物体 */
static bool s_recording    = false;     /* 是否正在录制 */
static bool s_rampThenPlay = false;     /* 插值结束后是进入播放(true)还是结束(false=回中) */

/* 录制相关 */
static int           s_recCount     = 0;   /* 本次录制已写入的条目数 */
static int           s_recLen       = 0;   /* 已保存的条目数 */
static bool          s_hasRec       = false; /* 是否有一份可播放的录制 */
static bool          s_recOverflow  = false; /* 缓冲区是否被写满过 */
static unsigned long s_recStartMs   = 0;
static unsigned long s_recDurationMs = 0;
static unsigned long s_tickNow      = 0;   /* 当前采样到的 tick 序号 */
static unsigned long s_lastEventTick = 0;  /* 上一次写出条目的 tick 序号 */
static double        s_snap[4]      = {0.0, 0.0, 0.0, 0.0};  /* 上一条事件时的真实角度 */
static double        s_recStart[4]  = {0.0, 0.0, 0.0, 0.0};  /* 录制起点的四个角度 */
static double        s_recTravel    = 0.0;                   /* 末端最大位移 */
static double        s_minX = 0.0, s_maxX = 0.0;
static double        s_minY = 0.0, s_maxY = 0.0;
static double        s_minZ = 0.0, s_maxZ = 0.0;

/* 插值相关 */
static double        s_rampFrom[4] = {0.0, 0.0, 0.0, 0.0};
static double        s_rampTo[4]   = {0.0, 0.0, 0.0, 0.0};
static unsigned long s_rampStartMs = 0;

/* 播放相关 */
static int           s_playIdx   = 0;
static unsigned long s_playDueMs = 0;

/* 按键消抖 */
static bool          s_btnStable[BTN_COUNT] = {false, false, false, false};
static bool          s_btnArmed[BTN_COUNT]  = {false, false, false, false};
static unsigned long s_btnChangeAt[BTN_COUNT] = {0UL, 0UL, 0UL, 0UL};

/* ---------------- 小工具 ---------------- */

static double btnGetAngle(int joint) {
  switch (joint) {
    case 0:  return Pos.ser.angle1;
    case 1:  return Pos.ser.angle2;
    case 2:  return Pos.ser.angle3;
    default: return Pos.ser.angle4;
  }
}

static void btnSetAngle(int joint, double deg) {
  switch (joint) {
    case 0:  Pos.ser.angle1 = deg; break;
    case 1:  Pos.ser.angle2 = deg; break;
    case 2:  Pos.ser.angle3 = deg; break;
    default: Pos.ser.angle4 = deg; break;
  }
}

/* 改完角度统一做一次限位 + 正解，保证 Pos.ser 与 Pos.rec 始终自洽。
 * 录制本来就是在本工程行程内动的，正常情况下 clampServoAngles() 不会改动任何值；
 * 这里只是防守，避免越界角度残留在 Pos 里。 */
static void btnCommitAngles(void) {
  SER tmp = Pos.ser;
  (void) clampServoAngles(&tmp);
  Pos.ser = tmp;
  (void) recFromServo(&Pos.rec, &Pos.ser);
}

/* 角度增量 -> 记录单位（0.5 度，四舍五入），并夹在 int8 能表达的范围里 */
static int btnToUnits(double deltaDeg) {
  double u = deltaDeg * BTN_ANGLE_UNIT;
  u = (u >= 0.0) ? (u + 0.5) : (u - 0.5);
  if (u >  127.0) u =  127.0;
  if (u < -127.0) u = -127.0;
  return (int) u;
}

static double btnFromUnits(int units) {
  return (double) units / BTN_ANGLE_UNIT;
}

/* 录制的位移统计：只看末端在 x/y/z 上的最大变化量 */
static void btnUpdateTravel(const REC *rec) {
  if (rec->x < s_minX) s_minX = rec->x;
  if (rec->x > s_maxX) s_maxX = rec->x;
  if (rec->y < s_minY) s_minY = rec->y;
  if (rec->y > s_maxY) s_maxY = rec->y;
  if (rec->z < s_minZ) s_minZ = rec->z;
  if (rec->z > s_maxZ) s_maxZ = rec->z;
}

static const char *btnObjectName(int object) {
  static const char *const name[PICK_OBJECT_COUNT] = { "A", "B", "C" };
  if (object < 0 || object >= PICK_OBJECT_COUNT) return "?";
  return name[object];
}

/* ---------------- 录制 ---------------- */

static void btnEmit(uint8_t code, int8_t arg) {
  if (s_recCount >= BTN_REC_ENTRIES) {
    s_recOverflow = true;
    return;
  }
  s_buf[s_recCount].code = code;
  s_buf[s_recCount].arg  = arg;
  s_recCount++;
}

static int  btnStopRecording(void);   /* 前向声明：缓冲写满时要自动收尾 */

static void btnStartRecording(void) {
  s_recCount    = 0;
  s_recOverflow = false;
  s_recording   = true;
  s_recStartMs  = millis();
  s_tickNow     = 0;
  s_lastEventTick = 0;
  s_recDurationMs = 0;
  s_recTravel   = 0.0;
  for (int j = 0; j < 4; j++) {
    s_snap[j]     = btnGetAngle(j);
    s_recStart[j] = s_snap[j];
  }
  s_minX = s_maxX = Pos.rec.x;
  s_minY = s_maxY = Pos.rec.y;
  s_minZ = s_maxZ = Pos.rec.z;

#if WEARM_DEBUG_SERIAL
  Serial.println(F("[btn] 开始录制：请用摇杆操控机械臂（时长需 >10 秒，且要有明显位移）"));
  Serial.println(F("[btn] 提示：本次录制会覆盖上一次的录制数据"));
#endif
}

/* 每个采样周期跑一次：把这一小段时间内的摇杆动作合并成条目写进缓冲 */
static void btnRecTick(unsigned long now) {
  unsigned long tick = (now - s_recStartMs) / BTN_TICK_MS;
  if (tick <= s_tickNow) return;
  s_tickNow = tick;

  btnUpdateTravel(&Pos.rec);

  int  d[4];
  bool moved = false;
  for (int j = 0; j < 4; j++) {
    d[j] = btnToUnits(btnGetAngle(j) - s_snap[j]);
    if (d[j] != 0) moved = true;
  }
  if (!moved) return;

  /* 距离上一条事件的周期数 gap：
   *   gap >= 2 -> 用 WAIT 条目把时间轴推过去（一条最多 127 个周期，可能拆几条）
   *   gap == 1 -> 不写 WAIT，改为把"推进 1 个周期"塞进本周期第一条增量条目的 bit7
   *   gap == 0 -> 同一个周期里的动作，时间不用再推进
   * 于是"每个周期都在动"时只花"动了几个关节"条，不会再多花一条 WAIT。 */
  unsigned long gap = s_tickNow - s_lastEventTick;
  while (gap >= 2UL && !s_recOverflow) {
    unsigned long chunk = (gap > (unsigned long) BTN_WAIT_MAX) ? (unsigned long) BTN_WAIT_MAX : gap;
    btnEmit(BTN_WAIT_CODE, (int8_t) chunk);
    gap -= chunk;
  }
  bool markTick = (gap == 1UL);

  for (int j = 0; j < 4; j++) {
    int u = d[j];
    while (u != 0 && !s_recOverflow) {
      int8_t step;
      if (u > 127)       step = 127;
      else if (u < -127) step = -127;
      else               step = (int8_t) u;
      uint8_t code = (uint8_t) j;
      if (markTick) {          /* 本周期的时间推进只挂在第一条增量上 */
        code |= (uint8_t) BTN_TICK_FLAG;
        markTick = false;
      }
      btnEmit(code, step);
      u -= (int) step;
    }
    s_snap[j] = btnGetAngle(j);
  }
  s_lastEventTick = s_tickNow;

  if (s_recOverflow) {
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 录制缓冲已满（512 条），自动结束录制"));
#endif
    (void) btnStopRecording();
  }
}

/* 收尾：把"最后一个整周期 → 按下结束键"之间那不到一个周期的动作补成条目。
 * 不做这一步的话，回放终点会停在最后一个整周期上：快速档下每个周期能走 10 度，
 * 差距肉眼可见（实测四关节同时动时末态差正好一个周期的位移）。 */
static void btnRecFlushTail(unsigned long now) {
  if (s_recOverflow) return;

  int  d[4];
  bool moved = false;
  for (int j = 0; j < 4; j++) {
    d[j] = btnToUnits(btnGetAngle(j) - s_snap[j]);
    if (d[j] != 0) moved = true;
  }
  if (!moved) return;                 /* 松手前本来就没动，时间轴不必再推进 */

  btnUpdateTravel(&Pos.rec);

  /* 已经跨进新的周期 -> 收尾动作挂在下一个周期上（回放时同样多等一个周期）；
   * 还在同一个周期内 -> 不挂标记，与本周期的条目同时落下。 */
  unsigned long tick = (now - s_recStartMs) / BTN_TICK_MS;
  bool markTick = (tick > s_lastEventTick);

  for (int j = 0; j < 4; j++) {
    int u = d[j];
    while (u != 0 && !s_recOverflow) {
      int8_t step;
      if (u > 127)       step = 127;
      else if (u < -127) step = -127;
      else               step = (int8_t) u;
      uint8_t code = (uint8_t) j;
      if (markTick) {
        code |= (uint8_t) BTN_TICK_FLAG;
        markTick = false;
      }
      btnEmit(code, step);
      u -= (int) step;
    }
    s_snap[j] = btnGetAngle(j);
  }
  s_lastEventTick = tick;
}

/* 结束录制并判断是否保存；返回值是给串口回话用的 PROTO_RES_*。
 * 不达标（太短 / 没位移 / 缓冲写满）时，本次数据丢弃，且上一次的录制也已经
 * 被这次录制覆盖掉了，所以一并失效。 */
static int btnStopRecording(void) {
  unsigned long now = millis();
  btnRecFlushTail(now);               /* 先把最后不到一个周期的动作补上 */
  s_recording = false;
  s_recDurationMs = now - s_recStartMs;

  double dx = s_maxX - s_minX;
  double dy = s_maxY - s_minY;
  double dz = s_maxZ - s_minZ;
  s_recTravel = dx;
  if (dy > s_recTravel) s_recTravel = dy;
  if (dz > s_recTravel) s_recTravel = dz;

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
    Serial.println(F("[btn] 不合格：动作太长把 512 条缓冲写满了，尾部动作丢失"));
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

/* ---------------- 平滑插值（播放前定位 / 回中） ---------------- */

static void btnStartRamp(const double *target, bool thenPlay) {
  for (int j = 0; j < 4; j++) {
    s_rampFrom[j] = btnGetAngle(j);
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
    btnSetAngle(j, s_rampFrom[j] + (s_rampTo[j] - s_rampFrom[j]) * t);
  }
  btnCommitAngles();

  if (t < 1.0) return;

  /* 收尾精确落位，避免插值公式留下 1e-12 级别的残差 */
  for (int j = 0; j < 4; j++) {
    btnSetAngle(j, s_rampTo[j]);
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

/* ---------------- 播放 ---------------- */

static void btnApplyDelta(int joint, int units) {
  btnSetAngle(joint, btnGetAngle(joint) + btnFromUnits(units));
  btnCommitAngles();
}

static void btnPlayTick(unsigned long now) {
  int guard = 0;
  while (s_playIdx < s_recLen) {
    struct btnRecEntry e = s_buf[s_playIdx];

    if (e.code == BTN_WAIT_CODE) {
      /* 等待条目：先把时间轴推过去；还没到点就原地不动，且不消费这条 */
      unsigned long span = (unsigned long) ((uint8_t) e.arg) * BTN_TICK_MS;
      if ((long) (now - (s_playDueMs + span)) < 0L) break;
      s_playDueMs += span;
    } else {
      /* 增量条目：带 0x80 的那条先把时间轴推进 1 个采样周期（这是一个周期的时间成本） */
      unsigned long due = (e.code & BTN_TICK_FLAG) ? (s_playDueMs + BTN_TICK_MS) : s_playDueMs;
      if ((long) (now - due) < 0L) break;   /* 没到点：不消费、也不改 s_playDueMs */
      s_playDueMs = due;
      btnApplyDelta((int) (e.code & 0x03), (int) e.arg);
    }

    s_playIdx++;
    guard++;
    if (guard > BTN_REC_ENTRIES) break;   /* 极端情况下也不让 loop 卡死 */
  }

  if (s_playIdx >= s_recLen) {
    s_state = BS_IDLE;
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 播放结束"));
#endif
  }
}

/* ---------------- 四个按键的动作 ---------------- */

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
  s_cycleIdx = (object + 1) % PICK_OBJECT_COUNT;
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
  if (!s_hasRec || s_recLen <= 0) {
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

  SER home;
  home.angle1 = Pos.ser.angle1;
  home.angle2 = Pos.ser.angle2;
  home.angle3 = Pos.ser.angle3;
  home.angle4 = Pos.ser.angle4;
  if (!posGetHomeAngles(&home)) {
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[btn] 回中失败：初始位姿反解不成功"));
#endif
    return PROTO_RES_BUSY;
  }

  double target[4];
  target[0] = home.angle1;
  target[1] = home.angle2;
  target[2] = home.angle3;
  target[3] = Pos.ser.angle4;   /* 末端开合保持现状（posGetHomeAngles 不动 angle4） */
  btnStartRamp(target, false);

#if WEARM_DEBUG_SERIAL
  Serial.println(F("[btn] 按键4 回中：平滑回到开机初始位姿"));
#endif
  return PROTO_RES_HOME_STARTED;
}

static int btnAction(int key) {
  switch (key) {
    case BTN_KEY_CYCLE:  return btnActionCycle();
    case BTN_KEY_RECORD: return btnActionRecord();
    case BTN_KEY_PLAY:   return btnActionPlay();
    case BTN_KEY_HOME:   return btnActionHome();
    default:             return PROTO_RES_UNKNOWN;
  }
}

/* ---------------- 按键扫描 ---------------- */

/* 返回 true 表示"这一次扫描确认了一次按下沿"（松手不返回 true） */
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

/* ---------------- 对外接口 ---------------- */

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
    (void) btnAction(k);   /* 动作本身负责打日志与串口回话 */
  }
}

int buttonHandleCommand(char c) {
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
