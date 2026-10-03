/*
// serial_protocol.cpp
// Serial command protocol: fixed commands + multi-servo sync angles (x/y/z).
// Also hosts the start entry of the A/B/C pick-and-place sequences (while a
// sequence runs this layer holds back every other motion command), the serial
// twins N/R/P/M of the four physical buttons (implementation lives in
// button_control.cpp) and the drawing commands F/D/G/E/Q/U/W plus the paper
// calibration commands p/n/o (implementation lives in draw_control.cpp).
// Handles character input, line buffering, command parsing and servo writes.
*/

#include "Arduino.h"
#include "constant_and_positions.h"
#include "pick_place.h"
#include "protocol_constants.h"
#include "serial_protocol.h"
#include "button_control.h"
#include "draw_control.h"
#include "weArm_config.h"

/* Verbose per-step traces (off by default). Leaving them off is what keeps the
 * whole Arduino print/float formatting layer out of the image. */
#ifndef WEARM_DEBUG_SERIAL
#define WEARM_DEBUG_SERIAL 0
#endif

#if !WEARM_DEBUG_SERIAL
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(x, y)
#else
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
#define DEBUG_PRINTF(x, y) Serial.print(x, y)
#endif

/* ---------- line buffer and state ---------- */
static char s_line[PROTO_LINE_BUF_SIZE];
static int  s_len = 0;
static bool s_pending = false;          /* characters buffered but not dispatched yet */
static bool s_dropUntilEol = false;     /* this line is too long, drop up to the EOL */
static unsigned long s_lastCharMs = 0;

/* ---------- helper declarations ---------- */
/* No noinline here on purpose: measured on the real AVR build, the attribute is
 * a no-op. serialProtocolLoop() reaches this from three places, but gcc keeps a
 * single out-of-line copy anyway, and the whole dispatcher only exists once
 * because protoHandleLine() has no caller outside this file. */
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

/* Skip blanks (space / tab) and return the first significant character.
 * Out of line on purpose: a dozen call sites share this one copy. Measured:
 * forcing always_inline here costs +40 bytes, the call is cheaper than the
 * expanded loop. */
static const char *protoSkipBlanks(const char *p)
{
  while (*p == ' ' || *p == '\t') {
    p++;
  }
  return p;
}

/* Skip blanks in place */
#define PROTO_SKIP_BLANKS(p) do { (p) = protoSkipBlanks(p); } while (0)

/* The PC self-check mock has no pgmspace.h; the AVR core defines both of these. */
#ifndef PSTR
#define PSTR(s) (s)
#endif
#ifndef pgm_read_byte
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#endif
#ifndef PROGMEM
#define PROGMEM
#endif

/* ========== response layer (WEARM_SERIAL_RESPONSES) ========== */
#if WEARM_SERIAL_RESPONSES
/* One byte out. Print::print(char) forwards straight to
 * HardwareSerial::write(uint8_t), so none of the Arduino number formatting
 * code is reachable from here - that is what keeps this layer at a few hundred
 * bytes instead of the ~9.5 KB that Serial.print(double) would drag in. */
static void protoPut(char c)
{
  Serial.print(c);
}

/* Gripper echo: angle4 is clamped into servoLimit f, i.e. 0..999, so a plain
 * three digit printer is enough. */
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

/* Walk <text> straight out of flash and send it; every reply ends with a
 * newline and a '#' becomes the clamped gripper angle in decimal. Kept out of
 * line on purpose: with -flto gcc otherwise clones this loop into every reply
 * site, where the per site argument setup costs more than the call. */
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

/* One flash copy per reply text. Naming the arrays keeps gcc from emitting a
 * separate copy (plus alignment padding) for every call site. */
static const char s_rOk[] PROGMEM = "OK";
static const char s_rErr[] PROGMEM = "ERR";
static const char s_rRej[] PROGMEM = "REJECTED";
static const char s_rOkN[] PROGMEM = "OK #";
static const char s_rBoot[] PROGMEM = "CMD OSHL 123 kK ABC NRPM0 FDGEQUW pno ANG x,y,z ! !P !B !D";

#define R_OK()   protoReply(s_rOk)
#define R_ERR()  protoReply(s_rErr)
#define R_REJ()  protoReply(s_rRej)
#define R_OKN()  protoReply(s_rOkN)
#define R_BOOT() protoReply(s_rBoot)
#define R_FEATURE() do { protoPut('P'); protoPut('='); protoPut((s_runtimeFeatures & PROTO_FEATURE_PICK) ? '1' : '0'); protoPut(' '); protoPut('B'); protoPut('='); protoPut((s_runtimeFeatures & PROTO_FEATURE_BUTTON) ? '1' : '0'); protoPut(' '); protoPut('D'); protoPut('='); protoPut((s_runtimeFeatures & PROTO_FEATURE_DRAW) ? '1' : '0'); protoPut(10); } while (0)
#else
#define R_OK()   do { } while (0)
#define R_ERR()  do { } while (0)
#define R_REJ()  do { } while (0)
#define R_OKN()  do { } while (0)
#define R_BOOT() do { } while (0)
#define R_FEATURE() do { } while (0)
#endif

/* ========== command class bitmaps ========== */
/* Index i = cmd - '0' spans '0'(0x30) .. 'p'(0x70), which covers every command
 * character; anything else falls off the end and is rejected.
 * s_liveBits: the command is still accepted while a module is busy.
 * s_fastBits: the command runs as soon as its single character arrives, i.e.
 * without waiting for the end of the line. */
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

/* ========== main loop interface ========== */
/* Called once per loop(): collect the characters that arrived into one line and run it */
void serialProtocolLoop(void)
{
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;

    /* remember when the last character arrived */
    s_lastCharMs = millis();

    /* over-long line: stay in the dropping state until the EOL */
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
      continue;  /* drop characters up to the EOL */
    }

    /* line buffer overflow */
    if (s_len >= PROTO_LINE_BUF_SIZE - 1) {
      s_dropUntilEol = true;
      s_len = 0;
      s_pending = false;
      continue;
    }

    /* store the character */
    s_line[s_len++] = (char)c;
    s_pending = true;

    /* single character fixed commands run immediately */
    if (s_len == 1 && protoCmdBit(s_fastBits, s_line[0])) {
      protoFlushLine();  /* dispatch single character commands right away */
    }
  }

  /* line timeout: buffered data that went quiet is treated as a complete line */
  if (s_pending && (millis() - s_lastCharMs) >= PROTO_LINE_TIMEOUT_MS) {
    protoFlushLine();
  }

  /* over-long line drop timeout */
  if (s_dropUntilEol && (millis() - s_lastCharMs) >= PROTO_LINE_TIMEOUT_MS) {
    s_dropUntilEol = false;
    s_line[0] = '\0';
    s_len = 0;
    s_pending = false;
  }
}

/* ========== dispatch ========== */
/* Dispatch the buffered line to the command handler */
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

static void protoFlushLine(void)
{
  s_line[s_len] = '\0';
  if (s_len > 0) {
    protoHandleLine(s_line);
  }
  s_len = 0;
  s_pending = false;
}

/* ========== command handling ========== */
/* Handle one complete line (newline already removed) */
int protoHandleLine(const char *line)
{
  if (line == NULL) {
    return PROTO_RES_NONE;
  }

  /* Work directly on the caller's string. Every parser below skips blanks on
   * its own, so the old "collapse blanks into a local copy" pass - and the
   * 40 byte stack frame it needed - is gone. */
  const char *p = line;
  PROTO_SKIP_BLANKS(p);

  const char cmd = *p;
  if (cmd == '\0') {
    return PROTO_RES_NONE;
  }

  /* "single" is the old "normalized length == 1": blanks only after the command
   * character. buf[0] of the old copy was exactly this cmd. */
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
    return PROTO_RES_SPEED_LEVEL;
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
  if (single && (cmd == PROTO_CMD_PICK_A || cmd == PROTO_CMD_PICK_B || cmd == PROTO_CMD_PICK_C)) {
    if (!WEARM_ENABLE_PICK_PLACE || !protoRuntimeEnabled(PROTO_FEATURE_PICK)) {
      R_REJ();
      return PROTO_RES_UNKNOWN;
    }
  }
  if (single && (cmd == PROTO_CMD_BTN_CYCLE || cmd == PROTO_CMD_BTN_RECORD ||
      cmd == PROTO_CMD_BTN_PLAY || cmd == PROTO_CMD_BTN_HOME ||
      cmd == PROTO_CMD_BTN_HOME_ALT)) {
    if (!WEARM_ENABLE_BUTTONS || !protoRuntimeEnabled(PROTO_FEATURE_BUTTON)) {
      R_REJ();
      return PROTO_RES_UNKNOWN;
    }
  }
  if (single && (cmd == PROTO_CMD_DRAW_TASK || cmd == PROTO_CMD_DRAW_START ||
       cmd == PROTO_CMD_DRAW_RECORD || cmd == PROTO_CMD_DRAW_UNDO ||
       cmd == PROTO_CMD_DRAW_PAUSE || cmd == PROTO_CMD_DRAW_RESUME ||
       cmd == PROTO_CMD_DRAW_CANCEL || cmd == PROTO_CMD_DRAW_PAPER_Z ||
       cmd == PROTO_CMD_DRAW_HALF || cmd == PROTO_CMD_DRAW_CENTER) &&
      (!WEARM_ENABLE_DRAW || !protoRuntimeEnabled(PROTO_FEATURE_DRAW))) {
    R_REJ();
    return PROTO_RES_UNKNOWN;
  }

  /* Busy decision: while a pick/place sequence runs, or the button module is
   * recording/playing/homing, or a drawing task is running, every other serial
   * motion command gives way.
   *
   * Two exceptions must pass (the callee decides again and answers
   * PROTO_RES_BUSY when it disagrees):
   *   1) speed commands (H, L, 1, 2, 3) - you may want to change speed in the
   *      middle of a pick/place sequence or a recording;
   *   2) button commands (N, R, P, M) - sending R to stop a recording must not
   *      be blocked here, or the recording could never be stopped.
   *   3) drawing commands (F, D, G, E, Q, U, W and p, n, o) - pause/resume/
   *      cancel/teach/calibrate are exactly what you send while drawing, so
   *      blocking them would disable all three control functions.
   *
   * The three predicates are pure queries, so || order cannot change the
   * answer; it is written cheapest-first only to keep the code short.
   * Measured: -8 bytes versus pickPlaceIsBusy() first. */
  if (drawControlBusy() || buttonControlBusy() || pickPlaceIsBusy()) {
    if (!protoCmdBit(s_liveBits, cmd)) {
      R_REJ();
      return PROTO_RES_BUSY;
    }
  }

  /* single character commands */
  if (single) {
    /* k/K are pulled out of the switch below: 'k' is the only case above 'W',
     * so the emitted jump table stays 40 entries wide instead of 60. */
    if (cmd == PROTO_CMD_TOOL_OPEN_STEP || cmd == PROTO_CMD_TOOL_CLOSE_STEP) {
      if (cmd == PROTO_CMD_TOOL_OPEN_STEP) {
        posToolOpen(PROTO_TOOL_STEP_DEG);
      } else {
        posToolClose(PROTO_TOOL_STEP_DEG);
      }
      DEBUG_PRINT(F("[tool] angle4 -> "));
      DEBUG_PRINTLN(Pos.ser.angle4);
      R_OKN();
      return PROTO_RES_TOOL_STEP;
    }

    /* '0' (home) and '1'..'3' (speed presets) sit far below the high end of the
     * switch, so they are handled here: the emitted jump table stays 23 entries
     * wide instead of 40. */
    if (cmd == PROTO_CMD_BTN_HOME_ALT) {
      int result = buttonHandleCommand(cmd);
      if (result >= 0) R_OK(); else R_REJ();
      return result >= 0 ? result : PROTO_RES_UNKNOWN;
    }
    if (cmd >= PROTO_CMD_SPEED_SLOW && cmd <= PROTO_CMD_SPEED_FAST) {
      /* '1','2','3' are SPEED_SLOW..SPEED_FAST in that order */
      int level = cmd - PROTO_CMD_SPEED_SLOW;
      adjustSpeed(level);
      DEBUG_PRINT(F("[speed] serial cmd -> "));
      DEBUG_PRINTLN(speedLevelName(level));
      R_OK();
      return PROTO_RES_SPEED_LEVEL;
    }

    /* An if chain instead of a switch on purpose: the case labels span 'O'..'W',
     * but '0' and '1' keep the table's low bound, so switch emits a 64 entry
     * jump table while the chain only pays for the comparisons it needs.
     * Measured on the real AVR build: -78 bytes for this whole dispatcher. */
    if (cmd == PROTO_CMD_GRIPPER_OPEN) {
        posSetAngle4(servoLimit.maxF);
        DEBUG_PRINT(F("[tool] angle4 -> "));
        DEBUG_PRINTLN(Pos.ser.angle4);
        R_OKN();
        return PROTO_RES_GRIPPER_OPEN;

    } else if (cmd == PROTO_CMD_GRIPPER_CLOSE) {
        posSetAngle4(servoLimit.minF);
        DEBUG_PRINT(F("[tool] angle4 -> "));
        DEBUG_PRINTLN(Pos.ser.angle4);
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
        DEBUG_PRINT(F("[pick] "));
        DEBUG_PRINT(cmd);
        DEBUG_PRINT(F(" start -> "));
        DEBUG_PRINTLN(rc == 0 ? F("OK") : F("REJECTED"));
        /* 0 = started; -2 = busy; -1/-3 = could not start this time (bad number /
         * path validation failed). Both are reported as BUSY so the host knows
         * the command was understood but the sequence did not run; the reason is
         * on the debug serial. */
        if (rc == 0) {
          R_OK();
          return PROTO_RES_PICK_STARTED;
        }
        R_REJ();
        return PROTO_RES_BUSY;

    /* N/R/P/M and '0': serial twins of the four physical buttons. Whether they
     * can run right now (recording/playing/pick-place) is decided inside
     * button_control.cpp, which answers PROTO_RES_BUSY when it cannot - the
     * busy guard above deliberately lets these characters through, otherwise
     * sending R to stop a recording would be blocked. */
    } else if (cmd == PROTO_CMD_BTN_CYCLE || cmd == PROTO_CMD_BTN_RECORD ||
               cmd == PROTO_CMD_BTN_PLAY || cmd == PROTO_CMD_BTN_HOME) {
      int result = buttonHandleCommand(cmd);
      if (result == PROTO_RES_BUSY || result == PROTO_RES_REC_REJECTED ||
          result == PROTO_RES_PLAY_NO_RECORD) R_REJ();
      else if (result < 0) R_ERR();
      else R_OK();
      return result < 0 ? PROTO_RES_UNKNOWN : result;

    /* F/D/G/E/Q/U/W: drawing commands (pick task / start / teach point / undo /
     * pause / resume / cancel). Whether they can run (already drawing, not
     * enough teach points, path validation) is decided inside draw_control.cpp,
     * which answers PROTO_RES_BUSY or PROTO_RES_DRAW_REJECTED - the busy guard
     * above deliberately lets these characters through, otherwise pause/cancel
     * while drawing would be blocked. */
    } else if (cmd == PROTO_CMD_DRAW_TASK || cmd == PROTO_CMD_DRAW_START ||
               cmd == PROTO_CMD_DRAW_RECORD || cmd == PROTO_CMD_DRAW_UNDO ||
               cmd == PROTO_CMD_DRAW_PAUSE || cmd == PROTO_CMD_DRAW_RESUME ||
               cmd == PROTO_CMD_DRAW_CANCEL) {
      int result = drawHandleCommand(cmd);
      if (result == PROTO_RES_BUSY || result == PROTO_RES_DRAW_REJECTED) R_REJ();
      else if (result < 0) R_ERR();
      else R_OK();
      return result;
    }
    /* unknown single character command: fall through to angle parsing */
  }

  /* Paper calibration p/n/o: multi character (p12.5 / n6 / o20,0). Handled
   * before angle parsing because the 'o' line contains a comma and would
   * otherwise be taken for "an angle command with a syntax error". */
  if ((cmd == PROTO_CMD_DRAW_PAPER_Z ||
       cmd == PROTO_CMD_DRAW_HALF ||
       cmd == PROTO_CMD_DRAW_CENTER) && !single) {
    if (!WEARM_ENABLE_DRAW || !protoRuntimeEnabled(PROTO_FEATURE_DRAW)) {
      R_REJ();
      return PROTO_RES_UNKNOWN;
    }
    int result = protoHandleDrawCalib(p, cmd);
    if (result == PROTO_RES_DRAW_REJECTED) R_REJ();
    return result;
  }

  int rc = PROTO_RES_UNKNOWN;

  if (protoAxisIndexFromChar(cmd) >= 0) {
    /* angle command */
    double angles[3];
    uint8_t seen = 0;

    rc = PROTO_RES_BAD_SYNTAX;
    if (protoParseAxisLine(p, angles, &seen)) {
      protoApplyAngles(angles, seen);
      {
#if WEARM_DEBUG_SERIAL
        Serial.print(F("[proto] sync angles b="));
        Serial.print(Pos.ser.angle1);
        Serial.print(F(" r="));
        Serial.print(Pos.ser.angle2);
        Serial.print(F(" c="));
        Serial.println(Pos.ser.angle3);
#endif
        rc = PROTO_RES_ANGLES_SET;
      }
    }
    DEBUG_PRINTLN(F("[proto] bad syntax, ignored"));
  } else {
    /* Not an axis letter, but it contains a comma or starts with '=': it looks
     * like an angle command that was mistyped. Everything else is unknown. */
    const char *scan = p;
    while (*scan != '\0' && *scan != ',') {
      scan++;
    }
    if (*scan == ',' || cmd == '=') {
      rc = PROTO_RES_BAD_SYNTAX;
    }
  }

  if (rc == PROTO_RES_ANGLES_SET) {
    R_OK();
  } else {
    R_ERR();
  }
  return rc;
}

/* ========== angle parsing ========== */
/* Parse an angle command: group (',' group)*, group = [blank] axis letter [blank]
 * [optional '='] [blank] value. The whole line must parse; an axis given twice
 * keeps its last value. Anything that does not match the grammar returns false
 * and touches nothing but angles/seen. */
static bool protoParseAxisLine(const char *s, double angles[3], uint8_t *seen)
{
  const char *p = s;
  int groups = 0;

  for (;;) {
    /* 1) blanks before the group */
    PROTO_SKIP_BLANKS(p);

    /* 2) axis letter (either case) */
    int axis = protoAxisIndexFromChar(*p);
    if (axis < 0) return false;
    p++;

    /* 3) blanks between the letter and the '=' */
    PROTO_SKIP_BLANKS(p);

    /* 4) optional '=' */
    if (*p == '=') {
      p++;
      PROTO_SKIP_BLANKS(p);
    }

    /* 5) the value (must really start with a digit) */
    if (!protoParseNumber(&p, &angles[axis])) return false;

    /* 6) record it (an axis given twice keeps the last value) */
    *seen |= (uint8_t)(1u << axis);
    groups++;

    /* 7) only the end of line or a comma may follow */
    PROTO_SKIP_BLANKS(p);
    if (*p == '\0') break;
    if (*p != ',') return false;
    p++;   /* eat the comma and go to the next group */
  }

  return (groups > 0);
}

/* Hand written number parser: avoids strtod/atof */
static bool protoParseNumber(const char **pp, double *out)
{
  const char *p = *pp;
  bool neg = false;
  unsigned int ip = 0;
  double value;

  /* optional sign */
  if (*p == '+') {
    p++;
  } else if (*p == '-') {
    neg = true;
    p++;
  }

  /* integer part: at least one digit */
  if (*p < '0' || *p > '9') {
    return false;
  }
  /* Digits are collected in an integer and converted once. Saturating at
   * 1000000 leaves the clamped result identical to the old per digit double
   * accumulation: every value this firmware accepts as an angle is far below
   * the clamp ceiling, so both forms end up at the same limit. */
  do {
    if (ip < 9999u) {
      ip = ip * 10u + (unsigned int)(*p - '0');
    }
    p++;
  } while (*p >= '0' && *p <= '9');

  value = (double)ip;

  /* fraction: a '.' must be followed by digits, and the digits are still added
   * one decimal place at a time so the rounding is unchanged */
  if (*p == '.') {
    double decimal_place = 0.1;
    p++;
    if (*p < '0' || *p > '9') {
      return false;  /* "12." is not valid */
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

/* ========== drawing parameter calibration ========== */
/* p<paper z> / n<half width> / o<center x>,<center y>
 * A syntax error or an out-of-range value answers PROTO_RES_DRAW_REJECTED and
 * changes no parameter. */
static int protoHandleDrawCalib(const char *line, char cmd)
{
  const char *p = line + 1;   /* skip the command letter */
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
    DEBUG_PRINT(F("[draw] calib -> paper z="));
    DEBUG_PRINTF(drawGetPaperZ(), 2);
    DEBUG_PRINT(F(" half="));
    DEBUG_PRINTF(drawGetHalfSize(), 2);
    DEBUG_PRINT(F(" center=("));
    DEBUG_PRINTF(drawGetCenterX(), 2);
    DEBUG_PRINT(F(","));
    DEBUG_PRINTF(drawGetCenterY(), 2);
    DEBUG_PRINTLN(F(")"));
    return PROTO_RES_DRAW_CALIBRATED;
  }

  R_ERR();
  DEBUG_PRINTLN(F("[draw] calib syntax error or bad value, parameters unchanged"));
  return PROTO_RES_DRAW_REJECTED;
}

/* ========== landing the parsed angles ========== */
/* Write the parsed angles to the servos */
static void protoApplyAngles(const double angles[3], uint8_t seen)
{
  for (int a = 0; a < PROTO_AXIS_COUNT; a++) {
    if (!(seen & (uint8_t)(1u << a))) continue;

    /* angle1 = b, angle2 = r, angle3 = c, so the servo index of axis a is a
     * itself; protoAxisServoIndex[] only spells that same mapping out. */
    (&Pos.ser.angle1)[a] = angles[a];
  }

  /* sync semantics: clamp every joint into its travel range (the same check the
   * rest of the firmware shares, which keeps six double comparisons out of this
   * file) and run the forward kinematics once. The caller only reaches this
   * point with at least one axis seen, so the old guard was always true. */
  clampServoAngles(&Pos.ser);
  if (!recFromServo(&Pos.rec, &Pos.ser)) {
    DEBUG_PRINTLN(F("[proto] warning: recFromServo failed"));
  }
}

/* ========== speed control ========== */
/* Step the speed level */
static int protoSpeedStep(int delta)
{
  int current_level = speedGetLevel();

  /* a custom level falls back to the default one */
  if (current_level < PROTO_SPEED_LEVEL_MIN || current_level > PROTO_SPEED_LEVEL_MAX) {
    current_level = PROTO_SPEED_LEVEL_DEF;
  }

  int want_level = current_level + delta;

  /* clamp to the level range */
  if (want_level < PROTO_SPEED_LEVEL_MIN) {
    want_level = PROTO_SPEED_LEVEL_MIN;
  } else if (want_level > PROTO_SPEED_LEVEL_MAX) {
    want_level = PROTO_SPEED_LEVEL_MAX;
  }

  /* already at the end: do not call adjustSpeed */
  if (want_level == current_level) {
    DEBUG_PRINT(F("[speed] already at "));
    DEBUG_PRINTLN(speedLevelName(current_level));
    return current_level;
  }

  /* change the speed level */
  want_level = adjustSpeed(want_level);
  DEBUG_PRINT(F("[speed] "));
  DEBUG_PRINT(delta > 0 ? F("H") : F("L"));
  DEBUG_PRINT(F(" -> "));
  DEBUG_PRINTLN(speedLevelName(want_level));
  return want_level;
}

/* ========== initialization ========== */
/* Serial.begin(PROTO_BAUD) plus the command table */
void serialProtocolBegin(void)
{
  Serial.begin(PROTO_BAUD);
  s_runtimeFeatures = protoCompiledFeatures();

  /* command table: kept under WEARM_SERIAL_RESPONSES so the host still gets the
   * command list with WEARM_DEBUG_SERIAL=0. Deliberately terse: every character
   * here is flash, so the command list and the angle syntax share one line. */
  R_BOOT();

  /* verbose table */
  DEBUG_PRINTLN(F("[proto] ===== serial command table ====="));
  DEBUG_PRINTLN(F("[proto] O            gripper OPEN  (angle4 -> f max)"));
  DEBUG_PRINTLN(F("[proto] S            gripper CLOSE (angle4 -> f min)"));
  DEBUG_PRINTLN(F("[proto] H / L        speed up / down one level"));
  DEBUG_PRINTLN(F("[proto] x deg,y deg,z deg  sync 3 servos, e.g. x10,y30,z20"));
  for (int a = 0; a < PROTO_AXIS_COUNT; a++) {
    double lo, hi;
    if (!protoAxisGetLimit(a, &lo, &hi)) continue;
    /* the axis letter and the joint letter are printed as one character C
     * strings so they are not taken for code values */
    DEBUG_PRINT(F("[proto] "));
    DEBUG_PRINT(protoAxisChar[a]);
    DEBUG_PRINT(F(" -> angle"));
    DEBUG_PRINT(protoAxisServoIndex[a]);
    DEBUG_PRINT(F(" ("));
    DEBUG_PRINT(protoAxisJoint[a]);
    DEBUG_PRINT(F(") "));
    DEBUG_PRINTF(lo, 1);
    DEBUG_PRINT(F(".."));
    DEBUG_PRINTF(hi, 1);
    DEBUG_PRINTLN();
  }
  DEBUG_PRINTLN(F("[proto] legacy: 1/2/3 = slow/normal/fast, k/K = tool step open/close"));
  DEBUG_PRINTLN(F("[proto] A/B/C = start pick/place sequence for object A/B/C"));
  DEBUG_PRINTLN(F("[proto] ================================"));
}
