/*
// serial_protocol.cpp
// 串口命令协议实现：固定指令通信 + 多舵机协同（x/y/z 三舵机同步角度）
// 另含 A/B/C 自动取放序列的启动入口（序列执行期间本层挡下其它动作指令）
// 实现串口字符读取、行缓冲、命令解析和舵机控制
*/

#include "Arduino.h"
#include "constant_and_positions.h"
#include "pick_place.h"
#include "protocol_constants.h"
#include "serial_protocol.h"
#define WEARM_DEBUG_SERIAL 1

/* ---------- 行缓冲与状态 ---------- */
static char s_line[PROTO_LINE_BUF_SIZE];
static int  s_len = 0;
static bool s_pending = false;          /* 缓冲区里有还没派发的字符 */
static bool s_dropUntilEol = false;     /* 本行超长，丢弃到行尾为止 */
static unsigned long s_lastCharMs = 0;

/* ---------- 辅助函数声明 ---------- */
static void protoFlushLine(void);
static bool protoParseAxisLine(const char *s, double angles[3], bool seen[3]);
static bool protoParseNumber(const char **pp, double *out);
static int protoApplyAngles(const double angles[3], const bool seen[3]);
static int protoSpeedStep(int delta);

/* ========== 主循环接口 ========== */
/* 每轮 loop() 调用一次：把串口收到的字符攒成一行并执行 */
void serialProtocolLoop(void)
{
  while (Serial.available() > 0) {
    int c = Serial.read();
    if (c < 0) break;

    /* 记录最后收到字符的时间 */
    s_lastCharMs = millis();

    /* 处理超长行的丢弃状态 */
    if (s_dropUntilEol) {
      if (c == '\n' || c == '\r') {
        s_dropUntilEol = false;
        s_line[0] = '\0';
        s_len = 0;
        s_pending = false;
      }
      continue;  /* 丢弃字符直到行尾 */
    }

    /* 处理换行符 - 派发当前行 */
    if (c == '\n' || c == '\r') {
      protoFlushLine();
      continue;
    }

    /* 检查行缓冲区溢出 */
    if (s_len >= PROTO_LINE_BUF_SIZE - 1) {
      s_dropUntilEol = true;
      s_len = 0;
      s_pending = false;
#if WEARM_DEBUG_SERIAL
      Serial.println(F("[proto] line too long, dropped"));
#endif
      continue;
    }

    /* 存储字符到行缓冲区 */
    s_line[s_len++] = (char)c;
    s_line[s_len] = '\0';
    s_pending = true;

    /* 单字符固定命令立即执行 */
    if (s_len == 1) {
      char cmd = s_line[0];
      if (cmd == PROTO_CMD_GRIPPER_OPEN || cmd == PROTO_CMD_GRIPPER_CLOSE ||
          cmd == PROTO_CMD_SPEED_UP || cmd == PROTO_CMD_SPEED_DOWN ||
          cmd == PROTO_CMD_SPEED_SLOW || cmd == PROTO_CMD_SPEED_NORMAL ||
          cmd == PROTO_CMD_SPEED_FAST ||
          cmd == PROTO_CMD_TOOL_OPEN_STEP || cmd == PROTO_CMD_TOOL_CLOSE_STEP ||
          cmd == PROTO_CMD_PICK_A || cmd == PROTO_CMD_PICK_B || cmd == PROTO_CMD_PICK_C) {
        protoFlushLine();  /* 立即派发单字符命令 */
      }
    }
  }

  /* 处理行超时：如果缓冲区有数据且超时，则派发 */
  if (s_pending && (millis() - s_lastCharMs) >= PROTO_LINE_TIMEOUT_MS) {
    protoFlushLine();
  }

  /* 处理超长行丢弃超时 */
  if (s_dropUntilEol && (millis() - s_lastCharMs) >= PROTO_LINE_TIMEOUT_MS) {
    s_dropUntilEol = false;
    s_line[0] = '\0';
    s_len = 0;
    s_pending = false;
  }
}

/* ========== 派发函数 ========== */
/* 派发当前行到命令处理器 */
static void protoFlushLine(void)
{
  s_line[s_len] = '\0';
  if (s_len > 0) {
    protoHandleLine(s_line);
  }
  s_len = 0;
  s_pending = false;
}

/* ========== 命令处理 ========== */
/* 直接处理一整行命令（不含换行符） */
int protoHandleLine(const char *line)
{
  if (line == NULL) {
    return PROTO_RES_NONE;
  }

  /* 创建局部副本并裁剪空白 */
  char buf[PROTO_LINE_BUF_SIZE];
  int i = 0, j = 0;
  bool in_space = true;

  /* 复制并跳过前导空白 */
  while (line[i] != '\0' && (line[i] == ' ' || line[i] == '\t')) {
    i++;
  }

  /* 复制非空白字符，记录最后一个非空白位置 */
  while (line[i] != '\0' && j < PROTO_LINE_BUF_SIZE - 1) {
    if (line[i] == ' ' || line[i] == '\t') {
      if (!in_space) {
        buf[j++] = ' ';  /* 保留单个空格分隔符 */
        in_space = true;
      }
    } else {
      buf[j++] = line[i];
      in_space = false;
    }
    i++;
  }

  /* 裁剪尾部空白 */
  while (j > 0 && (buf[j-1] == ' ' || buf[j-1] == '\t')) {
    j--;
  }
  buf[j] = '\0';

  /* 空行处理 */
  if (j == 0) {
    return PROTO_RES_NONE;
  }

  /* 取放序列执行期间的忙判定：取放序列独占 b/r/c 三个关节角，其它串口动作指令与摇杆都让位 */
  if (pickPlaceIsBusy()) {
    char cmd = buf[0];
    /* 只有调速指令集合（H、L、1、2、3）在取放序列执行期间仍然有效 */
    if (cmd != PROTO_CMD_SPEED_UP && cmd != PROTO_CMD_SPEED_DOWN &&
        cmd != PROTO_CMD_SPEED_SLOW && cmd != PROTO_CMD_SPEED_NORMAL &&
        cmd != PROTO_CMD_SPEED_FAST) {
      return PROTO_RES_BUSY;
    }
  }

  /* 单字符命令处理 */
  if (j == 1) {
    char cmd = buf[0];
    switch (cmd) {
      case PROTO_CMD_GRIPPER_OPEN:
        posSetAngle4(servoLimit.maxF);
#if WEARM_DEBUG_SERIAL
        Serial.print(F("[tool] angle4 -> "));
        Serial.println(Pos.ser.angle4);
#endif
        return PROTO_RES_GRIPPER_OPEN;

      case PROTO_CMD_GRIPPER_CLOSE:
        posSetAngle4(servoLimit.minF);
#if WEARM_DEBUG_SERIAL
        Serial.print(F("[tool] angle4 -> "));
        Serial.println(Pos.ser.angle4);
#endif
        return PROTO_RES_GRIPPER_CLOSE;

      case PROTO_CMD_SPEED_UP:
        protoSpeedStep(+1);
        return PROTO_RES_SPEED_UP;

      case PROTO_CMD_SPEED_DOWN:
        protoSpeedStep(-1);
        return PROTO_RES_SPEED_DOWN;

      case PROTO_CMD_SPEED_SLOW:
      case PROTO_CMD_SPEED_NORMAL:
      case PROTO_CMD_SPEED_FAST:
      {
        int level = (cmd == PROTO_CMD_SPEED_SLOW) ? SPEED_SLOW :
                    (cmd == PROTO_CMD_SPEED_NORMAL) ? SPEED_NORMAL : SPEED_FAST;
#if WEARM_DEBUG_SERIAL
        Serial.print(F("[speed] serial cmd -> "));
        Serial.println(speedLevelName(level));
#endif
        adjustSpeed(level);
        return PROTO_RES_SPEED_LEVEL;
      }

      case PROTO_CMD_TOOL_OPEN_STEP:
      {
        bool result = posToolOpen(PROTO_TOOL_STEP_DEG);
#if WEARM_DEBUG_SERIAL
        if (result) {
          Serial.print(F("[tool] angle4 -> "));
          Serial.println(Pos.ser.angle4);
        } else {
          Serial.print(F("[tool] at limit, angle4 = "));
          Serial.println(Pos.ser.angle4);
        }
#endif
        return PROTO_RES_TOOL_STEP;
      }

      case PROTO_CMD_TOOL_CLOSE_STEP:
      {
        bool result = posToolClose(PROTO_TOOL_STEP_DEG);
#if WEARM_DEBUG_SERIAL
        if (result) {
          Serial.print(F("[tool] angle4 -> "));
          Serial.println(Pos.ser.angle4);
        } else {
          Serial.print(F("[tool] at limit, angle4 = "));
          Serial.println(Pos.ser.angle4);
        }
#endif
        return PROTO_RES_TOOL_STEP;
      }

      case PROTO_CMD_PICK_A:
      {
        int rc = pickPlaceStart(PICK_OBJECT_A);
#if WEARM_DEBUG_SERIAL
        Serial.print(F("[pick] A start -> "));
        Serial.println(rc == 0 ? "OK" : "REJECTED");
#endif
        /* 0 = 已启动；-2 = 正忙；-1/-3 = 本次没能启动（编号非法 / 路径校验失败），
         * 都归到 BUSY，让上位机知道"指令认了但序列没跑"，具体原因看调试串口 */
        return (rc == 0) ? PROTO_RES_PICK_STARTED : PROTO_RES_BUSY;
      }

      case PROTO_CMD_PICK_B:
      {
        int rc = pickPlaceStart(PICK_OBJECT_B);
#if WEARM_DEBUG_SERIAL
        Serial.print(F("[pick] B start -> "));
        Serial.println(rc == 0 ? "OK" : "REJECTED");
#endif
        return (rc == 0) ? PROTO_RES_PICK_STARTED : PROTO_RES_BUSY;
      }

      case PROTO_CMD_PICK_C:
      {
        int rc = pickPlaceStart(PICK_OBJECT_C);
#if WEARM_DEBUG_SERIAL
        Serial.print(F("[pick] C start -> "));
        Serial.println(rc == 0 ? "OK" : "REJECTED");
#endif
        return (rc == 0) ? PROTO_RES_PICK_STARTED : PROTO_RES_BUSY;
      }
    }
    /* 单字符命令不识别，继续往下走角度解析 */
  }

  /* 角度指令解析 */
  if (protoAxisIndexFromChar(buf[0]) >= 0) {

    double angles[3] = {0};
    bool seen[3] = {false};

    if (protoParseAxisLine(buf, angles, seen)) {
      int written = protoApplyAngles(angles, seen);
      if (written > 0) {
#if WEARM_DEBUG_SERIAL
        Serial.print(F("[proto] sync angles b="));
        Serial.print(Pos.ser.angle1);
        Serial.print(F(" r="));
        Serial.print(Pos.ser.angle2);
        Serial.print(F(" c="));
        Serial.println(Pos.ser.angle3);
#endif
        return PROTO_RES_ANGLES_SET;
      }
    }
#if WEARM_DEBUG_SERIAL
    Serial.println(F("[proto] bad syntax, ignored"));
#endif
    return PROTO_RES_BAD_SYNTAX;
  }

  /* 不是轴字母开头，但含逗号或以等号开头：像角度指令却写错了，归 BAD_SYNTAX；其余归 UNKNOWN */
  if (strchr(buf, ',') != NULL || buf[0] == '=') {
    return PROTO_RES_BAD_SYNTAX;
  }

  /* 未知命令 */
#if WEARM_DEBUG_SERIAL
  Serial.println(F("[proto] unknown cmd"));
#endif
  return PROTO_RES_UNKNOWN;
}

/* ========== 角度解析 ========== */
/* 解析角度指令：组 (逗号分隔的组)*，组 = [空白] 轴字母[空白] [可选的等号] [空白] 角度值。
 * 整行必须正好解析完；同一个轴出现两次以最后一次为准。
 * 任何不符合语法的输入都返回 false，且不改动 angles/seen 以外的任何东西。 */
static bool protoParseAxisLine(const char *s, double angles[3], bool seen[3])
{
  const char *p = s;
  int groups = 0;

  for (;;) {
    /* 1) 组前空白 */
    while (*p == ' ' || *p == '\t') p++;

    /* 2) 轴字母（大小写都认） */
    int axis = protoAxisIndexFromChar(*p);
    if (axis < 0) return false;
    p++;

    /* 3) 字母与等号之间的空白 */
    while (*p == ' ' || *p == '\t') p++;

    /* 4) 可选等号 */
    if (*p == '=') {
      p++;
      while (*p == ' ' || *p == '\t') p++;
    }

    /* 5) 角度值（必须真的有数字） */
    double value = 0.0;
    if (!protoParseNumber(&p, &value)) return false;

    /* 6) 记录（同轴重复以最后一次为准） */
    angles[axis] = value;
    seen[axis] = true;
    groups++;

    /* 7) 后面必须是行尾或逗号 */
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') break;
    if (*p != ',') return false;
    p++;   /* 吃掉逗号，进入下一组 */
  }

  return (groups > 0);
}

/* 手写数字解析：避免使用 strtod/atof */
static bool protoParseNumber(const char **pp, double *out)
{
  const char *p = *pp;
  double result = 0.0;
  int sign = 1;
  bool has_digits = false;
  double decimal_place = 0.1;

  /* 可选正负号 */
  if (*p == '+') {
    p++;
  } else if (*p == '-') {
    sign = -1;
    p++;
  }

  /* 整数部分：至少一位数字 */
  while (*p >= '0' && *p <= '9') {
    result = result * 10 + (*p - '0');
    has_digits = true;
    p++;
  }

  /* 小数部分：必须有数字 */
  if (*p == '.') {
    p++;
    bool decimal_has_digits = false;

    while (*p >= '0' && *p <= '9') {
      result += (*p - '0') * decimal_place;
      decimal_place *= 0.1;
      decimal_has_digits = true;
      p++;
    }

    if (!decimal_has_digits) {
      return false;  /* "12." 不合法 */
    }

  }

  /* 必须有数字 */
  if (!has_digits) {
    return false;
  }

  result *= sign;
  *out = result;
  *pp = p;
  return true;
}

/* ========== 落地写入 ========== */
/* 将解析的角度应用到舵机 */
static int protoApplyAngles(const double angles[3], const bool seen[3])
{
  int written_count = 0;

  for (int a = 0; a < PROTO_AXIS_COUNT; a++) {
    if (!seen[a]) continue;

    /* 获取该轴的行程限制 */
    double min_angle, max_angle;
    if (!protoAxisGetLimit(a, &min_angle, &max_angle)) {
      continue;  /* 理论上不会发生 */
    }

    /* 夹角到行程范围内 */
    double angle = angles[a];
    if (angle < min_angle) angle = min_angle;
    if (angle > max_angle) angle = max_angle;

    /* 写入对应的舵机角度 */
    switch (protoAxisServoIndex[a]) {
      case 1:  /* angle1 = b 基座回转 */
        Pos.ser.angle1 = angle;
        break;
      case 2:  /* angle2 = r 上臂俯仰 */
        Pos.ser.angle2 = angle;
        break;
      case 3:  /* angle3 = c 下臂俯仰 */
        Pos.ser.angle3 = angle;
        break;
    }

    written_count++;
  }

  /* 同步语义：所有轴写完后，只调用一次正运动学 */
  if (written_count > 0) {
    bool success = recFromServo(&Pos.rec, &Pos.ser);
    if (!success) {
#if WEARM_DEBUG_SERIAL
      Serial.println(F("[proto] warning: recFromServo failed"));
#endif
    }
  }

  return written_count;
}

/* ========== 速度控制 ========== */
/* 速度档位步进 */
static int protoSpeedStep(int delta)
{
  int current_level = speedGetLevel();

  /* 如果当前是自定义档位，则使用默认档位 */
  if (current_level < PROTO_SPEED_LEVEL_MIN || current_level > PROTO_SPEED_LEVEL_MAX) {
    current_level = PROTO_SPEED_LEVEL_DEF;
  }

  int want_level = current_level + delta;

  /* 夹到档位边界 */
  if (want_level < PROTO_SPEED_LEVEL_MIN) {
    want_level = PROTO_SPEED_LEVEL_MIN;
  } else if (want_level > PROTO_SPEED_LEVEL_MAX) {
    want_level = PROTO_SPEED_LEVEL_MAX;
  }

  /* 如果已经在端点，不调用 adjustSpeed */
  if (want_level == current_level) {
#if WEARM_DEBUG_SERIAL
    Serial.print(F("[speed] already at "));
    Serial.println(speedLevelName(current_level));
#endif
    return current_level;
  }

  /* 调整速度档位 */
  want_level = adjustSpeed(want_level);
#if WEARM_DEBUG_SERIAL
  Serial.print(F("[speed] "));
  if (delta > 0) Serial.print(F("H"));
  else Serial.print(F("L"));
  Serial.print(F(" -> "));
  Serial.println(speedLevelName(want_level));
#endif
  return want_level;
}

/* ========== 初始化 ========== */
/* 初始化串口：Serial.begin(PROTO_BAUD) 并打印命令表 */
void serialProtocolBegin(void)
{
  Serial.begin(PROTO_BAUD);

#if WEARM_DEBUG_SERIAL
  /* 打印命令表 */
  Serial.println(F("[proto] ===== serial command table ====="));
  Serial.println(F("[proto] O            gripper OPEN  (angle4 -> f max)"));
  Serial.println(F("[proto] S            gripper CLOSE (angle4 -> f min)"));
  Serial.println(F("[proto] H / L        speed up / down one level"));
  Serial.println(F("[proto] x角度,y角度,z角度   sync 3 servos, e.g. x10,y30,z20"));
  for (int a = 0; a < PROTO_AXIS_COUNT; a++) {
    double lo = 0.0, hi = 0.0;
    if (!protoAxisGetLimit(a, &lo, &hi)) continue;
    /* 轴字母与关节字母用单字符 C 串打印，避免被当成码值输出 */
    char axis_ch[2] = { protoAxisChar[a], '\0' };
    char joint_ch[2] = { protoAxisJoint[a], '\0' };
    Serial.print(F("[proto] "));
    Serial.print(axis_ch);
    Serial.print(F(" -> angle"));
    Serial.print(protoAxisServoIndex[a]);
    Serial.print(F(" ("));
    Serial.print(joint_ch);
    Serial.print(F(") "));
    Serial.print(lo, 1);
    Serial.print(F(".."));
    Serial.print(hi, 1);
    Serial.println();
  }
  Serial.println(F("[proto] legacy: 1/2/3 = slow/normal/fast, k/K = tool step open/close"));
  Serial.println(F("[proto] A/B/C = start pick/place sequence for object A/B/C"));
  Serial.println(F("[proto] ================================"));
#endif
}
