//
// move.cpp
// 关节角步进实现（角度模式）
//
// 统一流程（每一步都保证 Pos.ser 与 Pos.rec 严格自洽）:
//   在副本 next = Pos.ser 上试探性把目标关节角加减 stepSize 度
//   -> 角度夹到 servoLimit：夹完与当前 Pos.ser 相同 => 已到机械行程尽头 => MOVE_AT_LIMIT
//   -> recFromServo() 用新角度算出真实坐标
//   -> 坐标一越出 rangeLimit（工作空间）=> 直接返回 MOVE_AT_LIMIT
//      （摇杆表现为"推到头了"）
//   -> 全部通过才一次性提交 Pos.ser / Pos.rec，返回 MOVE_OK
//
// 所有失败路径都一个字节都不写 Pos —— 这正是原来"备份 + 整步回退"的等价效果，
// 但省掉了备份变量和两处回退代码。
//
// 注意: 只改角度、绝不动坐标。坐标永远由正运动学重算，
// 因为 clampToRange() 是**原地修改** Pos.rec 的（返回"是否夹到过"），
// 曾经为了给 Pos.rec 一个"临时落点"而顺手挪坐标轴，结果 clampToRange()
// 把那个临时落点夹回了边界值，返回 false，整步回退被跳过 ——
// 表现为末端坐标可以停在界外（x = -0.5 < limit.minX）。
//
#include "move.h"
#include "constant_and_positions.h"

/* ---------- 内部: 方向码 -> 目标关节 / 步进符号 ---------- */

/* 六个 JointDir 方向码各自的目标关节与步进符号，按方向码数值 1..6 直接索引。
 * 下标 0/1/2 对应 servoAngle 里的 angle1(b) / angle2(r) / angle3(c)。
 *
 * 用查表取代原来六个展开的 case：每个 case 都要复制一份
 * "取角度 -> 32 位浮点加减 -> 写回"的代码，六份加起来比一张表贵得多。
 * 表放在 PROGMEM 里，用 pgm_read_byte 读，不占 RAM。
 *
 * 关节下标与符号打包进同一字节（低 1 位 = 是否取负，其余位 = 关节下标），
 * 这样只有一次 pgm_read_byte、一套表基址，符号判断也变成一位测试；
 * 拆成两张表时那第二套“取表基址 + 加下标 + lpm”实测要多花十几字节。 */
static const unsigned char DIR_CODE[7] PROGMEM = {
  0,                        /* 0: 占位，非法方向不会走到这里 */
  (2 << 1) | 0,             /* JOINT_C_UP    -> c (angle3), 步进取正 */
  (2 << 1) | 1,             /* JOINT_C_DOWN  -> c (angle3), 步进取负 */
  (0 << 1) | 1,             /* JOINT_B_LEFT  -> b (angle1), 步进取负 */
  (0 << 1) | 0,             /* JOINT_B_RIGHT -> b (angle1), 步进取正 */
  (1 << 1) | 0,             /* JOINT_R_FWD   -> r (angle2), 步进取正 */
  (1 << 1) | 1              /* JOINT_R_BWD   -> r (angle2), 步进取负 */
};

/* next 与 Pos.ser 的前三个姿态角（b/r/c）是否逐位相同。
 * 这里用逐位比较代替三次浮点 ==，两者在这条路径上等价：
 *   - 未被转动的两个角在夹取前后只可能"被夹了"（值确实变了）或原样不动；
 *   - 被转动的那个角 = 原值 ± stepSize（stepSize 恒 > 0），
 *     唯一能"值相等但位不同"的情形是 +0.0 / -0.0，而这条路径产生不了
 *     （a-b 恰好抵消时 IEEE 给的是 +0.0，且此时 Pos 那边是非 0 的 stepSize）。
 * angle4 不参与：它由末端专用接口控制，不走这条路径。 */
static bool samePose(const SER *a, const SER *b) {
  const unsigned char *pa = (const unsigned char *)a;
  const unsigned char *pb = (const unsigned char *)b;
  for (unsigned char i = 0; i < 3 * sizeof(double); i++) {
    if (pa[i] != pb[i]) return false;
  }
  return true;
}

/* 坐标是否越出 rangeLimit（只读判断，不改动 Pos）。
 * 与原来的 6 次展开比较逐条等价，只是改成 3 轴紧凑循环：
 *   limit 的内存布局是 {minX,maxX, minY,maxY, minZ,maxZ}，REC 是 {x,y,z}，
 *   两者都是连续的 double，所以按轴前进即可。
 *
 * 【实测记录】这里试过改成复用别人已经 out-of-line 的 clampToRange()
 * （把落点放进 pos 副本再靠它返回的"是否夹到过"当越界判据）：
 * move.cpp 自身确实从约 300 B 降到 206 B，但 LTO 的连锁反应让
 * draw_control.cpp 从 13966 B 涨到 14178 B，全程序净增 82 B，故回退。 */
static bool posOutOfRange(const REC *r) {
  const double *c = &r->x;
  const double *l = &limit.minX;
  for (unsigned char i = 0; i < 3; i++) {
    if (*c < l[0] || *c > l[1]) return true;
    c++;
    l += 2;
  }
  return false;
}

/* ---------- 单步关节移动 ---------- */

int moveJointStep(int dir, double stepSize) {
  if (stepSize <= 0) stepSize = speed.stepSize;  /* <=0 = 跟随全局调速 */

  /* 1) 试探性转动目标关节。整步都在 Pos 的副本上试算，
   *    这样任何一步不通过时 Pos 根本没被动过，不需要"备份 + 回退"那一整套读改写。
   *    目标关节与加减方向都从 PROGMEM 表里取（见文件开头的表）。 */
  if (dir <= 0 || dir > JOINT_R_BWD) {
    return MOVE_NONE;      /* 方向非法，不动 */
  }
  SER next = Pos.ser;
  unsigned char code = pgm_read_byte(&DIR_CODE[dir]);
  double *ap = &next.angle1 + (code >> 1);
  double step = stepSize;
  if (code & 1) step = -step;
  *ap += step;

  /* 2) 关节硬限位（最终防线）。夹完三个姿态角都等于原值 => 已到机械行程尽头。
   *    注意只比较 b/r/c：angle4 由末端专用接口控制，不走这条路径。
   *    直接返回即等价于原来的整步回退（Pos 未被改动）。 */
  (void) clampServoAngles(&next);
  if (samePose(&next, &Pos.ser)) {
    return MOVE_AT_LIMIT;
  }

  /* 3) 正运动学：用新角度算出真实末端坐标（先落在临时变量里，失败就不提交）。 */
  REC nextRec;
  if (!recFromServo(&nextRec, &next)) {
    return MOVE_UNREACHABLE;
  }

  /* 4) 位置边界：这个角度把末端带出了 rangeLimit（工作空间）就整步回退。
   *    角度模式下坐标是派生量，所以这不算"不可达"，而是"这个方向到头了"。 */
  if (posOutOfRange(&nextRec)) {
    return MOVE_AT_LIMIT;
  }

  /* 5) 全部通过，一次性提交：Pos.ser 与 Pos.rec 严格对应。 */
  Pos.ser = next;
  Pos.rec = nextRec;
  return MOVE_OK;
}

/* 旧名字：同一实现，历史上按"方向轴"理解调用方的兼容入口。 */
int moveAxisStep(int dir, double stepSize) {
  return moveJointStep(dir, stepSize);
}

/* 定长函数共用的封装：走一步并忽略结果（到限位时保持原位） */
static void stepFixed(int dir) {
  (void) moveJointStep(dir, 1.0);
}

/* ---------- 固定 1 度的六个方向步进 ---------- */

void moveup(void)       { stepFixed(JOINT_C_UP); }     /* 下臂 c +1 */
void movedown(void)     { stepFixed(JOINT_C_DOWN); }   /* 下臂 c -1 */
void moveleft(void)     { stepFixed(JOINT_B_LEFT); }   /* 基座 b -1 */
void moveright(void)    { stepFixed(JOINT_B_RIGHT); }  /* 基座 b +1 */
void moveforward(void)  { stepFixed(JOINT_R_FWD); }    /* 上臂 r +1 */
void movebackward(void) { stepFixed(JOINT_R_BWD); }    /* 上臂 r -1 */
