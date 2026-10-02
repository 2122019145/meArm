//
// move.cpp
// 关节角步进实现（角度模式）
//
// 统一流程（每一步都保证 Pos.ser 与 Pos.rec 严格自洽）:
//   备份角度与坐标
//   -> 试探性把目标关节角加减 stepSize 度
//   -> 角度夹到 servoLimit：夹完与备份相同 => 已到机械行程尽头 => MOVE_AT_LIMIT
//   -> recFromServo() 用新角度算出真实坐标
//   -> 坐标夹到 rangeLimit：这一步会把末端带出工作空间 => 整步回退
//      MOVE_AT_LIMIT（摇杆表现为"推到头了"）
//   -> 接受，返回 MOVE_OK
//
// 注意: 只改角度、绝不动坐标。坐标永远由正运动学重算，
// 因为 clampToRange() 是**原地修改** Pos.rec 的（返回"是否夹到过"），
// 曾经为了给 Pos.rec 一个"临时落点"而顺手挪坐标轴，结果 clampToRange()
// 把那个临时落点夹回了边界值，返回 false，整步回退被跳过 ——
// 表现为末端坐标可以停在界外（x = -0.5 < limit.minX）。
//
#include "move.h"
#include "constant_and_positions.h"

/* ---------- 内部: 把角度夹到关节硬限位 ---------- */

/* 按 servoSelfCheck() 建好的限位把四个角度吸附回区间内。
 * 返回是否有角度被改动。只改 Pos.ser，坐标由调用方随后重算。 */
static bool clampJointAngles(void) {
  SER before = Pos.ser;
  (void) clampServoAngles(&Pos.ser);
  return Pos.ser.angle1 != before.angle1 ||
         Pos.ser.angle2 != before.angle2 ||
         Pos.ser.angle3 != before.angle3 ||
         Pos.ser.angle4 != before.angle4;
}

/* 坐标是否越出 rangeLimit（只读判断，不改动 Pos）。
 * 不直接用 clampToRange() 的原因：它原地改坐标，会污染 Pos.rec。 */
static bool posOutOfRange(const REC *r) {
  return r->x < limit.minX || r->x > limit.maxX ||
         r->y < limit.minY || r->y > limit.maxY ||
         r->z < limit.minZ || r->z > limit.maxZ;
}

/* ---------- 单步关节移动 ---------- */

int moveJointStep(int dir, double stepSize) {
  if (stepSize <= 0) stepSize = speed.stepSize;  /* <=0 = 跟随全局调速 */

  SER oldSer = Pos.ser;
  double oldX = Pos.rec.x;
  double oldY = Pos.rec.y;
  double oldZ = Pos.rec.z;

  /* 1) 试探性转动目标关节。坐标一个字节都不碰。 */
  switch (dir) {
    case JOINT_C_UP:    Pos.ser.angle3 += stepSize; break;
    case JOINT_C_DOWN:  Pos.ser.angle3 -= stepSize; break;
    case JOINT_R_FWD:   Pos.ser.angle2 += stepSize; break;
    case JOINT_R_BWD:   Pos.ser.angle2 -= stepSize; break;
    case JOINT_B_LEFT:  Pos.ser.angle1 -= stepSize; break;
    case JOINT_B_RIGHT: Pos.ser.angle1 += stepSize; break;
    default:            return MOVE_NONE;      /* 方向非法，不动 */
  }

  /* 2) 关节硬限位（最终防线）。夹完三个姿态角都等于原值 => 已到机械行程尽头。
   *    注意只比较 b/r/c：angle4 由末端专用接口控制，不走这条路径。 */
  (void) clampJointAngles();
  if (Pos.ser.angle1 == oldSer.angle1 &&
      Pos.ser.angle2 == oldSer.angle2 &&
      Pos.ser.angle3 == oldSer.angle3) {
    Pos.ser = oldSer;
    return MOVE_AT_LIMIT;
  }

  /* 3) 正运动学：用新角度算出真实末端坐标。 */
  if (!recFromServo(&Pos.rec, &Pos.ser)) {
    Pos.ser = oldSer;
    Pos.rec.x = oldX; Pos.rec.y = oldY; Pos.rec.z = oldZ;
    return MOVE_UNREACHABLE;
  }

  /* 4) 位置边界：这个角度把末端带出了 rangeLimit（工作空间）就整步回退。
   *    角度模式下坐标是派生量，所以这不算"不可达"，而是"这个方向到头了"。 */
  if (posOutOfRange(&Pos.rec)) {
    Pos.ser = oldSer;
    Pos.rec.x = oldX; Pos.rec.y = oldY; Pos.rec.z = oldZ;
    return MOVE_AT_LIMIT;
  }

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
