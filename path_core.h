/*
 * path_core.h -- 取放（pick_place.cpp）与绘图（draw_control.cpp）共用的运动核心
 *
 * 这两个模块原先各自复制了一份完全相同的运动学小工具（反解 solveJoint、
 * 点校验 pointOk、直线段校验 segmentOk、写关节角 setJoints），两份实现的差别
 * 只有采样步长与"反解分支跳变"门限这几个常量（PICK_* vs DRAW_*）。
 *
 * 这里把它们提成带参数的 inline 函数：两个编译单元看到的是同一个符号，
 * 链接器只会保留一份（C++ inline 走 COMDAT，重复定义被丢弃），于是省掉
 * 一整份重复代码。参数化只把常量搬成实参，算式与运算次序一个字没动，
 * 所以每一个原调用点的结果都与原来逐位相同。
 *
 * 只有 C++11 裸机 AVR 代码；本文件不产生任何数据段，也不打印任何东西。
 *
 * 【本版本已删除】pathCorePointOk() 里的 limit 范围检查。笛卡尔坐标范围
 * 限制（rangeLimit/limit）已按需求整体移除；b/r/c 的物理行程 0~180 在
 * moveJointStep() 内硬编码，f 的 60~150 在 posSetAngle4() 内硬编码。
 */
#ifndef PATH_CORE_H
#define PATH_CORE_H

#include <math.h>
#include "constant_and_positions.h"

/* 反解一个工作区点，成功时给出 b/r/c。
 * 注意必须给 angle4 一个合法值：getAngleEx() 会把四个关节一起夹，
 * angle4 非法（例如 0）会让 clamped 恒为 true（.selfcheck 的历史教训）。 */
inline bool pathCoreSolveJoint(double x, double y, double z,
                               double *b, double *r, double *c) {
  pos p;
  p.rec.x = x;
  p.rec.y = y;
  p.rec.z = z;
  p.ser = Pos.ser;                 /* 顺带带上传一个合法的 angle4 */
  bool clamped = false;
  if (!getAngleEx(&p, &clamped)) return false;
  if (clamped) return false;       /* 靠吸附才能表示的姿态不算可达 */
  if (b) *b = p.ser.angle1;
  if (r) *r = p.ser.angle2;
  if (c) *c = p.ser.angle3;
  return true;
}

/* 这个工作区点能不能用：几何可达 + 反解成功且没被吸附。
 * 【本版本已删除】原来的 limit 三轴范围检查。笛卡尔坐标范围限制已按需求
 * 整体移除；剩下的几何可达性 isReachable() 属于反解的数学前提（acos 定义域），
 * 不是"范围限制"，仍然保留。 */
inline bool pathCorePointOk(double x, double y, double z,
                            double *b, double *r, double *c) {
  REC rec;
  rec.x = x;
  rec.y = y;
  rec.z = z;
  if (!isReachable(&rec)) return false;

  return pathCoreSolveJoint(x, y, z, b, r, c);
}

/* 三个轴一次写完，只做一次正解刷新（与串口角度指令同一约定） */
inline void pathCoreSetJoints(double b, double r, double c) {
  Pos.ser.angle1 = b;
  Pos.ser.angle2 = r;
  Pos.ser.angle3 = c;
  (void) recFromServo(&Pos.rec, &Pos.ser);
}

/* 校验一条直线段：逐点检查能不能用，并且相邻采样点的反解分支不跳变。
 *
 * 两个端点改成了两个三元素点数组：AVR 上传六个 double 会让每个调用点
 * 都要压栈再出栈，而两个指针只需要取地址。传进去的数值与原来六参数
 * 形式所传的逐位相同。
 *
 * sampleStep / sampleMax / jumpDeg 就是原来各文件里写死的采样步长、
 * 单段最多采样点数与分支跳变门限（0.5 / PICK 64 / DRAW 96 / 25.0 度）。 */
inline bool pathCoreSegmentOk(const double *p0, const double *p1,
                              double sampleStep, int sampleMax, double jumpDeg) {
  double x0 = p0[0], y0 = p0[1], z0 = p0[2];
  double dx = p1[0] - x0;
  double dy = p1[1] - y0;
  double dz = p1[2] - z0;
  double len = sqrt(dx * dx + dy * dy + dz * dz);

  int steps = (int)(len / sampleStep) + 1;
  if (steps < 2) steps = 2;
  if (steps > sampleMax) steps = sampleMax;

  double pb = 0.0, pr = 0.0, pc = 0.0;
  for (int i = 0; i <= steps; i++) {
    double t = (double)i / (double)steps;
    double b = 0.0, r = 0.0, c = 0.0;
    if (!pathCorePointOk(x0 + dx * t, y0 + dy * t, z0 + dz * t, &b, &r, &c)) {
      return false;
    }
    if (i > 0) {
      if (fabs(b - pb) > jumpDeg ||
          fabs(r - pr) > jumpDeg ||
          fabs(c - pc) > jumpDeg) {
        return false;   /* 反解在段中间换了分支，说明这条直线不能走 */
      }
    }
    pb = b;
    pr = r;
    pc = c;
  }
  return true;
}

#endif