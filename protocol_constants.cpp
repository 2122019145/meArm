/*
// protocol_constants.cpp
// 串口命令协议层常量的实现
// 定义轴字符映射、舵机索引映射和关节字母映射
// 实现轴字符转换和行程限制查询函数
*/

#include "Arduino.h"
#include "protocol_constants.h"
#include "constant_and_positions.h"

/* 轴字符定义：x y z */
const char protoAxisChar[PROTO_AXIS_COUNT] = { 'x', 'y', 'z' };

/* 轴对应的关节字母，用于串口提示：b r c。
 * 轴 a 写入的就是 angle(a+1)，所以这里不再单独维护一张舵机索引表。 */
const char protoAxisJoint[PROTO_AXIS_COUNT] = { 'b', 'r', 'c' };

/* 把 x/X/y/Y/z/Z 转成轴下标 0..2，其它字符返回 -1 */
int protoAxisIndexFromChar(int c)
{
  /* 转换为小写进行比较 */
  char lower_c = (char)c;
  if (c >= 'A' && c <= 'Z') {
    lower_c = (char)(c - 'A' + 'a');
  }

  for (int i = 0; i < PROTO_AXIS_COUNT; i++) {
    if (protoAxisChar[i] == lower_c) {
      return i;
    }
  }
  return -1;  /* 不认识的轴字符 */
}

/* 取第 axis 个轴（0..2）的关节行程上下限，直接转发 servoLimit。
 * axis 越界或指针为空时返回 false。 */
/* 取第 axis 个轴（0..2）的关节行程上下限。
 * 【本版本】已删除 servoLimit 配置，统一硬编码为舵机物理行程 0~180 度。
 * axis 越界或指针为空时返回 false。 */
bool protoAxisGetLimit(int axis, double *minAngle, double *maxAngle)
{
  /* 检查参数有效性 */
  if (axis < 0 || axis >= PROTO_AXIS_COUNT || minAngle == NULL || maxAngle == NULL) {
    return false;
  }

  /* b/r/c 三轴行程均为 0~180 度（硬编码，无需 switch 分支） */
  *minAngle = 0.0;
  *maxAngle = 180.0;

  return true;
}
