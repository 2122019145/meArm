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
