/* 临时诊断（放在被 .gitignore 忽略的 out/ 里，不入库）：
 * 给定目标点，打印反解出的 b/r/c 与正解回代的实际到达点，
 * 用来看"二维水平残差判据"在竖直方向到底放行了多大的误差。 */
#include <stdio.h>
#include "constant_and_positions.h"

static void probe(double x, double y, double z) {
  pos p;
  p.rec.x = x; p.rec.y = y; p.rec.z = z;
  p.ser.angle1 = 90; p.ser.angle2 = 90; p.ser.angle3 = 90; p.ser.angle4 = 90;
  bool clamped = false;
  const bool ok = getAngleEx(&p, &clamped);
  if (!ok) { printf("(%.1f, %.1f, %.1f)  REJECT\n", x, y, z); return; }
  REC back;
  recFromServo(&back, &p.ser);
  printf("(%.1f, %.1f, %.1f)  b=%.1f r=%.1f c=%.1f  -> 实际到达 (%.3f, %.3f, %.3f)"
         "  dz=%+.3f  xy_err=%.3f  clamped=%d\n",
         x, y, z, p.ser.angle1, p.ser.angle2, p.ser.angle3,
         back.x, back.y, back.z, back.z - z,
         sqrt((back.x - x) * (back.x - x) + (back.y - y) * (back.y - y)),
         clamped ? 1 : 0);
}

int main(void) {
  printf("目标                     角解                      实际到达点\n");
  probe(0.0, 80.0, 118.0);   /* POS_HOME */
  probe(0.0, 20.0, -18.5);   /* 物理可达带下界附近 */
  probe(0.0, 62.0, -20.0);
  probe(0.0, 122.0, -42.0);  /* 二维判据新放行的近满伸低点 */
  probe(0.0, 122.0, -60.0);
  probe(0.0, 125.0, -63.5);
  probe(0.0, 131.0, -63.0);
  probe(0.0, 122.0, 0.0);
  probe(0.0, 131.0, -63.5);  /* 最恶劣 */
  probe(0.0, 14.0, 172.0);   /* 上界附近 */
  probe(0.0, 40.0, 175.0);
  return 0;
}
