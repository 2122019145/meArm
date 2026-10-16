# 运动学模型替换后的“旧模型 / 旧尺度”遗留站点审计

审计对象：`D:\dsh1\meArm`（meArm 三舵机固件）。本文件是本次审计**唯一**新建/修改的文件，仓库其余部分未改动（只读审计）。

背景：固件的正/反运动学已替换为参照仓库 `Fifiiiii-123/MeArm@a7d2ee1`（本地副本 `D:\dsh1\meArm\others\f\MeArm`）的模型。本次审计逐行找出仍然编码**旧模型**（两节 20 单位、`arm1 = {20,20,0}`、`x >= 0` 半空间、`alpha = 90-r / beta = c-90`、`POS_HOME = (40,0,0)`、`±40` 盒子）或**旧尺度**（1 工作区单位 = 4 mm）的位置，并给出可推导的替换值。

---

## 0. 新模型基线（审计判据，全部来自当前源码）

| 事实 | 出处 | 值 |
| --- | --- | --- |
| 几何参数 | `constant_and_positions.cpp:26` | `L1 = 38`（底盘高）、`L2 = 80`（上臂）、`L3 = 80`（前臂），单位 **mm** |
| 结构体 | `constant_and_positions.cpp:43` | `struct arm arm1 = { 80, 80, 38 };` |
| 初始位姿 | `constant_and_positions.cpp:34` | `static const REC POS_HOME = { 0, 80, 118 };`（= 舵机姿态 90/90/90） |
| 正解 | `constant_and_positions.cpp:131-143` | `*outR = L2*cos(alpha) - L3*cos(alpha+betaRel);` `*outZ = arm1.armheight + L2*sin(alpha) - L3*sin(alpha+betaRel);` 其中 `theta = b-90`、`alpha = 180-r`、`betaRel = c`；`x = -R_h*sin(theta)`、`y = R_h*cos(theta)` |
| 反解门限 | `constant_and_positions.cpp:222,226` | 需 `d > fabs(L2-L3)+1 = 1` 且 `d < (L2+L3)-1 = 159`，`d = sqrt(rho² + (z-38)²)` |
| 关节窗口 | `constant_and_positions.h:72-77` | `KIN_BASE_MIN/MAX 0.0/180.0`、`KIN_RARM_MIN/MAX 45.0/160.0`、`KIN_FARM_MIN/MAX 30.0/110.0` |
| 坐标盒子 | `constant_and_positions.h:118` | `x/y = ±160`、`z = [-122, 198]` |
| **可达集约束** | `constant_and_positions.h:126-131` | “新模型的底座只有 b ∈ [0,180]，b = 90 指向 +y（正前方），所以**真实可达集全在 y >= 0 的半空间**里” |

**本审计独立推导出的硬约束（用于判定表格里的点是否可达）**：反解在 `R_h < 5` 时直接拒绝（`constant_and_positions.cpp:244`），故 `R_h > 0`；`theta = b - 90 ∈ [-90°, 90°]` ⇒ `cos(theta) >= 0` ⇒ **`y = R_h*cos(theta) >= 0` 恒成立**。因此：
> 任何 `y < 0` 的笛卡尔目标点在当前固件下**永远无解**（`pathCorePointOk()` 返回 false，`pickPlaceStart()` 返回 -4）。

辅助接口：`path_core.h:49 pathCorePointOk(x,y,z,&b,&r,&c)`（先查盒子 `:51-53`，再 `isReachable()`，再 `pathCoreSolveJoint`）、`path_core.h:80 pathCoreSegmentOk(p0,p1,sampleStep,sampleMax,jumpDeg)`。

---

## 1. 三条根因（几乎所有发现都归到这三条）

1. **尺度**：旧模型 1 工作区单位 = 4 mm（旧 `arm1 = {20,20,0}`，“两节各 80 mm”）。新模型坐标**直接就是毫米**。所有仍按旧单位书写的字面量都小了 **4×**。
2. **可达半空间翻转**：旧的不可达侧是 `x < 0`；新的不可达侧是 **`y < 0`**（`b = 90` 指向 +y）。所有把点放在 `y < 0` 或用 `x < 0` 描述“后方”的代码/注释都反了。
3. **舵机→几何约定改变**：旧 `alpha = 90 - r`、`beta = c - 90`；新 **`alpha = 180 - r`、`beta = c`**。所有引用旧公式的注释/推导失效（数值影响：`cos(180°−r)` 与 `cos(90°−r)` 符号相反，竖直分量 `sin` 的符号也相反）。
4. **坐标原点改变**（尺度换算的隐含坑）：旧模型肩关节在 `z = 0`；新模型 `z = 0` 是**底盘顶面**，肩关节在 `z = +38`。所以 `z` 不能简单 ×4，必须 **`z_new = 38 + 4*z_old`**；而 `x/y` 以基座轴为原点，可以 ×4。

**可复用的换算公式**（把任何旧模型点搬到新模型）：

```
x_new =  4 * x_old
y_new =  4 * |y_old|      // 旧 x<0 的“后方”搬过来会落到 y<0，必须镜像到 +y
z_new = 38 + 4 * z_old    // 肩轴从 z=0 抬到 z=38
```

---

## 2. 逐文件发现

### 2.1 `pick_place.cpp` / `pick_place.h`

> 名称对照（任务里用的老名字 → 现源码）：`PICK_APPROACH_DZ` → **`PICK_LIFT_DZ`**（由 `pickPlaceApproachDz()` 返回，见 `pick_place.h:79-81`）；`PICK_GRASP_Z` → 就是 `PICK_SRC[].z`（每个物体的抓取高度）；`PICK_CLEAR_DZ`、`PICK_BACK_DZ` 名字不变。

#### 尺度注释与常量

**`pick_place.cpp:5-6`**
```cpp
 * 1 工作区单位 = 4 mm（上臂/下臂各 20 单位 = 80 mm），所以 5 cm = 12.5 单位、2 cm = 5.0 单位。
```
- 问题：整段尺度换算前提已废（新坐标 = mm）。
- 建议：删掉“1 单位 = 4 mm”，改为“坐标单位 = mm；用户口径的 5cm/2cm 直接写 50/20”。

**`pick_place.cpp:62-63`**
```cpp
/* 尺度换算：1 工作区单位 = 4 mm（arm1 = {20, 20, 0}，两节各 80 mm）。
   用户口径的 5cm / 2cm / 10cm 在这里换成工作区单位。 */
```
- 问题：引用的 `arm1 = {20,20,0}` 已不存在（现为 `{80,80,38}`）；换算系数 4 已废。
- 建议：整段删除或改为“坐标单位 = mm（arm1 = {80,80,38}），5cm/2cm/10cm 直接是 50/20/100”。

**`pick_place.cpp:64`** `#define PICK_LIFT_DZ      12.5`
- 问题：旧单位的 5 cm，现在只有 **12.5 mm**（1.25 cm）。
- 建议：`#define PICK_LIFT_DZ      50.0`

**`pick_place.cpp:65`** `#define PICK_CLEAR_DZ      2.5`
- 问题：旧单位的 1 cm = 2.5 mm。
- 建议：`#define PICK_CLEAR_DZ      10.0`

**`pick_place.cpp:66`** `#define PICK_BACK_DZ       5.0`
- 问题：旧单位的 2 cm = 5 mm（注释 `pick_place.h:83` 写“物体后方 2cm”）。
- 建议：`#define PICK_BACK_DZ       20.0`

**`pick_place.cpp:67`** `#define PICK_TRAVEL_STEP   1.0`
- 问题：旧单位的 0.25 cm = 1 mm；`planTravelZ()` 用它逐档降飞行高度（`pick_place.cpp:321`），配合新 `LIFT−CLEAR = 50−10 = 40 mm` 只有 8 档，尚可，但 1 mm 档过密。
- 建议：`#define PICK_TRAVEL_STEP   5.0`（(50−10)/5 = 8 档，与旧行为的 8 档一致）。

**`pick_place.cpp:68-69`**
```cpp
#define PICK_HEIGHT_MIN   (-25.0)
#define PICK_HEIGHT_MAX   ( 25.0)
```
- 问题：旧单位的 ±10 cm = ±25 mm；现在只有 ±2.5 cm。另外由于第 1 节的第 4 条根因，这个门限的比较基准（`z` 直接和物体高度比）也要跟着换基准。
- 建议：改为“肩轴 z=38 ± 100 mm” → `#define PICK_HEIGHT_MIN (-62.0)` / `#define PICK_HEIGHT_MAX ( 138.0)`；或保留对称 ±100 但必须**显式声明基准是肩轴**并在比较处减 38。

#### 表格点（**确定 bug**：全部 3 个物体都不可达）

**`pick_place.cpp:94-98`**
```cpp
static const struct pickPoint PICK_SRC[PICK_OBJECT_COUNT] PROGMEM = {
    { 24.0,  16.0, -20.0 },
    { 24.0, -12.0,   0.0 },
    { 26.0,   0.0,  20.0 },
};
```
- 问题：**B 的 `y = -12 < 0` → 永远无解**；A/C 的半径只有 24~26 mm，而新臂展是 160 mm —— 这不是“误差”，是物理上把物体塞进了底座柱里（旧模型里 24/40 = 60% 臂展，需 96/160 才等价）。
- 建议（换算 + 把 y<0 镜像到 +y）：`A { 96.0, 64.0, -42.0 }`、`B { 96.0, 48.0, 38.0 }`、`C { 104.0, 20.0, 118.0 }`（C 取 y = 20 而不是 0，避免正落在 `y = 0` 的可达边界上）。可达性核对 `d = sqrt(rho²+(z-38)²) < 159`：A 140.4、B 107.3、C 132.7 ✓，且 y > 0 ✓。

**`pick_place.cpp:112-116`**
```cpp
static const struct pickPoint PICK_DST[PICK_OBJECT_COUNT] PROGMEM = {
    { 15.0, -18.0, -20.0 },
    { 18.0,  16.0,   0.0 },
    { 20.0, -16.0,  20.0 },
};
```
- 问题：**A 的 `y=-18`、C 的 `y=-16` → 均永远无解**。即使修好 PICK_SRC，A/C 也会在 `planTravelZ()` 探 `dst` 时失败。
- 建议：`A { 60.0, 72.0, -42.0 }`、`B { 72.0, 64.0, 38.0 }`、`C { 80.0, 64.0, 118.0 }`。可达性：A 123.3、B 96.3、C 130.0 ✓。

> **连锁后果**：`planTravelZ()`（`pick_place.cpp:314-339`）对每个候选飞行高度都对 `behind/src/dst` 调 `pathCorePointOk()`（`:327-333`），负 y 点恒定 false ⇒ 返回 false ⇒ `pickPlaceStart()` 在 `:620-628` 返回 **-4**。于是 `probe_pick_place.cpp:182`/`:187-189` 及后续级联全部失败。**A/B/C 三个物体当前都是死的**。

#### 路径采样

**`pick_place.cpp:143-144`**
```cpp
#define PICK_PATH_SAMPLE_STEP 0.5
#define PICK_PATH_SAMPLE_MAX  64
```
- 问题：0.5 旧单位 = 2 mm；表格按 mm 重标后（点间距变成 ~4×、单段可达 200 mm），2 mm 步长需要 ~100 个采样点 > 64，`pathCoreSegmentOk()`（`path_core.h:88-90`）会把步长**静默抬高**到 `段长/64`，不再是声明的精度。
- 建议：`PICK_PATH_SAMPLE_STEP 2.0` + `PICK_PATH_SAMPLE_MAX 160`。

#### 注释/推导（旧模型）

| 位置 | 当前行（要点） | 问题 | 建议 |
| --- | --- | --- | --- |
| `pick_place.cpp:33-41` | “物体高度范围 -10cm ~ +10cm（±25 单位）”“末端到肩的距离 sqrt(rho²+z²) ≤ 40” | 旧包络 40（现 159）；且距离基准必须是肩轴 z=38 | 改成“d = sqrt(rho²+(z-38)²) ≤ 159”“±100 mm” |
| `pick_place.cpp:49` | “改高度时只要落在 ±25 单位（±10cm）以内就不会被 -4 拒绝” | 同上 | “±100 mm（肩轴基准）” |
| `pick_place.cpp:81-88` | “两节各 20 单位，末端到肩的距离 ≤ 40 单位”“x_max(z) = sqrt(1600 - z*z)”“alpha = 90-r、beta = c-90 都落在 [-90,90]” | 公式、包络、舵机约定全旧 | 换成 `d < 159`、`alpha = 180-r`、`beta = c` |
| `pick_place.cpp:90-93` | “表里 C 在 +5cm / rho=26，飞行高度只能到 29.5（抬升 2.4cm）” | 基于旧几何的数值 | 按新表重算（C src rho=105.9、zv=80） |
| `pick_place.cpp:100-111` | Δx/Δy 与“门限 5.0”的推理 | 全旧单位 | 门限改 ~20 mm |
| `pick_place.cpp:302-313` | `planTravelZ` 文档重复“两节各 20 单位…≤ 40”“+10cm（z=+25）”“rho ≤ 13.9 单位” | 旧几何/旧单位 | 重写 |
| `pick_place.cpp:324-325` | 注释“起点位于可达边界（上电初始位姿 (40,0,0)）” | 旧 POS_HOME；新 home `(0,80,118)` 的 `d = 113.1 < 159`，**不在边界上** | 重写 |
| `pick_place.cpp:635-636` | “起点位于可达边界（上电初始位姿 (40,0,0)）时是‘物体正上方’…” | 同上 | 重写 |
| `pick_place.cpp:693-695` | 同上的首段注释 | 同上 | 重写 |

#### `s_riseAtSrc` 特例（现在几乎不可能触发）

**`pick_place.cpp:215-222`**
```cpp
/* 上电初始位姿 (b,r,c)=(90,90,90) 对应 (40,0,0)，rho 已取到最大值 L1+L2=40，
   而 rho=40 只在 z=0 成立，所以任何抬升都无解…… */
static bool s_riseAtSrc = false;
```
- 问题：前提已消失。新 home `(0,80,118)`：`rho = 80`、`zv = 118-38 = 80`、`d = 113.1`，离外边界 159 很远，`z` 还能升到 198 —— 从初始位姿出发**谈不上“任何抬升都无解”**。
- 结论：该分支变成**死代码**，除非起点恰好落在满伸位（`b = 0/180` 附近使 `rho → 160`）。`pick_place.cpp:334` 的 `s_riseAtSrc = !riseInPlace;` 逻辑本身没错，但注释与它存在的理由要重写；`pick_place.cpp:637-645` 的 `path[1][0] = s_riseAtSrc ? src[0] : Pos.rec.x;` 可保留。
- 建议：注释改为“仅在起点已贴近满伸边界（rho ≈ 160）时才需要从物体正上方起步”；或若确认实机不会出现该姿态，直接删除该分支并简化 `path[8][3]`。

#### `pick_place.h`

| 位置 | 当前行 | 问题 | 建议 |
| --- | --- | --- | --- |
| `pick_place.h:5` | “路径按用户口径：以物体高度为基准（1 工作区单位 = 4 mm，5cm = 12.5、2cm = 5.0）” | 旧尺度 | “坐标单位 = mm（5cm = 50、2cm = 20）” |
| `pick_place.h:17` | “物体与目标的放置高度允许 -10cm ~ +10cm（±25 单位）” | 旧单位 | “±100 mm（肩轴 z=38 基准）” |
| `pick_place.h:14` | “「后方 / 向前」取的是基座的水平半径方向，不是固定 ±x” | **仍然正确**，但值得补一句“不可达侧是 y<0” | 补注 |
| `pick_place.h:33` | “最后回初始位姿那一段走的是关节空间插值（POS_HOME 本身就是关节姿态，不必再反解）” | 仍然正确 | 保留 |

---

### 2.2 `draw_control.cpp` / `draw_control.h`

**`draw_control.cpp:66-70`**（纸面高度理由）
```cpp
/* 纸面高度：铅笔尖刚好落在纸上时的 Pos.rec.z。
   初始位姿末端是 (40,0,0)（两节水平朝前、完全伸直），纸面在肩轴**下方**
   （z < 0）才可能被笔尖碰到。默认取 -20（肩轴下方 20 单位 ≈ 80 mm 的台面）。
   上机可用 p<z> 随时重标（注意新模型下 x 越小、可达的竖向深度越浅，纸面别离肩轴太近）。 */
```
- 问题：旧 POS_HOME `(40,0,0)`；“肩轴下方 20 单位 ≈ 80 mm”是 4 mm/单位时代的说法；且 `z = 0` 现在是**底盘顶面**，肩轴在 `z = +38`，所以 `z = -20` 其实是“底盘顶面下 20 mm”（若底盘就坐在台面上，台面约在 `z ≈ -38`）。新模型下 `x` 与可达竖向深度无关（对称轴是 y）。
- 建议：数值 `-20.0` 可保留（它确实在底盘顶面之下、且可达：`zv = -58`，`rho` 上限 147.9 > 需要值），但**理由段必须重写**：`z = 0` = 底盘顶面，`z = -38` = 台面；`p<z>` 上机重标。

**`draw_control.cpp:71-74`**
```cpp
#define DRAW_PEN_Z_DEFAULT    -20.0
#define DRAW_CENTER_X_DEFAULT 27.0
#define DRAW_CENTER_Y_DEFAULT  0.0
#define DRAW_HALF_DEFAULT      6.0
```
- 问题：`CENTER_Y_DEFAULT = 0.0` 让默认图形**一半落在 `y < 0` 的不可达半空间**里（`y ∈ [-6, 6]`）；`x = 27` 与半宽 6 是旧尺度的图形尺寸（新臂展 160 mm，一张 12×12 mm 的图相当于旧模型的 3×3 单位）；`HALF = 6` 现在只有 6 mm。
- 建议：`DRAW_CENTER_X_DEFAULT 0.0`、`DRAW_CENTER_Y_DEFAULT 100.0`、`DRAW_HALF_DEFAULT 20.0`（区域 `x ∈ [-20,20]`、`y ∈ [80,120]`：整块 `y > 0` ✓；最大 `rho = sqrt(20²+120²) = 121.7`，`z = -20` 时 `d = sqrt(121.7²+58²) = 134.9 < 159` ✓）。

**`draw_control.cpp:78-80`**
```cpp
#define DRAW_HALF_MIN          2.0
#define DRAW_HALF_MAX         12.0
#define DRAW_CENTER_LIMIT     35.0   /* 中心允许的 |x|、|y| 上限（工作区单位） */
```
- 问题：三个都是旧 ±40 盒子的子集（35/40、12/40）；新盒子 ±160，且中心还必须满足 `y > 0` 与 `d < 159`。
- 建议：`HALF_MIN 10.0`、`HALF_MAX 60.0`、`CENTER_LIMIT 120.0`，并**追加可达性约束**（中心 `y` 必须 `>= half`、整块 `d < 159`）。注意：改这两个宏会连带打破 `probe_draw.cpp:413-414`（`!drawSetHalfSize(1.0) && !drawSetHalfSize(13.0)`、`!drawSetCenter(100.0, 0.0)`）与 `:877`（`o100,0` 必须被拒）—— 探针要同步改，否则是新引入的失败。

**`draw_control.cpp:84`** `#define DRAW_LIFT_DZ           4.0`
- 问题：旧单位 4.0 × 4 = 16 mm 抬笔量，现在只有 4 mm（可能仍够，但不再是设计值）。
- 建议：`15.0`（或 `20.0`）。

**`draw_control.cpp:451-461`** `liftZFor()`
```cpp
double z = pt[2] + DRAW_LIFT_DZ;
while (z > pt[2]) { if (pointOk(pt[0], pt[1], z, NULL, NULL, NULL)) return z; z -= 0.5; }
```
- 问题：`z -= 0.5` 是旧单位的 2 mm 降档；`DRAW_LIFT_DZ = 4.0` 时最多 8 次尝试。
- 建议：`z -= 2.0`（mm），配合 `DRAW_LIFT_DZ 15.0` 给出 7~8 档。

**`draw_control.cpp:91-99`** 速度/步长
```cpp
#define DRAW_V_MAX 3.0            /* 单位/秒 */
#define DRAW_ACCEL 6.0
#define DRAW_STEP_MAX 0.02
```
- 问题：旧 `3 单位/秒 = 12 mm/s`；单位换 mm 后同样的数值只有原来 **1/4 的物理速度**，但轨迹细分预算与 `MAX_DPS 70` 的配合都变了；`0.02` 现在是 0.02 mm/循环（注释“一条 12 单位直线分 600 步”已废）。
- 建议：`DRAW_V_MAX 12.0`、`DRAW_ACCEL 24.0`、`DRAW_STEP_MAX 0.08`（保持旧物理速度与每步比例），`DRAW_MAX_DPS 70.0` 保留。

**`draw_control.cpp:107-108`**
```cpp
#define DRAW_PATH_SAMPLE_STEP  0.25
#define DRAW_PATH_SAMPLE_MAX   96
```
- 问题：0.25 旧单位 = 1 mm；mm 尺度下图形跨度更大，`home → lift` 那一段（`draw_control.cpp:498-499`）长度可达数百 mm，`sampleMax 96` 会把步长静默抬高。
- 建议：`DRAW_PATH_SAMPLE_STEP 1.0` + `DRAW_PATH_SAMPLE_MAX 256`。

**`draw_control.cpp:112`** `#define DRAW_POINT_TOL 0.8`
- 问题：0.8 旧单位 = 3.2 mm 容差，现在只有 0.8 mm（偏严，会导致 `liftZFor` 找不到点而降级）。
- 建议：`1.0`~`3.0`（取 `1.0` 起步，上机可调）。

**`draw_control.cpp:115-122`**
```cpp
/* x/y/z 是"工作区单位"（0~40 左右）……满偏速度约 6 单位/秒 */
#define DRAW_JOG_STEP_MAX 0.25
#define DRAW_JOG_STEP_MIN 0.02
```
- 问题：注释的“0~40”是旧盒子；`0.25/0.02` 旧单位步长在 mm 下过小（满偏 0.25 mm/格）。
- 建议：注释改“mm（盒子 ±160 / z [-122,198]）”；`JOG_STEP_MAX 1.0`、`JOG_STEP_MIN 0.08`。

**`draw_control.cpp:128-130`** `DRAW_JOG_INVERT_X/Y/Z 0`
- 问题：不是旧模型残留，但**基座正方向在新模型里翻转了**（`x = -R_h*sin(theta)`），动点动时可能某个轴的“左右/前后”反了。
- 建议：上机验证；这是**标定项（B）**，不是代码错误。

**`draw_control.cpp:465-577` `validatePath()` step 1（home → 起点正上方）**
- 问题：**流程本身正确**（`posGetHomeAngles()` 取自 `POS_HOME`、`pathCoreSegmentOk()` 是模型无关的通用校验）。它当前的失败是**被喂了旧尺度/负 y 的目标**：默认图形 `y ∈ [-6,6]`、`x ≈ 27`，`home = (0,80,118)` 到那些点的连线穿过 `y < 0`，于是 `:498-499` 拒绝、`:501` 打印 `"[draw] vp1: home->lift segment rejected"`。
- 建议：**不改这段逻辑**；改 `DRAW_CENTER_*` / `DRAW_HALF_*`（见上）让默认图形落在 `y > 0` 且在 `d < 159` 内。

**`draw_control.cpp:37`** `/* 路径校验的采样步长也同步从 0.5 收紧到 0.25 */`
- 问题：历史注释，前一句的尺度前提已废。
- 建议：随 `DRAW_PATH_SAMPLE_STEP` 改值一并更新。

**`draw_control.h:92-97`**（纸面坐标与几何）
```cpp
 * 默认区域 x[14,26] y[-6,6]（中心 20,0，半宽 6），整块都在可达空间内部：
 * 该区域 |rho| 最大 26.7，R = sqrt(rho^2+z^2) 最大约 29.6 << L1+L2 = 40。
```
- 问题：三重错误 —— (a) 中心与代码默认 `(27,0)` 都不一致（写的是 20,0）；(b) `y[-6,6]` 有一半在不可达半空间；(c) 包络 `40` 与 `L1+L2` 是旧模型。
- 建议：整段按新的 `(0,100)` / 半宽 20 / `d < 159` 重写。

**`draw_control.h:25-27`**（速度/步长说明）、**`:62-64`**（`p<数字>`/`n<数字>`/`o<x>,<y>` 默认值说明）、**`:82-84`**（jog 步长）、**`:88-90`**（invert 说明）
- 问题：数值随上表的宏一起改；`o<x>,<y>` 说明里的“|x|,|y| ≤ 35”同样过期。
- 建议：同步更新为新宏值与 mm 单位。

**`draw_control.cpp:199-202`、`:317-319`、`:1219`、`:1222`、`:1228`、`:1231`、`:1237-1238`、`:1241-1242`、`:1023-1029`**
- 结论：这些只是初始值赋值/标定入口/串口回显，**不含旧模型常量**，随宏改值即可，无需单独修改。

---

### 2.3 `joystick_control.cpp` / `joystick_control.h`

**结论先行：`joystick_control.cpp` 本身没有任何几何/尺度常量**（它只按度步进关节、步长由 `speed.stepSize` 给），因此**无旧尺度残留**；遗留全在头文件注释。

**`joystick_control.h:40`**
```cpp
 *  每个关节都只受舵机电气行程限制（b/r/c 0~180，f 0~180 —— f 原来还有
```
- 问题：**stale**。现为 `b 0~180`、`r 45~160`、`c 30~110`（`KIN_*` 宏），`f` 仍是 `0~180`。
- 建议：改成“b 0~180、r 45~160、c 30~110、f 0~180（见 `constant_and_positions.h` 的 `KIN_*` 宏）”。

**`joystick_control.h:52-67`**（整个“【坐标模型（用于显示与工作空间校验）】”块）
```cpp
 * 肩关节在坐标原点，上臂 L1 = 下臂 L2 = 20，armheight = 0。
 * b (angle1) 90° -> 基座朝 +x 方向；b 行程 0~180° 使可达空间只占 x >= 0 一侧
 * 初始位姿 POS_HOME = (40, 0, 0) -> (b,r,c,f) = (90, 90, 90, 105)
 * 正运动学: alpha = 90 - r, beta = c - 90, x_planar = L1 cos alpha + L2 cos beta,
 *           z = L1 sin alpha + L2 sin beta + armheight, theta = 90 - b
 *           -> x = x_planar cos theta, y = x_planar sin theta
 * 【修正】肩轴反转（alpha = 90 - r）、肘轴正转（beta = c - 90）
```
- 问题：**整块旧模型** —— L1=L2=20、armheight=0、`b=90 → +x`、`POS_HOME (40,0,0)`、`alpha=90-r`、`beta=c-90`、可达侧 `x >= 0`。
- 建议：整块替换为新模型（`L1=38、L2=L3=80`、`b=90 → +y`、`POS_HOME (0,80,118) ↔ (90,90,90)`、`alpha=180-r`、`beta=c`、不可达侧为 `y<0`）。

**`joystick_control.h:28`** “左手柄 左右推 (A0) b = angle1 基座回转 角度增大（末端往 +y 侧转）”
- 结论：**正确**（与新模型一致），保留。

**`joystick_control.h:5-9`**（直接控制关节角、`Pos.rec.x/y/z` 只用于显示）、**`:76-82`**（死区/迟滞/标定参数）、**`:114-127`**（`JOY_DIR_*` / `JOY_ACT_*`）
- 结论：与运动学无关，**无需改动**。

**`joystick_control.cpp`（需注意但非 bug 的点）**
- `:10-13` 轴→关节映射注释、`:53-57` `JOY_LX_PIN A0 / JOY_LY_PIN A1 / JOY_RX_PIN A2 / JOY_RY_PIN A3`、`:105-126` `JOY_DIR_NEG[]` + `JOY_RAW_SLOT[] = {0,1,3,2}`：**尺度无关**（都是关节角），但**方向语义**在新模型下需要上机复核（对应 `meArm.ino:37-38` 那两句描述的方向）。
- `:69-70`（死区 40 / 满偏 500）、`:85`（`JOY_STEP_MIN_DEG 0.1`）、`:88`/`:90`：与模型无关。
- `:331` `double step = speed.stepSize * ((double)amp / (double)JOY_FULL_SCALE);`、`:338` `posSetAngle4(...)`、`:479` `moveJointStep(dir, step)`：单位是**度**，无需换算；但**同样的角速度在新臂上末端线速度是 4×**，默认 `stepSize` 建议减半（标定项 B）。
- `:456-460` 关于 `tx` 符号的注释（“If it feels inverted on the machine, swap the two”）、`:470-471`（提到旧 `p1.txt` 投诉）：历史叙述，建议保留但注明结论已随换模型重验。

---

### 2.4 `move.cpp`

**`move.cpp:7`**（文件头流程注释）
```cpp
 *   -> b/r/c 三轴硬编码夹到 0~180：夹完与当前 Pos.ser 相同 => 已到行程尽头 => MOVE_AT_LIMIT
```
- 问题：三轴已不再是统一 `0~180`（代码本身已用 `KIN_*`，见 `:72-77`），描述与实现矛盾。
- 建议：改成“b/r/c 按 `KIN_*` 窗口夹取（b 0~180、r 45~160、c 30~110）”。

**`move.cpp:19`**
```cpp
 * 把关。可配置的关节限位（clampServoAngles）仍然是删除状态：b/r/c 的 0~180 物理行程在 moveJointStep() 内部直接硬编码
```
- 问题：同上，`0~180` 已废。
- 建议：改成“b/r/c 的物理行程窗口（`KIN_*`）在 `moveJointStep() → clampBCR()` 内部硬编码”。

**`move.cpp:118`**
```cpp
/* 2) 关节硬限位：b/r/c 三轴硬编码 0~180（最终防线）。
```
- 问题：措辞 stale（紧跟的 `:122 clampBCR(&next);` 用的是 `KIN_*`）。
- 建议：改成“三轴按 `KIN_*` 窗口夹取（最终防线）”。

**`move.cpp:66-69`**（“【v1.6.7 运动逻辑替换】行程不再是三轴统一的 0~180……b 0~180、r 45~160、c 30~110”）、**`:72-77` `clampBCR()`**、**`:102-103` `moveJointStep()`**、**`:150` `stepFixed()`（固定 1°）**
- 结论：**已更新 / 与模型无关**，无需改动。

**`move.cpp:133-141`**
```cpp
 * 【本版】盒子比真实包络宽，这一步现在拦不住任何姿态，但它是配置的唯一判据
```
- 问题：结论仍成立，但“真实包络”已变 —— 现在盒子（±160 / [-122,198]）不只宽，**还把 `y < 0` 的整个不可达半空间包了进来**（盒子拦不住任何姿态，`y<0` 由反解兜底）。
- 建议：补一句限定“位置盒子的负 y 侧是声明，实际可达集由 `isReachable()`（`y>=0`、`d<159`）兜底”。

---

### 2.5 `button_control.cpp`

**结论：干净，无旧模型/旧尺度遗留。**
- `button_control.cpp:167-179` 注释与代码已同步到新模型：
```cpp
/* b/r/c 夹到各自的**机械行程窗口**（v1.6.7 起：b 0~180、r 45~160、c 30~110，
 * 见 constant_and_positions.h 的 KIN_* 宏），f 夹到舵机电气行程
 * TOOL_ANGLE_MIN..MAX(0~180) */
...
if (tmp.angle1 < KIN_BASE_MIN) tmp.angle1 = KIN_BASE_MIN;
if (tmp.angle2 < KIN_RARM_MIN) tmp.angle2 = KIN_RARM_MIN;
if (tmp.angle3 < KIN_FARM_MIN) tmp.angle3 = KIN_FARM_MIN;
```
- `:603-615` 用 `posGetHomeAngles()` 回中（`:603` 注释“`posGetHomeAngles()` 只写 angle1..3”），**与 `POS_HOME` 自洽**，不硬编码坐标。
- `:127-128`、`:244-247` 只涉及关节角增量，与尺度无关。

---

### 2.6 `serial_protocol.cpp`

**`serial_protocol.cpp:705-708`（确定 stale 注释）**
```cpp
/* 超出机械行程是夹取而不是拒绝：b/r/c 三轴的上下限已硬编码为 0..180，
 * 所以 "x200" 会落到该关节行程的上限，
 * 而不是把整条命令变成什么都不做的空操作。 */
```
- 问题：`0..180` 是旧模型；实际夹取窗口是 `KIN_*`（见下两行代码）。举例也不再准确：`x200` 现在落到 **c/b 各自窗口上限**（b→180，c→110），而 `y0` 会落到 **r 的下限 45**（不是 0）—— 探针正是栽在这里。
- 建议：改成“按各轴 `KIN_*` 窗口夹取：b 0~180、r 45~160、c 30~110；例如 `x200 → 180`、`y0 → 45`、`z0 → 30`”。

**`serial_protocol.cpp:836-844`（已更新，可作正确样板）**
```cpp
 * 角度在这里按各轴的**机械行程窗口**夹取（取代了原来的 clampServoAngles()
 * 调用）：b 0~180、r 45~160、c 30~110（v1.6.7 起，见
 * constant_and_positions.h 的 KIN_* 宏；旧版三轴统一 0~180）。
static const double axisMin[PROTO_AXIS_COUNT] = {KIN_BASE_MIN, KIN_RARM_MIN, KIN_FARM_MIN};
static const double axisMax[PROTO_AXIS_COUNT] = {KIN_BASE_MAX, KIN_RARM_MAX, KIN_FARM_MAX};
```
- 结论：**正确**（“旧版三轴统一 0~180”是明确的历史说明，不是遗留假设）。`serial_protocol.cpp:853` `(void) recFromServo(&Pos.rec, &Pos.ser);` 也是正确做法（写完角度重算坐标）。
- `:491`、`:500` 的“旧版”指**命令解析**的旧实现，与运动学无关。

---

### 2.7 `meArm.ino`

**`meArm.ino:37-38`（确定 bug：旧舵机→几何公式）**
```cpp
//   左手柄 前后推 (A1) -> r = angle2 上臂俯仰   角度增大 = 上臂下沉（α = 90°−r 变小）
//   右手柄 前后推 (A3) -> c = angle3 下臂俯仰   角度增大 = 下臂上抬（β = c−90° 变大）
```
- 问题：公式是旧的。新模型 `alpha = 180 - r`、`beta = c`。“r 增大 → alpha 减小 → 上臂下沉”这个**方向结论仍成立**（`d(alpha)/dr = -1`），但 `β = c - 90°` 完全错（新 `β = c`，所以“c 增大 → 下臂上抬”是否成立取决于新模型下的装配符号，必须按 `beta = c` 重述）。
- 建议：改为“r 增大 = 上臂下沉（α = 180°−r 变小）”“c 增大 = 下臂 … （β = c）”，并按实机复验方向。

**`meArm.ino:36`** “b = angle1 基座回转 角度增大 = 末端往 +y 侧转”
- 结论：**与新模型一致**，保留。

**`meArm.ino:115-128`（安全边界块，确定 stale）**
```cpp
//   1. 关节行程硬限位 —— b[0,180] r[0,180] c[0,180]，f[TOOL_ANGLE_MIN,TOOL_ANGLE_MAX]=[0,180]
//   2. rangeLimit 位置边界   —— "工作空间盒子"（constant_and_positions.cpp 的 limit）：
//                              x/y ±40、z -40~40。
```
- 问题：`r/c` 的 `0~180` 已废（现 `r 45~160`、`c 30~110`）；盒子已废（现 `x/y ±160`、`z [-122,198]`）。
- 建议：`:116` 改为“b[0,180] r[45,160] c[30,110]，f[0,180]”；`:122-123` 改为“x/y ±160、z [-122,198]（外加 `y >= 0` 与 `d < 159` 的真实可达判据）”。

**`meArm.ino:133-149`（坐标模型块，整块旧模型）**
```cpp
//   肩关节在坐标原点，上臂 L1 = 下臂 L2 = 20，armheight = 0。
//     b (angle1) 90° -> 基座朝 +x，b 偏离 90° 就转向 ±y；
//   alpha = 90 - r、beta = c - 90（肩轴反转、肘轴正转，见 RC-robot-hand 标定）。
//   初始位姿 POS_HOME = (40, 0, 0)，即 (b,r,c,f) = (90, 90, 90, f 中位)：
//   上臂、下臂都水平朝前，完全伸直的姿态，落在可达外边界 rho = L1+L2 = 40 上
//     x[0.00, 40.00] y[-40.00, 40.00] z[-40.00, 40.00]
//   limit 本版取 x/y ±40、z -40~40（与可达包络一致，两杆各 20、关节 0~180°）。
//   ★ 末端能到哪仍由几何决定（L1 = L2 = 20）：水平半径越大竖向行程越小，最大竖向行程在 ρ≈0 处（z ∈ [-40, 40]）。
//   ★ 上机提示: 基座可摆到 x<0 的后方、末端最低到 z≈-40.0，请先在慢速档、空载、抬离台面的情况下单步试。
```
- 问题：`L1=L2=20`、`armheight=0`、`b=90→+x`、`alpha=90-r`、`beta=c-90`、`POS_HOME (40,0,0)`、`rho=40`、三轴 `±40`、“基座可摆到 x<0 的后方”—— **全部 8 行都是旧模型**。
- 建议：整块按 `constant_and_positions.h:17-34` 的措辞重写（`L1=38、L2=L3=80`、`b=90→+y`、`alpha=180-r`、`beta=c`、`POS_HOME (0,80,118) ↔ (90,90,90)`、`x/y ±160`、`z [-122,198]`、可达集 `y>=0` 且 `d<159`、“末端最低到 z=-122，但 `y>=0` 半空间之外到不了”）。

**`meArm.ino:186-198` `writeServo()`（潜在 bug，镜像和）**
```cpp
servoDriveWrite(0, g_mirrorBase ? (180.0 - Pos.ser.angle1) : Pos.ser.angle1);
servoDriveWrite(1, g_mirrorShoulder ? (180.0 - Pos.ser.angle2) : Pos.ser.angle2);
servoDriveWrite(2, g_mirrorElbow ? (180.0 - Pos.ser.angle3) : Pos.ser.angle3);
#if WEARM_MIRROR_TOOL
servoDriveWrite(3, TOOL_MIRROR_SUM - Pos.ser.angle4);
#else
servoDriveWrite(3, Pos.ser.angle4);
#endif
```
- 问题：`180.0 - angle` 只在**该关节整行程恰好是 0~180 且关于 90 对称**时才是正确镜像。基座 `b` 满足（`0~180`，和 = 180 ✓）、夹爪用 `TOOL_MIRROR_SUM = 210` 已是装配角 ✓；但 `r ∈ [45,160]`（镜像和应为 `45+160 = 205`）、`c ∈ [30,110]`（应为 `30+110 = 140`）。若把 `WEARM_MIRROR_SHOULDER`/`WEARM_MIRROR_ELBOW` 置 1，`180-r` 会把 `[45,160]` 映到 `[20,135]` —— **窗口中心错位 25°**，等于换了一个安装角，不是镜像。
- 建议：改成窗口通用式 `(KIN_RARM_MIN + KIN_RARM_MAX) - Pos.ser.angle2`（= `205.0 - r`）与 `(KIN_FARM_MIN + KIN_FARM_MAX) - Pos.ser.angle3`（= `140.0 - c`），与 `weArm_config.h:208-221` 宣称的 `physical = (min+max) - logical` 一致。
- 现状缓解：两个标志默认 `0`（`weArm_config.h:229-239`），所以当前是**潜伏 bug**，不是现行故障。

**`meArm.ino:173-185`（镜像说明块）**：内容与新模型一致（`physical = (min+max) - logical`、基座和 180、夹爪和 210、v1.6.7 注明基座正方向翻转 + `mB` 开关），**保留**；建议把“和 180”的措辞改成“基座窗口 0~180，和恰为 180”，以免读者以为所有关节都是 180。

**`meArm.ino:200-233` setup()**：`:203 posInit()` → `:230 writeServo()`，注释 `:201-202` 说明必须先 `posInit()` 再 `writeServo()`（否则开机瞬间被拉到 0）——**正确且与模型无关**，保留。`:258` 第二次 `writeServo()` 同理。

**`meArm.ino:49-56`（串口 x/y/z 是关节角，不是坐标）**：与运动学无关，保留。

---

### 2.8 `weArm_config.h`

**`weArm_config.h:223-224`（需要上机重标，见 B）**
```cpp
#ifndef WEARM_MIRROR_BASE
#  define WEARM_MIRROR_BASE 1
```
- 问题：这不是“旧模型文字”，但**默认值建立在其语义上**。新模型 `x = -R_h*sin(b-90)` 让基座正方向与旧模型相反（`weArm_config.h:242-246` 自己也这么说），所以 `WEARM_MIRROR_BASE = 1` 现在**可能正好是错的**。
- 建议：上机用 `mB` 运行时开关实测两种方向，把确认值写回默认；不要凭推理直接翻转。

**`weArm_config.h:214`（措辞 stale）**
```cpp
 * 修法不动 0~180 的整套映射，也不动任何逻辑角，只在唯一写出口 writeServo() 里
```
- 问题：`0~180` 已不是“整套映射”（`r/c` 已变窗口）。
- 建议：改成“不动关节角的整套映射（b 0~180、r 45~160、c 30~110），也不动任何逻辑角”。

**`weArm_config.h:229-236`（`WEARM_MIRROR_SHOULDER 0` 的理由）**
```cpp
 * ...置 1 只会把上臂的**竖直**分量取反（水平分量 cos(180°−r) 不变）
```
- 问题：这句话**已经是用新模型写的**（`cos(180°−r)`），与新模型一致 ✓；但它同时也就说明了 `meArm.ino:188` 的 `180.0 - angle` 公式与这里声称的“不动逻辑角、只在写出口镜像”在 `r` 窗口上不自洽（见 2.7 的建议）。
- 建议：保留此理由，并按 2.7 修正 `writeServo()` 的镜像式。

**`weArm_config.h:226-227`（`WEARM_MIRROR_TOOL 1`）、`:238-239`（`WEARM_MIRROR_ELBOW 0`）、`:204-205`（`WEARM_ENABLE_DRAW 0`）、`:79-89`（flash/RAM 预算）、`:93-143`（版本字节历史）**
- 结论：**无需改动**（`TOOL_MIRROR_SUM = 210` 与夹爪 0~180 窗口自洽）。

---

### 2.9 额外发现：`path_core.h:15-19`（未被任务列出，但同类）

```cpp
 b/r/c 的物理行程 0~180 在 moveJointStep() 内硬编码，f 的电气行程 0~180 在 posSetAngle4() 内硬编码
```
- 问题：`b/r/c 0~180` stale（现 r 45~160、c 30~110）。
- 建议：改成“b/r/c 的物理行程窗口（`KIN_*`）在 `moveJointStep() → clampBCR()` 内硬编码”。
- 附注：`path_core.h:28-32 pathCoreSolveJoint()` 的注释“`*clamped` 恒为 false”在当前实现下成立（`getAngleEx` 走的是会被拒绝的路径），但**语义脆弱**：一旦 IK 真的返回 clamped，调用方会立刻开始拒绝点。建议把注释改成“恒为 false；若将来为 true，调用方按不可达处理”。

---

### 2.10 额外发现：`single/meArm/meArm.ino`（git 跟踪的生成物，整体是旧固件快照）

该文件由 `.selfcheck/make_single.ps1` 生成，目前内容仍是**替换前**的固件，整个文件都是旧模型：

| 位置 | 当前行 |
| --- | --- |
| `single/meArm/meArm.ino:1777` | `static const REC POS_HOME = { 40, 0, 0 };` |
| `single/meArm/meArm.ino:1780` | `struct arm arm1 = { 20, 20, 0 };` |
| `single/meArm/meArm.ino:1827` | `sqrt(x²+y²) <= L1 + L2 = 40`（旧 `isReachable`） |
| `single/meArm/meArm.ino:1866-1874` | 旧正解 `x = L1·cos(alpha) + L2·cos(beta)` 等 |
| `single/meArm/meArm.ino:1930` | `double argW = atan2(L1 * sin(delta), L2 + L1 * cos(delta));`（旧反解） |

- 问题：它是一个**整文件级别的遗留产物**；作为“单文件发布版”会发布错误的运动学。
- 建议：重新运行 `.selfcheck/make_single.ps1` 覆盖生成（不要手改）；若暂时不重新生成，至少在 `single/` 下加一个说明文件。**本次审计不改它（只读 + 未在任务清单）**。

---

### 2.11 额外发现：`REVIEW.md:26-36`

- 该段落讨论旧 `atan2` 分支修正，并断言“当前 `constant_and_positions.cpp:27` 是 `arm1 = {20, 20, 0}`” —— **已失效**（现 `:43` 为 `{80, 80, 38}`）。
- 建议：加一句“本节为换模型前的历史记录，参数已变”，或整段归档。

---

## 3. 文档中需要重写的段落（精确行范围）

### 3.1 `D:\dsh1\meArm\README.md`（共 384 行）

| 行范围 | 内容 | 过期点 |
| --- | --- | --- |
| **`:60-131`** | 整个 `## 运动学模型` 小节 | 全节旧模型，需整节重写 |
| `:62-63` | “几何参数：上臂 L1 = 下臂 L2 = 20，下臂末端相对肩部不外伸（`armheight` = 0）……（本例按 20 标定）” | `arm1 = {80,80,38}`，单位 mm |
| `:64-65` | 肩关节在坐标原点 | 现肩轴在 `z = +38` |
| `:71` | “\| b \| 基座回转 \| `θ = 90° − b`（b = 90° 时面朝 +x） \|” | 现 `b = 90` 面朝 **+y**；`θ = b − 90` |
| `:72` | “`α = 90° − r`” | 现 `α = 180° − r` |
| `:73` | “`β = c − 90°`” | 现 `β = c` |
| `:76-82` | “修正历程” | 需并入新模型的历史 |
| `:84-93` | 正解推导 | 全旧公式 |
| `:95-104` | 反解推导 | 全旧公式（含 `b = 90° − atan2(y,x)`、`r = 90° − α`、`c = β + 90°`） |
| `:106-117` | 分支优先级说明 | 分支数/判据随新 IK 变 |
| `:119-123` | “基座 b ∈ [0°,180°] 只覆盖前半空间，所以 `x < 0` 的点反解失败（可达包络是 x[0,40]、y[-40,40]、z[-40,40]）” | 现不可达侧是 **`y < 0`**，包络 `±160` / `[-122,198]` |
| `:125-126` | “`isReachable()` 只检查水平半径 **ρ ≤ L1 + L2 = 40**（容差 1e-9）” | 现为 `d = sqrt(rho²+(z-38)²) < 159` |
| `:128-131` | “各轴硬行程 0~180° 在 `moveJointStep()` 里。初始位姿 `POS_HOME = (40,0,0)`……正好落在可达外边界 ρ = L1+L2 = 40 上” | 现 `POS_HOME = (0,80,118)`、`d = 113.1`（不在边界上）、窗口为 `KIN_*` |
| `:173` | “越界按 0~180° **夹取**（如 `x200` 写 180）” | 现按各轴 `KIN_*` 窗口夹取（`y0 → 45`、`z0 → 30`） |
| `:178` | “高度（默认 **−20.0**，即初始位姿末端 (40,0,0) 再往下 20 单位）……中心（\|x\|,\|y\| ≤ 35，默认 27,0）” | 旧 home + 旧单位 + 旧中心上限 |
| `:237-238` | “纸面：高度默认 −20.0（初始位姿末端 (40,0,0) 往下 20 单位）……图形半宽 6.0（仍限 2.0~12.0）、纸面中心 (27.0,0.0)（仍限 \|x\|,\|y\| ≤ 35.0）、抬笔高度 4.0” | 全部旧尺度 |
| `:372-376` | “b/r/c 的 0~180° 硬编码在 `moveJointStep()`。末端笛卡尔盒子 `rangeLimit` 已恢复（x/y ±40、z −60~60）……z 比真实包络（−20~40）还宽” | 全旧 |

### 3.2 `D:\dsh1\meArm\.selfcheck\README.md`（共 2202 行）

| 行范围 | 内容 | 过期点 |
| --- | --- | --- |
| `:79-89` | 探针总表 | 多行引用旧值：`:79`(probe_rt 5 标定点/1950 组)、`:80`(probe_move “limit = x/y/z 都 ±40”“home `(40,0,0)` 通过”“解出 90/90/90”)、`:82`(probe_axes “配置值必须 = x/y/z 都 ±40”)、`:84`(probe_protocol “0~180 夹取”)、`:85`(probe_pick_place “Δx、Δy 各 ≥5”“A 305/B 334/C 335 轮”“抬升 6.0”)、`:88`(probe_draw “纸面 z=-20.0（= 初始位姿末端 (40,0,0) 往下 20 单位）/半宽 6/中心 (27,0)”“x 21.00→33.00”“每格 ≤0.26 单位”) |
| `:91-112` | `### 当前实测包络（r、c 都放开到 0~180 之后）` 整节 | `:94-97` 代码块 `x[-40,40] y[-40,40] z[-20,40]` / `limit z[-60,60]` / 余量；`:99-100` 四个极端姿态；`:102-103` 逐高度 —— 全旧 |
| `:175-176` | “`isReachable()` 判断 `R > L1+L2` 必须带浮点容差……恰为 40，浮点算出来是 40.000000000000004” | 现门限 159、基准含 `z-38` |
| `:183-185` | “反解要试 4 个候选……而 `rhoP = -rho` 的反向平面分支是 `x<0` 类目标的唯一出路” | “`x<0` 类目标”现在**可达**（不可达的是 `y<0`） |
| `:348` | “`bool posGetHomeAngles(SER *ser)` 对 `POS_HOME`（40,0,0）反解得到（`b/r/c` = 90/90/90）” | 现 `POS_HOME = (0,80,118)`（结论 90/90/90 仍对） |
| `:359-420` | `## 新增绘图功能（铅笔绘制，v1.0.0）` | `:364-368`(“默认纸面高度 `z=-20.0` —— 初始位姿末端是 `(40,0,0)`……1 单位 = 4 mm，即铅笔露出约 80 mm”)、`:378-379`(“`DRAW_LIFT_DZ 4.0`；每次降 0.5”)、`:398`(“`DRAW_JOG_STEP_MIN 0.02 ~ DRAW_JOG_STEP_MAX 0.25` 工作区单位”) |
| `:1187-1272` | 示教点动修复记录 | `:1214`、`:1251` 用“工作区单位”描述步长 |
| `:1345-1435` | 删除范围限制 | `:1423` “`probe_draw`：默认纸面高度 12.0 → **-12.0**” |
| `:1436-1497` | `rangeLimit` 恢复记录 | `:1441`(“x/y 保持 ±40、z 改成 **−60~60**”)、`:1448`、`:1454`、`:1460-1464`(“真实末端范围是 **z ∈ [−20, 40]**（L1 = L2 = 20、`armheight = 0`）”)、`:1473`(“改成 80/80 后包络变成 z ∈ [−80, 160]、x/y ±160，那时盒子也要一起放宽”) |
| `:1498-1670` | v1.6.6 记录（`α=90−r`、`β=c−180`） | `:1525-1526`、`:1536-1537`(“`POS_HOME`：`(20,0,20)` → **`(20,0,-20)`**”)、`:1608-1621`(纸面 -12 → -28，“1 单位 = 4 mm ⇒ 铅笔露出约 32 mm”) |
| `:1671-1756` | RC-robot-hand 记录（`β=c−90`） | `:1696-1712`(“`POS_HOME`：`{40,0,0}` → **`{20,0,-10}`**”、包络 `x[0,40] y[-40,40] z[-40,40]`、`rangeLimit` 三轴 ±40)、`:1713-1719`(纸面/中心 x)、`:1728`(“`probe_axes` §4 标定表（新模型）：`(90,90,90) → (40.00,0.00,0.00)`”)、`:1734`(“`probe_draw`：纸面 z=−20、中心 (27,0)、线 x 21→33”) |
| `:1885-2028` | `上机才能确认项` | `:1898-1899`(“盒子取 x/y ±40、z −60~60：x/y 与包络端值重合（余量 0）……”)、`:1940-1941`(“`DRAW_PEN_Z_DEFAULT **-20.0**`（= 初始位姿末端 `(40,0,0)` 再往下 20 单位）”)、`:2017-2022`(纸面朝向)、`:2037`(“臂长 20”力矩)、`:2044`(“`c` 的绝对角 = `r − c`（两节各 20 单位臂）”——**这句本身也是错公式**，新模型应为 `abs = c`) |
| `:2063-2128` | `single` 单文件整合 | 生成物已过期（见 2.10） |
| `:2151-2202` | `## 初始位姿改回 (40,0,0) —— 上电仍是 x90,y90,z90` | **整节**描述换模型前最后一版：`:2155-2160`、`:2164-2167`(`constant_and_positions.cpp:30` `POS_HOME {40,0,0}`)、`:2168-2174`(“A/B/C 的飞行高度不变（**−7.50 / +12.50 / +29.50**）”)、`:2175-2180`、`:2181`(“注释同步：`meArm.ino:140-142`、`joystick_control.h:58-59`、`draw_control.cpp:67-70`”)、`:2183-2195`、`:2197-2202` |

---

## 4. `.selfcheck/probe_*.cpp`：模型相关的断言清单（**不要改探针**，此处只登记）

### 4.1 确定会失败（A 类）

| 探针断言 | 行 | 断言内容 | 失败原因 |
| --- | --- | --- | --- |
| `probe_axes.cpp:73-81` | `:73-81` | `limit.minX == -40.0 && limit.maxX == 40.0`（y / z 同） | 现为 `±160` / `[-122,198]` |
| `probe_axes.cpp:82-84` | `:82-84` | 三轴余量计算 | 依赖 `±40` 盒子 |
| `probe_axes.cpp:142-146 + :155` | `:142-146` 表、`:155` `err < 0.01` | 5 个旧标定点 `(90,90,90)→(40.00,0,0)` 等 | 新正解给出完全不同的坐标，全部超差 |
| `probe_move.cpp:66 + :71-73` | `:66` `dist(&Pos.rec, 40.0, 0.0, 0.0)`、`:71-73` “home=(40,0,0)、在 limit 内（±40 闭区间边界）……” | 旧 `POS_HOME` 坐标 | 现 home = `(0,80,118)` |
| `probe_move.cpp:158-160` | `:158-160` | “水平超出臂展的点 `isReachable() == false`”，`x = 100` | `x = 100`（`y = 0`）现**可达**，`RHO_MAX` 已是 160 |
| `probe_move.cpp:174-177` | `:174-177` | “limit 配置 = x/y ±40、z -40~40” | 现 `±160` / `[-122,198]` |
| `probe_move.cpp:196-198` | `:196-198` | “`z = 41`（超出 maxZ=40）被点校验拒绝” | `41` 现在**在盒内且可达**，断言取反 |
| `probe_move.cpp:199-200` | `:199-200` | “`x = 41`（超出 maxX=40）被点校验拒绝” | 同上 |
| `probe_rt.cpp:3-9` | 文件头惯例说明 | 旧 `alpha/beta/theta` 与 `rho∈[0,40]` | 说明过时（随后断言即失败） |
| `probe_rt.cpp:38-49` | `:38-43` 表、`:49` `check("anchor", …)` | 4 个旧锚点 `{90,90,90,40.00,0.00}` 等 | 新正解不同，4/4 失败 |
| `probe_rt.cpp:54-73` | `:54-60` 表、`:72-73` `good = ok && err < 0.05` | 5 个旧反解用例 `{20,0,20}→(90,90,90)` 等 | 5/5 失败 |
| `probe_rt.cpp:79-83` | `:79-83` | 3 个端点用例 `{(20,0,20),(20,0,-20),(0,0,-40)}` | 失败 |
| `probe_protocol.cpp:71-77` | `static void probeTravel(...) { (void)axis; *lo = 0.0; *hi = 180.0; }` | 自称“直接取固件自己的 `servoLimit`”，实际硬编码 `0/180`（该符号已不存在） | r/c 现为 `45~160`/`30~110`，期望值生成器整体错，级联失败 |
| `probe_protocol.cpp:212-221` | `"x20,y0,z40"` → 期望 `want2 = probeClampToTravel(1, 0.0)` = 0 | 固件把 r 夹到 **45** | 不符 |
| `probe_protocol.cpp:263-265` | `"x20,y10,z40"` 同族 | 同上 | 不符 |
| `probe_protocol.cpp:291-293` | 同族 | 同上 | 不符 |
| `probe_protocol.cpp:309` | 期望 `12.5` | 同上 | 不符 |
| `probe_protocol.cpp:319-321` | 同族 | 同上 | 不符 |
| `probe_protocol.cpp:345-352` | 越界表 `"x200,y0,z40"` / `"x20,y-90,z40"` / `"x20,y0,z999"` / `"x0,y200,z40"` | 按 `0/180` 生成期望 | 不符 |
| `probe_protocol.cpp:445-449` / `:470-474` / `:518-522` | 同族（含 `10.0`、`12.5`） | 同上 | 不符 |
| `probe_protocol.cpp:679` | `angWant[a] = probeClampToTravel(a, v[a]);`（随机 INV8） | 同上 | 不符 |
| `probe_draw.cpp:750-753` | `check("先回初始位姿再走向第一个目标点", has && fabs(first[0]-40.0) < 1.0 && fabs(first[1]) < 1.0 && fabs(first[2]) < 1.0, d)`（文案 `:750` “抬笔起点 (%.2f,%.2f,%.2f)（初始位姿 40,0,0）”） | 硬编码旧 home 坐标 | 现 home `(0,80,118)` |
| `probe_pick_place.cpp:182` | `check("取放点与整条飞行路径可用", ok, d)` | 复合条件含 `pointUsable(src)`/`pointUsable(dst)`（负 y 点 + 旧尺度） | 3 个物体全失败 |
| `probe_pick_place.cpp:187-189` | `check("启动 A 返回 0", rc == 0, d)`（`rc = pickPlaceStart(PICK_OBJECT_A)`） | 固件返回 **-4** | 不符 |
| `probe_pick_place.cpp:190-191 / :195-196 / :207-260 / :257 / :275-279 / :287-289 / :294` | 序列跑完 / 回到 idle / 张开到位 / 最终回 `POS_HOME` / 阶段顺序 / 逐轮不变量 | 全部依赖 `:187` 启动成功 | 级联失败 |

### 4.2 依赖默认值、目前通过但会被“正确修复”打破（联动项）

| 位置 | 断言 | 说明 |
| --- | --- | --- |
| `probe_draw.cpp:366-368` | “默认纸面 z=-20 / 半宽 6 / 中心 (27,0)” | 只要改 `DRAW_PEN_Z_DEFAULT`/`DRAW_HALF_DEFAULT`/`DRAW_CENTER_*` 就失败 —— **必须与代码同步改** |
| `probe_draw.cpp:412-414` | `!drawSetHalfSize(1.0) && !drawSetHalfSize(13.0)`、`!drawSetCenter(100.0, 0.0)` | 改 `DRAW_HALF_MIN/MAX`、`DRAW_CENTER_LIMIT` 后 `13.0` / `100.0` 会变成合法值 → 断言翻转 |
| `probe_draw.cpp:877-878` | `protoHandleLine("o100,0") == PROTO_RES_DRAW_REJECTED`、`p-20 / n6` 默认 | 同上 |
| `probe_draw.cpp:452/:457/:459-460/:465/:584` | 硬编码 `-20.0` / `27.0` / `±6.0` / `21→33` | 同上（数值型联动） |
| `probe_pick_place.cpp:148` | `worstClear >= 5.0`（“放置点不会蹭到别的物体（离别人的初始点 >=5）”） | 只依赖表格自身几何，**当前通过**；表格按 mm 重标后阈值应改 ~`20.0`，否则形同虚设 |
| `probe_pick_place.cpp:151-159` | 注释 “`sqrt(rho²+z²) ≤ 40`”、`clearDz = lift / 5.0` | `lift/5` 只是复刻 `PICK_CLEAR_DZ/PICK_LIFT_DZ = 1/5` 的比例，改宏后仍自洽 ✓，但注释要改。`pickPlaceApproachDz()` 返回的是 **`PICK_LIFT_DZ`**（见 `pick_place.h:79-81`），不要与 `PICK_CLEAR_DZ` 混淆 |

### 4.3 不受影响（自洽，无需改动）

- `probe_axes.cpp:125 + :135`（单轴满行程扫描 `0~180` 正解都要成功）—— 当前 `recFromServo()`（`constant_and_positions.cpp:156-179`）只在空指针/非有限值才失败，故仍通过；但 `:12-15` 的 `BMIN/BMAX/RMIN/RMAX/CMIN/CMAX = 0/180` 已**不再是行程真值**，建议改成读 `KIN_*`。
- `probe_joystick.cpp:303-310`（4000 轮随机推杆不出 `limit`）—— 新盒子更宽，通过。
- `probe_joystick.cpp` 其余（`:78-181` 轴→关节映射、`:197/:220` 死区/迟滞、`:250/:387` 步长缩放、`:328-404` 标定）—— 全部是关节角/模拟量，与运动学无关。
- `probe_button.cpp:295-311`（回中后关节角 == `POS_HOME` 反解值）—— 用 `posGetHomeAngles()`，自洽。
- `wearm_ino_test.cpp:113-121 / :131 / :141 / :147 / :156 / :173-181` —— 全部走 `posGetHomeAngles()`/`recFromServo()`，`TOOL_MIRROR_SUM == 210.0` 仍成立。
- `probe_draw.cpp:917-924` —— 有 `pr == 0 || pr == -3` 的容错，可能仍通过（未验证）。
- `probe_rt.cpp:97-143`（round-trip 统计 `bad == 0 && worst < 0.2 && branchBad == 0`）与 `:145-192`（残差统计）—— **不确定**：窗口内组合 `r∈[45,160]/c∈[30,110]` 看起来仍满足 `d < 159`，窗口外由 `if (limClamped) { clampedN++; continue; }` 跳过；需要实跑确认（本次为只读审计，未构建）。
- `probe_protocol.cpp:90` 注释“换算成 20mm 臂长的坐标误差”、`:100-101 PROBE_RAND_LO -40.0 / HI 220.0`（角度随机范围）—— 前者文字 stale，后者无害。

---

## 5. 分类

### (A) 确定的 bug / 自相矛盾（必须改代码或注释，否则功能就是坏的）

1. **`pick_place.cpp:94-98` / `:112-116`**：`PICK_SRC[1].y = -12`、`PICK_DST[0].y = -18`、`PICK_DST[2].y = -16` 均在不可达半空间 → **三个物体全部无法启动（返回 -4）**。
2. **`pick_place.cpp:64-69 / :143-144`**（`PICK_LIFT_DZ`、`PICK_CLEAR_DZ`、`PICK_BACK_DZ`、`PICK_TRAVEL_STEP`、`PICK_HEIGHT_*`、`PICK_PATH_SAMPLE_*`）与 **`pick_place.h:5/:17`**：旧尺度常量，与 mm 尺度相差 4×；采样上限会静默劣化路径校验。
3. **`pick_place.cpp:215-222 / :302-313 / :324-325 / :635-636 / :693-695`**：`s_riseAtSrc` 的存在理由（旧 home 在满伸边界）已消失 → 死代码 + 错误注释。
4. **`draw_control.cpp:73` 及 `:71-74`、`:78-80`**：默认图形中心 `y = 0` + 半宽 `6` → 默认图形**一半在不可达区**，`validatePath()` step 1（`:498-499`）必然拒绝 → **默认绘图完全不可用**。
5. **`draw_control.cpp:91-99 / :107-112 / :115-122`、`liftZFor()` `:451-461`**：旧单位的线速度/步长/抬笔量/采样，物理含义已变。
6. **`meArm.ino:37-38`**：旧舵机→几何公式（`α = 90°−r`、`β = c−90°`）与新模型矛盾。
7. **`meArm.ino:115-128`、`:133-149`**：安全边界与坐标模型整块旧模型（含 `±40`、`L1=L2=20`、`POS_HOME (40,0,0)`、可达侧 `x >= 0`）。
8. **`meArm.ino:186-190`（潜伏）**：肩/肘的 `180.0 - angle` 镜像和与该关节的 `KIN_*` 窗口不符（应为 `205 − r`、`140 − c`），与 `weArm_config.h:214-216` 的 `(min+max)-logical` 声明矛盾；当前被默认 `0` 掩盖。
9. **`serial_protocol.cpp:705-708`**：注释坚持“b/r/c 上下限 0..180”，并给出会被实机证伪的例子（`y0 → 0` 实为 `45`）。
10. **`move.cpp:7 / :19 / :118`**、**`path_core.h:17`**：`0~180` 硬编码的过期描述（代码已正确）。
11. **`joystick_control.h:40`、`:52-67`**：旧行程 + 整块旧坐标模型注释。
12. **`draw_control.h:92-97`**：默认区域、中心、包络三者都与代码/模型不符。
13. **探测链**：`probe_protocol.cpp:73-77` 的 `probeTravel()` 硬编码 `0/180`（自称读固件），是它全部失败的根；`probe_axes.cpp:73-84/:142-155`、`probe_move.cpp:66/71-73/158-160/174-177/196-200`、`probe_rt.cpp:38-83`、`probe_draw.cpp:750-753`、`probe_pick_place.cpp:182/187+` 的期望值全部对应旧模型 —— 这些**不是探针的错**，是固件换代后探针未同步（按任务要求：只登记，不修改）。
14. **文档**：`README.md` §`:60-131`、`:173`、`:178`、`:237-238`、`:372-376`；`.selfcheck/README.md` §`:79-112`、`:175-185`、`:348`、`:359-420`、`:1187-1272`、`:1345-1497`、`:1498-1670`、`:1671-1756`、`:1885-2028`、`:2151-2202`；`REVIEW.md:26-36`。
15. **`single/meArm/meArm.ino`**：整文件旧固件快照（`:1777`、`:1780`、`:1827`、`:1866-1874`、`:1930`），作为发布物会发布错误运动学。

### (B) 只是需要在实机上重新标定 / 复验（不是逻辑错误）

1. **`WEARM_MIRROR_BASE = 1`**（`weArm_config.h:223-224`）：基座正方向在新模型里翻转（`x = -R_h·sin(b-90)`），默认值可能已错；用 `mB` 运行时开关实测后回写。
2. **`WEARM_MIRROR_SHOULDER / ELBOW`**（`:229-239`）：默认 `0` 与“上臂/下臂按正向装配”判断一致，但换模型后要上机复验一次。
3. **`draw_control.cpp:71 DRAW_PEN_Z_DEFAULT`** 的实际纸面高度：`-20.0` 在几何上可行（底盘顶面下 20 mm），但台面/纸厚要实测（`p<z>`），建议以台面 `z ≈ -38` 为参照。
4. **`draw_control.cpp:128-130 DRAW_JOG_INVERT_X/Y/Z`**：动点动时 x/y/z 的左右/前后是否反了（基座方向翻转）。
5. **`joystick_control.cpp` 的方向语义**（`JOY_DIR_NEG` / `JOY_RAW_SLOT` / A0~A3 映射 / `tx` 符号）：映射本身与模型无关，但“推左往哪边走”要按新模型复验。
6. **速度类**：`PICK_RATE_*_DPS`、`JOY_STEP_MIN_DEG`/默认 `stepSize`、`DRAW_MAX_DPS 70` —— 臂长 ×4 后同样角速度的末端线速度也 ×4，手感需重标（我在 §2 给出的 `DRAW_V_MAX 12.0` 等是“保持旧物理速度”的建议值，不是唯一解）。
7. **`KIN_BASE/RARM/FARM` 窗口本身**（`constant_and_positions.h:72-77`）：它们是照参照仓库抄的，必须在本机三舵机上逐一撞限位确认真值（尤其 `r 45~160`、`c 30~110`）。
8. **新取放点的实际位置**（§2.1 给的 `PICK_SRC/PICK_DST` 是几何上可达的换算值，但物体实际摆在哪要按台面layout 决定）。

---

## 6. 建议的改动顺序（供后续实施参考，本次未执行）

1. `pick_place.cpp:94-116` 换表 + `:64-69/:143-144` 换常量（恢复取放功能）。
2. `draw_control.cpp:71-84/:91-122` 换常量 + `draw_control.h:92-97` 换文档（恢复绘图功能）。
3. `meArm.ino:115-149`、`joystick_control.h:40-67`、`move.cpp:7/19/118`、`path_core.h:17`、`serial_protocol.cpp:705-708` 注释同步。
4. `meArm.ino:186-190` 镜像式改窗口通用式（即使当前默认不触发）。
5. 探针同步（改动 1/2 会连带 `probe_draw.cpp:366-368/:412-414/:877`、`probe_pick_place.cpp:148`）。
6. `README.md` / `.selfcheck/README.md` 文档重写；`single/` 重新生成。
7. 上机标定项（B 类）。
