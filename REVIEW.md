# 代码审查与清理报告

范围：`D:\dsh1\meArm` 多文件工程全部 9 个编译单元 + `.selfcheck/` 探针与脚本，
并对照 `others/` 下的参考实现。目标是三件事：全量审查 + 冗余清理、修掉夹爪在
取放过程中会松开的毛病、把自研舵机驱动换成 Arduino 官方 `Servo` 库并让它装得进
Uno 的 flash。

> **历史标注（v1.6.7）**：本报告写在**运动模型整体替换之前**。v1.6.7 把整套运动逻辑换成了
> 参照仓库 `Fifiiiii-123/MeArm@a7d2ee1` 的那一版。因此下面凡是拿"两节各 20 单位 /
> 1 工作区单位 = 4mm / `β = c − 90` / `θ = 90 − b` / `x ≥ 0` / 末端盒子 ±40 /
> `POS_HOME = (40,0,0)` / `ikBranch()` 四候选 / `normRevAngle()`"说话的段落，都只是
> **v1.6.7 之前的历史**（运动模型已被整体替换，见 `README.md` 的「运动学模型」与
> 「v1.6.7 运动逻辑替换」两节）。历史结论一律保留、不删除；受影响的段落已就地标注。
> 容量数字与探针结论同样要在换模型后重测（§6）。

## 0. v1.6.7 之后的运动模型（本报告的历史前提已变）

以 `constant_and_positions.{h,cpp}` 为准：

- 几何：原点在**底盘顶面中心**，z = 0 是底盘顶面（肩轴在 z = +38），单位 mm；
  `L1 = 38`（底盘高）、`L2 = 80`（上臂）、`L3 = 80`（前臂），`arm1 = { 80, 80, 38 }`。
- 角度映射：`theta = b − 90`（自 +y 量起，所以 `b = 90°` = 正前方 +y）、
  `alpha = 180 − r`、`beta = c`。
- 正解：`R_h = L2·cos(alpha) − L3·cos(alpha + beta)`、`x = −R_h·sin(theta)`、
  `y = R_h·cos(theta)`、`z = 38 + L2·sin(alpha) − L3·sin(alpha + beta)`。
- 反解：`d = sqrt(rho² + (z−38)²)`（`rho = sqrt(x²+y²)`），要求
  `d ∈ [|L2−L3|+1, L2+L3−1] = [1, 159]` mm；
  `beta = acos(clamp((L2²+L3²−d²)/(2·L2·L3), ±1))`；`phi = atan2(z−38, rho)`；
  `alpha = phi + atan2(L3·sin(beta), L2 − L3·cos(beta))`；`R_h < 5` 视为奇异；
  `theta = −atan2(x, y)`。**单分支，没有镜像候选，也没有 `normRevAngle()` 折回。**
- 行程窗口：`b 0~180`、`r 45~160`、`c 30~110`（`KIN_*` 宏）；钳到窗口后用钳后角度重跑
  正解，**水平残差 `sqrt(dx²+dy²) > 5.0` mm 判失败**（与参照工程
  `Task5_LineN2_TeachSmooth.ino:237-239` 逐点等价；z 偏差大时只留一条日志提示）。
- `POS_HOME = (0, 80, 118)`，对应 `(b,r,c) = (90,90,90)`：同一个物理姿态，但从旧模型
  贴在可达外边界 ρ = 40 上的 `(40,0,0)` 挪到了工作空间中央（`d ≈ 113.1mm`）。
- 可达半空间是 **y ≥ 0**（旧模型是 x ≥ 0，整整差了 90°）；工作空间盒子
  `x/y ±160`、`z [−18, +171]`（**实测可达竖直带**，实测边界 `[−18.5,+171.5]`）；
  几何极限是 `38 ± 160 = [−122, +198]`，但真正的上下限由关节行程给出，见下一条。
- 旧模型被换掉的两个硬理由：旧 `b = 90` 指向 +x，与"x = 左右、y = 前后"的实物约定
  自相矛盾；旧 `POS_HOME = (40,0,0)` 正好贴在可达外边界上，取放只能写特例。

本报告里 §1.1（把两节臂长用反的修复）、§1.6（`normRevAngle()` 相关的死代码清理）、
§4 的 IK↔FK 互逆、§5 的 ±40 范围盒与"透镜"、以及 §6 的全部数字，针对的都是**换模型前的
代码**。

## 结论摘要

| 项目 | 结果 |
| --- | --- |
| 默认配置实编 | **32026 B / 1481 B**（余 **230 B**，Arduino IDE 原样参数） |
| 是否还需要 `-mrelax` | **不需要**（`-mrelax` 从来不是 IDE 能给的选项，见 §3） |
| 编译告警 | 9 个编译单元在 `-Wall -Wextra -Wshadow -Wconversion` 下**零警告** |
| 自检 | 11 个探针 **ALL PASS**（`run_all.ps1` 退出码 0） |
| 修掉的真实缺陷 | 5 处（§1），另有 2 处"活函数里的死代码"与 1 处自相矛盾的统计口径 |
| 认定**不是**缺陷的 | 3 处子代理误报（§4） |

## 1. 真正修掉的缺陷

### 1.1 逆解把两节臂长用反了（潜伏缺陷，零字节修复）

> **（v1.6.7 之前的历史，运动模型已被整体替换，见 README 的对应章节）** —— 下面描述的
> `argW` 操作数对调、`arm1 = {20, 20, 0}`、四候选与 0.05° 门限都属于旧模型；
> v1.6.7 之后 `constant_and_positions.cpp` 的逆解整段被换成参照仓库的单分支 `solveIK`，
> 这处历史修复的对象已不存在。

`constant_and_positions.cpp:177`：

```cpp
double argW    = atan2(L1 * sin(delta), L2 + L1 * cos(delta));   // 修复后
// 原来是 atan2(L2 * sin(delta), L1 + L2 * cos(delta)) —— 两个臂长对调了
```

推导：平面两连杆的末端矢量是 `R = e^{i·beta}·(L1·e^{i·delta} + L2)`，所以
`beta = phiBase - atan2(L1·sinδ, L2 + L1·cosδ)`。

**为什么今天看不出问题**：当 `L1 == L2` 时两式恒等（因为 `atan2(sinδ, 1+cosδ) ≡ δ/2`），
而当前 `constant_and_positions.cpp:27` 是 `arm1 = {20, 20, 0}` —— 两节臂一样长。
**为什么必须修**：`constant_and_positions.h:103` 明确邀请使用者"把某一节臂改长"。
数值验证：取 L1=20、L2=10、α=90°、β=0°，旧式算出 β=36.87°（真值 0），回代残差 14.1°
远超 0.05 的接收门限 ⇒ 四个候选全被否掉 ⇒ `getAngleEx()` **永远返回 false**，
机械臂彻底不动。修复只是交换两个操作数，不占 flash。

### 1.2 取放开始前不把夹爪归位（"夹"会变成"松"）

`pick_place.cpp:514` 新增：

```cpp
(void) posSetAngle4(PICK_TOOL_OPEN_ANGLE);   // 紧接着 beginMove(...SRC_APPROACH...)
```

（**v1.6.7 之前的历史，运动模型已被整体替换，见 README 的对应章节**：宏名
`PICK_TOOL_OPEN_ANGLE` 现在是 `TOOL_ANGLE_OPEN`；"启动前先把爪子归位"这条缺陷与
修复思路本身与运动模型无关，仍然有效。）

触发路径：上一轮取放序列若在夹持阶段被打断（或操作者用串口 `'S'` 关过爪），
`angle4` 会停在 60°（名义合拢）或 45°（夹紧）。此时启动新序列，`DESCEND_SRC` 会
**闭着爪子**压到物体上，紧接着 `CLOSE_TOOL` 把爪子从 45° 张到 60° —— 本该"夹住"
的动作实际在"松开"，物体掉落或根本没被夹到。

### 1.3 `beginDwell()` 无钳位的窄化转换

`pick_place.cpp` `beginDwell()`：`unsigned long` → `unsigned short` 直接强转。
感谢 `PICK_DWELL_MS = 600` 今天撞不上，但 `durationMs()` 的 `PICK_MAX_SEG_MS`
一旦被调大或复用到别处就会静默截断。改成先钳到 60000 再转。

### 1.4 阶段名数组不查边界

`pick_place.cpp:565-573` `pickPlaceStageName()`：`s_stage` 是 `int8_t`，负值转
`unsigned` 会回绕成一个巨大的下标，而 `PICK_STAGE_NAME[]`（`pick_place.cpp:160`）
只有 12 项。这是探针会直接调用的对外 API，越界读会返回垃圾字符串。改成越界返回 `"?"`。

### 1.5 `speedStepDown()` 名不副实

`constant_and_positions.cpp:496-499`：档位 0/1/2 = 慢(0.5°/80ms)/中(1°/40ms)/
快(2°/20ms)，而原式 `(cur + 1) % 3` 除了 2→0 之外**全都在升档** —— 函数名和注释
（"循环降一档"）都在骗人。改为 `(cur + 2) % 3`。

**注意**：grep 全工程（`*.cpp`、`*.ino`、`.selfcheck/`、`single/`、`others/`）确认
**这个函数当前没有任何调用者**（只有 `others/` 里的旧单文件版还带着它的定义）。
这里只是让名字与行为一致，不是在修一个会跑到的路径。

### 1.6 活函数里的死代码

`constant_and_positions.cpp` `getAngleEx()` 末尾原本有两行"容差内的微小负角归零
（避免把 -0.0 传给舵机）"。实际上 `bestR`/`bestC` 只可能来自 `rDeg`/`cDeg`，而它们
都过了 `normRevAngle()`（`constant_and_positions.cpp:179-184`，值域恒为 `[0,360)`），
再经 0~180 的行程判据筛选，永远落不进 `(-1e-6, 0)` —— 连 `-0.0` 都到不了
（`-0.0 < 0` 本身是 false）。删掉这两行，顺带删掉因此变成纯中转的
`angle1/angle2/angle3` 三个局部量，`getAngleEx()` 直接写
`pos1->ser.angle1 = bestB; angle2 = bestR; angle3 = bestC;`。

> **（v1.6.7 之前的历史，运动模型已被整体替换，见 README 的对应章节）**：`normRevAngle()`
> 与这里的"0~180 行程判据筛选"都属于旧的四候选反解；新模型下这些中转量与
> `normRevAngle()` 已随整段反解一起消失。

### 1.7 过点统计的分母自相矛盾

`draw_control.cpp:704`：原为 `s_hitTotal = s_isTeach ? s_ptCount : 0;`，
但 `pathTick()` 里的"过点统计"循环**不区分**是不是示教任务，而内置图形走
`buildShapePath()`（`draw_control.cpp:307`）同样把 `s_ptCount` 设成顶点数（2/3/4）。
于是画内置图形时会报出"命中 4 / 共 0"这样一对自相矛盾的数。两份数据同源
（`s_ptArc` 是同一个规划器填的），分母改回 `s_ptCount`。绘制完成的打印本来就只在
示教任务里给出这对数，所以这次改动不动任何输出。

## 2. 冗余清理：哪些真的省了 flash

判断标准只有一条：**改完再编，`Program:` 数字动了才算省**。

### 2.1 省下来了

**`draw_control.cpp` `validatePath()` 曲线分支复用共用校验（约 −300 B，最大一项）**
原来是一套自写的 `SAMPLE_PER_SPAN 24` 逐点采样：`curvePoint()` + `pointOk()` +
自己三轴的"分支跳变"比较。这与 `path_core.h` 的 `pathCoreSegmentOk()` 完全重复，
只有采样密度不同。改为直接复用共用函数（按子弦 `SUB_PER_SPAN 6` 切分，每子弦再按
0.5 mm 一步采样、最多 96 点）—— 冗余消失的同时**采样反而更密、校验更严**。

**`serial_protocol.cpp` 回复层（约 −144 B）**

- 删掉 `protoWriteInt()` 与 `protoReply()` 里的 `'#'` 替换分支。它唯一的用户是
  `'O'`/`'S'`，作用是把编译期常量 150/60 回显成 `"OK 150"`/`"OK 60"`；两个调用点
  改用 `R_OK()`，输出文本不变（`probe_protocol.cpp:125`/`:535` 断言的是
  `P=1 B=1 D=1`）。
- 删掉 `s_rOkN`（`"OK #"`）与 `R_OKN()`。
- `R_FEATURE()` 是个被展开两次、每次约 20 个 `protoPut` 的宏，收敛成一个出线函数
  `protoReplyFeature()`，两处调用改成函数调用（`#else` 分支给一个空宏）。
- 开机横幅 `s_rBoot` 去掉没有任何处理器的 `123 `、`kK `、`x,y,z` 三段。
- grep 确认：没有任何探针断言开机横幅或旧的 `OK 150`/`OK 60`。

### 2.2 改了但**一字节都没省**（重要教训）

删掉 `protocol_constants.cpp` 里无人引用的 `protoAxisGetLimit(int, double*, double*)`
与 `protoAxisJoint[]`，以及它们在 `protocol_constants.h` 里的声明：
**实测 32586 → 32586，零变化**。链接器的 `-Wl,--gc-sections` 早就把它们丢掉了。

> **教训：在这个工程里删"没被引用的符号"不省 flash，只有改"活的代码"才省。**

同理，下面几种"尺寸游戏"全部实测无效或更差，不要再试：

| 尝试 | 结果 |
| --- | --- |
| 给 `path_core.h` 的 `pathCoreSegmentOk` 加 `noinline` | 32586 → 32586，无变化 |
| 给它加 `noinline, noclone` | 32586 → 32642，**更差 56 B**（LTO 的 `.constprop` 克隆是净赚） |
| `-ExtraDefs '-fno-inline'` | **35144 B**，大幅变差 |

### 2.3 已经重复的代码（查过，没有可收割的）

`path_core.h` 已经承载了共用的 `pathCoreSolveJoint()` / `pathCorePointOk()` /
`pathCoreSegmentOk()` / `pathCoreSetJoints()`；`draw_control.cpp:253-266` 只留了两个
极薄的 `static inline` 转发（`pointOk()`、`setJoints()`）。符号表里 `dist3` 只有一份
（218 B）。**取放与绘图之间已经不存在重复的运动学代码。**

## 3. 舵机驱动：换用 Arduino 官方 `Servo` 库

`servo_drive.cpp` 现在只有 34 行，是官方库的薄封装：`Servo::attach()` 绑定、
`Servo::write()` 输出（保持原来的 `(int)` 截断 + 夹取 0~180 语义）。脉宽映射与
20ms 帧时序**全部交给官方库**，本文件不再持有任何时序状态；旧自研驱动的
`servoDriveTicksForDeg()` / `servoDriveStep()` 已随驱动一起删除，`probe_servo_drive`
改测这个封装的行为，PC 自检加了一个 `.selfcheck/mock/Servo.h` 以便脱离真实库链接。

**换完之后一度装不下**：默认配置冲到 **32586 B**，比 Uno 的 32256 B 上限多 330 B。

`avr_build.ps1` 里加了 `-Relax` 开关（编译加 `-mrelax`，链接另外加
`-mrelax -Wl,--relax`）可以压到 32080 B —— 但**这条路走不通**：Arduino IDE 用的
`%LOCALAPPDATA%\Arduino15\packages\arduino\hardware\avr\1.8.8\platform.txt` 里
`compiler.c.elf.flags` 并不含 `-mrelax`，`compiler.c.elf.extra_flags` 与
`compiler.ldflags` 都是空的。而 `README.md` 写明使用者是**直接在 Arduino IDE 里打开
仓库根目录（或 `single/meArm/`）**，所以必须以 IDE 原样参数为准。

于是靠 §2 的清理把空间腾了出来：**32586 → 32026 B**，余 230 B，
**不再需要 `-mrelax`**。`-Relax` 开关保留在脚本里，作为分析工具而非构建前提。

## 4. 查过但确认**不是**缺陷（避免白改）

1. **曲线分支 `if (nu > 1.0) nu = 1.0;`**（`draw_control.cpp` `pathAdvance()`）
   被怀疑会让"弧长与笔尖脱同步、PATH 提前报完成"。逐行核对后否定：进入循环前
   `step` 已被 `s_totalLen - s_done` 截断，而 `s_totalLen` 是 64 点/段的弦长积分、
   `du = tryStep / |dP/du|` 用的是解析速度，两者误差在万分之一量级；这个钳位是
   吸收数值过冲的护栏，不是失步。
2. **`rangeLimitConfig()` 里的 "例如 minZ = 60、maxZ = -60"** 只是举了一个"上下限
   写反"的例子，不是残留的 ±60 旧模型（当时真正的范围盒是 ±40，
   `constant_and_positions.cpp:32-40`）。**（v1.6.7 之前的历史，运动模型已被整体替换，
   见 README 的对应章节）**：换模型后盒子是 `x/y ±160`、`z [−18, +171]`（实测可达竖直带），
   但"这只是配置函数的举例、不是说有 ±60 旧模型"这条判断本身仍然成立。
3. **`p`/`n`/`o` 绘图标定命令"够不着"** —— 误报。`serial_protocol.cpp:671` 会调
   `protoHandleDrawCalib()`，进而调 `drawSetPaperZ()` / `drawSetHalfSize()` /
   `drawSetCenter()`，链路是通的。

另外逐项确认无缺陷的：`millis()` 回绕（都是无符号减法）、除零（`durationMs()`
有 `rateDps > 0` 门、`s_stageMs > 0` 门）、缓冲区越界（无下标写入）、未初始化字段、
状态机（12 个阶段各只有一个后继 + 一个兜底中止，`advanceStage()` 每轮最多跑一次）、
IK↔FK 互逆（**（v1.6.7 之前的历史，运动模型已被整体替换，见 README 的对应章节）**：
当时是 `r = 90 − α`、`c = β + 90` 与其反式严格互逆，镜像分支也能映回同一个 `(x,y,z)`；
换模型后对应关系变成 `r = 180 − alpha`、`c = beta`，镜像候选随单分支反解一起消失，
正解/反解仍严格互逆）、行程判据 `b < minB−EPS || b > maxB+EPS` 与
`>= minR−EPS && <= maxR+EPS` 的等价性（同一个 `ANGLE_EPS`）。

**有意保留的一处"不精确"**：`s_liveBits[0]=0x0F` / `s_fastBits[0]=0x0E` 仍给早已
删除的 `'1'/'2'/'3'`（档位直达键）留着位。保留这三个位没有害处 —— 这几个字符因此
照常进入派发流程、收到一个干净的 `ERR`；清掉只会让"忙时"的回复从 `ERR` 变成 `BUSY`。
已在 `serial_protocol.cpp` 的表注释里写明。

## 5. 文档与注释订正

- `constant_and_positions.cpp:32-38`：范围盒 ±40 只是给外部调用者的**粗**范围；真实
  可达集（β=c−90 ⇒ α,β ∈ [−90,90] ⇒ 两节臂的 cos 都 ≥ 0 ⇒ x ≥ 0）是一个**透镜**，
  既不是圆盘也不是球。**（v1.6.7 之前的历史，运动模型已被整体替换，见 README 的
  对应章节）**：换模型后可达半空间是 y ≥ 0、盒子是 `x/y ±160`、`z [−18, +171]`（实测可达竖直带） ——
  "盒子只是粗范围、真判据在反解里"这条结论仍然成立。
- `constant_and_positions.cpp` `isReachable()` 注释：删掉"机械臂的实际可达空间是一个球"
  的错误说法，换成与模型无关的必要条件（末端到肩的距离 ≤ L1+L2 ⇒ 水平半径 ≤ 40）。
  **（v1.6.7 之前的历史…）**：换模型后 `isReachable()` 已经是**三维**判据
  （`d = sqrt(rho² + (z−38)²) ∈ [1,159]` mm 且 y ≥ 0），不再是"水平半径 ≤ 40"。
- `constant_and_positions.h:39-50`：`RADtoDEG` 的 π 截断论证原先挂在一个已经不存在的
  1e-9 行程容差上（现在是 `ANGLE_EPS 1e-6`）；端点示例也写错了 ——
  `(0,0,-40)` 的解是 **`r=180, c=0`**，不是 `c=90`（`probe_rt.cpp:82` 就是这么断言的）。
  **（v1.6.7 之前的历史，运动模型已被整体替换，见 README 的对应章节）**：这个端点示例与
  `probe_rt.cpp` 的断言都是旧模型的；探针期望值待更新，见 `.selfcheck/README.md` 的警告。
- `pick_place.cpp` `beginMove()` 里"保证任何关节都不超过设定角速度"是**假的**：
  `durationMs()` 的 4000 ms 上限会把实际速率顶上去（180° 按 25°/s 要 7200 ms，被夹到
  4000 ms 就成 45°/s）。改成如实描述这是"宁可快一点也不要一段走 7 秒"的取舍。
- `README.md` 取放小节：`PICK_APPROACH_DZ` 41.0（旧值）→ **30.0**；夹爪补上三段角
  （开 150 / 名义合拢 60 / 夹紧 45，即"再往里压 15°"）与**两步合爪**（先 60°/s 合到
  名义角，再 12°/s 慢慢压到夹紧角）；停顿 400 → **600** ms；阶段数 11 → **12**
  （多出 `squeeze-tool`）；`−3` 的**两个**来源。
  **（v1.6.7 之前的历史，运动模型已被整体替换，见 README 的对应章节）**：
  `PICK_APPROACH_DZ` / `PICK_GRASP_Z` 这两个宏名在 v1.6.7 已不存在，现在的抬升高度是
  `PICK_LIFT_DZ = 50.0` mm（`pickPlaceApproachDz()` 返回它），点位表也按新尺度整体重标。
- `serial_protocol.h`：删掉"按 `servoLimit` 夹取"（该配置表已删除，现在是
  `protoApplyAngles()` 里硬夹 0~180）；回复码说明里删掉已不存在的 `OK #`。
- `protocol_constants.h`：原来说"每个关节的角度范围唯一真值是 `TOOL_ANGLE_MIN/MAX`" ——
  这两个宏是**夹爪**(angle4/f) 的 0~180，不是三个关节 b/r/c 的
  （**v1.6.7 之前**三者数值碰巧都是 0~180；换模型后 b/r/c 的窗口是 `KIN_*` 的
  0~180 / 45~160 / 30~110，夹爪与三个关节再也不一样了）；
  `'1'/'2'/'3'` 的说明改成如实记录"已删除、数字位留空"。
- `weArm_config.h:66-88` 与 `.selfcheck/README.md`：容量表按本轮实测刷新（§6），
  并记录本轮相对上一版的行差与原因。**（v1.6.7 之前的历史）**：§6 的数字是旧 40mm
  模型下的实测，换模型后待重测。

## 6. 验证证据

> **v1.6.7 已重测**（见本节末尾的 v1.6.7 表）。下表是**旧 40mm 模型**下的实测，
> 只作历史对照 —— 运动逻辑整体替换、取放点位与绘图参数按新尺度重标之后，
> 所有容量数字都比 v1.6.6 时代大 180 B 左右。

`avr_build.ps1`（Arduino 1.8.8 核心 + avr-gcc 7.3.0，**不含 `-mrelax`**）：

| 配置 | Program | Data |
| --- | --- | --- |
| 取放 + 按键 + 绘图（默认） | **32026 B（97.7%），余 230 B** | 1481 B（72.3%） |
| 取放 + 绘图（关按键） | 28434 B（86.8%） | 564 B（27.5%） |
| 取放 + 按键（关绘图） | 19628 B（59.9%） | 1231 B（60.1%） |
| 绘图 + 按键（关取放） | 29048 B（88.6%） | 1447 B（70.7%） |

模块之间共用 `path_core.h`/FK-IK，所以这些差值互相重叠、**不能相加**。
单文件版 `single/meArm/meArm.ino`（`make_single.ps1` 生成，322981 字节 / 7012 行）
实编 **32030 B / 1481 B**，与多文件版只差 4 B —— 合并成单个 TU 后 LTO 的内联结果
略有不同，属正常。**（v1.6.7 之前的历史，运动模型已被整体替换，见 README 的对应章节）**：
单文件版已随新模型重新生成过，**字节数 / 行数 / 实编数字都必须重跑
`make_single.ps1` + `avr_build.ps1` 才能引用**，上面这组数只代表换模型之前。

**v1.6.7 实测（同样不含 `-mrelax`，`.selfcheck/avr_build.ps1` + `-ExtraDefs` 消融）**：

| 配置 | Program | Data |
| --- | --- | --- |
| 取放 + 按键（默认，绘图关） | **22734 B（69.4%），余 9522 B** | 1236 B（60.4%） |
| 绘图 + 按键（关取放） | **28824 B（88.0%），余 3432 B** | 1447 B（70.7%） |
| 取放 + 绘图（关按键） | 31242 B（95.3%） | 569 B（27.8%） |
| 取放 + 按键 + 绘图（三开） | 34648 B（105.7%）**装不进** | 1486 B（72.6%） |
| 三开 + `-mrelax` | 34128 B（104.2%）**仍超限** | 1486 B（72.6%） |

单文件版本分支实编 **22754 B / 1236 B**（比多文件版多 20 B），
`make_single.ps1` 生成 **347671 字节 / 7337 行**。

`run_all.ps1` 退出码 0：9 个编译单元在 `-Wall -Wextra -Wshadow -Wconversion` 下
**零警告**，11 个探针（`probe_axes`、`probe_rt`、`probe_move`、`probe_joystick`、
`wearm_ino_test`、`probe_protocol`、`probe_pick_place`、`probe_button`、`probe_draw`、
`probe_servo_drive`、`probe_pins_off`）**ALL PASS**。

> **（v1.6.7 之前的历史，运动模型已被整体替换，见 README 的对应章节）**：这批探针的
> 期望值是照旧运动模型写死的，换模型后**会变红**，而且当前已被用户停用（不跑）。
> 恢复探针之前必须先更新期望值 —— 要改哪几处的清单见 `.selfcheck/README.md`
> 的「探针期望值与 v1.6.7 运动模型不匹配」一节。

## 7. 没做 / 待上机确认

- **本轮所有结论都来自 PC 自检与静态审查**；v1.6.7 的运动学方向随后**已上机验证过**
  （位置正确、三条映射与实物一致），但下面"待确认清单"里的标定项仍未做。
- 上机时请重点确认（**v1.6.7 新模型**）：三条角度映射是否与实物姿态方向一致 ——
  `theta = b − 90`（基座，`b = 90°` 正对前方 +y）、`alpha = 180 − r`（上臂）、
  `beta = c`（前臂相对肘部的折角）。本机已实测这三条都对，不需要 `mB`/`mS`/`mE`
  （`WEARM_MIRROR_BASE = 1` 保持默认，`WEARM_MIRROR_SHOULDER`/`ELBOW` 保持 0）；
  以及用 `p<z>` 标定纸面高度 —— 默认 `DRAW_PEN_Z_DEFAULT = −6.0` 是"能放进
  50×50mm 图形的最低平面"（`z = 0` 是底盘顶面、臂尖最低只到约 −14），
  **纸铺在台面上够不到、必须垫块**，垫块高 = 台面到目标 z；
  `p<z>` 重标后中心/半宽可再放大（`.selfcheck/drawcheck.cpp` 有表）。
- §7 上一版写的是旧模型（`β = c − 90`、`S=0` 镜像开关、`DRAW_PEN_Z_DEFAULT = −28.0`
  按"home −20 / 纸面比 home 低 8"推出来），**均已被 v1.6.7 作废**，此处只作历史记录。
- **已提交并打标签 `v1.6.7`**（用户裁决：只提交源码与自检工具/文档 —— 构建产物、
  `.selfcheck/out/`、`others/` 第三方克隆不入库；`esp8266/` 与 `arduino_keypad/` 保留并入库）。
  **但实体机上机验证仍未做**，上面那条待确认清单依然有效。
- `others/` 是参考实现目录，仍未纳入版本控制（原样保留，未改动）。
- 根目录下 `meArm.zip` / `mearm (2).zip` / `meArm (3).zip` 是历史手工打包产物，
  **未删除**（已在 `.gitignore` 里，不影响仓库）；本轮只清掉了自己的临时脚本
  `.selfcheck/tmp_ablate.ps1` 与几个可再生的编译残留（`*.o`、`probe_button.exe`、`p1.txt`）。
