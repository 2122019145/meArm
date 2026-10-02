# .selfcheck —— PC 端编译自检与回归探针

这个目录**不参与烧录**，只是让你在电脑上（不需要 Arduino 板子）验证固件的
语法、类型、运动学模型与手柄逻辑。烧录时忽略整个目录即可。

> 当前固件是**角度模式**：摇杆直接改变 4 个舵机角 `Pos.ser.angle1..angle4`，
> `Pos.rec.x/y/z` 是由正运动学 `recFromServo()` 算出的派生显示量。
> 旧的"末端坐标控制"版本已备份到 `D:\wearm-backup-cartesian-20261002-205737\`。

## 为什么需要它

`arduino-cli` 没有装，无法做真正的 AVR 编译；而历史上真正致命的错误
（`#endif}` 少一个花括号、`-Wenum-compare` 枚举混用、`-Wunused-but-set-variable`）
**全部是编译器抓到的，人工审查两次都没看出来**。所以改动固件后请务必跑一遍这里。

更要紧的是：这套自检抓到过**探针自己写错、反过来冤枉固件**的多次假失败
（详见下面"写新探针时的教训"），也抓到过真正的固件 bug。

## 工具链

- 编译器：`C:\ProgramData\mingw64\mingw64\bin\g++.exe`（16.1.0）
- mock 头文件在本目录 `mock\`：`Arduino.h`、`Servo.h`，仅用于 PC 端替代真机 API。
- 串口日志默认被丢弃；想看到固件里的 `Serial.println` 输出，先设环境变量
  `$env:WEARM_MOCK_SERIAL='1'`。

## 一键跑全部

```powershell
D:\dsh1\wearm\.selfcheck\run_all.cmd
```

（`.cmd` 只是个壳，用 `-ExecutionPolicy Bypass` 调 `run_all.ps1`——
Windows 默认禁止直接运行 `.ps1`。）

它做两件事，任一失败就 `exit 1`：

1. **严格编译 7 个固件 TU**（`-Wall -Wextra -Wshadow -Wconversion`），要求
   **零警告**；
2. **编译并运行 8 个自检程序**，逐个要求输出里有 `ALL PASS` 且退出码 0。

每个探针的完整输出会存到 `.selfcheck\out\<探针>.log`。

通过时输出：

```
================ 1) 固件严格编译（0 警告才算过）================
  [ OK ] constant_and_positions.cpp  EXIT=0  输出 0 行（零警告）
  [ OK ] move.cpp  EXIT=0  输出 0 行（零警告）
  [ OK ] joystick_control.cpp  EXIT=0  输出 0 行（零警告）
  [ OK ] protocol_constants.cpp  EXIT=0  输出 0 行（零警告）
  [ OK ] serial_protocol.cpp  EXIT=0  输出 0 行（零警告）
  [ OK ] pick_place.cpp  EXIT=0  输出 0 行（零警告）
  [ OK ] button_control.cpp  EXIT=0  输出 0 行（零警告）

================ 2) 自检程序（必须 ALL PASS）================
  [ OK ] probe_axes  零警告编译 + ALL PASS
  [ OK ] probe_rt  零警告编译 + ALL PASS
  [ OK ] probe_move  零警告编译 + ALL PASS
  [ OK ] probe_joystick  零警告编译 + ALL PASS
  [ OK ] wearm_ino_test  零警告编译 + ALL PASS
  [ OK ] probe_protocol  零警告编译 + ALL PASS
  [ OK ] probe_pick_place  零警告编译 + ALL PASS
  [ OK ] probe_button  零警告编译 + ALL PASS

>>> 全部通过（固件零警告 + 8 个自检 ALL PASS）
```

## 每个探针在测什么

| 探针 | 覆盖内容 |
|---|---|
| `probe_rt` | 5 个标定点核对、反解→正解残差 0.05 阈值、关节角往复（1950 组：反解 false 0 / 被吸附 0 / 镜像分支 22 / 非镜像回转角最大误差 0.000°）、全网格残差（6270 组全部成功、最大残差 0.000000） |
| `probe_move` | 六向定长步进（`moveup` = 下臂 c +1°、`moveright` = 基座 b +1°、`moveforward` = 上臂 r +1°，其余关节一字节不动）、推到头返回 `MOVE_AT_LIMIT` 且零位移、非法方向 `MOVE_NONE`、`stepSize=0` 跟随全局调速、坐标与角度严格自洽且不出 `limit`、超臂展 `isReachable()==false`、三档调速参数、4000 步随机压力（零越界/零 NaN/零关节越限） |
| `probe_joystick` | 四路轴各控一个关节角且互不干扰、坐标 = `recFromServo(Pos.ser)`、末端 A2 方向与限位、斜推可同时动多个关节、死区不动作、串口 `1/2/3` 调速、串口 `k/K` 开合、快速转角 > 慢速、四路推到头都停在 `servoLimit` 内、4000 轮随机推杆 |
| `probe_axes` | 5929741 个关节角组合扫出真实可达包络、验证 `limit` 完全覆盖包络且余量非负、单轴满行程自查（每个关节都能走到行程两端）、5 个标定点核对 |
| `wearm_ino_test` | **直接 `#include "../wearm.ino"`**（不手抄复刻）：4 个舵机 attach 引脚、上电姿态 = POS_HOME、上电不是 0（不会甩向原点）、静置 20 轮 loop 不动、推杆后 loop 真的把新角度写进舵机 |
| `probe_protocol` | 串口协议完整测试：O/S/H/L 命令（爪子开/关/升档/降档）、角度指令格式（x10,y30,z20 及其变体）、行缓冲与超时处理（PROTO_LINE_BUF_SIZE=40, PROTO_LINE_TIMEOUT_MS=300）、速度档位边界（PROTO_SPEED_LEVEL_MIN/MAX/DEF）、MockSerial 64 字节缓冲边界测试、无换行超时场景、单字符打印 vs 码值打印 |
| `probe_pick_place` | A/B/C 自动取放：位置表（Δx、Δy 各 ≥5，三初始点/三放置点两两相距 ≥5，放置点离"别人的初始点" ≥5，实测最紧 7.21）、6 个取放点与接近点都"在 limit 内 + 可达 + 反解未被吸附"、启动语义（0 / -2 忙 / -1 编号非法）、完整跑三轮（A 305 轮、B 334 轮、C 335 轮，逐轮零违规、末点 = 放置点 + 抬升 6.0、`angle4 == servoLimit.maxF`）、忙时让位（O/S/x45/k 全返 BUSY 且状态零改动，H/L 仍生效）、空闲时反复调用不动状态 |
| `probe_button` | 四按键接口 9 段：`pinMode` 记录断言 D2~D5 都是 `INPUT_PULLUP`、初始无录制/不忙/不让位；按键1 物理连按四次 A→B→C→A 且序列执行中再按不推进序号；录制门槛（未录制时播放回 16、录制中 `busy=true` 但 `locked=false`、录制中 O 被挡回 11 而 H/L 仍回 3/4、只录 3 秒回 14、录 11.5 秒无位移回 14）；真实摇杆录制 11.6 秒 → 保存回 13、61 条、位移 24.95；播放回 15、只在"仍被独占"时推摇杆验证让位、回放末态与录制末态误差 0.00、耗时 12.52 s = 录制 11.6 s + 预摆 1.5 s；**播放中按按键4 / 发别名 `0` 都被拒（回 11）且播放不被劫持**；回中回 17 且 b/r/c 与 `posGetHomeAngles()` 位级一致、angle4 不动、别名 `0` 同样有效；串口路径 N/R/P/M 与忙守卫豁免（序列忙时 N 回 11、录制中 R 回 14）；空闲 100 次调用零改动；**[9] 最坏情况**：四路摇杆每 700 ms 翻向、连续推 11.2 秒 → 仍保存成功（444/512 条、位移 36.45；旧"每周期另写一条 WAIT"的格式要 ~560 条，必然溢出判废），回放后四轴末态差 0.00 度 |

### 当前实测包络（r、c 都放开到 0~180 之后）

```
x[-40.00, 40.00]  y[-40.00, 40.00]  z[-20.00, 40.00]
limit    : x[-40.0,40.0] y[-40.0,40.0] z[-20.0,40.0]
余量     : x[0.00,0.00] y[0.00,0.00] z[0.00,0.00]   （非负 = limit 完整覆盖包络）
```

四个极端姿态：`x=+40` 在 (b,r,c)=(90,0,0)，`x=-40` 在 (90,180,0)，
`z=+40` 在 (90,90,0)，`z=-20` 在 (90,0,90)。

逐高度（每 2° 一层，y 恒为 `[-X, X]` 对称）：`z=0.0 → x[-40.00,40.00]`（最远）、
`z=-20.0 → x[0,26.18]`、`z=20.0 → x[-35.15,35.15]`、`z=40.0 → x[-8.66,8.66]`。

两点容易看错，先说明：

- 每一层是 `|z - 层高| <= 1` 的**一层**，不是"z 恰好等于层高"的精确平面。
  所以 `z=40.0` 那层不是 0 而是 ±8.66 —— 它含了 `z=39.05` 的姿态
  （`r=103,c=1` 给 -8.66；`r=78,c=1` 给 +8.66）。真正的 z=40 只有
  `(r,c)=(90,0)` 一个姿态，此时 x=y=0。
- 负 x 来自 **ρ<0 的"朝后伸直/反折"姿态**（`20cos(r)+20cos(r-c)` 变负），
  例如 `r=180,c=0` 时 ρ=-40，b=90° 就落在 x=-40。不是算错。

## 写新探针时的教训（都踩过，代价很大）

1. **直接 `#include "constant_and_positions.h"` 调用固件函数，不要重写公式。**
   探针里重写运动学必然引入符号错：曾经把基座角写成 `theta=(b-90)`（应为
   `(90-b)`），整张可达表镜像，于是把"可达"误判成"不可达"，浪费好几轮。
   `wearm_ino_test` 同理：**直接 include 真 `.ino`**。手抄复刻版曾把引脚写成
   9/10/11/6（真 sketch 是 9/7/8/6）却照样编译通过、零警告——
   接线漂移只有"直接编译真文件 + 核对行为"才抓得到。
2. **必须调 `servoSelfCheck()` 再反解**，否则 `jointMin/jointMax` 映射表仍是全 0，
   会把所有角度夹成 0（表现为打印全 0.00、可达率虚高）。
3. **不要把 `getAngle(&t)` 和读取 `t.ser.*` 写在同一个 `printf` 参数里** ——
   参数求值顺序未指定，会打印出"陈旧但看起来合理"的角度。先 `bool ok = getAngle(&t);`
   再打印。
4. **循环边界别硬编码关节行程**（`c <= 105` 这类）。行程一改（c 已从 105 放到 180），
   探针就悄悄只测一半网格。改成从 `servoLimit.minC/maxC` 取。
5. **探针自己报的失败要先怀疑探针。** 踩过的假失败：
   - "慢速测试的终点接着测快速，剩余行程不足导致快速位移反而更小" ——
     两次测量前都要 `resetInputs(); posInit();` 回到同一起点；
   - 把**不在 `limit` 体积内**的点（如 x<0）当反解失败 —— 那些点本来就会被
     `clampToRange` 挡住，应先用 `limit.minX/maxX/...` 过滤；
   - 用 `==` 比较浮点角度：`posInit()` 走"设坐标→反解→落角度"往返，
     角度带 ~1e-13 舍入误差，`89.99999999999999 != 90.0` 会报假失败
     （而 `%.1f` 打印出来完全正常）→ 用容差比较。

6. **串口必须单一读者**。旧函数 `handleSerialSpeedCmd()` 自己调用 `Serial.read()`，新的行缓冲也要读同一串数据，两个读者会把字节各吃一半、行永远拼不完整。因此旧函数已**整体删除**（连同 `constant_and_positions.h` 里的声明），串口字节只由 `serialProtocolLoop()` 读，调用点只在 `weArm.ino` 的 `loop()` 里（放在 `joystickLoop()` 之前）。`joystickSetup()` 也不再调用 `Serial.begin()`。
7. **mock 的串口输入缓冲只有 64 字节且只追加不压缩**：`.selfcheck\mock\Arduino.cpp` 里是 `static char inBuf[64]`，`mockSerialFeed()` 只 append，读空后 `inPos` 等于 `inLen` 但 `inLen` 不回退。所以连续喂超过 64 字节的测试，后面的字符根本进不去 —— 测的是 mock 缓冲被塞满，而不是固件行为。正确做法是穿插 `mockSerialClear()`（或分段重建缓冲）。反例：`probe_protocol` 原超长行用例注释写"60 个 A"，字面量实际是 70 个，行尾换行符根本没能进缓冲。
8. **无换行超时用例必须调用两次 `serialProtocolLoop()`**：固件在读到字符时把 `s_lastCharMs` 记成 `millis()`，若先喂数据、再一次性推进 400 毫秒、再调 loop，则超时判断里的时间差恒为 0。正确写法是先 `mockSerialFeed` 再调一次 loop，再 `g_mockMillis` 加 400，再调第二次 loop。
9. **mock 的 `MockSerial` 没有 `print` 的单字符重载**：写 `Serial.print(某个 char 变量)` 会被隐式提升到打印 int，串口里打出的是码值（例如字母 x 打成 120）。要打印单个字符必须用长度 2 的 C 串（`char ch[2] = { c, 0 }; Serial.print(ch);`），真机与 mock 都能出字母。
10. **`PROTO_ANGLE_MIN` 与 `PROTO_ANGLE_MAX` 已被删除**：本工程不为解析阶段定义名义角度区间。解析时不该因为角度超出某个区间就拒绝指令（那样 x200 就变成废指令），而是落地前按该关节真实的 `servoLimit` 夹取（x200 最终写 180）。探针要造随机角度请用自己的局部常量。
11. **探针直接赋值 `Pos.ser` 会造出固件永远不会产生的脏状态**：探针的 `resetInputs()` 直接写 `Pos.ser.angle1` 到 `angle4`，不走固件写入路径，因此 `Pos.rec` 不刷新；而固件唯一写角度的地方 `protoApplyAngles()` 在 `written` 大于 0 时会调一次 `recFromServo(&Pos.rec, &Pos.ser)`，即固件任何一次成功写入都保证 `Pos.rec` 自洽。曾因此在压力测试里出现 violations 等于 4 的假警报（全是"Pos.rec 必须等于 recFromServo(Pos.ser)"这一条不变量）。正确做法：压力测试开始前自己补一句 `(void) recFromServo(&Pos.rec, &Pos.ser);`。
12. **PowerShell 的逗号优先级高于加号**：把两个 `-I` 参数用 `@("-I" + $root, "-I" + $mock)` 这样写会拼成**一个**字符串，表现为编译报 `fatal error: Arduino.h: No such file or directory`。必须给每个拼接加括号。
13. **给子智能体的长文本（例如 workflow 脚本里的 prompt）不能出现反引号**：脚本里用模板字符串包住 prompt 时，正文里的 markdown 反引号会直接终结模板字符串，报 `workflow script does not parse` 与 `SyntaxError: Unexpected identifier`。
14. **探针不许引用只存在于某个 `.cpp` 里的私有宏**。`pick_place.cpp` 的 `PICK_GRASP_Z` / `PICK_APPROACH_DZ` 是文件内 `#define`，探针 include 了头文件也看不到它们（表现为 `'PICK_APPROACH_DZ' was not declared`）。修法不是把宏抄一遍（那会制造第二处真值），而是在头文件里加一个访问器：`double pickPlaceApproachDz(void);`。
15. **`REC` / `SER` / `pos` 是 `typedef`，不能写 `struct REC r;`**（C 语言习惯，C++ 里报 `expected primary-expression`）。写 `REC r;` 即可。
16. **子智能体说"编译通过/测试通过"一律要自己再验一遍**。本轮一个子智能体在**从未编译成功**的情况下写出"预期输出应该是所有测试通过"，还往 `.selfcheck\` 扔了 11 个一次性垃圾文件（`hello.c`、`probe_minimal.py`、`probe_report.txt` 之类）。验收口径：自己跑 `run_all.cmd`，看它在**你改完的磁盘文件**上是否真的全绿；子智能体贴的原始输出只当线索，不当证据。
17. **测"某期间摇杆被让位"时，别把"期间结束的那一瞬间"算进去**。`probe_button` 的收尾循环写的是 `buttonLoop()` 之后紧接着 `joystickLoop()`：播放恰好在 `buttonLoop()` 里收尾，于是同轮的 `joystickLoop()` 立刻用还按着的摇杆走了 1.0°（`angle2 86.1124 -> 85.1124`），报成"让位失效"。固件是对的 —— 播放一结束摇杆就该立刻恢复。正确写法：循环条件用 `buttonControlLocked()`，并且**只在仍然锁着时才调 `joystickLoop()`**，然后单独收尾跑完剩余流程。
18. **探针里对"循环序号"这类有状态的量，不要写死第几次的期望值**。`probe_button` 断言"mock 喂 `N\n` 应该夹 A"，但循环序号在前面用例里已经推进到 C，于是假失败。正确写法：调用**前**先取 `int nextBefore = buttonPickNext();`，再断言 `pickPlaceCurrentObject() == nextBefore`。
19. **用 `edit` 工具改 `.ps1` 会丢 UTF-8 BOM**（第 274 行那条坑的另一面：`write`/`edit` 都按无 BOM 写）。改完必须补：读成字节、前置 `EF BB BF`、`[System.IO.File]::WriteAllBytes()` 写回，再确认 `CRLF` 数正常、`loneLF` 为 0。`run_all.ps1` 因为要被 `powershell -File` 解析，BOM 丢了会直接语法报错。
20. **`run_all.ps1` 的汇总行用 `$probes.Count` 自动计数**，加探针不用手改文案；但 `README.md` 里的"6 个固件 TU / 7 个自检"是手写的，每加一个 TU 或探针都要同步改示例输出，否则文档与实测输出对不上。
21. **"缓冲能撑多久"必须按最坏情况的每周期条目数推一遍**。`button_control.cpp` 头部原写"条目速率上限约 25 条/秒、512 条 ≈ 20 秒"，把采样周期误当成了条目速率；实际每个有动作的周期最多写 `1 + 动的关节数` 条 → 四关节同时动 = 125 条/秒，512 条只够 4.1 秒 → 必然写满、必然低于 10 秒门槛、录完就判废。修法是换格式：把"推进一个周期"塞进增量条目的 bit7（`BTN_TICK_FLAG`），每周期只花"动了几个关节"条，并把 tick 从 40 ms 放宽到 100 ms；容量结论按 `512 / (1 + 关节数) × tick` 重算后才写进注释。
22. **"周期性采样 → 回放"必须显式处理首尾零头**。采样是离散的，`btnRecTick()` 只在周期边界结算增量，于是"最后一个整周期 → 松手"之间的动作永远进不了缓冲。四关节在快速档每周期能走 10 度，回放终点就整整差 10.00 度（不是量化误差，是丢了一段）。修法：`btnStopRecording()` 一开头调 `btnRecFlushTail(now)` 把这一小段补成条目，只有跨进新周期时才给第一条挂 `BTN_TICK_FLAG`。修完 `probe_button` 的末态差从 `10.00/10.00/10.00/9.50` 变成 `0.00/0.00/0.00/0.00`。

## 编译器的坑（本机实测）

1. **`C:\msys64\ucrt64\bin\g++.exe` 在本沙箱下静默失败**：退出码 1、stderr 一个字都没有、连 `-v` 都不打印，`.exe`/`.o` 永不产出（看起来像"什么都没发生"）。**一切编译/链接都用 `C:\ProgramData\mingw64\mingw64\bin\g++.exe`** —— 它也是 `run_all.ps1` 里 `$gpp` 用的那一个。
2. **`cmd /c "... & echo ERRORLEVEL=%errorlevel%"` 的 `%errorlevel%` 是解析期展开的**，打印出来是命令**执行前**的值，会给出"编译成功"的错觉。PowerShell 里一律用 `$LASTEXITCODE`。

## 固件里的坑（自检抓到过的真 bug）

写固件时这几条都是"看起来对、实际错"的，已修：

1. **`clampToRange()` 会原地改 `Pos.rec` 并返回"是否夹到过"**，不能当只读判断用。
   旧 `moveJointStep()` 为了"给坐标一个临时落点"先挪坐标再判断越界，
   结果越界时该函数把坐标夹回边界值、返回值恰好让整步回退被跳过，
   末端坐标能停在界外（压力测试报 `violations=5722`）。修法是新增只读的
   `posOutOfRange()`，整步回退。
2. **`isReachable()` 判断 `R > L1+L2` 必须带浮点容差**。两杆完全伸直时 R 数学上
   恰为 40，浮点算出来是 40.000000000000004，严格比较会把"全伸直"整类姿态
   判成不可达（实测 19 组反解失败）。现在用 `GEOM_EPS = 1e-9`。
3. **`90 - RADtoDEG*atan2()` 的浮点误差会把"正好 0°"算成 -2.4e-10**，
   归一化 `while (b < 0) b += 360` 于是把它变成 359.9999999998，
   再和 `maxB = 180` 一比就丢掉这个分支：目标 `(0,20,20)` 的正确回转角恰好是 0°
   就踩在这里（表现为反解 false、`Pos.ser` 停在 `(0,0,0)`）。
   修法：归一化后把 `> 360-1e-6` 的值折回 0°。
4. **反解要试 4 个候选**：`{rhoP = ±rho} × {delta = ±|c|}`。
   只试一个分支会漏解（镜像姿态里常常只有一个满足机械行程）；
   而 `rhoP = -rho` 的反向平面分支是 `x<0` 类目标的唯一出路
   （正向平面会算出 190°~270° 的回转角，超出 `b` 的行程 `[0,180]`）。
5. **舵机角 `c = alpha - beta`，不是 `beta`、也不是 `|c|`**。模型是
   `beta = alpha - c`，所以反解必须先求出 `alpha` 与 `beta` 才能得到 `c`。
   曾把 `beta` 或 `|c|` 直接当舵机角，导致 `(28.28,0,0)` 这类点"解"残差很大。
6. **单个标定点无法区分模型**：`beta = r-c`、`r+c-180`、`180-r-c` 在 c=90 时
   全部退化成同一个点 (20,20)，只有代 c=105 才分道（正确模型给 (19.32,14.82)，
   `r+c-180` 给 (19.32,25.18)）。**凡是"唯一命中"的结论，先找第二个参数不同的标定点。**
7. **`getAngleEx()` 返回 false 时不能改 `Pos.ser`**。`applyJointLimits()` 在 CLAMP
   策略下就地吸附，写完才发现要拒绝时角度已被改掉一半——
   老代码踩过：一个反解失败的 `posInit()` 把 `Pos.ser` 留成 `(0,0,0)`，
   **上电时机械臂直接甩向原点**。现在先存 `oldA1..oldA4`，失败全部回滚。
8. **夹限位判断的容差不能用 `ANGLE_EPS = 1e-6`**：反解落在行程端点上的解常有
   -1e-14 量级舍入误差，于是几乎每个"正好在端点"的姿态都被判越界，
   `clamped` 标志天天误报。改用 `LIM_EPS = 1e-9`（远小于舵机可分辨的 0.1° 步进）。
9. **固件只能有一处运动学实现**。`posInit()` 的开机自检曾自己重写正解并写成
   `(angle2 + angle3)`（应为 `r - c`），公式错了照样打印一串"看起来很合理"的数字。
   现在整段改成调用固件自己的 `recFromServo()`。

10. **反解的"镜像分支"不是 bug，但必须用正解回代验证。** 同一个末端点可以有两组
    关节角到达：基座转 180°、两节臂在相反的竖直平面里镜像，两组都合法，反解器
    只能给出其中一个。r 放开到 0~180 之后镜像解也落进了行程，反解器可能先选到
    它，于是只比回转角就会误报 180° 的"误差"（实测 22 组）。修法**不是**放宽阈值
    或跳过这些点，而是**加强**判定：镜像分支（回转角与原来相差 180° 左右）必须用
    固件 fk() 正解回代、残差 < 0.05 才算通过，否则计入 branchBad 判 FAIL。
    （实测 1950 组里 22 组是合法镜像解，无法解释的 0 组。）

## mock 的引脚索引（容易搞错）

- `g_mockAnalog[]` 用**A0 起算的下标**：`MOCK_AX 0`(=A0)、`MOCK_AY 1`(=A1)、
  `MOCK_TX 2`(=A2)、`MOCK_TY 3`(=A3)。
- `g_mockDigital[]` 用**展开后的数字引脚号**：A0=14、A1=15、A2=16、A3=17。
- `mockSerialFeed("1")` 模拟串口收到字符，`mockSerialClear()` 清空输入。
- 推进时间：直接改 `g_mockMillis += ms;`（固件里没有 `delay`，全靠时间门控）。
- `mockServoPin(i)` / `mockServoAngle(i)` / `mockServoWriteCount()` 记录每个
  `Servo` 实例最后一次 attach 的引脚与写入的角度（`wearm_ino_test` 靠它核对接线）。

## 新增串口协议模块

本轮新增了完整的串口协议处理模块，包含：

- **协议常量**（`protocol_constants.h` 与 `.cpp`）：
  - 命令字符：`O`（爪子张开）、`S`（爪子关闭）、`H`/`L`（速度档升/降）、`x/y/z`（轴指令）
  - 兼容保留字符：`1/2/3`、`k/K`
  - 轴到舵机映射：`protoAxisServoIndex = {1,2,3}`（x→基座、y→上臂、z→下臂）
  - 关节字母表：`protoAxisJoint = {b,r,c}`
  - 协议参数：`PROTO_LINE_BUF_SIZE=40`、`PROTO_BAUD=115200`、`PROTO_LINE_TIMEOUT_MS=300`、`PROTO_TOOL_STEP_DEG=5.0`
  - 速度档位：`PROTO_SPEED_LEVEL_MIN`/`MAX`/`DEF`

- **串口协议处理**（`serial_protocol.h` 与 `.cpp`）：
  - 字符读取、行缓冲、命令解析与落地写入
  - 对外接口：`serialProtocolBegin()`、`serialProtocolLoop()`、`protoHandleLine(const char*)`
  - 命令语义：`O`=爪子开、`S`=爪子关、`H/L`=速度档升降、`x角度,y角度,z角度`格式指令

- **探针更新**：
  - `probe_joystick` 中的旧串口用例已删除，移至 `probe_protocol` 专门测试
  - 新增 `probe_protocol` 探针（约 582 行，12 段）专门测试串口协议功能

## 新增 A/B/C 自动取放模块

本轮新增 `pick_place.h` / `pick_place.cpp`：上位机发送单个字母 `A` 或 `B` 或 `C`，
机械臂自动把对应物体夹起来搬到另一个位置。

- **对外接口**（`pick_place.h`）：
  - `int pickPlaceStart(int object)`：启动某个物体的取放序列。返回 `0` 已启动、
    `-1` 编号非法、`-2` 已有序列在执行、`-3` 路径校验失败（**失败时不改任何全局状态**）。
  - `void pickPlaceLoop(void)`：非阻塞推进，每轮 `loop()` 调一次，不调用 `delay()`，
    不做等待式循环。
  - `bool pickPlaceIsBusy(void)` / `int pickPlaceCurrentObject(void)` / `const char *pickPlaceStageName(void)`
  - `bool pickPlaceGetSource(int, double*, double*, double*)` / `...GetTarget(...)`：
    读出物体初始位置与放置位置。
  - `double pickPlaceApproachDz(void)`：接近点相对物体的抬升高度（探针用它，避免抄死 6.0）。
  - 物体编号：`PICK_OBJECT_A/B/C` = 0/1/2，`PICK_OBJECT_COUNT` = 3。

- **位置表**（都写在 `pick_place.cpp` 顶部，改这里即可自定义）：

  | 物体 | 初始位置 | 放置位置 | x 位移 | y 位移 |
  |---|---|---|---|---|
  | A | (24, 12, 12) | (16, -14, 12) | -8 | -26 |
  | B | (24, -12, 12) | (18, 16, 12) | -6 | +28 |
  | C | (12, 20, 12) | (28, -6, 12) | +16 | -26 |

  三个初始点两两相距 24.00 / 14.42 / 34.18，三个放置点两两相距 30.07 / 14.42 / 24.17。
  另外每个放置点离**别人**的初始点都 ≥7.21（实测最近的是三对并列：B 放对 A 初始、B 放对 C 初始、
  C 放对 B 初始，都是 7.2111），免得给一个物体放件时夹爪蹭到还在地上的另一个物体。

- **动作拆解**（每个物体都是这套）：到物体上方 → 垂直下降到抓取高度 → 合爪
  （`angle4` 写成 `servoLimit.minF`，与串口 `S` 同角度）→ 停顿 400ms → 垂直抬起
  → 平移到放置点上方 → 垂直下降到放置高度 → 张爪（`servoLimit.maxF`，与 `O` 同角度）
  → 停顿 400ms → 垂直撤离。

- **启动前校验**：把 6 段直线按 0.5 的步长采样（每段最多 64 点），逐点要求
  "在 `limit` 内 + `isReachable()` + 反解成功且未被吸附"，且相邻采样点的关节角
  跳变不超过 25°（防止反解中途换分支）；还要求当前位姿与手里角度同分支（差 ≤10°）。
  任何一项不过就拒绝启动并保持现状 —— 宁可不动，也不能半路甩臂。

- **串口集成**：`serial_protocol.cpp` 在单字符命令 `switch` 里加了 `A/B/C` 三个分支，
  返回 `PROTO_RES_PICK_STARTED`（10）或 `PROTO_RES_BUSY`（11）；序列执行期间
  `protoHandleLine()` 最前面有忙守卫（`pickPlaceIsBusy() || buttonControlBusy()`），
  **只放行调速指令 `H/L/1/2/3` 与按键命令 `N/R/P/M`**，其余动作指令
  一律返回 `PROTO_RES_BUSY` 且不改状态，并回一行
  `[proto] busy: pick/place or record/play running, command ignored`（在
  `#if WEARM_DEBUG_SERIAL` 里，不影响返回值）。摇杆侧在
  `joystick_control.cpp` 的轴步进循环开头 `if (pickPlaceIsBusy() || buttonControlLocked()) break;` ——
  序列执行与播放/回中期间独占 b/r/c 与末端角（录制期间**不**独占，见下一节）。
  D13 指示灯把"序列在执行"也算作在动（`updateLed(moved || pickPlaceIsBusy() || buttonControlLocked())`），
  否则那十几秒灯是灭的，看着像死机。

## 新增四按键操作接口

本轮新增 `button_control.h` / `button_control.cpp`：四个按键的全部调用接口都封装在
**这一个 cpp** 里（这是需求原文的硬性要求），`weArm.ino` 只调 `buttonSetup()` /
`buttonLoop()`。

- **按键接线**：D2 = 按键1（循环执行取放）、D3 = 按键2（录制）、D4 = 按键3（播放）、
  D5 = 按键4（回中）。四脚 `INPUT_PULLUP`，按下为**低电平**（另一端接 GND），
  25 ms 时间消抖，只认按下沿。D6~D9 是舵机、D13 是灯、D0/D1 是串口、A0~A3 是摇杆，
  D2~D5 是唯一空闲好接的四个脚。
- **串口等价命令**（外部接口不只在按键上）：`N` = 按键1、`R` = 按键2、`P` = 按键3、
  `M`（别名 `0`）= 按键4。**不用 1/2/3/4**，因为 `1/2/3` 已经是既有的调速命令。
- **按键1 循环执行**：每按一次依次夹 A、B、C，第四次回到 A。忙时（序列执行中）
  被拒绝且循环序号**不推进**。
- **按键2 录制**：第一次按下开始录（状态「录制中」，串口回 `PROTO_RES_REC_STARTED`=12），
  人工用摇杆操控；第二次按下结束并保存（13）。保存门槛：时长 **>10000 ms** 且
  末端位移 **≥10.0**（`BTN_REC_MIN_MS` / `BTN_REC_MIN_TRAVEL`），不达标回
  `PROTO_RES_REC_REJECTED`=14。**录制失败会连带废掉上一次的录制**（缓冲区已被本次
  录制复用），代码注释与串口提示都写明了这一点。
- **录制格式**：条目只有两种 —— "某关节角的增量（0.5 度量化，`BTN_ANGLE_UNIT`）"或
  "等待若干个采样周期"。采样周期 `BTN_TICK_MS` = **100 ms**（**不是** 40 ms：四关节
  同时动时 40 ms 只够录 4.1 秒，低于 10 秒门槛，见教训 21）。**"推进一个周期"不单独占
  条目**，而是挂在本次动作第一条增量条目的最高位（`BTN_TICK_FLAG` = 0x80），所以
  "每个周期都在动"时每周期只花"动了几个关节"条。缓冲 `BTN_REC_ENTRIES` = 512 条 =
  1024 字节 SRAM，按最坏情况（每周期 1 条时间推进 + 每关节 1 条增量）的**连续动作
  上限**：单关节 ≈51 s、两关节 ≈26 s、三关节 ≈17 s、四关节 ≈12.8 s；几乎不动时
  每条 WAIT 可表 12.7 秒，理论上限约 43 分钟。写满会自动停止录制并明确提示。
- **收尾增量**：结束录制时会把"最后一个整周期 → 按下结束键"之间那不到一个周期的
  动作补成条目（`btnRecFlushTail()`）。不做这一步，回放终点会停在最后一个整周期上 ——
  实测快速档四关节同时动时末态正好差一个周期的位移（10.00 度），见教训 22。
- **按键3 播放**：先用 `BTN_RAMP_MS` = 1500 ms 在关节空间线性预摆到录制起点，
  再按录制时的时间轴复现，结束回空闲。没有录制时回 `PROTO_RES_PLAY_NO_RECORD`=16。
- **按键4 回中**：目标角由 `constant_and_positions.cpp` 新增的
  `bool posGetHomeAngles(SER *ser)` 对 `POS_HOME`（20,0,20）反解得到（`b/r/c` = 90/90/90），
  **angle4 保持当前值不动**；同样是 1500 ms 插值而不是瞬间跳变。探针直接拿这个函数
  当期望值，避免在探针里抄死 90。
- **忙与让位是两套语义**（这是本模块最容易写错的地方）：
  - `buttonControlBusy()` = 录制/播放/回中任一 → 串口动作指令（除调速与 N/R/P/M）被挡。
  - `buttonControlLocked()` = 播放/回中（**不含录制**） → 摇杆让位。
  - 忙守卫必须**豁免 N/R/P/M 本身**，否则录制中发 `R` 想结束录制会被自己的忙守卫吞掉
    （probe_button 专门有一条用例守着这个）。
- **新增返回值**：12 录制开始、13 录制已保存、14 录制被拒、15 播放开始、
  16 无可播放录制、17 回中开始（都定义在 `serial_protocol.h`）。

## PowerShell 的两个坑

1. **`& $gpp ... 2>&1` 赋给变量后 `$LASTEXITCODE` 仍是 0** —— g++ 的 warning 走
   stderr，只看退出码会漏掉警告（历史上就这样漏过两个 `-Wunused-*`）。
   判定失败要**同时看"输出行数 > 0"**。`run_all.ps1` 已经这么做了。
2. **`.ps1` 必须存成 UTF-8 带 BOM**。Windows PowerShell 5.1 读无 BOM 的 `.ps1`
   会按 ANSI(GBK) 解码，中文注释变乱码并破坏语法分析（报一堆
   `Unexpected token`/`Missing closing '}'`）。`run_all.ps1` 已带 BOM——
   **改完这个文件记得确认 BOM 还在**（用 `write` 工具覆盖会丢掉 BOM）。

## 已知的"上机才能确认"项

代码层面无法验证、需要接上真机试的：

1. **哪根摇杆轴接在 A0/A1 与 A2/A3**：若左右手反了，把
   `joystick_control.cpp` 里 `JOY_L*` / `JOY_R*` 两组引脚对调。
2. **末端开合方向**：现按旧手柄版约定「A2 **右推 = 收回**、左推 = 张开」
   （`joystick_control.cpp` 里 `(raw > JOY_CENTER) ? -1 : 1`）。
   若手感相反，把那个三元式对调即可。
3. **`c` 接近 180° 的机械干涉**：`servoLimit.maxC` 已放开到 180（理论行程），
   但下臂在接近 180° 时会往后折回，代码注释已警告这是"理论行程上限、
   不保证不撞"。**请先在慢速档、空载、抬离台面的情况下单步试。**
4. **`limit` 现在是软护栏**：基座可摆到 x<0 后方、末端最低到 z≈-20.0，
   同样建议先慢速空载试。

5. **`r` 走到 0° / 180° 的机械干涉**：`servoLimit` 已把上臂 r 放开到 0~180
   （r=0 上臂水平朝前、r=180 上臂水平朝后）。与 c 组合后末端最低可到肩关节
   以下约 20，可能撞台面或底座。**请先在慢速档、空载、抬离台面的情况下
   单步试**，确认行程两端不会撞到东西再放开跑。

6. **A/B/C 三个物体的实际摆放位置**：`pick_place.cpp` 顶部的 `PICK_SRC` 与
   `PICK_DST` 只是"设计值"，真机上是把物体摆在哪儿就改哪一行（单位与坐标
   模型一致，`x/y` 是水平面、`z` 是高度）。第一次跑取放序列请务必：
   先进慢速档（串口发 `L` 或 `1`），**卸掉夹爪里的物体**，用 `A` 试跑一遍，
   观察夹爪是否真的夹到物体中段高度（`PICK_GRASP_Z` / `PICK_APPROACH_DZ`
   也在同一个文件顶部）。序列速度 = 当前档位（慢 25 / 中 50 / 快 90 度每秒），
   夹爪段固定 60 度每秒。

7. **四个按键的接线与手感**：D2~D5 一端接按键、另一端接 GND，按下为低电平
   （固件用 `INPUT_PULLUP`，接反或悬空会一直读到"按下"）。上机先逐个试：
   按一下 D2 是否开始夹 A、再按是否换 B；按 D3 是否进入「录制中」、再按是否
   报保存成功或"太短/没有位移"；D4 是否复现；D5 是否回到 `POS_HOME` 姿态。
   注意**回放按录制时的时间轴走**，与当前调速档位无关，所以第一次录制建议
   用慢速档录一小段、观察回放是否与手动操作一致。
