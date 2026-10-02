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

1. **严格编译 5 个固件 TU**（`-Wall -Wextra -Wshadow -Wconversion`），要求
   **零警告**；
2. **编译并运行 6 个自检程序**，逐个要求输出里有 `ALL PASS` 且退出码 0。

每个探针的完整输出会存到 `.selfcheck\out\<探针>.log`。

通过时输出：

```
================ 1) 固件严格编译（0 警告才算过）================
  [ OK ] constant_and_positions.cpp  EXIT=0  输出 0 行（零警告）
  [ OK ] move.cpp  EXIT=0  输出 0 行（零警告）
  [ OK ] joystick_control.cpp  EXIT=0  输出 0 行（零警告）
  [ OK ] protocol_constants.cpp  EXIT=0  输出 0 行（零警告）
  [ OK ] serial_protocol.cpp  EXIT=0  输出 0 行（零警告）

================ 2) 自检程序（必须 ALL PASS）================
  [ OK ] probe_axes  零警告编译 + ALL PASS
  [ OK ] probe_rt  零警告编译 + ALL PASS
  [ OK ] probe_move  零警告编译 + ALL PASS
  [ OK ] probe_joystick  零警告编译 + ALL PASS
  [ OK ] wearm_ino_test  零警告编译 + ALL PASS
  [ OK ] probe_protocol  零警告编译 + ALL PASS

>>> 全部通过（固件零警告 + 6 个自检 ALL PASS）
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
