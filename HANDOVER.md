# meArm 项目交接说明（换到新对话时的起点）

> 这份文件是给"下一个对话/下一个人"看的：项目在哪、现在是什么状态、怎么跑、有哪些坑、
> 还剩什么没做。生成于 **2026-10-10**，对应 **v1.6.4**（已发布）。
> 先说三个最容易踩的前提：
>
> 1. **项目目录是 `D:\dsh1\meArm`**（不是会话默认工作目录 `D:\dsh1\wearm`）。DSH 里引用
>    它一律用绝对路径，工具里写成 `@"D:\dsh1\meArm"` 或直接 `D:\dsh1\meArm\...`。
> 2. **`.selfcheck/README.md` 是本项目的权威设计说明**（1500+ 行，中文），任何"为什么
>    这么写"的问题先去那里查；本文件只是导航 + 当前状态。
> 3. **硬件从未上机验证过**（只有 PC 端自检 + AVR 实编尺寸）。所有"实机现象"类描述
>    都来自用户口述，不是我们测出来的。

---

## 0. 30 秒上手

| 想知道什么 | 去哪儿 |
| --- | --- |
| 这个固件现在怎么工作 | 本文件 §4 + `.selfcheck/README.md` |
| 改完代码怎么验 | 本文件 §5（两条命令：`run_all.ps1`、`avr_build.ps1`） |
| 现在还剩多少 flash | 本文件 §6（默认配置 **余量 240 B**，单文件 228 B） |
| 上一次发布说了什么 | `https://github.com/2122019145/meArm/releases/tag/v1.6.4` |
| 无注释版源码包在哪 | 本文件 §9（`dist\meArm_v1.6.4_multifile_nocomment.zip`、`dist\meArm_v1.6.4_single_nocomment.zip`，已验证与带注释版逐记号相同、AVR 尺寸不变） |
| 还有哪些没决的事 | 本文件 §10 |
| 想直接开工 | 用 §11 的"开场白"粘给新对话 |

---

## 1. 项目是什么

Arduino Uno（ATmega328P，16 MHz）驱动的 meArm 四自由度机械臂固件。属于课程/考试交付物，
所以它有两套并存的结构约束：

* **多文件版**：仓库根目录 21 个文件（9 个 `.cpp` + 11 个 `.h` + `meArm.ino`），用
  `.selfcheck/mock/` 里的假 Arduino 头在 PC 上编译自检。
* **单文件版**：`single/meArm/meArm.ino`，是 `.selfcheck/make_single.ps1` 把上面 21 个文件
  合并成的**生成物**（考试/交作业只交一个 `.ino`）。**改任何固件文件后必须重跑
  `make_single.ps1`**，否则 `wearm_ino_test` 探针立刻红。

四个关节：`b`(基座) / `r`(上臂) / `c`(下臂/小臂) / `f`(夹爪)；舵机引脚基座 9、上臂 7、
下臂 8、夹爪 6。摇杆占 A0~A3，另有四个按键。

---

## 2. 当前版本与线上状态

| 项 | 值 |
| --- | --- |
| 版本 | **v1.6.4** |
| 仓库 | `https://github.com/2122019145/meArm.git`（remote `origin`，分支 `main`） |
| 提交 | **`28333f6`** —— "v1.6.4：串口回到角度语义、绘图恢复 7 个任务、摇杆手感重做两轮"（25 files changed, +1491 / −1013） |
| 标签 | 附注标签 `v1.6.4`（注解 4734 B 中文版本说明，`git tag -n99 v1.6.4` 可看） |
| Release | **https://github.com/2122019145/meArm/releases/tag/v1.6.4**（id `408020083`，`draft=false`，`make_latest` 生效，assets 0） |
| 单文件 | `single/meArm/meArm.ino` = **7344 行 / 338092 B / SHA256 `EE53561BACF86B9812D36F78C5A1C6EB2AFCE8257B248E85B2819DEFB7B205F9`**（v1.6.3 是 7310 行 / 332015 B / `EA1C32C1…`） |
| 未入库（有意） | `docs/`、`dist/`、`meArm.zip`、`"mearm (2).zip"`、`p1.txt`、`HANDOVER.md`（本文件） |

`git status --porcelain` 在 v1.6.4 提交后是干净的，只剩上面这些未跟踪项。

---

## 3. 代码地图

| 文件 | 职责（一句话） |
| --- | --- |
| `meArm.ino` | 主程序：`setup()` 复位→串口→摇杆标定→按键/绘图→Timer1 舵机；`loop()` 每轮依次跑 serial/pickPlace/button/draw/joystick/writeServo |
| `weArm_config.h` | 全部开关与容量约束注释（`WEARM_ENABLE_*`、镜像开关、录制深度…），**第 63-76 行是实编容量记录** |
| `constant_and_positions.h/.cpp` | `Pos`（`ser` 四个关节角是唯一被控量、`rec` 是正解派生量）、`servoLimit`、`rangeLimit`、正反解 |
| `path_core.h` | 笛卡尔直线/圆弧采样（供绘图用） |
| `move.h/.cpp` | `moveToPoint()` —— 全工程唯一"按坐标移动"入口（内部反解→写 `Pos.ser`→刷新 `Pos.rec`） |
| `servo_drive.h/.cpp` | Timer1 硬件 PWM 四路舵机驱动（已弃用 Arduino Servo 库），含基座/夹爪镜像 |
| `serial_protocol.h/.cpp` | 串口协议：`x/y/z` 直接写三个舵机角度、`F` 切 7 个绘图任务、`?` 查状态；回 `OK/ERR/BUSY`，模式回执 `OK F=<tag>` |
| `joystick_control.h/.cpp` | 摇杆读取 + 标定 + 死区/迟滞 + 全局调速（直接加减关节角），板载 LED |
| `button_control.h/.cpp` | 四个按键：录制/播放/回中/示教点动，非阻塞插值 |
| `pick_place.h/.cpp` | 取放序列 A/B/C（非阻塞状态机） |
| `draw_control.h/.cpp` | 7 个绘图任务状态机：回待机→走位→落笔→绘制→抬笔 |
| `protocol_constants.h/.cpp` | 协议常量/结果码/模式标签（`drawTaskTag()`） |
| `single/meArm/meArm.ino` | **生成物**，单文件交付版 |

---

## 4. 行为规格（改代码前必须知道的四条）

1. **串口 `x/y/z` 是"直接写舵机角度"，不是笛卡尔坐标**（考试要求，v1.6.4 从坐标语义改回来的）：
   `x`→基座角 `angle1`、`y`→上臂角 `angle2`、`z`→下臂角 `angle3`；支持大小写、空格、
   `x=20`、`x12.5` 小数、只写部分轴（其余不动）。**超出行程不报错，按 `servoLimit` 夹取**
   （如 `x200` → 夹到 `maxB`=180），成功回 `OK`；语法错回 `ERR`；忙（取放/按键/绘图占用）
   回 `BUSY` 且**状态零改动**。实现入口：`protoParseAxisLine()` → `protoApplyAngles()` →
   `clampServoAngles()` → `recFromServo()`。
2. **绘图任务是 7 个**：`LINE 0 / N 1 / TRIANGLE 2 / Z 3 / V 4 / POLYLINE 5 / CURVE 6`，
   串口 `F` 切换后回 `OK F=<tag>`，tag 为 `LINE/N/TRI/Z/V/POLY/CURVE`（`drawTaskTag()`）。
3. **摇杆手感（v1.6.4 两轮改完的样子）**：死区 `JOY_DEADZONE=40`、标定 32 次、
   `JOY_CAL_MAX_OFF=80`、**施密特迟滞 `JOY_HYST=25`**（静止门槛 65、已起控 40）、
   每格关节角 = `stepSize × 偏转/500`（下限 `JOY_STEP_MIN_DEG 0.1°`）、
   固定步长间隔 慢 0.5°/80 ms、中 1.0°/40 ms（默认）、快 2.0°/20 ms。
   **"推得越狠走得越快"已被用户要求整段删除**（`speedIntervalMs()`/`jogIntervalMs()` 都没了，
   `struct speedCfg` 只剩 `{ double stepSize; int stepDelayMs; }`），不要再加回来。
4. **`Pos.ser` 是唯一真值**，`Pos.rec` 只是正解派生量；任何改角度的入口最后都要
   `recFromServo()` 刷新一次，否则显示与工作空间校验会漂。

---

## 5. 构建 · 自检 · 打包设施（照抄命令）

> ⚠️ 本机（会话 shell）里 **`pwsh` 命令不存在**，实际宿主是 **Windows PowerShell 5.1**；
> 而且直接 `& .\xxx.ps1` 会被 ExecutionPolicy 拒绝。**一律这样调**：
>
> ```powershell
> powershell -NoProfile -ExecutionPolicy Bypass -File D:\dsh1\meArm\.selfcheck\<脚本>.ps1
> ```

| 脚本 | 作用 | 关键点 |
| --- | --- | --- |
| `.selfcheck/run_all.ps1` | **PC 端全量自检**：9 个固件 TU 严格编译（`-Wall -Wextra -Wshadow -Wconversion`，**退出码 0 且零输出**才算过，只看退出码会漏警告）+ 11 个探针必须 `ALL PASS` | 探针：`probe_axes / probe_rt / probe_move / probe_joystick / wearm_ino_test / probe_protocol / probe_pick_place / probe_button / probe_draw / probe_servo_drive / probe_pins_off`。编译器硬编码 `C:\ProgramData\mingw64\mingw64\bin\g++.exe`；探针用 `-DWEARM_BUTTON_PINS=1`，只有 `probe_pins_off` 用 `=0` |
| `.selfcheck/avr_build.ps1` | **真 AVR 实编**（avr-gcc 7.3.0 + core 1.8.8，`-Os -flto`），报 Program/Data | 参数 `-SketchDir`（多文件版指 `D:\dsh1\meArm`，单文件版指 `D:\dsh1\meArm\single\meArm`）、`-Full`、`-NoLto`、`-Symbols`、`-ExtraDefs` |
| `.selfcheck/make_single.ps1` | 21 个文件 → `single/meArm/meArm.ino` | 生成物，LF-only；改固件后必跑 |
| `.selfcheck/ensure_bom.ps1` | 给 `.selfcheck/*.ps1` 里含中文又没 BOM 的脚本补 UTF-8 BOM | **中文脚本没 BOM 会被 5.1 按 CP936 解码、乱码甚至吃掉引号**；`-Check` 只报告 |
| `.selfcheck/gh_release.ps1` | 发/更 GitHub Release | 本机**没有 gh CLI、没有 GITHUB_TOKEN**；token 从 `git credential fill`（host=github.com）现取、不落盘；`-Tag -BodyFile [-Name] [-Force] [-MakeLatest] [-CheckOnly] [-DryRun]`；正文 <300 字符会拒绝发布；发布名取正文首行 `# vX.Y.Z — …` |
| `.selfcheck/strip_comments.py` | **本轮新增**：去注释（Python 词法状态机） | `python <in> <out> [--lines]`，默认 compact；见 §9 |
| `.selfcheck/make_nocomment.ps1` | **本轮新增**：生成"无注释"源码包 | 见 §9 |

**提交与发版的中文处理**：正文/提交信息/标签注解一律写成 UTF-8 **无 BOM** 文件，再用
`git commit -F` / `git tag -a -F`，不要走命令行参数（会被 ANSI 重编码）。

---

## 6. 当前容量与硬约束

AVR Uno 实编（16 MHz，`-Os -flto`，即 `.selfcheck/avr_build.ps1`）：

| 配置 | flash | SRAM | flash 余量 |
| --- | --- | --- | --- |
| **默认（取放+按键+绘图）多文件** | **32016 B (97.7%)** | 1475 B (72.0%) | **240 B** |
| 默认 单文件 | 32028 B | 1475 B | **228 B** |
| 只关按键 | 28814 B | 558 B | 1762 B |
| 只关绘图 | 19260 B | 1224 B | 1316 B |
| 只关取放 | 29114 B | 1441 B | 962 B |

⇒ **默认配置只剩 240 B flash**，任何"再加点功能"之前必须先减（这是为什么 v1.6.4 顺手删了
`moveAxisStep()`、`joystickGetMode()`、`atRangeEdge()`、`posToolOpen/Close()`、串口 `1/2/3` 与
`k/K`）。容量记录同时写在 `weArm_config.h:63-76` 和 `.selfcheck/README.md` §5。

---

## 7. 文档与发布稿地图

| 位置 | 内容 |
| --- | --- |
| `.selfcheck/README.md` | **权威设计说明**（1500+ 行）。重点：角度语义 + 七任务 + 删减清单（:548 起）、摇杆两轮修正（:1251 起，末尾 :1456 附"小臂抖动"分析）、实编容量 §5（:1141-1156）、单文件对比表（:1530-1538） |
| `README.md`（根） | 面向使用者的说明：命令表、任务表、接线 |
| `docs/make_program_guide_docx.py` → `docs/meArm_program_guide.docx/.pdf` | 程序说明文档（含**单文件行号表**，已按 7344 行同步）；生成后用 Word COM 导 PDF |
| `docs/validate_program_guide.py` | 校验行号表/关键词漂移（`MODULE_BANNERS` 里是 21 个模块在单文件里的起始行：4,250,487,579,655,795,915,999,1134,1421,1557,1662,1792,2654,2717,2934,3458,3964,5364,6142,7106），报告写 `.selfcheck/out/program_guide_validate_report.txt` |
| `docs/make_kinematics_docx.py`、`kinematics_derivation.docx/.pdf` | 运动学推导 |
| `docs/make_review_guide_docx.py`、`meArm_review_guide.docx/.pdf`、`review_content.py` | **旧 34 页复习手册**（是否更新/删除仍未决，见 §10） |
| `.selfcheck/out/rel_v1.6.4.md` | v1.6.4 的 Release 正文（8443 B，六节改动 + 验证 + 已知限制）；同目录另有 v0.1.0…v1.6.3 共 16 份历史发布稿 |
| `.selfcheck/out/commit_v1.6.4.txt`、`tag_v1.6.4.txt` | 本次提交信息 / 标签注解原文 |

`docs/` 与 `dist/` 都**不在 git 里**（历史惯例），`dist/` 里两个旧 zip 内容已陈旧。

---

## 8. 这个项目里的坑（照抄可避雷）

1. **别用命令行传中文**（提交信息、标签注解、Release 正文）→ 一律 `-F <UTF-8 无 BOM 文件>`。
2. **`.ps1` 含中文必须有 BOM**，新写完跑一次 `ensure_bom.ps1`。
3. **`$ErrorActionPreference='Stop'` + 外部命令往 stderr 写一个字 = 脚本当场中断**
   （`NativeCommandError`）。要像 `make_nocomment.ps1` 的 `Invoke-Capture` 那样临时把
   ErrorActionPreference 放成 `Continue`，自己判退出码 + 输出行数。
4. **`.NET` 静态调用（`[System.IO.File]::…`）用绝对路径**：它按进程 CWD 解析，
   PowerShell 的 `Set-Location/cd` 不改变进程 CWD。
5. **`.ino` 不是自足 C++ 翻译单元**（要靠包含者先 `#include "Arduino.h"`），g++ 也不按扩展名
   认它 → 单独编译要 `-x c++ -include Arduino.h`。
6. **`g++` 的警告走 stderr**：判断"编译零警告"必须同时看退出码和输出行数。
7. **单文件是生成物**：改固件 → 重跑 `make_single.ps1` → 再跑 `run_all.ps1`（
   `wearm_ino_test.cpp` 直接 `#include "../meArm.ino"`，会立刻暴露不一致）。
8. **探针里的假失败史**：`posInit()` 反解会有 ~1e-13 舍入，比较用容差；夹爪上电取的是
   `[60,150]` 中位 105 而不是 90；fuzz 用例角度按 0.1° 生成以配合 `%.1f` 打印。
9. **`weArm_config.h` 里 `WEARM_BUTTON_PINS` 的默认值是 0**，探针侧才用 1（`probe_pins_off`
   专门验 0 的情况）。
10. `.selfcheck/out/`、`build/`、`dist/`（未跟踪）、`*.o/*.exe/*.log` 都在 `.gitignore` 里。
11. **PowerShell 里逗号优先于加号**：`@("-I" + $a, "-I" + $b)` 会被解析成 `"-I" + ($a, $b)`，
    两个 `-I` 被拼成**一个**参数（`-I...\single\meArm -I...\.selfcheck\mock`），于是预处理找不到
    `<Arduino.h>`、退出码非 0，最后表现为"记号流不同"的**假失败**。每个元素单独括起来：
    `@(("-I" + $a), ("-I" + $b))`。
12. **`edit` 工具改完含中文的 `.ps1` 会把 UTF-8 BOM 弄丢**（本轮 `make_nocomment.ps1` 被
    `ensure_bom.ps1` 补了三次）：用 edit 改过这类脚本之后，**必须再跑一次 `ensure_bom.ps1`**。

---

## 9. 无注释版源码包（用户 m16978 要求）—— 已完成并验证

**需求原话**："给多文件和单文件版本都打包一个无注释的版本"。

做了什么：

* `.selfcheck/strip_comments.py` —— 去注释专用词法状态机（Python）。只删注释，**不动字符串/
  字符字面量里的 `//`、`/*`**，处理行尾反斜杠续行的 `//`，丢 CR，保留 BOM 状态。两种模式：
  `compact`（默认，纯注释行整行删掉、作者原本的空行保留、连续空行折叠成 1、去首尾空行）与
  `--lines`（换行一律保留、行号与带注释版一一对应，代价是长注释块位置一片空行）。
  **选 compact 当交付形态**，等价性靠"预处理记号流相同"而不是靠行号。
* `.selfcheck/make_nocomment.ps1` —— 一条命令产出两个包：去注释 → 断言不再有 `/*`、`//` →
  用 `run_all.ps1` 同款参数严格编译每个文件（要求零警告）→ `g++ -E -P` 取"原文 vs 去注释"
  记号流折叠空白后比 SHA256（**全功能 / 功能全关两种宏配置各比一遍**）→ 可选
  `-ProbeCheck` 在 `build\nocomment\verify` 里搭"去注释固件 + 完整 `.selfcheck`"的树跑一遍
  9 TU + 11 探针 → 写 `README.txt` → 打 zip → 报告 `.selfcheck/out/nocomment_report.md`。
* 去注释体积实测（22 个文件，`in → out`）：总计 **677979 B → 238155 B（35.1%）**；
  其中多文件版 21 个文件 **339887 → 120038 B**，单文件版 **338092 → 118117 B（34.9%）**。
  典型：`weArm_config.h` 17339 → 652 B（3.8%，它几乎全是注释）、`meArm.ino` 16822 → 982 B、
  `serial_protocol.cpp` 34725 → 18655 B。**去注释后剩余非 ASCII 字节全为 0**（中文只存在于注释里）。
  完整表格在 `.selfcheck/out/nocomment_report.md`。
* 产物路径（`build/` 在 .gitignore 里，是暂存区）：
  `build\nocomment\pkg\meArm_multifile_nocomment\{README.txt,meArm\<21 文件>}`、
  `build\nocomment\pkg\meArm_single_nocomment\{README.txt,meArm\meArm.ino}`；
  zip 目标 `dist\meArm_v1.6.4_multifile_nocomment.zip`、`dist\meArm_v1.6.4_single_nocomment.zip`。

**结果（2026-10-10 全部通过，`-ProbeCheck` 一趟跑完）**：

* 命令：`powershell -NoProfile -ExecutionPolicy Bypass -File D:\dsh1\meArm\.selfcheck\make_nocomment.ps1 -ProbeCheck`
  → 末尾打印 `>>> 无注释版打包完成，全部检查通过`（`失败项：0`）。
* 22 个文件逐个通过：**去注释后无 `/*` 与 `//`** + **`-Wall -Wextra -Wshadow -Wconversion` 严格编译零警告** +
  **`g++ -E -P` 预处理记号流折叠空白后 SHA256 与原文相同**（全功能 / 功能全关两种宏配置各比一遍）。
* 另有一步端到端：在 `build\nocomment\verify` 里搭"**去注释固件 + 完整 `.selfcheck`**"的树，
  跑那份 `run_all.ps1` → **9 个 TU 零警告 + 11 个探针 ALL PASS**（与带注释版同一结果）。
* **AVR 复核（证明去注释没动到代码）**：多文件 **32016 B / 1475 B**、单文件 **32028 B / 1475 B**
  —— 与 §6 带注释版的数字**完全一致**。
* 交付 zip 已生成：`dist\meArm_v1.6.4_multifile_nocomment.zip`（**38725 B**）、
  `dist\meArm_v1.6.4_single_nocomment.zip`（**29947 B**）；每个包内是
  `<包名>\README.txt`（说明这个包是什么、怎么验证的）+ `<包名>\meArm\{21 个文件 | meArm.ino}`，
  Arduino IDE 可直接打开。
* 报告：`.selfcheck/out/nocomment_report.md`（逐文件 in/out 字节/行数表 + 失败项 0）。
* 调试这段脚本踩的两个坑已记在 §8 第 11、12 条（PowerShell 逗号优先级 → "单文件记号流假失败"；
  `edit` 弄丢 BOM → 中文脚本被按 CP936 读）。

**后续（用户没要求，等你决定）**：

1. 本轮新增 `.selfcheck/strip_comments.py` 与 `.selfcheck/make_nocomment.ps1`（未跟踪文件），
   `ensure_bom.ps1` 还顺手给 `.selfcheck/make_single.ps1` 补了 BOM（一行 diff）。
   要不要提交、要不要顺带发 v1.6.5？`docs/`、`dist/` 按惯例不入库。
2. 若希望"无注释版"随版本自动更新：脚本是幂等的（同输入必同输出），可挂进发版清单。

---

## 10. 未决问题 / 待办

1. **`docs/` 要不要入库？** 已问过用户，尚未答复（历史惯例是不入库）。
2. **旧 34 页手册 `docs/meArm_review_guide.docx/.pdf` + `docs/review_content.py`** 要不要按
   v1.6.4 更新或删除？已问过，未答复。
3. **小臂（`c`，摇杆 A3，舵机 D8）异常抖动**：只做了原因分析（全文在
   `.selfcheck/README.md:1456` 附节：分流表 + 6 条候选 + 硬件优先建议），**一行缓解代码都没实装**。
4. **Uno 余量只剩 240 B**（单文件 228 B）：想继续加功能，先做减法。
5. **从没上过机**：探针覆盖的是逻辑与协议，真实舵机抖动/供电/机械干涉只能靠实机。

---

## 11. 新对话开场白（直接粘）

> 继续 `D:\dsh1\meArm` 这个 meArm 机械臂项目（Arduino Uno 固件：v1.6.4 已发布，
> 无注释版源码包也已完成并验证）。先读 `D:\dsh1\meArm\HANDOVER.md` 与
> `D:\dsh1\meArm\.selfcheck\README.md`，再跟我确认 §10 的未决问题：
> ①`docs/` 要不要入库 ②旧 34 页手册 `meArm_review_guide.*` 要不要按 v1.6.4 更新/删除
> ③小臂抖动（`c`/A3/D8）要不要实装缓解 ④本轮的 `strip_comments.py`、`make_nocomment.ps1`
> 要不要提交（可能顺带发 v1.6.5）。改代码后必跑 `run_all.ps1`；发版前跑 `avr_build.ps1`
> （默认配置只剩 240 B flash）。

---

## 附：常用命令速查

```powershell
# 全量自检（9 TU 零警告 + 11 探针 ALL PASS）
powershell -NoProfile -ExecutionPolicy Bypass -File D:\dsh1\meArm\.selfcheck\run_all.ps1

# 真 AVR 实编（先多文件、再单文件）
powershell -NoProfile -ExecutionPolicy Bypass -File D:\dsh1\meArm\.selfcheck\avr_build.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File D:\dsh1\meArm\.selfcheck\avr_build.ps1 -SketchDir D:\dsh1\meArm\single\meArm

# 重新生成单文件（改固件后必做）
powershell -NoProfile -ExecutionPolicy Bypass -File D:\dsh1\meArm\.selfcheck\make_single.ps1

# 无注释包
powershell -NoProfile -ExecutionPolicy Bypass -File D:\dsh1\meArm\.selfcheck\make_nocomment.ps1 -ProbeCheck

# 文档行号表校验
python D:\dsh1\meArm\docs\validate_program_guide.py

# 发 Release（中文正文一律走 -BodyFile）
powershell -NoProfile -ExecutionPolicy Bypass -File D:\dsh1\meArm\.selfcheck\gh_release.ps1 -Tag v1.6.4 -BodyFile D:\dsh1\meArm\.selfcheck\out\rel_v1.6.4.md -MakeLatest

# git 中文提交/标签（先把正文写成 UTF-8 无 BOM 文件）
git -C D:\dsh1\meArm commit -F .selfcheck\out\commit_v1.6.5.txt
git -C D:\dsh1\meArm tag -a v1.6.5 -F .selfcheck\out\tag_v1.6.5.txt
```
