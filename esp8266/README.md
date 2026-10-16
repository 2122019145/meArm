# meArm WiFi 遥控桥（ESP8266）

给 `others/f/MeArm/` 里的机械臂固件配一块**无线遥控板**：机械臂的串口接到 ESP8266，
手机/电脑浏览器就能控制，不用装 App、不用连互联网（ESP 自己开热点也能用）。
原来 `others/f/MeArm/KeypadBoard_Task5` 那块插满按键的 UNO，本程序可以完全替代。

```
  手机浏览器 ──WiFi──> ESP8266（网页 + 命令队列 + 状态缓存） ──9600 8N1──> 机械臂（Arduino）
                          │
                          └── telnet <IP> 23：串口控制台（和网页日志同源）
```

---

## 1. 文件

| 文件 | 说明 |
|---|---|
| `meArm_wifi/meArm_wifi.ino` | ESP8266 主程序（网页、串口桥、配网、OTA 全在里面，无第三方库） |
| `README.md` | 本文件 |

`meArm_wifi.ino` 内部结构：第 1 节配置区 → 第 2 节**按钮表** → 第 3 节全局状态 →
第 4 节日志环 → 第 5 节机械臂串口/发送队列 → 第 6 节 telnet → 第 7 节 WiFi 配置(EEPROM) →
第 8 节网页与 HTTP 路由 → 第 9 节 setup/loop。**想加按钮只改第 2 节的 `BTNS[]` 表**，
网页会自动多出按钮，不用动 HTML。

第 2 节里有三个预设（`PRESET_TASK5` / `PRESET_VTEST` / `PRESET_MEARM165`），各自的
按钮表、关节表（有没有关节指令）、波特率、状态解析都不一样，默认用 **Task5**。

---

## 2. 硬件与接线

机械臂（Arduino Uno / ATmega328P）是 **5V 逻辑**，ESP8266 是 **3.3V 逻辑且不耐 5V**：

| 机械臂 Uno | ESP8266 (NodeMCU) | 说明 |
|---|---|---|
| D1 (TX) | D5 / GPIO14 | **必须分压**：1kΩ 串联 + 2kΩ 到 GND，5V→3.33V |
| D0 (RX) | D6 / GPIO12 | 3.3V 直连（3.3V > 3.0V 门限，可靠） |
| GND | GND | 必须共地，否则串口全是乱码 |
| 5V | VIN → AMS1117-3.3 | 不要用 Uno 的 3.3V 脚（只有约 50mA） |

**三个坑**

1. **Uno 的 D0/D1 同时接在 USB 转串口上**。给 Uno 烧录时，USB 芯片和 ESP 会同时驱动 D0
   → 烧录前拔掉 ESP 的 D6 线（或加一个拨动开关），烧完再插回。
   （现场症状：avrdude 反复 `not in sync: resp=0x00`，见第 9 节。）
2. ESP8266 峰值电流 300mA+，AMS1117 输入输出各并 100nF + 470uF，否则动作时容易复位。
3. 如果嫌分压麻烦，可以在 Uno 和 ESP 之间加一块 3.3V 双向电平转换模块，接线不变。

不想用软串口（软串口在 9600 下很稳，但 ESP 同时跑 WiFi 时会偶发丢字节）可以把
`ARM_LINK_SOFTWARE` 改成 `0`，改用 GPIO3(RX)/GPIO1(TX) 硬件串口——代价是 ESP 的 USB
串口监视器被机械臂占用（telnet/网页日志仍然可用）。

---

## 3. 编译与烧录

**Arduino IDE**：开发板管理器装 "esp8266 by ESP8266 Community"（本机已装 3.1.1）→ 打开
`meArm_wifi/meArm_wifi.ino`（文件夹名必须等于文件名）→ 开发板选 **NodeMCU 1.0 (ESP-12E
Module)** → 选 ESP 的 USB 端口 → 上传。

**命令行**（本机实测可用的命令，无需装 arduino-cli 到 PATH）：

```powershell
$env:ARDUINO_DIRECTORIES_DATA="$env:LOCALAPPDATA\Arduino15"
& 'C:\Users\m2122\AppData\Local\Programs\arduino-ide\resources\app\lib\backend\resources\arduino-cli.exe' `
    compile --fqbn esp8266:esp8266:nodemcuv2 'D:\dsh1\meArm\esp8266\meArm_wifi'
```

实测占用（默认 Task5 预设）：Flash 322 760 / 1 048 576 B（30%）、RAM 34 212 / 80 192 B（42%）、
IRAM 62 467 / 65 536 B（95%，ESP8266 核心本身就要这么多，正常）。
四个配置全部 `--warnings more` 下 **0 warning / 0 error**（见第 10 节）。

---

## 4. 首次使用

1. 插上 ESP8266 电源。它先用 15 秒尝试连你存在里面的 WiFi（第一次没有，所以直接跳过）。
2. 连不上就自己开热点 **`meArm-XXXXXX`**（PASSWORD = `mearm1234`）。
3. 手机连这个热点 → 一般会自动弹出配网页 → 填你家的 WiFi 名和密码 → 保存并重启。
4. 重启后 ESP 连上你家 WiFi，串口（115200）会打印 IP；也可以用 **http://mearm.local**。
5. 打开网页就是遥控台：状态卡片 + 按钮区（+ 该固件有角度指令时才有滑条） + 命令框 + 日志。

固定 IP / 换主机名：改 `WIFI_SSID`、`WIFI_PASS`、`HOST_NAME`（默认 `mearm`，即
`mearm.local`）。清掉已存配置：在配网页里把两个框都留空再保存。

---

## 5. 网页能干什么

* **按钮区**：全部由 `BTNS[]` 生成，按功能分组；标红/带"会动"标记的按钮点击前会弹确认框。
* **关节滑条**：只有"固件接受角度指令"的预设才显示（V-test / meArm v1.6.5）。
  V-test 一条命令只能带一个关节角，"依次发送全部关节"会**一条一条排队发、等固件回
  `DONE` 再发下一条**；meArm v1.6.5 则把三条滑条拼成一条 `x90,y80,z100` 一次发。
  **Task5 固件没有角度指令 → 这一区自动隐藏。**
* **手动命令框**：直接敲协议命令（Task5 敲 `7`、`?`；V-test 敲 `REC`、`B90`…），
  等于一个网页版串口。
* **日志**：机械臂回包实时显示，可一键下载 `log.txt`。
* **自动查状态**：默认每 3 秒发一次查询字符（三个预设都用 `?`，meArm v1.6.5 用 `!`），
  只是查询、不会打断动作；不想要就取消勾选。
* **状态卡片**：由 ESP 按预设组装（Task5：状态 / 当前动作 / 笔尖坐标 / 示教点 / WiFi /
  堆内存 / 运行 / 队列；V-test：状态 / 关节角 / 摇杆爪 / 记录 / …）。
* **telnet 控制台**：`telnet <ESP_IP> 23`，直接敲命令、实时看回包。

> **按钮含义对照（默认 Task5 固件）**：`1` 示教/记录（`t` 同义）`2` 暂停 `3` 继续 `4` 取消
> `5` 折线 `6` 曲线 `7` 直线演示 `8` 基础图形（字母 N）`?` 状态查询 `h`（或任意未知字符）指令表。
> 对应原来那块按键板：键1×1=1、键2×1=2、键2×2=3、键3×1=4、键3×2=5、键3×3=6、键4×1=7、键4×2=8。
> 折线/曲线要先"示教"记录点（最多 5 点，≥2 点才能折线、≥3 点才能曲线）。
>
> **按钮含义对照（V-test 固件）**：`?`查询 `X`急停保持 `D`舵机脱力（**手臂会垂下来**）
> `E`使能并回 B90R90F90C120+开摇杆 `H0`回记录起点 `C+`/`C-`爪子±1° `REC`/`STOP`录制
> `SAVE`存EEPROM `TRIM5`裁掉末5秒 `LOAD`读回 `V`回放 `P`暂停/继续 `DUMP`导出记录
> `JOY`/`K`摇杆开停 `CJ`/`CL`爪子随摇杆/锁爪 `CAL`摇杆校准 `H`/`L`调速 `J`看ADC `HELP`帮助。

---

## 6. 换成别的固件（三个预设）

| 预设 | 目标 | 波特率 | 命令 |
|---|---|---|---|
| `PRESET_TASK5`（默认） | `others/f/MeArm/Task5_LineN2_TeachFix`（及整条 Task5 线） | 9600 | 单字符 `1`~`8`、`g t s c p x ?` |
| `PRESET_VTEST` | `others/RC-robot-hand/V-test` v3.3 | 9600 | 单词命令 + `B90`/`R120`… |
| `PRESET_MEARM165` | 本仓库主固件 v1.6.5 | 115200 | `x90,y80,z100`、`A/B/C`、… |

命令行换预设（不用改文件）：

```powershell
$env:ARDUINO_DIRECTORIES_DATA="$env:LOCALAPPDATA\Arduino15"
$cli = 'C:\Users\m2122\AppData\Local\Programs\arduino-ide\resources\app\lib\backend\resources\arduino-cli.exe'
# V-test：
& $cli compile --fqbn esp8266:esp8266:nodemcuv2 `
    --build-property 'compiler.cpp.extra_flags=-DPRESET=1' 'D:\dsh1\meArm\esp8266\meArm_wifi'
# meArm v1.6.5：
& $cli compile --fqbn esp8266:esp8266:nodemcuv2 `
    --build-property 'compiler.cpp.extra_flags=-DPRESET=2' 'D:\dsh1\meArm\esp8266\meArm_wifi'
```

或者直接在文件里把 `#define PRESET PRESET_TASK5` 改成 `PRESET_VTEST` / `PRESET_MEARM165`
（用 V-test 时 `ARM_BAUD` 保持 9600；用 meArm v1.6.5 要改成 `115200`）。
用 meArm v1.6.5 时关节滑条变成 `x/y/z` 三个，三条值会拼成**一条** `x90,y80,z100` 发出去
（`JOINT_ONE_PER_LINE=0`），不排队。

---

## 7. 协议要点

### 7.1 Task5 系列（默认，实机对照以 `others/f/MeArm/Task5_LineN2_TeachFix` 为准）

串口 9600 8N1，**一条命令一行**（`\n` 结束），**单个 ASCII 字符**，大小写不敏感：

| 字符 | 作用 | 字符 | 作用 |
|---|---|---|---|
| `1` / `t` | 示教/记录（未在示教则进入示教模式） | `2` | 暂停 |
| `3` | 继续 | `4` / `x` | 取消，回待机 |
| `5` / `s` | 折线绘制（示教 ≥2 点） | `6` / `c` | 曲线绘制（示教 ≥3 点） |
| `7` / `g` | 直线演示（6 坐标） | `8` | 基础图形（字母 N） |
| `p` | 暂停/继续切换 | `?` | 状态查询 |

* 回包用前缀区分：`>>> …` 进度、`! …` 拒绝/错误，开机横幅 `=== Task5 轨迹绘制 ===`。
* **没有 `DONE`/`OK` 完成行**，所以本预设**不用发送队列**（按钮点一下就发一条），
  自动查状态靠 `?` 的回包（`状态:n 示教点数:m  笔尖(x,y,z): x, y, z`，n：0待机 1示教 2绘制 3暂停）。
* 绘制/示教**中途**再发 `7`/`8` 会被固件忽略并回 `! 绘制/示教中，忽略`（这是固件设计）。
* 没有角度指令 → 网页不显示关节滑条；想手动摆位就用原来的摇杆，或切到别的预设。

### 7.2 V-test v3.3（实机对照以 `others/RC-robot-hand/V-test` 为准）

串口 9600 8N1，命令以 `\n` 或 `\r` 结束，**大小写不敏感**，空行忽略。

两条最容易踩的约束：

1. **一条命令只能带一个关节角**。固件对角度参数要求"整串解析完"（`strtod` 后必须到结尾），
   所以 `B90 R120 F70 C122` 一行会回 `ERR:ANGLE`；要发四行。
2. **机械臂在动作中会拒绝新命令**。动作类命令前有 `if(mode!=IDLE){ BUSY; STOP or X first }`，
   连发会被 `BUSY` 拒掉。必须等 `DONE:POSITION` / `DONE:E_HOME; joystick ON` /
   `DONE:V_REPLAY` 这类完成行。**本程序的发送队列就是干这个的**（等 `DONE`/`ERR`/`BUSY`，
   超时 3.5 秒继续下一条）。

回包格式：`STATE:IDLE`、`B=90.00 R=90.00 F=90.00 C=120.00`、`JOY:0 CLAW_LOCK:1`、
`RAM:1 samples=300 ms=30000`、`DONE:*`、`ERR:*`。

---

## 8. HTTP 接口（二次开发/自己写上位机用）

| 请求 | 作用 |
|---|---|
| `GET /` | 遥控网页 |
| `GET /buttons` | 按钮表 JSON：`[{"g":组,"l":标签,"c":命令,"f":标志,"d":是否要确认}]` |
| `GET /joints` | `{"one":1,"list":[{"n":"B","lo":0,"hi":180,"v":90},…]}`；无关节的预设回 `{"one":0,"list":[]}` |
| `GET /state` | 状态 + `cards`（状态卡片）+ **只含上次以来新回包**的 `new` 数组 |
| `GET /cmd?c=7` | 直接发一条命令；加 `&q=1` 则只排进队列 |
| `GET /joint?i=0&v=45` | 设第 0 个关节；`one=0` 的固件再带 `&all=v0,v1,v2` |
| `GET /queue?clear=1` | 清空发送队列 |
| `GET /auto?on=0` | 关/开自动查状态 |
| `GET /log.txt` | 下载最近 28 行回包 |
| `GET/POST /wifi` | WiFi 配置页 / 保存并重启 |
| `GET /reboot` | 重启 |

---

## 9. 常见问题

| 现象 | 原因/处理 |
|---|---|
| 给 Uno 烧录失败，`not in sync: resp=0x00` 刷屏 | **先拔掉 ESP 的 D6（和 D5）**：Uno 的 D0/D1 被 ESP 和 USB 芯片同时驱动。也可能是端口选错（ESP/Uno 各占一个 COM）、串口监视器占口、克隆板要选 `Processor: ATmega328P (Old Bootloader)` |
| 网页能开，但发命令没反应 | 机械臂串口线接反（TX↔RX 交叉）、没共地、波特率不对（Task5/V-test 都是 9600） |
| 日志里全是乱码 | 波特率不匹配或没共地；也有可能是 ESP 收到的 5V 电平没分压导致丢位 |
| 偶发丢一两个字符 | 软串口 + WiFi 同跑的长短中断冲突。改用 `ARM_LINK_SOFTWARE 0` 走硬件串口 |
| Task5 发 `7`/`8` 回 `! 绘制/示教中，忽略` | 固件正在绘制/示教，先发 `4` 取消或 `2` 暂停 |
| Task5 发 `5`/`6` 回 `! 示教点不足…` | 折线/曲线要先按键 `1` 进示教、移动摇杆、再按 `1` 记录点（折线 ≥2 点、曲线 ≥3 点） |
| V-test 发一条命令回 `BUSY; STOP or X first` | 机械臂还在动作中，等 `DONE` 或先发 `X` |
| V-test 发 `B90 R120` 回 `ERR:ANGLE` | 一条命令只能一个关节角，分开发（网页滑条已经自动排队） |
| V-test 点"舵机脱力"后手臂垂下来 | 正常，`D` 就是切断舵机力矩；发 `E` 恢复 |
| ESP 一动就重启 | 供电不足：换独立 AMS1117-3.3 + 470uF，别用 Uno 的 3.3V |
| `mearm.local` 打不开 | 有些安卓/路由器不支持 mDNS，直接用串口打印的 IP |
| 想恢复出厂 | 配网页里两个框留空保存，或重烧固件后 15 秒内连 `meArm-XXXXXX` 热点 |

---

## 10. 已验证 / 未验证

* 已验证（编译）：四个配置在 arduino-cli 1.5.1 + esp8266 3.1.1 下**全部 exit 0、0 warning
  0 error** —— 默认 Task5、`-DPRESET=1`、`-DPRESET=2`、`-DARM_LINK_SOFTWARE=0`
  （Flash 322 760 / 322 784 / 322 976 / 318 896 B，RAM 约 42%，IRAM 92~95%）。
* 已验证（网页）：把内嵌 HTML 抠出来用 `node --check` 过语法；再用三份假数据在浏览器里
  渲染 —— Task5 形状（无滑条、中文状态、8 张卡片、按钮分组与红/确认标记）、V-test 形状
  （4 滑条 + `one=1` 排队）、meArm 形状（3 滑条 + `one=0` 一次发），均无横向溢出。
  ESP 端解析 `?` 回包的字节偏移也用 node 按字节验算过（`状态:`7 字节、`示教点数:`13 字节、
  `笔尖(x,y,z):`14 字节）。
* 未验证（需要硬件）：与真实机械臂的串口收发、Task5 中文回包的实时状态跟踪、WiFi 配网、OTA。
