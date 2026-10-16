param(
  [string]$Repo = (Split-Path -Parent $PSScriptRoot),
  [string]$OutDir = $null
)
$ErrorActionPreference = 'Stop'
if (-not $OutDir) { $OutDir = Join-Path $Repo 'single\arduino_keypad' }
New-Item -ItemType Directory -Force -Path $OutDir | Out-Null
$outFile = Join-Path $OutDir 'arduino_keypad.ino'

$src = Join-Path $Repo 'arduino_keypad\arduino_keypad.ino'
if (-not (Test-Path -LiteralPath $src)) { throw "missing source: $src" }
$lines = [System.IO.File]::ReadAllLines($src, [System.Text.UTF8Encoding]::new($false))

$chunks = New-Object System.Collections.Generic.List[string]
$chunks.Add('// ============================================================================')
$chunks.Add('// meArm 按键板 —— 单文件交付稿（第二块 Arduino：UNO / Nano）')
$chunks.Add('// ----------------------------------------------------------------------------')
$chunks.Add('// 这是什么：一块独立的"按键遥控板"。4 个按键 + 1 个 LED，通过软串口把命令字符')
$chunks.Add('//   发给跑 meArm 主控固件的那块 Arduino，代替它板上那 4 个物理按键。')
$chunks.Add('//   本文件只依赖 Arduino 核心自带的 <SoftwareSerial.h>，不依赖本仓库其它文件。')
$chunks.Add('//')
$chunks.Add('// 从哪来：由仓库里的 .selfcheck/make_keypad_single.ps1 生成 ——')
$chunks.Add('//   arduino_keypad/arduino_keypad.ino 逐字拷贝 + 本说明头。')
$chunks.Add('//   要改功能请回去改源文件、再重新生成，别只改这一份。')
$chunks.Add('//')
$chunks.Add('// 怎么上传：Arduino IDE 要求 **.ino 文件名与所在文件夹同名**。')
$chunks.Add('//   本文件叫 arduino_keypad.ino，所以它必须在名为 arduino_keypad 的文件夹里')
$chunks.Add('//   （仓库里就是 single/arduino_keypad/arduino_keypad.ino）。双击打开 →')
$chunks.Add('//   选开发板 Arduino Uno（Nano 也一样）→ 选端口 → 上传。不需要装任何库。')
$chunks.Add('//')
$chunks.Add('// 配套文件（都不影响上传，只是说明与辅助）：')
$chunks.Add('//   arduino_keypad/README.md    接线、两套预设、按键映射的详细说明')
$chunks.Add('//   arduino_keypad/keysim.js    把按键状态机搬到 PC 上用 node 跑的波形复核')
$chunks.Add('//                               （纯离线模拟，不连板子、与上传无关）')
$chunks.Add('//')
$chunks.Add('// 实测占用（avr-gcc 7.3.0 + Arduino AVR core 1.8.8，atmega328p）：')
$chunks.Add('//   Program 6170 字节（18.8%） / Data 975 字节（47.6%）—— UNO 上很宽裕。')
$chunks.Add('//')
$chunks.Add('// 主控固件要求：本板发出的 N/R/P/M/0 属于主控的"按键模块"。本仓库 v1.6.8 的')
$chunks.Add('//   默认功能组合是「取放 + 绘图，按键关」，那 5 个槽位会被主控回 REJECTED；')
$chunks.Add('//   要配合本板，请把主控改成「取放 + 按键（绘图关）」那套开关组合。')
$chunks.Add('//   详细对照见下面源码开头的注释（"和主控固件的配套关系"一节）。')
$chunks.Add('// ============================================================================')
$chunks.Add('')
foreach ($line in $lines) { $chunks.Add($line) }

# LF-only：与 .gitattributes 的 "* text=auto eol=lf" 及 make_single.ps1 保持一致。
$text = [string]::Join("`n", $chunks) + "`n"
[System.IO.File]::WriteAllText($outFile, $text, [System.Text.UTF8Encoding]::new($false))
$lineCount = $text.Split("`n").Length - 1
Write-Host ("generated {0} ({1} bytes, {2} lines; source {3} lines)" -f `
  $outFile, (Get-Item -LiteralPath $outFile).Length, $lineCount, $lines.Count)
