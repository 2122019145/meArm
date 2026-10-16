#
# compile_only.ps1 -- 只做编译验证（跳过全部探针）
#
# 【为什么有这个脚本】探针要跑 11 个程序（含 4000 步随机压力、整条绘图轨迹），
# 一轮几十秒到几分钟；改代码时大多数时候只想确认"还能编过、还是零警告"。
# 这个脚本就是 run_all.ps1 的第 1 步，参数与判据完全一致：
#   9 个固件 TU 用 -Wall -Wextra -Wshadow -Wconversion 严格编译，**输出 0 行才算过**。
#
# 用法：
#   powershell -NoProfile -ExecutionPolicy Bypass -File D:\dsh1\meArm\.selfcheck\compile_only.ps1
#   ... -File ...\compile_only.ps1 -Avr        # 额外跑 AVR 体积校验（avr_build.ps1）
#
# 【注意 PowerShell / g++ 的坑】g++ 的 warning 走 stderr；`& $gpp ... 2>&1` 赋给变量后
#   $LASTEXITCODE 仍是 0，所以判定必须同时看"输出行数 > 0"。
# 【本文件的编码】必须存成 UTF-8 **带 BOM**（Windows PowerShell 5.1 读无 BOM 的 .ps1
#   时会按 ANSI(GBK) 解码，中文注释变乱码并破坏语法分析）。
#
param(
    [switch]$Avr        # 顺带跑 avr_build.ps1（Arduino Uno 目标体积/内存）
)

$ErrorActionPreference = 'Continue'
$gpp  = 'C:\ProgramData\mingw64\mingw64\bin\g++.exe'
$root = Split-Path -Parent $PSScriptRoot          # .selfcheck 的上一级 = 工程根
$sc   = $PSScriptRoot
$out  = Join-Path $sc 'out'
$mock = Join-Path $sc 'mock'

if (-not (Test-Path $out)) { New-Item -ItemType Directory -Path $out | Out-Null }

$fw  = @('constant_and_positions.cpp', 'servo_drive.cpp', 'move.cpp', 'joystick_control.cpp', 'protocol_constants.cpp', 'serial_protocol.cpp', 'pick_place.cpp', 'button_control.cpp', 'draw_control.cpp') | ForEach-Object { Join-Path $root $_ }
$inc = @("-I$root", "-I$mock")

# 与 run_all.ps1 第 1 步一致：PC 端固定用「全功能 + 调试日志开」的配置。
$cfg = @('-DWEARM_DEBUG_SERIAL=1', '-DWEARM_ENABLE_PICK_PLACE=1', '-DWEARM_ENABLE_BUTTONS=1', '-DWEARM_ENABLE_DRAW=1')
$fail = 0

Write-Host '================ 固件严格编译（0 警告才算过；探针已跳过）================'
foreach ($f in $fw) {
    $name = Split-Path -Leaf $f
    $log = & $gpp -std=gnu++17 -O2 -Wall -Wextra -Wshadow -Wconversion @cfg -c @inc $f -o (Join-Path $out ($name + '.o')) 2>&1
    $n = ($log | Measure-Object).Count
    if ($LASTEXITCODE -ne 0 -or $n -gt 0) {
        Write-Host ("  [FAIL] {0}  EXIT={1}  输出 {2} 行" -f $name, $LASTEXITCODE, $n) -ForegroundColor Red
        $log | Select-Object -First 20 | ForEach-Object { Write-Host "         $_" }
        $fail++
    } else {
        Write-Host ("  [ OK ] {0}  EXIT=0  输出 0 行（零警告）" -f $name) -ForegroundColor Green
    }
}

if ($Avr) {
    Write-Host ''
    Write-Host '================ AVR 体积校验（avr_build.ps1）================'
    & (Join-Path $sc 'avr_build.ps1')
    if ($LASTEXITCODE -ne 0) { $fail++ }
}

Write-Host ''
if ($fail -eq 0) {
    Write-Host '>>> 编译验证通过（固件零警告）' -ForegroundColor Green
    exit 0
} else {
    Write-Host (">>> 有 {0} 项失败" -f $fail) -ForegroundColor Red
    exit 1
}
