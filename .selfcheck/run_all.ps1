#
# run_all.ps1 -- wearm 工程 PC 端全量自检
#
# 用法（任意目录）：
#     D:\dsh1\wearm\.selfcheck\run_all.cmd
# 或  powershell -NoProfile -ExecutionPolicy Bypass -File D:\dsh1\wearm\.selfcheck\run_all.ps1
#
# 做三件事：
#   1. 严格编译 3 个固件 TU（-Wall -Wextra -Wshadow -Wconversion）
#   2. 严格编译并运行 5 个自检程序，逐个要求 "ALL PASS" 且退出码 0
#   3. 汇总退出码（任一失败则 exit 1）
#
# 【注意 PowerShell / g++ 的坑】
#   g++ 的 warning 走 stderr；`& $gpp ... 2>&1` 赋给变量后 $LASTEXITCODE 仍是 0。
#   所以判定编译失败必须同时看"输出行数 > 0"，只查退出码会漏掉警告
#   （历史上就这样漏过两个 -Wunused-* 警告）。
# 【本文件的编码】必须存成 UTF-8 **带 BOM**：Windows PowerShell 5.1 读无 BOM 的
#   .ps1 时会按 ANSI(GBK) 解码，中文注释会变成乱码并破坏语法分析。
#
$ErrorActionPreference = 'Continue'
$gpp  = 'C:\ProgramData\mingw64\mingw64\bin\g++.exe'
$root = Split-Path -Parent $PSScriptRoot          # .selfcheck 的上一级 = 工程根
$sc   = $PSScriptRoot
$out  = Join-Path $sc 'out'
$mock = Join-Path $sc 'mock'

if (-not (Test-Path $out)) { New-Item -ItemType Directory -Path $out | Out-Null }

$fw  = @('constant_and_positions.cpp', 'move.cpp', 'joystick_control.cpp') | ForEach-Object { Join-Path $root $_ }
$inc = @("-I$root", "-I$mock")
$fail = 0

Write-Host '================ 1) 固件严格编译（0 警告才算过）================'
foreach ($f in $fw) {
    $name = Split-Path -Leaf $f
    $log = & $gpp -std=gnu++17 -O2 -Wall -Wextra -Wshadow -Wconversion -c @inc $f -o (Join-Path $out ($name + '.o')) 2>&1
    $n = ($log | Measure-Object).Count
    if ($LASTEXITCODE -ne 0 -or $n -gt 0) {
        Write-Host ("  [FAIL] {0}  EXIT={1}  输出 {2} 行" -f $name, $LASTEXITCODE, $n) -ForegroundColor Red
        $log | Select-Object -First 20 | ForEach-Object { Write-Host "         $_" }
        $fail++
    } else {
        Write-Host ("  [ OK ] {0}  EXIT=0  输出 0 行（零警告）" -f $name) -ForegroundColor Green
    }
}

Write-Host ''
Write-Host '================ 2) 自检程序（必须 ALL PASS）================'
# 每个自检程序都链接全部固件 TU + 两个 mock。
# 【为什么不再按探针分别配源文件】早期按"这个探针需要哪些 .cpp"逐个列，
# 结果 $fw 里已经含 move.cpp、又在 extra 里再列一次，触发
# "multiple definition of moveJointStep(int, double)" 链接错误。
# 全部链接既简单又不会漏（未用到的目标文件由链接器按需取舍）。
$probes = @('probe_axes', 'probe_rt', 'probe_move', 'probe_joystick', 'wearm_ino_test')
foreach ($n in $probes) {
    $src  = Join-Path $sc ($n + '.cpp')
    $exe  = Join-Path $out ($n + '.exe')
    $srcs = @($src) + $fw + @((Join-Path $mock 'Arduino.cpp'), (Join-Path $mock 'Servo.cpp'))
    $clog = & $gpp -std=gnu++17 -O2 -Wall -Wextra -Wshadow @inc @srcs -o $exe 2>&1
    $cn = ($clog | Measure-Object).Count
    if ($LASTEXITCODE -ne 0 -or $cn -gt 0) {
        Write-Host ("  [FAIL] {0} 编译  EXIT={1}  输出 {2} 行" -f $n, $LASTEXITCODE, $cn) -ForegroundColor Red
        $clog | Select-Object -First 20 | ForEach-Object { Write-Host "         $_" }
        $fail++
        continue
    }
    $rlog = & $exe 2>&1
    $rc   = $LASTEXITCODE
    $txt  = ($rlog | Out-String)
    if ($txt -match 'ALL PASS' -and $rc -eq 0) {
        Write-Host ("  [ OK ] {0}  零警告编译 + ALL PASS" -f $n) -ForegroundColor Green
    } else {
        Write-Host ("  [FAIL] {0}  EXIT={1}" -f $n, $rc) -ForegroundColor Red
        $rlog | Where-Object { $_ -match 'FAIL|>>>' } | ForEach-Object { Write-Host "         $_" }
        $fail++
    }
    # 完整输出存盘，方便回看细节
    $rlog | Set-Content -Encoding UTF8 (Join-Path $out ($n + '.log'))
}

Write-Host ''
if ($fail -eq 0) {
    Write-Host '>>> 全部通过（固件零警告 + 5 个自检 ALL PASS）' -ForegroundColor Green
    exit 0
} else {
    Write-Host (">>> 有 {0} 项失败" -f $fail) -ForegroundColor Red
    exit 1
}
