# make_nocomment.ps1 -- 生成"无注释"源码包（多文件版 + 单文件版）
#
# 用法：
#     pwsh -File .selfcheck\make_nocomment.ps1 [-SkipZip] [-ProbeCheck] [-KeepStage]
#
# 做什么：
#   1) 调 .selfcheck/strip_comments.py（compact 模式）把 21 个固件文件与单文件版的
#      single/meArm/meArm.ino 去注释，落到 build\nocomment\pkg 下的两个交付目录；
#   2) 断言去注释结果里不再出现 /* 与 //；
#   3) 用与 run_all.ps1 完全相同的严格参数（-Wall -Wextra -Wshadow -Wconversion）
#      编译每一个去注释文件，要求退出码 0 且零输出（零警告）；
#   4) 用 g++ -E -P 取"原文"与"去注释版"的预处理记号流，折叠空白后比 SHA256，
#      两种宏配置（全功能 / 全关）各比一遍 —— 只有注释变了才会哈希相同；
#   5) -ProbeCheck 时在 build\nocomment\verify 里搭一棵"去注释固件 + 完整 .selfcheck"
#      的树，跑那份 run_all.ps1（9 个 TU 零警告 + 11 个探针 ALL PASS）；
#   6) 每个包写一份 README.txt，Compress-Archive 成
#      dist\meArm_v1.6.4_multifile_nocomment.zip 与 dist\meArm_v1.6.4_single_nocomment.zip；
#   7) 报告写 .selfcheck\out\nocomment_report.md。
#
# 为什么这样验证：去注释是纯文本手术，怕的是动到字符串字面量、行尾续行、条件编译
# 分支。所以既做"严格编译零警告"，又做"预处理记号流逐字符相同"（与换行/空白无关），
# 再用 11 个探针跑一遍去注释后的固件。AVR 尺寸另用 avr_build.ps1 -SketchDir 复核。

[CmdletBinding()]
param(
    [switch]$SkipZip,
    [switch]$ProbeCheck,
    [switch]$KeepStage
)

$ErrorActionPreference = 'Stop'

$root  = Split-Path -Parent $PSScriptRoot
$mock  = Join-Path $PSScriptRoot 'mock'
$gpp   = 'C:\ProgramData\mingw64\mingw64\bin\g++.exe'
$py    = 'python'
$stage = Join-Path $root 'build\nocomment'
$pkg   = Join-Path $stage 'pkg'
$outd  = Join-Path $PSScriptRoot 'out'
New-Item -ItemType Directory -Force -Path $outd | Out-Null
if (-not $KeepStage) {
    Remove-Item -Recurse -Force $pkg -ErrorAction SilentlyContinue
    Remove-Item -Recurse -Force (Join-Path $stage 'verify') -ErrorAction SilentlyContinue
}

$mfSketch = Join-Path $pkg 'meArm_multifile_nocomment\meArm'
$sfSketch = Join-Path $pkg 'meArm_single_nocomment\meArm'
New-Item -ItemType Directory -Force -Path $mfSketch, $sfSketch | Out-Null

# 21 个固件文件（git ls-files 核过）
$firmware = @(
    'button_control.cpp', 'button_control.h',
    'constant_and_positions.cpp', 'constant_and_positions.h',
    'draw_control.cpp', 'draw_control.h',
    'joystick_control.cpp', 'joystick_control.h',
    'meArm.ino',
    'move.cpp', 'move.h',
    'path_core.h',
    'pick_place.cpp', 'pick_place.h',
    'protocol_constants.cpp', 'protocol_constants.h',
    'serial_protocol.cpp', 'serial_protocol.h',
    'servo_drive.cpp', 'servo_drive.h',
    'weArm_config.h'
)

$cfg  = @('-DWEARM_DEBUG_SERIAL=1', '-DWEARM_ENABLE_PICK_PLACE=1',
          '-DWEARM_ENABLE_BUTTONS=1', '-DWEARM_ENABLE_DRAW=1')
$cfgOff = @('-DWEARM_ENABLE_PICK_PLACE=0', '-DWEARM_ENABLE_BUTTONS=0',
            '-DWEARM_ENABLE_DRAW=0')

$fail = 0
$rows = New-Object System.Collections.Generic.List[object]

function Get-Sha256([string]$s) {
    $sha = [System.Security.Cryptography.SHA256]::Create()
    $bytes = [System.Text.Encoding]::UTF8.GetBytes($s)
    return -join ($sha.ComputeHash($bytes) | ForEach-Object { $_.ToString('x2') })
}

# g++ 的警告走 stderr；本脚本 $ErrorActionPreference='Stop' 会让"本地命令往 stderr 写了
# 一个字"直接变成 NativeCommandError 并中断脚本（第一版就死在 meArm.ino 的
# "linker input file unused" 警告上）。所以所有外部命令都过这道门：临时把
# ErrorActionPreference 放成 Continue，由调用者自己看退出码与输出行数。
function Invoke-Capture([string[]]$argv) {
    $prev = $ErrorActionPreference
    $ErrorActionPreference = 'Continue'
    try {
        $out = & $argv[0] $argv[1..($argv.Count - 1)] 2>&1
        $code = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $prev
    }
    return [pscustomobject]@{ Out = @($out); Code = $code }
}

function Get-TokenHash([string]$path, [string[]]$incs, [string[]]$defs) {
    $r = Invoke-Capture (@($gpp, '-E', '-P', '-x', 'c++', '-std=gnu++17') + $defs + $incs + @($path))
    if ($r.Code -ne 0) { return $null }
    $flat = (($r.Out -join "`n") -replace '\s+', ' ').Trim()
    return Get-Sha256 $flat
}

Write-Host '=== 1) 去注释 ==='
$items = @()
foreach ($f in $firmware) {
    $items += [pscustomobject]@{ Src = (Join-Path $root $f); Dst = (Join-Path $mfSketch $f);
                                 Group = '多文件'; Sketch = 'multi' }
}
$items += [pscustomobject]@{ Src = (Join-Path $root 'single\meArm\meArm.ino');
                            Dst = (Join-Path $sfSketch 'meArm.ino');
                            Group = '单文件'; Sketch = 'single' }

foreach ($it in $items) {
    $line = (& $py (Join-Path $PSScriptRoot 'strip_comments.py') $it.Src $it.Dst) 2>&1
    if ($LASTEXITCODE -ne 0) { Write-Host "  [FAIL] 去注释失败：$($it.Src)" -ForegroundColor Red; $fail++; continue }
    $txt = [System.IO.File]::ReadAllText($it.Dst)
    $blk = ([regex]::Matches($txt, '/\*')).Count
    $lin = ([regex]::Matches($txt, '//')).Count
    $inB  = (Get-Item $it.Src).Length
    $outB = (Get-Item $it.Dst).Length
    $ok = ($blk -eq 0 -and $lin -eq 0)
    if (-not $ok) { $fail++; Write-Host "  [FAIL] 仍残留注释标记：$($it.Dst) /*=$blk //=$lin" -ForegroundColor Red }

    $rows.Add([pscustomobject]@{
        Group = $it.Group; Name = (Split-Path -Leaf $it.Dst)
        InB = $inB; OutB = $outB
        Lines = ($txt -split "`n").Count
        Marks = "$blk/$lin"; Ok = $ok
    })
    Write-Host ("  {0,-9} {1,-28} {2,7} -> {3,7} B  ({4,4}%)  注释标记={5}" -f `
        $it.Group, (Split-Path -Leaf $it.Dst), $inB, $outB,
        [math]::Round(100.0 * $outB / $inB, 1), "$blk/$lin")
}

Write-Host '=== 2) 严格编译（零警告） ==='
$objDir = Join-Path $stage 'obj'
Remove-Item -Recurse -Force $objDir -ErrorAction SilentlyContinue
New-Item -ItemType Directory -Force -Path $objDir | Out-Null
foreach ($it in $items) {
    $incs = if ($it.Sketch -eq 'multi') { @("-I$mfSketch", "-I$mock") } else { @("-I$sfSketch", "-I$mock") }
    $obj  = Join-Path $objDir ((Split-Path -Leaf $it.Dst) + '.o')
    # .ino 不是自足的 C++ 翻译单元（要靠包含者先引 Arduino.h），g++ 也不能按扩展名认它，
    # 所以显式 -x c++ 加 -include Arduino.h；.cpp/.h 上这两项无害。
    $argv = @($gpp, '-std=gnu++17', '-O2', '-Wall', '-Wextra', '-Wshadow', '-Wconversion') + $cfg + $incs
    if ($it.Dst -like '*.ino') { $argv += @('-include', 'Arduino.h') }
    $argv += @('-x', 'c++', '-c', $it.Dst, '-o', $obj)
    $r   = Invoke-Capture $argv
    $log = $r.Out
    $n   = @($log).Count
    if ($r.Code -ne 0 -or $n -gt 0) {
        $fail++
        Write-Host "  [FAIL] 编译有输出：$(Split-Path -Leaf $it.Dst)" -ForegroundColor Red
        $log | Select-Object -First 20 | ForEach-Object { Write-Host "        $_" -ForegroundColor DarkYellow }
    } else {
        Write-Host "  [ OK ] $(Split-Path -Leaf $it.Dst) 零警告"
    }
}

Write-Host '=== 3) 预处理记号流等价（两种宏配置） ==='
# 坑：PowerShell 里逗号优先于加号，写 "@('-I' + X, '-I' + Y)" 会被解析成
# "'-I' + (X, Y)"，两个 -I 拼成一个参数 → 预处理找不到 <Arduino.h> → 返回 $null 假失败。
# 所以每个元素都单独括起来。
$singleRoot = Join-Path $root 'single\meArm'
$shaEmpty   = Get-Sha256 ''
foreach ($it in $items) {
    $incs    = if ($it.Sketch -eq 'multi') { @("-I$mfSketch", "-I$mock") } else { @("-I$sfSketch", "-I$mock") }
    $incsSrc = if ($it.Sketch -eq 'multi') { @("-I$root", "-I$mock") } else { @(("-I" + $singleRoot), ("-I" + $mock)) }
    foreach ($pair in @(@{ Name = '全功能'; Defs = $cfg }, @{ Name = '全关'; Defs = $cfgOff })) {
        $h1 = Get-TokenHash $it.Src $incsSrc $pair.Defs
        $h2 = Get-TokenHash $it.Dst $incs  $pair.Defs
        if ($null -eq $h1 -or $null -eq $h2 -or $h1 -ne $h2) {
            $fail++
            Write-Host "  [FAIL] 记号流不同：$(Split-Path -Leaf $it.Dst) [$($pair.Name)]" -ForegroundColor Red
        } else {
            # 纯声明型头文件（weArm_config.h）预处理完就是空的：两边都空只能证明"都没展开东西"，
            # 标出来免得把"空 == 空"误读成强验证。
            $note = if ($h1 -eq $shaEmpty) { '  (预处理结果为空：只有宏声明，没有代码)' } else { '' }
            Write-Host ("  [ OK ] {0} [{1}] {2}{3}" -f (Split-Path -Leaf $it.Dst), $pair.Name, $h1.Substring(0, 16), $note)
        }
    }
}

if ($ProbeCheck) {
    Write-Host '=== 4) 去注释固件 + 完整自检（9 TU + 11 探针） ==='
    $ver = Join-Path $stage 'verify'
    Remove-Item -Recurse -Force $ver -ErrorAction SilentlyContinue
    New-Item -ItemType Directory -Force -Path $ver | Out-Null
    foreach ($it in $items) {
        if ($it.Sketch -eq 'multi') { Copy-Item $it.Dst (Join-Path $ver (Split-Path -Leaf $it.Dst)) -Force }
    }
    Copy-Item (Join-Path $root 'single\meArm\meArm.ino') (Join-Path $ver 'meArm_single.ino') -Force
    New-Item -ItemType Directory -Force -Path (Join-Path $ver 'single\meArm') | Out-Null
    Copy-Item (Join-Path $sfSketch 'meArm.ino') (Join-Path $ver 'single\meArm\meArm.ino') -Force
    Copy-Item $PSScriptRoot (Join-Path $ver '.selfcheck') -Recurse -Force
    $log = & (Join-Path $ver '.selfcheck\run_all.ps1') 2>&1
    $log | ForEach-Object { Write-Host "    $_" }
    if ($LASTEXITCODE -ne 0) { $fail++; Write-Host '  [FAIL] 去注释树的自检未全过' -ForegroundColor Red }
    else { Write-Host '  [ OK ] 去注释树：9 TU 零警告 + 11 探针 ALL PASS' }
}

Write-Host '=== 5) README.txt + 打包 ==='
$readme = @"
meArm v1.6.4 源码（无注释版，__GROUP__）
========================================

这个包和仓库里的源码**功能完全一样**，只是把所有注释删掉了。

怎么来的：.selfcheck\strip_comments.py（一个 C/C++ 词法状态机）逐个文件删注释，
字符串字面量里的 // 与 /* 不会被动到，行尾以反斜杠续行的 // 注释按 C 规则处理。

怎么保证没删错：每个文件都做了"预处理记号流逐字符相同"的比对
（g++ -E -P 的输出折叠空白后比 SHA256，全功能与功能全关两种宏配置各比一遍），
并且用 -Wall -Wextra -Wshadow -Wconversion 严格编译过、零警告。

__FILES__

要不要带注释的版本：仓库根目录的 21 个文件（多文件版）或 single/meArm/meArm.ino
（单文件版）就是带注释版；.selfcheck\README.md 里有全部设计说明。

注意：注释删掉后行号不再与仓库里的带注释版对应，串口调试或编译报错的行号会不一样。
"@

$mfList = ($firmware | ForEach-Object { "    meArm\$_" }) -join "`n"
$sfList = "    meArm\meArm.ino（__LINES__ 行，去注释后 __OUTB__ B）"
$mfText = $readme.Replace('__GROUP__', '多文件版').Replace('__FILES__', "包内文件（21 个，Arduino 草图目录 meArm\）：`n$mfList")
$sfRow  = $rows | Where-Object { $_.Group -eq '单文件' } | Select-Object -First 1
$sfText = $readme.Replace('__GROUP__', '单文件版').Replace('__FILES__',
    $sfList.Replace('__LINES__', $sfRow.Lines).Replace('__OUTB__', $sfRow.OutB))

$utf8 = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllText((Join-Path $pkg 'meArm_multifile_nocomment\README.txt'), ($mfText -replace "`r`n", "`n"), $utf8)
[System.IO.File]::WriteAllText((Join-Path $pkg 'meArm_single_nocomment\README.txt'),   ($sfText -replace "`r`n", "`n"), $utf8)

if (-not $SkipZip) {
    $z1 = Join-Path $root 'dist\meArm_v1.6.4_multifile_nocomment.zip'
    $z2 = Join-Path $root 'dist\meArm_v1.6.4_single_nocomment.zip'
    Compress-Archive -Path (Join-Path $pkg 'meArm_multifile_nocomment') -DestinationPath $z1 -Force
    Compress-Archive -Path (Join-Path $pkg 'meArm_single_nocomment')    -DestinationPath $z2 -Force
    Write-Host ("  {0}  {1} B" -f $z1, (Get-Item $z1).Length)
    Write-Host ("  {0}  {1} B" -f $z2, (Get-Item $z2).Length)
}

Write-Host '=== 6) 报告 ==='
$rep = New-Object System.Collections.Generic.List[string]
$rep.Add('# 无注释版打包报告')
$rep.Add('')
$rep.Add("生成时间：$(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')")
$rep.Add('')
$rep.Add('| 版本 | 文件 | 原始 B | 去注释 B | 比例 | 去注释后行数 | 注释标记 |')
$rep.Add('| --- | --- | ---: | ---: | ---: | ---: | --- |')
foreach ($r in $rows) {
    $rep.Add(("| {0} | {1} | {2} | {3} | {4}% | {5} | {6} |" -f `
        $r.Group, $r.Name, $r.InB, $r.OutB, [math]::Round(100.0 * $r.OutB / $r.InB, 1), $r.Lines, $r.Marks))
}
$totIn  = ($rows | Measure-Object -Property InB  -Sum).Sum
$totOut = ($rows | Measure-Object -Property OutB -Sum).Sum
$rep.Add('')
$rep.Add("合计：$totIn B -> $totOut B（$( [math]::Round(100.0*$totOut/$totIn,1) )%），多文件版不含单文件版。")
$rep.Add('')
$rep.Add('验证：每个文件均通过 ①去注释后无 /* 与 // ②严格编译零警告 ③g++ -E -P 记号流 SHA256 与原文相同（全功能/全关两种宏配置）。')
if ($ProbeCheck) { $rep.Add('另外在 build\nocomment\verify 里用去注释后的固件跑了一遍完整自检（9 TU 零警告 + 11 探针 ALL PASS）。') }
$rep.Add('')
$rep.Add("失败项：$fail")
$repText = ($rep -join "`n") + "`n"
[System.IO.File]::WriteAllText((Join-Path $outd 'nocomment_report.md'), $repText, $utf8)
Write-Host "  $outd\nocomment_report.md"

Write-Host ''
if ($fail -gt 0) { Write-Host ">>> 有 $fail 项失败" -ForegroundColor Red; exit 1 }
Write-Host '>>> 无注释版打包完成，全部检查通过' -ForegroundColor Green
exit 0
