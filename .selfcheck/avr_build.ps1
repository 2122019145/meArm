#
# avr_build.ps1 -- build the meArm sketch exactly the way the Arduino IDE does,
# with the locally installed AVR toolchain, and report flash/SRAM usage.
#
# Toolchain (found on this machine):
#   avr-gcc 7.3.0 : %LOCALAPPDATA%\Arduino15\packages\arduino\tools\avr-gcc\7.3.0-atmel3.6.1-arduino7\bin
#   AVR core 1.8.8: %LOCALAPPDATA%\Arduino15\packages\arduino\hardware\avr\1.8.8
#
# v1.6.6: the sketch's servo layer (servo_drive.cpp) is a thin wrapper over the
# official Arduino Servo library, so that library IS compiled and linked here.
# (v1.3.0~v1.6.5 used a self-written Timer1 driver and deliberately skipped it --
#  that is why the -I/-l for Servo disappeared back then.)
#
# Flags are copied from the core's platform.txt (LTO is enabled by that core!).
#
[CmdletBinding()]
param(
  [string]$SketchDir  = 'D:\dsh1\meArm',
  [string]$BuildRoot  = 'D:\dsh1\meArm\.selfcheck\avrbuild',
  [switch]$Full,       # also rebuild the core archive
  [switch]$Symbols,    # print the biggest symbols after linking
  [switch]$NoLto,      # disable LTO (per-object sizes become meaningful)
  [switch]$Relax,      # enable AVR jump/call relaxation (-mrelax) to shrink flash
  [string[]]$ExtraDefs = @()   # extra -D... switches (used for size ablations)
)

$ErrorActionPreference = 'Stop'

$ard      = Join-Path $env:LOCALAPPDATA 'Arduino15\packages\arduino'
$gccbin   = Join-Path $ard 'tools\avr-gcc\7.3.0-atmel3.6.1-arduino7\bin'
$coreDir  = Join-Path $ard 'hardware\avr\1.8.8\cores\arduino'
$varDir   = Join-Path $ard 'hardware\avr\1.8.8\variants\standard'
# 【v1.6.6】舵机层（servo_drive.cpp）封装官方 Servo 库，所以这里必须把
#   Servo 库的 src/avr/Servo.cpp 编译并链接进来（v1.3.0~v1.6.5 用的是自研
#   Timer1 驱动，当时刻意跳过了这个库；忘了把它加回来的话，构建立刻就是
#   "fatal error: Servo.h: No such file or directory"）。
$servoDir = Join-Path $env:LOCALAPPDATA 'Arduino15\libraries\Servo\src'

$cc   = Join-Path $gccbin 'avr-gcc.exe'
$cxx  = Join-Path $gccbin 'avr-g++.exe'
$ar   = Join-Path $gccbin 'avr-gcc-ar.exe'
$size = Join-Path $gccbin 'avr-size.exe'
$nm   = Join-Path $gccbin 'avr-nm.exe'

foreach ($p in @($cc,$cxx,$ar,$size,$nm,$coreDir,$varDir,$servoDir)) {
  if (-not (Test-Path -LiteralPath $p)) { throw "missing: $p" }
}

$mcu   = 'atmega328p'
$fcpu  = '16000000L'
$defs  = @(('-mmcu=' + $mcu), ('-DF_CPU=' + $fcpu), '-DARDUINO=10819',
           '-DARDUINO_AVR_UNO', '-DARDUINO_ARCH_AVR', '-w')
$cF    = @('-c','-g','-Os','-std=gnu11','-ffunction-sections','-fdata-sections',
           '-MMD','-flto','-fno-fat-lto-objects')
$cppF  = @('-c','-g','-Os','-std=gnu++11','-fpermissive','-fno-exceptions',
           '-ffunction-sections','-fdata-sections','-fno-threadsafe-statics',
           '-Wno-error=narrowing','-MMD','-flto')
$inc   = @(('-I' + $coreDir), ('-I' + $varDir), ('-I' + $servoDir), ('-I' + (Join-Path $servoDir 'avr')))
$defs   = $defs + $ExtraDefs
if ($Relax) {
  # -mrelax 让链接器把 CALL/JMP 放松成 RCALL/RJMP（调用点足够近时）。
  # 不改变任何行为，纯粹是编码更短；32 KB 的程序里通常能省几百字节。
  $cF  = $cF  + '-mrelax'
  $cppF = $cppF + '-mrelax'
  $defs = $defs + '-mrelax'
}
if ($NoLto) {
  $cF   = @($cF   | Where-Object { $_ -ne '-flto' -and $_ -ne '-fno-fat-lto-objects' })
  $cppF = @($cppF | Where-Object { $_ -ne '-flto' })
}

$coreObjDir = Join-Path $BuildRoot 'core'
$sketchBld  = Join-Path $BuildRoot 'sketch'
$sketchObj  = Join-Path $BuildRoot 'obj'
$coreA      = Join-Path $BuildRoot 'core.a'
$servoObjDir = Join-Path $BuildRoot 'servo'
$servoA     = Join-Path $BuildRoot 'servo.a'
$elf        = Join-Path $BuildRoot 'weArm.elf'

if ($Full) { Remove-Item -LiteralPath $BuildRoot -Recurse -Force -ErrorAction SilentlyContinue }
foreach ($d in @($BuildRoot,$coreObjDir,$sketchBld,$sketchObj,$servoObjDir)) {
  if (-not (Test-Path -LiteralPath $d)) { New-Item -ItemType Directory -Path $d -Force | Out-Null }
}

function Invoke-Tool {
  param([string]$Exe, [string[]]$Arguments, [string]$What)
  $out = & $Exe @Arguments 2>&1
  if ($LASTEXITCODE -ne 0) {
    Write-Host "!! $What failed (exit $LASTEXITCODE)" -ForegroundColor Red
    $out | ForEach-Object { Write-Host $_ }
    throw "$What failed"
  }
  return $out
}

function Compile-Tree {
  param([string]$Dir, [string]$ObjDir, [string[]]$ExtraInc)
  $sources = @()
  $sources += Get-ChildItem -LiteralPath $Dir -File -Filter *.S   -ErrorAction SilentlyContinue
  $sources += Get-ChildItem -LiteralPath $Dir -File -Filter *.c   -ErrorAction SilentlyContinue
  $sources += Get-ChildItem -LiteralPath $Dir -File -Filter *.cpp -ErrorAction SilentlyContinue
  $objs = @()
  foreach ($s in $sources) {
    $obj = Join-Path $ObjDir ($s.Name + '.o')
    $isC = $s.Extension -eq '.c'
    $flags = if ($isC) { $cF } else { $cppF }
    if ($s.Extension -eq '.S') { $flags = @('-c','-g','-x','assembler-with-cpp','-MMD','-flto') }
    $a = $flags + $defs + $inc + $ExtraInc + @($s.FullName, '-o', $obj)
    $exe = if ($isC -or $s.Extension -eq '.S') { $cc } else { $cxx }
    Invoke-Tool -Exe $exe -Arguments $a -What ("compile " + $s.Name) | Out-Null
    $objs += $obj
  }
  return $objs
}

# ---- 1) core archive (cached unless -Full) ----
if ($Full -or -not (Test-Path -LiteralPath $coreA)) {
  Write-Host '== compiling AVR core =='
  $coreObjs = Compile-Tree -Dir $coreDir -ObjDir $coreObjDir -ExtraInc @()
  Remove-Item -LiteralPath $coreA -Force -ErrorAction SilentlyContinue
  Invoke-Tool -Exe $ar -Arguments (@('rcs', $coreA) + $coreObjs) -What 'archive core.a' | Out-Null
  Write-Host ('   core.a: {0} objects' -f $coreObjs.Count)
}

# ---- 1b) Servo library archive (cached unless -Full) ----
# 只编译 AVR 架构那一个源文件（src/avr/Servo.cpp）——和 Arduino IDE 对
# 「库的 src/ + src/<arch>/ 都要编」的规则一致；Servo.h 里按 ARDUINO_ARCH_AVR
# 条件包含 avr/ServoTimers.h，所以 src 与 src/avr 都要在 -I 里。
if ($Full -or -not (Test-Path -LiteralPath $servoA)) {
  Write-Host '== compiling Servo library =='
  $servoSrc = @(Get-ChildItem -LiteralPath (Join-Path $servoDir 'avr') -File -Filter *.cpp -ErrorAction SilentlyContinue)
  if ($servoSrc.Count -eq 0) { throw "no Servo sources under $servoDir\avr" }
  $servoObjs = @()
  foreach ($s in $servoSrc) {
    $obj = Join-Path $servoObjDir ($s.Name + '.o')
    $a = $cppF + $defs + $inc + @($s.FullName, '-o', $obj)
    Invoke-Tool -Exe $cxx -Arguments $a -What ("compile servo " + $s.Name) | Out-Null
    $servoObjs += $obj
  }
  Remove-Item -LiteralPath $servoA -Force -ErrorAction SilentlyContinue
  Invoke-Tool -Exe $ar -Arguments (@('rcs', $servoA) + $servoObjs) -What 'archive servo.a' | Out-Null
  Write-Host ('   servo.a: {0} object(s)' -f $servoObjs.Count)
}

# ---- 2) sketch sources ----
Write-Host '== compiling sketch =='
Remove-Item -Path (Join-Path $sketchBld '*') -Recurse -Force -ErrorAction SilentlyContinue
# only the sketch's own sources -- never subdirectories such as .selfcheck or .git
foreach ($pat in @('*.ino','*.cpp','*.h','*.hpp','*.c')) {
  Copy-Item -Path (Join-Path $SketchDir $pat) -Destination $sketchBld -Force -ErrorAction SilentlyContinue
}
# the IDE compiles <sketch>.ino as <sketch>.ino.cpp with Arduino.h prepended
$ino = Get-ChildItem -LiteralPath $sketchBld -File -Filter *.ino | Select-Object -First 1
if (-not $ino) { throw "no .ino in $SketchDir" }
$inoText = [System.IO.File]::ReadAllText($ino.FullName, [System.Text.UTF8Encoding]::new($false))
$inoCpp  = Join-Path $sketchBld ($ino.BaseName + '.ino.cpp')
[System.IO.File]::WriteAllText($inoCpp, "#include <Arduino.h>`n" + $inoText, [System.Text.UTF8Encoding]::new($false))
Remove-Item -LiteralPath $ino.FullName -Force

$sketchObjs = @()
foreach ($s in (Get-ChildItem -LiteralPath $sketchBld -File -Filter *.cpp)) {
  $obj = Join-Path $sketchObj ($s.Name + '.o')
  $a = $cppF + $defs + $inc + @(('-I' + $sketchBld), $s.FullName, '-o', $obj)
  Invoke-Tool -Exe $cxx -Arguments $a -What ("compile " + $s.Name) | Out-Null
  $sketchObjs += $obj
}

# ---- 3) link ----
$link = @('-Os','-g','-flto','-fuse-linker-plugin','-Wl,--gc-sections',('-mmcu=' + $mcu),
          "-Wl,-Map=$BuildRoot\weArm.map",'-o', $elf) + $sketchObjs + @($servoA, $coreA, ('-L' + $BuildRoot), '-lm')
if ($Relax) { $link = $link + @('-mrelax', '-Wl,--relax') }
if ($NoLto) { $link = @($link | Where-Object { $_ -ne '-flto' -and $_ -ne '-fuse-linker-plugin' }) }
Invoke-Tool -Exe $cxx -Arguments $link -What 'link' | Out-Null

# ---- 4) report ----
Write-Host ''
$s = & $size --format=avr --mcu=$mcu $elf 2>&1
$s | ForEach-Object { Write-Host $_ }
Write-Host ''
$s | Select-String -Pattern 'Program|Data|text|data|bss' | ForEach-Object { $_.Line }

if ($Symbols) {
  Write-Host ''
  Write-Host '== biggest symbols =='
  $lines = & $nm --size-sort --print-size --radix=d $elf 2>&1
  $rows = foreach ($l in $lines) {
    if ($l -match '^\s*(\d+)\s+(\d+)\s+(\S)\s+(.+)$') {
      [pscustomobject]@{ addr=[int]$matches[1]; size=[int]$matches[2]; type=$matches[3]; name=$matches[4] }
    }
  }
  $rows | Where-Object { $_.size -gt 0 -and $_.type -match '[TtRrDdBb]' } |
    Sort-Object size -Descending | Select-Object -First 40 |
    ForEach-Object { '{0,7}  {1}  {2}' -f $_.size, $_.type, $_.name }
}
