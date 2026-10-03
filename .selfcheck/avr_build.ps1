#
# avr_build.ps1 -- build the weArm sketch exactly the way the Arduino IDE does,
# with the locally installed AVR toolchain, and report flash/SRAM usage.
#
# Toolchain (found on this machine):
#   avr-gcc 7.3.0 : %LOCALAPPDATA%\Arduino15\packages\arduino\tools\avr-gcc\7.3.0-atmel3.6.1-arduino7\bin
#   AVR core 1.8.8: %LOCALAPPDATA%\Arduino15\packages\arduino\hardware\avr\1.8.8
#
# v1.3.0: the sketch no longer uses the Arduino Servo library (it has its own
# 4-channel Timer1 driver in servo_drive.cpp), so the Servo library is neither
# compiled nor linked here.
#
# Flags are copied from the core's platform.txt (LTO is enabled by that core!).
#
[CmdletBinding()]
param(
  [string]$SketchDir  = 'D:\dsh1\wearm',
  [string]$BuildRoot  = 'D:\dsh1\wearm\.selfcheck\avrbuild',
  [switch]$Full,       # also rebuild the core archive
  [switch]$Symbols,    # print the biggest symbols after linking
  [switch]$NoLto,      # disable LTO (per-object sizes become meaningful)
  [string[]]$ExtraDefs = @()   # extra -D... switches (used for size ablations)
)

$ErrorActionPreference = 'Stop'

$ard      = Join-Path $env:LOCALAPPDATA 'Arduino15\packages\arduino'
$gccbin   = Join-Path $ard 'tools\avr-gcc\7.3.0-atmel3.6.1-arduino7\bin'
$coreDir  = Join-Path $ard 'hardware\avr\1.8.8\cores\arduino'
$varDir   = Join-Path $ard 'hardware\avr\1.8.8\variants\standard'
# 【v1.3.0】不再需要 Arduino Servo 库：固件改用自研 servo_drive.cpp（4 路 Timer1）。
#   去掉 -I 指向 Servo 与 servo.a 的编译，既省构建时间，也让尺寸对比与真机一致。

$cc   = Join-Path $gccbin 'avr-gcc.exe'
$cxx  = Join-Path $gccbin 'avr-g++.exe'
$ar   = Join-Path $gccbin 'avr-gcc-ar.exe'
$size = Join-Path $gccbin 'avr-size.exe'
$nm   = Join-Path $gccbin 'avr-nm.exe'

foreach ($p in @($cc,$cxx,$ar,$size,$nm,$coreDir,$varDir)) {
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
$inc   = @(('-I' + $coreDir), ('-I' + $varDir))
$defs   = $defs + $ExtraDefs
if ($NoLto) {
  $cF   = @($cF   | Where-Object { $_ -ne '-flto' -and $_ -ne '-fno-fat-lto-objects' })
  $cppF = @($cppF | Where-Object { $_ -ne '-flto' })
}

$coreObjDir = Join-Path $BuildRoot 'core'
$sketchBld  = Join-Path $BuildRoot 'sketch'
$sketchObj  = Join-Path $BuildRoot 'obj'
$coreA      = Join-Path $BuildRoot 'core.a'
$elf        = Join-Path $BuildRoot 'weArm.elf'

if ($Full) { Remove-Item -LiteralPath $BuildRoot -Recurse -Force -ErrorAction SilentlyContinue }
foreach ($d in @($BuildRoot,$coreObjDir,$sketchBld,$sketchObj)) {
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
          "-Wl,-Map=$BuildRoot\weArm.map",'-o', $elf) + $sketchObjs + @($coreA, ('-L' + $BuildRoot), '-lm')
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
