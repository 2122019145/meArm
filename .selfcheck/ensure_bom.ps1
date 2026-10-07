# ensure_bom.ps1 -- keep every repo .ps1 script readable by Windows PowerShell 5.1.
#
# Why this exists:
#   The tool shell here is Windows PowerShell 5.1 (NOT pwsh 7). For a .ps1 file
#   WITHOUT a byte order mark, 5.1 decodes it with the ANSI code page (CP936 on
#   this machine). Chinese comments then turn into mojibake, and -- worse -- the
#   leftover byte of a UTF-8 sequence can swallow the following ASCII byte, e.g.
#   a closing quote. A single eaten quote makes the whole script fail to parse
#   with a confusing "unexpected token" error, and some lines even merge.
#   Adding a UTF-8 BOM makes 5.1 decode the file as UTF-8, which is correct.
#
# The editor that writes these files emits UTF-8 without BOM, so run this after
# editing any .selfcheck\*.ps1 script (this file is ASCII-only on purpose, so it
# parses under any code page).
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File .selfcheck\ensure_bom.ps1
#   powershell -ExecutionPolicy Bypass -File .selfcheck\ensure_bom.ps1 -Check
param(
  [switch]$Check   # only report; do not write
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$enc  = New-Object System.Text.UTF8Encoding($true)
$bad  = 0

$files = Get-ChildItem -LiteralPath $root -Recurse -Filter *.ps1 -File |
         Where-Object { $_.FullName -notmatch '\\out\\' }

foreach ($f in $files) {
    $b = [System.IO.File]::ReadAllBytes($f.FullName)
    if ($b.Length -ge 3 -and $b[0] -eq 0xEF -and $b[1] -eq 0xBB -and $b[2] -eq 0xBF) {
        if ($Check) { Write-Host ("  [ OK ] {0} has BOM" -f $f.Name) }
        continue
    }
    $nonAscii = $false
    foreach ($x in $b) { if ($x -gt 0x7F) { $nonAscii = $true; break } }
    if (-not $nonAscii) {
        # Pure ASCII parses identically under every code page, so a BOM is optional.
        if ($Check) { Write-Host ("  [ OK ] {0} ASCII-only, BOM not required" -f $f.Name) }
        continue
    }
    $bad++
    if ($Check) {
        Write-Host ("  [WARN] {0} missing BOM (PowerShell 5.1 would mis-decode it)" -f $f.Name) -ForegroundColor Yellow
    } else {
        $txt = [System.Text.Encoding]::UTF8.GetString($b)
        [System.IO.File]::WriteAllText($f.FullName, $txt, $enc)
        Write-Host ("  [FIX ] {0} -> UTF-8 with BOM" -f $f.Name)
    }
}

if ($Check -and $bad -gt 0) { exit 1 }
if (-not $Check -and $bad -eq 0) { Write-Host "  all .ps1 files already have a BOM" }
exit 0
