# gh_release.ps1 -- create (or update) a GitHub release for 2122019145/meArm
# from a UTF-8 body file, without the gh CLI.
#
# Auth: reuses the credential that Git Credential Manager already stored for
# github.com ("git credential fill").  The token is never written to disk and
# never printed; it lives only inside this process.
#
# ASCII-ONLY ON PURPOSE: Windows PowerShell 5.1 mis-decodes .ps1 files that
# contain non-ASCII bytes unless they carry a UTF-8 BOM (see .selfcheck/README.md,
# the "PowerShell / script traps" section).  Chinese text belongs in the -BodyFile, not here.
#
# Release name: taken from the first line of the body file when it looks like
# "# vX.Y.Z - something" (leading "# " stripped), otherwise from -Name.
# Pass Chinese names through the FILE, never on the command line: a child
# powershell.exe re-encodes argv through the ANSI code page and garbles it.
#
# Examples:
#   powershell -NoProfile -ExecutionPolicy Bypass -File .selfcheck\gh_release.ps1 `
#       -Tag v1.2.0 -BodyFile .selfcheck\out\rel_v1.2.0.md
#   ... -Force            update an existing release body
#   ... -CheckOnly        only report whether the release exists
#   ... -MakeLatest       set the "Latest" badge on this release

[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string] $Tag,
    [Parameter(Mandatory = $true)][string] $BodyFile,
    [string] $Name = '',
    [switch] $Force,
    [switch] $MakeLatest,
    [switch] $CheckOnly,
    [switch] $DryRun
)

$ErrorActionPreference = 'Stop'
$repo = '2122019145/meArm'
$api  = "https://api.github.com/repos/$repo/releases"

if (-not (Test-Path -LiteralPath $BodyFile)) { throw "body file not found: $BodyFile" }
$full = (Resolve-Path -LiteralPath $BodyFile).Path
$body = [System.IO.File]::ReadAllText($full, [System.Text.UTF8Encoding]::new($false))

# ---- release name -----------------------------------------------------------
if ([string]::IsNullOrWhiteSpace($Name)) {
    $first = ($body -split "`n")[0].TrimEnd("`r").Trim()
    if ($first.StartsWith('#')) { $Name = $first.TrimStart('#').Trim() }
    else { $Name = $Tag }
}

# ---- token ------------------------------------------------------------------
$cred = "protocol=https`nhost=github.com`n`n" | git credential fill 2>$null
$pat  = ($cred | Select-String -Pattern '^password=(.+)$').Matches.Groups[1].Value
if ([string]::IsNullOrWhiteSpace($pat)) {
    throw 'no github credential: "git credential fill" returned no password'
}
$hdr = @{
    Authorization = "Bearer $pat"
    'User-Agent'  = 'dsh-gh-release'
    Accept        = 'application/vnd.github+json'
}

# ---- does the release already exist? ---------------------------------------
$existing = $null
try { $existing = Invoke-RestMethod -Method Get -Uri "$api/tags/$Tag" -Headers $hdr -TimeoutSec 40 }
catch { $existing = $null }

"tag        : $Tag"
"name       : $Name"
"body file  : $full"
"body chars : $($body.Length)"
if ($existing) {
    "existing   : $($existing.html_url)  id=$($existing.id)  body=$($existing.body.Length) chars  draft=$($existing.draft)"
} else {
    "existing   : none"
}

if ($CheckOnly) { exit 0 }
if ($existing -and -not $Force) { "SKIP: release already exists (add -Force to update)"; exit 0 }
if ($body.Length -lt 300) { throw "body too short ($($body.Length) chars) -- refusing to publish" }
if ($DryRun) { "DRY RUN: would $(if ($existing) { 'PATCH' } else { 'POST' })"; exit 0 }

# ---- publish ----------------------------------------------------------------
# ConvertTo-Json escapes non-ASCII as \uXXXX, which is what the API wants; the
# payload must be sent as UTF-8 BYTES, because PS 5.1 turns a [string] -Body
# into ISO-8859-1 ("latin1") and every Chinese character arrives as mojibake.
$payload = [ordered]@{
    tag_name    = $Tag
    name        = $Name
    body        = $body
    draft       = $false
    prerelease  = $false
    make_latest = $(if ($MakeLatest) { 'true' } else { 'false' })
}
$bytes = [System.Text.Encoding]::UTF8.GetBytes(($payload | ConvertTo-Json -Depth 4 -Compress))

if ($existing) {
    $r = Invoke-RestMethod -Method Patch -Uri "$api/$($existing.id)" -Headers $hdr `
         -ContentType 'application/json; charset=utf-8' -Body $bytes -TimeoutSec 60
    "UPDATED : $($r.html_url)"
} else {
    $r = Invoke-RestMethod -Method Post -Uri $api -Headers $hdr `
         -ContentType 'application/json; charset=utf-8' -Body $bytes -TimeoutSec 60
    "CREATED : $($r.html_url)"
}
"id         : $($r.id)"
"tag(remote): $($r.tag_name)"
"body sent  : $($r.body.Length) chars"
"latest     : $($r.make_latest)"
