# progress.ps1 -- one-shot (or looping) progress report for the meArm release/consolidation run.
#
# Sources it polls (all read-only):
#   1. the DSH session store  -> live Agent-Team board: members, tasks, message counts
#   2. the GitHub API         -> which tags already have a Release page and its body size
#   3. the working tree       -> rel_*.md notes, the single-file sketch, git status
#
# ASCII-ONLY ON PURPOSE: Windows PowerShell 5.1 mis-decodes non-BOM .ps1 files, so
# every literal here is English; Chinese data (task subjects, release names) is read
# from files/JSON at RUNTIME, which is safe.
#
# Usage:
#   powershell -NoProfile -ExecutionPolicy Bypass -File D:\dsh1\meArm\.selfcheck\progress.ps1
#   ... -Watch -IntervalSec 30        poll forever, one table per interval
#   ... -NoGitHub                     skip the API call (offline / rate limit)
#   ... -SessionId session-xxxx       override auto-detected session store

[CmdletBinding()]
param(
    [string] $Repo = 'D:\dsh1\meArm',
    [string] $SessionId = $env:DSH_SESSION_ID,
    [string] $StorageRoot = (Join-Path $env:USERPROFILE '.dsh\storages\session_projcache\sessions'),
    [string[]] $Tags = @('v0.1.0','v0.2.0','v0.3.0','v0.4.0','v0.5.0','v1.0.0','v1.1.0',
                         'v1.2.0','v1.3.0','v1.4.0','v1.5.0','v1.6.0','v1.6.1','v1.6.2'),
    [string[]] $NoteTags = @('v1.2.0','v1.3.0','v1.4.0','v1.5.0','v1.6.0','v1.6.1','v1.6.2'),
    [switch] $NoGitHub,
    [switch] $Watch,
    [int]    $IntervalSec = 30
)

$ErrorActionPreference = 'Continue'
try { [Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false) } catch { }

function Get-SessionStore {
    param([string] $Id, [string] $Root)
    if ($Id) {
        $p = Join-Path $Root "session-$Id.json"
        if (Test-Path -LiteralPath $p) { return Get-Item -LiteralPath $p }
    }
    $f = Get-ChildItem -LiteralPath $Root -Filter 'session-*.json' -ErrorAction SilentlyContinue |
         Sort-Object LastWriteTime -Descending | Select-Object -First 1
    return $f
}

function Show-Team {
    param([string] $Id, [string] $Root)
    $store = Get-SessionStore -Id $Id -Root $Root
    '== 1. AGENT TEAM (from DSH session store) =='
    if (-not $store) { '   (session store not found -- pass -SessionId)'; ''; return }
    "   store: $($store.Name)   mtime: $($store.LastWriteTime.ToString('HH:mm:ss'))   age: $([int]((Get-Date) - $store.LastWriteTime).TotalSeconds)s"
    try {
        $j = [System.IO.File]::ReadAllText($store.FullName, [System.Text.UTF8Encoding]::new($false)) | ConvertFrom-Json
        $team = $j.record.rows.agentTeam.val
    } catch { "   (could not parse session store: $($_.Exception.Message))"; ''; return }
    if (-not $team) { '   (no agent team in this session)'; ''; return }

    '   members:'
    foreach ($m in $team.members) {
        '     {0,-22} {1,-9} {2}' -f $m.name, $m.phase, $m.description
    }
    $tasks = @($team.tasks)
    $nameById = @{}
    foreach ($m in $team.members) { $nameById[$m.id] = $m.name }
    "   tasks: $($tasks.Count)   messages: $(@($team.messages).Count)   delivered: $(@($team.delivered).Count)"
    '     {0,-8} {1,-11} {2,-22} {3}' -f 'id', 'status', 'owner', 'subject'
    foreach ($t in ($tasks | Sort-Object { [int]($_.id -replace '\D','') })) {
        $owner = $t.ownerId
        if ($owner -and $nameById.ContainsKey($owner)) { $owner = $nameById[$owner] }
        '     {0,-8} {1,-11} {2,-22} {3}' -f $t.id, $t.status, $owner, $t.subject
    }
    ''
}

function Show-Releases {
    param([string[]] $TagList)
    '== 2. GITHUB RELEASES =='
    if ($NoGitHub) { '   (skipped: -NoGitHub)'; ''; return }
    $cred = "protocol=https`nhost=github.com`n`n" | git credential fill 2>$null
    $pat  = ($cred | Select-String -Pattern '^password=(.+)$').Matches.Groups[1].Value
    if ([string]::IsNullOrWhiteSpace($pat)) { '   (no github credential)'; ''; return }
    $hdr = @{ Authorization = "Bearer $pat"; 'User-Agent' = 'dsh-progress'; Accept = 'application/vnd.github+json' }
    try {
        $r = Invoke-RestMethod -Uri 'https://api.github.com/repos/2122019145/meArm/releases?per_page=100' -Headers $hdr -TimeoutSec 40
    } catch { "   (API failed: $($_.Exception.Message))"; ''; return }
    $byTag = @{}
    foreach ($x in $r) { $byTag[$x.tag_name] = $x }
    $have = 0
    foreach ($t in $TagList) {
        if ($byTag.ContainsKey($t)) {
            $have++
            $x = $byTag[$t]
            '     {0,-8} OK    id={1,-10} body={2,-6} {3}' -f $t, $x.id, $x.body.Length, $x.created_at
        } else {
            '     {0,-8} MISSING' -f $t
        }
    }
    "   total on GitHub: $($r.Count)   wanted: $($TagList.Count)   present: $have"
    ''
}

function Show-Local {
    param([string] $Root, [string[]] $TagList)
    '== 3. LOCAL ARTIFACTS =='
    foreach ($t in $TagList) {
        $f = Join-Path $Root ".selfcheck\out\rel_$t.md"
        if (Test-Path -LiteralPath $f) {
            $i = Get-Item -LiteralPath $f
            $txt = [System.IO.File]::ReadAllText($f, [System.Text.UTF8Encoding]::new($false))
            $lines = ([regex]::Matches($txt, "`n")).Count + 1
            '     rel_{0,-7} {1,6} B / {2,6} chars / {3,4} lines   {4}' -f "$t.md", $i.Length, $txt.Length, $lines, $i.LastWriteTime.ToString('HH:mm:ss')
        } else {
            '     rel_{0,-7} (not written yet)' -f "$t.md"
        }
    }
    foreach ($rel in @('single\meArm\meArm.ino', '.selfcheck\make_single.ps1', '.selfcheck\gh_release.ps1', '.selfcheck\out\single_ino_verify.md')) {
        $f = Join-Path $Root $rel
        if (Test-Path -LiteralPath $f) {
            $i = Get-Item -LiteralPath $f
            '     {0,-40} {1,7} bytes   {2}' -f $rel, $i.Length, $i.LastWriteTime.ToString('HH:mm:ss')
        } else {
            '     {0,-40} (missing)' -f $rel
        }
    }
    ''
    '   git status --short:'
    $st = & git -C $Root status --short 2>$null
    if ($st) { foreach ($l in $st) { "     $l" } } else { '     (clean)' }
    ''
}

function Invoke-Report {
    param()
    Clear-Host
    '=================================================================='
    "  meArm progress   $((Get-Date).ToString('yyyy-MM-dd HH:mm:ss'))"
    '=================================================================='
    ''
    Show-Team    -Id $SessionId -Root $StorageRoot
    Show-Releases -TagList $Tags
    Show-Local   -Root $Repo -TagList $NoteTags
}

if ($Watch) {
    while ($true) { Invoke-Report; Start-Sleep -Seconds $IntervalSec }
} else {
    Invoke-Report
}
