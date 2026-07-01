# Determinism regression harness (roadmap C2 — was scratch/determinism_det.ps1, cited by docs/DETERMINISM.md).
#
# Runs beat-box twice in BB_DET_STEPS mode (one sim step per render frame, wall-clock ignored, so both
# runs execute a bit-identical step sequence) and reports, per hashed field, the FIRST step at which the
# two runs diverge. "NONE" across every field == bitwise-deterministic. Paths are derived from this
# script's location (portable), not hardcoded, so it works from any clone.
#
# Usage:  pwsh tools/determinism_det.ps1     (build the Release preset first)

$ErrorActionPreference = "Stop"
$repo = Split-Path $PSScriptRoot -Parent

# Prefer the RelWithDebInfo output of the "Release" preset; fall back to any beat-box.exe under build/.
$exe = Join-Path $repo "build\Release\RelWithDebInfo\beat-box.exe"
if (-not (Test-Path $exe)) {
  $exe = Get-ChildItem -Path (Join-Path $repo "build") -Filter "beat-box.exe" -Recurse -EA SilentlyContinue |
         Select-Object -First 1 -ExpandProperty FullName
}
if (-not $exe -or -not (Test-Path $exe)) {
  Write-Error "beat-box.exe not found under $repo\build — build the Release preset first (cmake --build build/Release --config RelWithDebInfo)."
  exit 1
}

$logdir = Join-Path $repo "scratch"   # transient run logs (scratch/ is gitignored)
New-Item -ItemType Directory -Force -Path $logdir | Out-Null

$env:BB_DET_STEPS = "1000"
$fields = @('ph','cp2','cph','vhf','vhi','chash','lh','sh','wh','viol')
$Runs=@{}
foreach($run in 1,2){
  $log = Join-Path $logdir "det_run$run.log"; Remove-Item $log,"$log.err" -Force -EA SilentlyContinue
  $p = Start-Process -FilePath $exe -WorkingDirectory (Split-Path $exe) -RedirectStandardOutput $log -RedirectStandardError "$log.err" -PassThru
  for($t=0;$t -lt 120;$t++){ Start-Sleep -Seconds 2; $p.Refresh(); if($p.HasExited){break} }
  if(-not $p.HasExited){ Stop-Process -Id $p.Id -Force -EA SilentlyContinue }
  $m=@{}; foreach($f in $fields){ $m[$f]=@{} }
  Get-Content $log -EA SilentlyContinue | Select-String '^DET step=' | ForEach-Object {
    if($_.Line -match 'step=(\d+)'){ $st=[int]$matches[1] } else { return }
    foreach($f in $fields){ if($_.Line -match "$f=(\S+)"){ $m[$f][$st]=$matches[1] } }
  }
  $Runs[$run]=$m; Start-Sleep -Seconds 1
}
function First($f){ $keys = $Runs[1][$f].Keys | Where-Object { $Runs[2][$f].ContainsKey($_) } | Sort-Object; foreach($s in $keys){ if($Runs[1][$f][$s] -ne $Runs[2][$f][$s]){ return $s } }; return "NONE" }
Write-Output "run1 steps: $($Runs[1]['ph'].Count)  run2 steps: $($Runs[2]['ph'].Count)"
foreach($f in $fields){ Write-Output ("  {0,-5} first-diverge = {1}" -f $f, (First $f)) }
