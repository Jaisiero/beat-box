# Full regression gauntlet (review v3): runs scenes 1-8 headless with per-scene quality
# thresholds and reports a pass/fail table. One command = the whole physics regression gate.
#
# Thresholds were CALIBRATED from per-scene BB_METRICS_CSV baselines (post-warmup maxima on a
# healthy build, AVBD, 10 s) with ~3-4x headroom for the documented run-to-run chaos (the pool
# scenes are non-deterministic; scene_7's deep200 swings several-x between identical runs).
# They are meant to catch EXPLOSIONS and REGRESSIONS (NaN blowups, floor escapes, solver
# collapse), not jitter noise. Recalibrate by running each scene with BB_METRICS_CSV and
# taking post-warmup maxima if physics behavior changes intentionally.
#
# Shared gates (all scenes): pen <= 400 mm (physical clamp region is ~250; 400 = pathological),
# maxv <= 20000 mm/s (the 12 m/s terminal clamp is on ADVECTED velocity; AVBD's reconstructed
# velocity (pos delta/dt) legitimately overshoots it during heavy stacking — scene_2 measured
# 16072 in one run. A real runaway/NaN lands far beyond 20000 or trips miny/pen first),
# miny >= -2 m (floor top is y=0; below -2 = a body fell through the floor).
#
# Exit code = number of failing scenes (0 = all pass).
# Usage: pwsh tools/gauntlet.ps1    (build the Release preset first; warm shader cache advised)

$ErrorActionPreference = "Stop"
$repo = Split-Path $PSScriptRoot -Parent

$exe = Join-Path $repo "build\Release\RelWithDebInfo\beat-box.exe"
if (-not (Test-Path $exe)) {
  $exe = Get-ChildItem -Path (Join-Path $repo "build") -Filter "beat-box.exe" -Recurse -EA SilentlyContinue |
         Select-Object -First 1 -ExpandProperty FullName
}
if (-not $exe -or -not (Test-Path $exe)) { Write-Error "beat-box.exe not found — build first."; exit 99 }

# scene table: calibrated deep200 limit + a short description (run time is uniform)
$scenes = @(
  @{ n = 1; deep200 = 20;  desc = "restitution showcase" },
  @{ n = 2; deep200 = 900; desc = "1001-body stress field" },
  @{ n = 3; deep200 = 10;  desc = "stacks + pyramid pile" },
  @{ n = 4; deep200 = 10;  desc = "ramps" },
  @{ n = 5; deep200 = 30;  desc = "concave showcase" },
  @{ n = 6; deep200 = 10;  desc = "deterministic stability probe" },
  @{ n = 7; deep200 = 300; desc = "box pool (432 cubes)" },
  @{ n = 8; deep200 = 10;  desc = "single-cube free fall" }
)

$run_seconds = "10"
$results = @()
$failures = 0

foreach ($s in $scenes) {
  $env:BB_SCENE = "$($s.n)"; $env:BB_SOLVER = "2"; $env:BB_AUTOSTART = "1"
  $env:BB_RUN_SECONDS = $run_seconds
  $env:BB_ASSERT_MAX_DEEP200 = "$($s.deep200)"
  $env:BB_ASSERT_MAX_PEN = "400"; $env:BB_ASSERT_MAX_MAXV = "20000"; $env:BB_ASSERT_MIN_MINY = "-2"
  $log = Join-Path $repo "scratch\gauntlet_scene$($s.n).log"
  New-Item -ItemType Directory -Force -Path (Join-Path $repo "scratch") | Out-Null

  $p = Start-Process -FilePath $exe -WorkingDirectory (Split-Path $exe) -PassThru -WindowStyle Hidden `
       -RedirectStandardOutput $log -RedirectStandardError "$log.err"
  for ($t = 0; $t -lt 120; $t++) { Start-Sleep -Seconds 2; $p.Refresh(); if ($p.HasExited) { break } }
  if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force -EA SilentlyContinue; $code = 98 } else { $code = $p.ExitCode }

  # a run that never stepped the sim (no [PERF] "elapsed -> exiting" AND no assert line) is a
  # LAUNCH failure, not a physics pass — do not let it false-pass
  $ranOut  = Select-String -Path $log -Pattern "elapsed -> exiting" -Quiet -EA SilentlyContinue
  $asserted = Select-String -Path "$log.err" -Pattern "ASSERT FAILED" -EA SilentlyContinue | Select-Object -First 1
  if ($code -eq 0 -and -not $ranOut) { $code = 97 }

  $status = if ($code -eq 0) { "PASS" } elseif ($code -eq 2) { "FAIL: $($asserted.Line)" }
            elseif ($code -eq 97) { "FAIL: run ended without completing (launch/crash?)" }
            elseif ($code -eq 98) { "FAIL: timeout (hung)" }
            else { "FAIL: exit $code" }
  if ($code -ne 0) { $failures++ }
  $results += "scene $($s.n) [$($s.desc)]".PadRight(42) + $status
  Write-Output $results[-1]
}

foreach ($v in "BB_SCENE","BB_SOLVER","BB_AUTOSTART","BB_RUN_SECONDS","BB_ASSERT_MAX_DEEP200","BB_ASSERT_MAX_PEN","BB_ASSERT_MAX_MAXV","BB_ASSERT_MIN_MINY") {
  Remove-Item "env:$v" -EA SilentlyContinue
}

Write-Output ("-" * 60)
if ($failures -eq 0) { Write-Output "GAUNTLET: ALL 8 SCENES PASS" } else { Write-Output "GAUNTLET: $failures SCENE(S) FAILED" }
exit $failures
