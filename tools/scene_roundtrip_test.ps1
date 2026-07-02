# F3 text-scene parser round-trip test (review v3).
#
# Dumps a built-in scene to the BB_SCENE_FILE text format, loads that file back, dumps again,
# and requires both dumps to be byte-identical: dump(scene_3) == dump(load(dump(scene_3))).
# Catches parser/writer drift (lost fields, default mismatches, ordering changes).
# Uses BB_COMPILE_ONLY so each launch exits right after the scene builds (~2 s warm cache).
#
# Exit codes: 0 = round-trip identical, 1 = mismatch, 2 = launch/dump failure.
# Usage: pwsh tools/scene_roundtrip_test.ps1   (build the Release preset first)

$ErrorActionPreference = "Stop"
$repo = Split-Path $PSScriptRoot -Parent

$exe = Join-Path $repo "build\Release\RelWithDebInfo\beat-box.exe"
if (-not (Test-Path $exe)) {
  $exe = Get-ChildItem -Path (Join-Path $repo "build") -Filter "beat-box.exe" -Recurse -EA SilentlyContinue |
         Select-Object -First 1 -ExpandProperty FullName
}
if (-not $exe -or -not (Test-Path $exe)) { Write-Error "beat-box.exe not found — build first."; exit 2 }

$dir = Join-Path $repo "scratch"; New-Item -ItemType Directory -Force -Path $dir | Out-Null
$a = Join-Path $dir "roundtrip_a.txt"; $b = Join-Path $dir "roundtrip_b.txt"
Remove-Item $a, $b -Force -EA SilentlyContinue

function Run-Dump([hashtable]$extraEnv, [string]$dumpPath) {
  foreach ($k in $extraEnv.Keys) { Set-Item -Path "env:$k" -Value $extraEnv[$k] }
  $env:BB_COMPILE_ONLY = "1"; $env:BB_SCENE_DUMP = $dumpPath
  $p = Start-Process -FilePath $exe -WorkingDirectory (Split-Path $exe) -PassThru -WindowStyle Hidden
  for ($t = 0; $t -lt 120; $t++) { Start-Sleep -Seconds 2; $p.Refresh(); if ($p.HasExited) { break } }
  if (-not $p.HasExited) { Stop-Process -Id $p.Id -Force -EA SilentlyContinue }
  foreach ($k in $extraEnv.Keys) { Remove-Item -Path "env:$k" -EA SilentlyContinue }
  Remove-Item env:BB_COMPILE_ONLY, env:BB_SCENE_DUMP -EA SilentlyContinue
}

Write-Output "1) dump scene_3 -> roundtrip_a.txt"
Run-Dump @{ BB_SCENE = "3" } $a
if (-not (Test-Path $a)) { Write-Output "RESULT: FAIL (first dump missing)"; exit 2 }

Write-Output "2) load roundtrip_a.txt -> dump roundtrip_b.txt"
Run-Dump @{ BB_SCENE_FILE = $a } $b
if (-not (Test-Path $b)) { Write-Output "RESULT: FAIL (second dump missing)"; exit 2 }

$ha = (Get-FileHash $a -Algorithm SHA256).Hash
$hb = (Get-FileHash $b -Algorithm SHA256).Hash
Write-Output "a: $ha"
Write-Output "b: $hb"
if ($ha -ne $hb) {
  Write-Output "RESULT: ROUND-TRIP MISMATCH (parser/writer drift) — diff:"
  Compare-Object (Get-Content $a) (Get-Content $b) | Select-Object -First 10 | Format-Table -AutoSize | Out-String | Write-Output
  exit 1
}
Write-Output "RESULT: ROUND-TRIP OK ($( (Get-Content $a).Count ) lines byte-identical)"
exit 0
