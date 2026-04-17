$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

$release = Join-Path $root "release"
$work = Join-Path $root ".pyinstaller"

if (Test-Path $release) {
  Remove-Item -Recurse -Force $release
}

if (Test-Path $work) {
  Remove-Item -Recurse -Force $work
}

py -m PyInstaller `
  --noconfirm `
  --clean `
  --name car_host_app `
  --windowed `
  --paths $root `
  --distpath (Join-Path $release "onedir") `
  --workpath (Join-Path $work "onedir") `
  --specpath (Join-Path $work "spec") `
  main.py

py -m PyInstaller `
  --noconfirm `
  --clean `
  --name car_host_app `
  --windowed `
  --onefile `
  --paths $root `
  --distpath (Join-Path $release "onefile") `
  --workpath (Join-Path $work "onefile") `
  --specpath (Join-Path $work "spec") `
  main.py
