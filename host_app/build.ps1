$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

python -m PyInstaller `
  --noconfirm `
  --clean `
  --name car_host_app `
  --windowed `
  --paths $root `
  main.py
