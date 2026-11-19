param([switch]$Simulate, [int]$Port = 8090)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path $PSScriptRoot -Parent
$taskPython = Join-Path $env:LOCALAPPDATA 'arm4-tools/can-venv/Scripts/python.exe'
if (-not (Test-Path -LiteralPath $taskPython)) {
    $taskPython = Join-Path $env:USERPROFILE 'AppData/Local/Packages/OpenAI.Codex_2p2nqsd0c76g0/LocalCache/Local/arm4-tools/can-venv/Scripts/python.exe'
}
if (-not (Test-Path -LiteralPath $taskPython)) {
    throw '未找到现有 CAN Python 环境，请安装 manual_control/requirements.txt 后直接运行 server.py。'
}
$env:PYTHONUTF8 = '1'
$taskArgs = @((Join-Path $PSScriptRoot 'server.py'), '--port', "$Port")
if ($Simulate) { $taskArgs += '--simulate' }
& $taskPython @taskArgs
