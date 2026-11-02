param(
    [string]$Serial = $env:SCARA_ADB_SERIAL,
    [string]$Package = (Join-Path $PSScriptRoot '..\output\rv1106\scara-robot'),
    [string]$Destination = '/root/scara-robot',
    [string]$Adb = 'adb'
)
# 只校验、上传和设置执行权限；不启动程序、不改固件、系统库或网络。
$ErrorActionPreference = 'Stop'
$packagePath = (Resolve-Path -LiteralPath $Package).Path
if ($Destination -notmatch '^/(root|userdata|tmp|mnt)/[A-Za-z0-9_-][A-Za-z0-9_./-]*$' -or
    $Destination -match '(^|/)\.\.(/|$)') {
    throw '目标须为/root、/userdata、/tmp或/mnt下的独立应用目录，不能含空格或..。'
}
$Destination = $Destination.TrimEnd('/')
foreach ($name in @('scara_robot', 'run-robot.sh', 'camera-mis5001.sh', 'config/robot.json',
                    'models/xiangqi-yolov5s-relu.rknn', 'engine/pikafish', 'engine/pikafish.nnue', 'SHA256SUMS')) {
    if (-not (Test-Path -LiteralPath (Join-Path $packagePath $name) -PathType Leaf)) {
        throw "运行包缺少$name；请先运行build-robot.sh。"
    }
}
$root = $packagePath.TrimEnd([char[]]'\/') + [IO.Path]::DirectorySeparatorChar
foreach ($line in Get-Content -LiteralPath (Join-Path $packagePath 'SHA256SUMS')) {
    if ($line -notmatch '^([0-9a-fA-F]{64})\s+\*?(.+)$') { throw 'SHA256SUMS格式无效。' }
    $expected = $Matches[1]
    $file = [IO.Path]::GetFullPath((Join-Path $packagePath $Matches[2]))
    if (-not $file.StartsWith($root, [StringComparison]::OrdinalIgnoreCase)) { throw '清单路径超出运行包。' }
    if ((Get-FileHash -LiteralPath $file -Algorithm SHA256).Hash -ne $expected) { throw "校验失败：$file" }
}
$target = @()
if ($Serial) { $target = @('-s', $Serial) }
& $Adb @target shell mkdir -p $Destination
if ($LASTEXITCODE -ne 0) { throw '无法建立板端应用目录。' }
$preflight = & $Adb @target shell "test ! -L '$Destination/engine' && test ! -L '$Destination/lib' && printf 'SCARA_DEST_READY\n'"
if (-not (@($preflight | ForEach-Object { $_.ToString().Trim() }) -contains 'SCARA_DEST_READY')) {
    throw '目标engine或lib为符号链接，或无法核对目标；请先保留旧数据并选择独立应用目录。'
}
& $Adb @target push "$packagePath/." "$Destination/"
if ($LASTEXITCODE -ne 0) { throw 'ADB上传失败。' }
# 老版本ADB可能不传递远端退出码，必须收到远端完整校验后的独立成功标记。
$verify = "chmod +x '$Destination/scara_robot' '$Destination/run-robot.sh' '$Destination/camera-mis5001.sh' '$Destination/engine/pikafish' && ( test ! -f '$Destination/scara_tests' || chmod +x '$Destination/scara_tests' ) && cd '$Destination' && sha256sum -c SHA256SUMS >/dev/null && printf 'SCARA_DEPLOY_OK\n'"
$reply = & $Adb @target shell $verify
if ($LASTEXITCODE -ne 0 -or -not (@($reply | ForEach-Object { $_.ToString().Trim() }) -contains 'SCARA_DEPLOY_OK')) {
    throw '远端权限设置或SHA256校验失败，未收到SCARA_DEPLOY_OK。'
}
Write-Output "部署完成。手动运行：sh $Destination/run-robot.sh"
