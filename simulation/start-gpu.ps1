# Windows启动一次；Ubuntu仍用原run-interactive.sh。复用本机已验证的WSL CUDA环境。
param(
    [string]$Distro = 'Ubuntu-24.04-D',
    [string]$GpuRoot = '/home/lst/.cache/scara-xiangqi-train',
    [string]$Vm = 'ubuntu-codex'
)
$ErrorActionPreference = 'Stop'
function Read-Bytes($stream, $length) {
    $bytes = [byte[]]::new($length)
    $offset = 0
    while ($offset -lt $length) {
        $count = $stream.Read($bytes,$offset,$length-$offset)
        if (-not $count) { throw 'GPU服务提前断开' }
        $offset += $count
    }
    return ,$bytes
}
function Get-GpuStatus {
    $gpuClient = [System.Net.Sockets.TcpClient]::new()
    try {
        if (-not $gpuClient.ConnectAsync('127.0.0.1',8767).Wait(1500)) { return $null }
        $stream = $gpuClient.GetStream()
        $stream.ReadTimeout = 2000
        $stream.Write([byte[]]@(0,0,0,0),0,4)
        $header = Read-Bytes $stream 4
        [array]::Reverse($header)
        $length = [BitConverter]::ToUInt32($header,0)
        if ($length -gt 65536) { return $null }
        $body = Read-Bytes $stream $length
        return [Text.Encoding]::UTF8.GetString($body) | ConvertFrom-Json
    } catch { return $null }
    finally { $gpuClient.Dispose() }
}
$status = Get-GpuStatus
if (-not $status) {
    $windowsScript = ($PSScriptRoot -replace '\\','/') + '/gpu_server.py'
    $linuxScript = (& wsl -d $Distro -- wslpath -a $windowsScript).Trim()
    Start-Process -FilePath wsl.exe -WindowStyle Hidden -ArgumentList @(
        '-d',$Distro,'--exec',"$GpuRoot/venv/bin/python",'-u',
        $linuxScript,"$GpuRoot/yolov5","$GpuRoot/xiangqi14-s-best-portable.pt"
    ) -RedirectStandardOutput "$PSScriptRoot/gpu-service.log" `
      -RedirectStandardError "$PSScriptRoot/gpu-service-error.log" | Out-Null
    for ($attempt=0; $attempt -lt 30 -and -not $status; $attempt++) {
        Start-Sleep -Seconds 1
        $status = Get-GpuStatus
    }
}
if (-not $status -or $status.backend -ne 'cuda:0') {
    throw 'CUDA服务启动失败，请看gpu-service-error.log；不会回退CPU。'
}
# 只转发本机回环口，不打开防火墙，不要求VMware给客体直通NVIDIA。
& ssh $Vm 'ss -ltn sport = :8767' | Out-String -OutVariable listeners | Out-Null
if ($LASTEXITCODE) { throw 'RV虚拟机SSH连接失败' }
if ($listeners -notmatch '127\.0\.0\.1:8767') {
    Start-Process -FilePath ssh.exe -WindowStyle Hidden -ArgumentList @(
        '-N','-R','127.0.0.1:8767:127.0.0.1:8767','-o','ExitOnForwardFailure=yes',
        '-o','ServerAliveInterval=15','-o','ServerAliveCountMax=3',$Vm
    ) | Out-Null
    Start-Sleep -Seconds 1
}
$guestProbe = @'
import socket,struct,json
with socket.create_connection(('127.0.0.1',8767),3) as connection:
    stream=connection.makefile('rwb')
    stream.write(bytes(4)); stream.flush()
    length=struct.unpack('!I',stream.read(4))[0]
    print(json.loads(stream.read(length))['backend'])
'@
$bridge = $guestProbe | & ssh -T -o ConnectTimeout=5 $Vm python3 -
if ($LASTEXITCODE -or $bridge -ne 'cuda:0') {
    throw '主机GPU正常，但Ubuntu转发未连通；需关闭失效的旧转发后重试，不回退CPU。'
}
Write-Host "YOLO $($status.backend)：$($status.gpu)；已核对Ubuntu到GPU，可运行 ./run-interactive.sh black"
