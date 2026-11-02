param(
    [Parameter(Position=0)][ValidateRange(1,3000)][int]$Rpm = 50,
    [switch]$CheckOnly,
    [switch]$SetupOnly
)
$ErrorActionPreference = 'Stop'
try {
    if ($CheckOnly -and $SetupOnly) { throw 'Use either CheckOnly or SetupOnly.' }
    if (-not [Environment]::Is64BitOperatingSystem -or $env:PROCESSOR_ARCHITEW6432 -eq 'ARM64' -or $env:PROCESSOR_ARCHITECTURE -eq 'ARM64') {
        throw 'This demo requires Windows 10/11 on an x64 PC.'
    }
    $panelRoot = (Resolve-Path -LiteralPath (Join-Path $PSScriptRoot '..\..')).Path
    $panelMain = Join-Path $PSScriptRoot 'panel.py'
    $panelData = Join-Path $env:LOCALAPPDATA 'scara-z-axis-panel'
    $panelOffline = Join-Path $PSScriptRoot 'dependencies'
    $panelRequirements = Join-Path $PSScriptRoot 'requirements.txt'
    $panelRuntime = Join-Path $panelData 'python-libs'
    $panelPython = $null
    # Prefer an existing Python 3.11 with Tk; no PATH or administrator changes.
    foreach ($candidate in @((Join-Path $env:LOCALAPPDATA 'Programs\Python\Python311\python.exe'), (Join-Path $panelData 'Python311\python.exe'))) {
        if (Test-Path -LiteralPath $candidate) {
            & $candidate -I -c 'import sys, struct, tkinter; assert sys.version_info[:2] == (3, 11) and struct.calcsize(chr(80)) == 8' 2>$null
            if ($LASTEXITCODE -eq 0 -and (Test-Path -LiteralPath (Join-Path (Split-Path $candidate) 'pythonw.exe'))) {
                $panelPython = $candidate
                break
            }
        }
    }
    if (-not $panelPython) {
        if ($CheckOnly) { throw 'Python 3.11 with Tk is missing. Run the BAT normally to install it.' }
        $panelInstallerName = 'python-3.11.9-amd64.exe'
        $panelInstaller = Join-Path $panelOffline $panelInstallerName
        if (-not (Test-Path -LiteralPath $panelInstaller)) {
            New-Item -ItemType Directory -Path $panelData -Force | Out-Null
            $panelInstaller = Join-Path $panelData $panelInstallerName
            if (-not (Test-Path -LiteralPath $panelInstaller)) {
                Write-Host 'Downloading Python 3.11.9 from python.org...'
                [Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12
                Invoke-WebRequest -UseBasicParsing -Uri "https://www.python.org/ftp/python/3.11.9/$panelInstallerName" -OutFile $panelInstaller
            }
        }
        # Digest from the official Python Sigstore bundle; never execute an unchecked download.
        $panelInstallerHash = '5ee42c4eee1e6b4464bb23722f90b45303f79442df63083f05322f1785f5fdde'
        if ((Get-FileHash -LiteralPath $panelInstaller -Algorithm SHA256).Hash -ne $panelInstallerHash) {
            throw "Python installer SHA-256 mismatch. Remove this download and try again: $panelInstaller"
        }
        $panelPythonDir = Join-Path $panelData 'Python311'
        Write-Host 'Installing Python and Tk for the current user...'
        $panelInstallProcess = Start-Process -FilePath $panelInstaller -WindowStyle Hidden -Wait -PassThru -ArgumentList @(
            '/quiet', 'InstallAllUsers=0', ('TargetDir="' + $panelPythonDir + '"'),
            'Include_pip=1', 'Include_tcltk=1', 'Include_launcher=0', 'InstallLauncherAllUsers=0',
            'Include_test=0', 'Include_doc=0', 'Shortcuts=0', 'AssociateFiles=0', 'PrependPath=0'
        )
        if ($panelInstallProcess.ExitCode -notin @(0, 3010)) { throw "Python installation failed: $($panelInstallProcess.ExitCode)" }
        $panelPython = Join-Path $panelPythonDir 'python.exe'
    }
    $panelPythonw = Join-Path (Split-Path $panelPython) 'pythonw.exe'
    foreach ($required in @($panelMain, $panelPython, $panelPythonw)) {
        if (-not (Test-Path -LiteralPath $required)) { throw "Required file missing: $required" }
    }
    # Isolate from user PYTHONPATH and user-site packages on the demonstration PC.
    $env:PYTHONNOUSERSITE = '1'
    $env:PYTHONPATH = $panelRuntime
    $panelProbe = @'
import sys
try:
    import tkinter, usb.core, gs_usb, libusb_package, struct
    from importlib.metadata import version
    from pathlib import Path
    assert sys.version_info[:2] == (3, 11) and struct.calcsize('P') == 8
    for line in Path(sys.argv[1]).read_text().splitlines():
        if line.strip() and not line.startswith('#'):
            package, expected = line.split()[0].split('==')
            assert version(package) == expected
    import usb.backend.libusb1
    assert usb.backend.libusb1.get_backend(find_library=libusb_package.find_library) is not None
except Exception:
    sys.exit(1)
'@
    & $panelPython -c $panelProbe $panelRequirements
    if ($LASTEXITCODE -ne 0) {
        if ($CheckOnly) { throw 'Python dependencies are not ready. Run the BAT normally to install them.' }
        Write-Host 'Installing pinned panel dependencies into LocalAppData...'
        $panelPipArgs = @('-m', 'pip', '--isolated', 'install', '--disable-pip-version-check', '--require-hashes', '--only-binary=:all:', '--upgrade', '--target', $panelRuntime, '-r', $panelRequirements)
        $panelWheels = Join-Path $panelOffline 'wheels'
        if (Test-Path -LiteralPath $panelWheels) { $panelPipArgs += @('--no-index', '--find-links', $panelWheels) }
        & $panelPython @panelPipArgs
        if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed.' }
        & $panelPython -c $panelProbe $panelRequirements
        if ($LASTEXITCODE -ne 0) { throw 'Dependency check failed after installation.' }
    }
    if ($CheckOnly -or $SetupOnly) {
        Write-Host "OK: Python, Tk, USB libraries and panel entry are ready. Target RPM: $Rpm"
        Write-Host 'No USB connection, GUI, or motor commands were started.'
        exit 0
    }

    # Reuse the existing panel rather than opening a second USB controller.
    $panelExisting = Get-CimInstance Win32_Process -Filter "Name='pythonw.exe'" |
        Where-Object { $_.CommandLine -and $_.CommandLine.IndexOf($panelMain, [StringComparison]::OrdinalIgnoreCase) -ge 0 } |
        Select-Object -First 1
    if ($panelExisting) {
        $panelProcess = Get-Process -Id $panelExisting.ProcessId
        Add-Type -TypeDefinition @'
using System;
using System.Runtime.InteropServices;
public static class ScaraPanelWindow {
    [DllImport("user32.dll")] public static extern bool ShowWindowAsync(IntPtr handle, int command);
    [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr handle);
}
'@
        if ($panelProcess.MainWindowHandle -ne [IntPtr]::Zero) {
            [void][ScaraPanelWindow]::ShowWindowAsync($panelProcess.MainWindowHandle, 9)
            [void][ScaraPanelWindow]::SetForegroundWindow($panelProcess.MainWindowHandle)
        }
        Write-Host 'Panel is already running; kept its current settings and motion state.'
        exit 0
    }
    $panelOutput = Join-Path $panelRoot 'output\hardware-validation'
    New-Item -ItemType Directory -Path $panelOutput -Force | Out-Null
    $panelStdout = Join-Path $panelOutput 'z-panel-stdout.log'
    $panelStderr = Join-Path $panelOutput 'z-panel-stderr.log'
    $panelProcess = Start-Process -FilePath $panelPythonw -WindowStyle Normal -ArgumentList @(('"' + $panelMain + '"'), "--rpm=$Rpm") -WorkingDirectory $panelRoot -RedirectStandardOutput $panelStdout -RedirectStandardError $panelStderr -PassThru
    $panelProcess.Id | Set-Content -LiteralPath (Join-Path $panelOutput 'z-panel.pid')
    if ($panelProcess.WaitForExit(800)) {
        $panelDetails = Get-Content -LiteralPath $panelStderr -Raw
        throw "Panel exited during startup. $panelDetails"
    }
    Write-Host "Panel opened in manual standby. Target RPM: $Rpm"
    Write-Host "Startup errors: $panelStderr"
    exit 0
} catch {
    Write-Host $_.Exception.Message -ForegroundColor Red
    exit 1
}
