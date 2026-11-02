param([string]$Serial = '759dce9b268fd05e')
$ErrorActionPreference = 'Stop'
$packagePath = Join-Path $PSScriptRoot '..\output\rv1106\scara-vision-v1'
if (-not (Test-Path -LiteralPath (Join-Path $packagePath 'scara_vision'))) {
    throw 'Build the RV1106 package first.'
}
& adb -s $Serial push "$packagePath/." /root/scara-vision-v1/
if ($LASTEXITCODE -ne 0) { throw 'ADB upload failed.' }
& adb -s $Serial shell chmod +x /root/scara-vision-v1/scara_vision
if ($LASTEXITCODE -ne 0) { throw 'Could not set executable permission.' }
Write-Output 'Deployed. Run /root/scara-vision-v1/capture.sh on the board.'
