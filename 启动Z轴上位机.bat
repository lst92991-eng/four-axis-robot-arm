@echo off
setlocal
title SCARA Z-axis Control Panel
powershell.exe -NoLogo -NoProfile -ExecutionPolicy Bypass -File "%~dp0tools\z-axis-panel\run.ps1" %*
set "panelExit=%errorlevel%"
if not "%panelExit%"=="0" (
    echo.
    echo Launch failed. Read the message above and press any key to close.
    pause >nul
)
exit /b %panelExit%
