@echo off
setlocal
set "PYTHONIOENCODING=utf-8"
chcp 65001 >nul
rem 选择台架解释器，优先使用已指定的环境或项目虚拟环境。
if not defined SCARA_PYTHON if exist "%~dp0..\.venv\Scripts\python.exe" set "SCARA_PYTHON=%~dp0..\.venv\Scripts\python.exe"
rem Existing lab installation. A course workstation can use .venv or SCARA_PYTHON.
set "lab=%LOCALAPPDATA%\Packages\OpenAI.Codex_2p2nqsd0c76g0\LocalCache\Local\arm4-tools"
if not defined SCARA_PYTHON if exist "%lab%\can-venv\Scripts\python.exe" set "SCARA_PYTHON=%lab%\can-venv\Scripts\python.exe"
if not defined SCARA_PYTHON set "SCARA_PYTHON=python"
if not defined SCARA_ADB if exist "%lab%\platform-tools\adb.exe" set "SCARA_ADB=%lab%\platform-tools\adb.exe"
if exist "%lab%\openocd-2.1.0\bin\openocd.exe" set "PATH=%PATH%;%lab%\openocd-2.1.0\bin"
rem 把子命令和原有参数交给统一入口，并向调用方返回执行结果。
"%SCARA_PYTHON%" -u "%~dp0run.py" %*
exit /b %errorlevel%
