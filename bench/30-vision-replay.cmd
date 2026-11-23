@echo off
rem 回放已保存的图像和张量，核对 RGB 输入与识别结果。
call "%~dp0_run.cmd" vision %*
exit /b %errorlevel%
