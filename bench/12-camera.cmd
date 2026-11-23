@echo off
rem 从相机取一张原图，供核对清晰度和棋盘覆盖范围。
call "%~dp0_run.cmd" frame %*
exit /b %errorlevel%
