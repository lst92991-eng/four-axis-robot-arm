@echo off
setlocal
set PYTHONDONTWRITEBYTECODE=1
rem 启动棋盘面板，由页面选择开局或接续。--mock 使用离线演示。
call "%~dp0_run.cmd" game %*
exit /b %errorlevel%
