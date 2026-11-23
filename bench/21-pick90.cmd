@echo off
rem 用同一颗棋子依次验证九十格取放，--plan 查看离线路线。
call "%~dp0_run.cmd" pick90 %*
exit /b %errorlevel%
