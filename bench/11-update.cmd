@echo off
rem 核对选定轴的共用固件包，加 --write 后通过 CAN 在线更新。
call "%~dp0_run.cmd" update %*
exit /b %errorlevel%
