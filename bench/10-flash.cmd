@echo off
rem 按轴选择 SWD 首装镜像。默认校验，加 --write 才写板。
call "%~dp0_run.cmd" flash %*
exit /b %errorlevel%
