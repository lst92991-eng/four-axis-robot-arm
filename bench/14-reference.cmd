@echo off
rem 在已知实物姿态下接续三轴机械参考，加 --write 后写入。
call "%~dp0_run.cmd" reference %*
exit /b %errorlevel%
