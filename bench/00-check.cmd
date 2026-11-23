@echo off
rem 读取节点状态、固件版本、机械参考和故障，供上电后检查通信。
call "%~dp0_run.cmd" check %*
exit /b %errorlevel%
