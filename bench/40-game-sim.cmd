@echo off
rem 用预置场景运行 C++ 对局主程序，--all 核对全部回归场景。
call "%~dp0_run.cmd" sim %*
exit /b %errorlevel%
