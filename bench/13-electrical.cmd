@echo off
rem 选择一个轴做电角标定，--run 执行标定并保存结果。
call "%~dp0_run.cmd" electrical %*
exit /b %errorlevel%
