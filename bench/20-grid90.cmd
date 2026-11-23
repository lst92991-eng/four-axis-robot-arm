@echo off
rem 空吸头依次定位九十个交点，--plan 查看离线路线。
call "%~dp0_run.cmd" grid90 %*
exit /b %errorlevel%
