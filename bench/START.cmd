@echo off
rem 显示台架菜单，按编号选择检查、标定、巡点或对局。
call "%~dp0_run.cmd" menu
exit /b %errorlevel%
