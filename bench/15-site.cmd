@echo off
rem 用三个实测交点和工作高度生成安装数值表，--install 保存本机版本。
call "%~dp0_run.cmd" site %*
exit /b %errorlevel%
