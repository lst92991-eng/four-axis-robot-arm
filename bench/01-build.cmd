@echo off
setlocal
chcp 65001 >nul
rem 先定位 OpenCV SDK，再准备 Visual Studio 的 x64 编译环境。
if not defined OpenCV_DIR if exist "%USERPROFILE%\.cache\scara-geometry\sdk\opencv\build\OpenCVConfig.cmake" set "OpenCV_DIR=%USERPROFILE%\.cache\scara-geometry\sdk\opencv\build"
if not defined OpenCV_DIR (
  echo Set OpenCV_DIR to the OpenCV 5.0.0 CMake directory.
  exit /b 2
)
if not defined VSINSTALLDIR (
  for /f "usebackq tokens=*" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do call "%%i\Common7\Tools\VsDevCmd.bat" -no_logo -arch=x64
)
if not defined VSINSTALLDIR exit /b 2
set "tools=%VSINSTALLDIR%Common7\IDE\CommonExtensions\Microsoft\CMake"
set "cmake=%tools%\CMake\bin\cmake.exe"
if not exist "%cmake%" set "cmake=cmake"
set "SCARA_BUILD_PY="
if defined SCARA_PYTHON set SCARA_BUILD_PY=-DPython3_EXECUTABLE="%SCARA_PYTHON%"
if not defined SCARA_BUILD_PY if exist "%~dp0..\.venv\Scripts\python.exe" set SCARA_BUILD_PY=-DPython3_EXECUTABLE="%~dp0..\.venv\Scripts\python.exe"
rem 以项目根目录为工作目录，配置并构建 Windows 台架 DLL、巡点工具和回放程序。
pushd "%~dp0.."
"%cmake%" -S bench/field -B build/bench -G Ninja -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR="%OpenCV_DIR%" %SCARA_BUILD_PY%
if errorlevel 1 goto failed
"%cmake%" --build build/bench --parallel 6
if errorlevel 1 goto failed
popd
exit /b 0
:failed
popd
exit /b 1
