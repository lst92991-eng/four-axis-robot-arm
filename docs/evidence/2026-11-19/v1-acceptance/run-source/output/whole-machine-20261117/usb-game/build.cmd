@echo off
call "C:/Program Files/Microsoft Visual Studio/18/Community/Common7/Tools/VsDevCmd.bat" -no_logo -arch=x64
if errorlevel 1 exit /b 1
"C:/Program Files/Microsoft Visual Studio/18/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe" -S output/whole-machine-20261117/usb-game -B output/whole-machine-20261117/usb-game/build -G Ninja -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR=C:/Users/shuto/.cache/scara-geometry/sdk/opencv/build -DPython3_ROOT_DIR="D:/Program Files/Python/Python313"
if errorlevel 1 exit /b 1
"C:/Program Files/Microsoft Visual Studio/18/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe" --build output/whole-machine-20261117/usb-game/build --target scara_usb_game --parallel 6
exit /b %errorlevel%