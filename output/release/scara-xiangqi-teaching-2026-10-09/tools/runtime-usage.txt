RV1106 / MIS5001 / OpenCV 5.0.0 / C++17
Step 1: convert one NV12 frame to a color photo.

From Windows, inside the extracted package directory:
  adb push . /root/scara-vision-v1/
  adb shell chmod +x /root/scara-vision-v1/scara_vision
  adb shell sh /root/scara-vision-v1/capture.sh
  adb pull /root/scara-vision-v1/live/source.jpg ./source.jpg

For multiple devices, add -s DEVICE_SERIAL after adb.
This build targets ARMv7 hard-float / uClibc 1.0.31.

Capture uses /dev/video11, 1280x960 NV12 and the board's MIS5001 IQ file.
It prepares live/, clears the old frame/photo, and calls scara_vision.
The program reads the first tightly packed frame (1,843,200 bytes)
from camera.nv12 in its working directory, then saves live/source.jpg.

Exit codes: 0 = photo saved, 1 = conversion/read/write failure,
124 = capture timeout. A chessboard is not needed for this step.
