cmake_minimum_required(VERSION 3.16)
project(scara_vision LANGUAGES CXX)

# 固定到实际验证过的5.0.0，避免本机的4.x被意外找到并混用头文件/动态库。
# 当前只用Mat、颜色转换和图片保存三个模块。
find_package(OpenCV 5.0.0 EXACT REQUIRED COMPONENTS core imgproc imgcodecs)
add_executable(scara_vision main.cpp)
# 用C++17标准特性，并关闭GNU语言扩展，便于保持宿主与交叉工具链的语法一致。
target_compile_features(scara_vision PRIVATE cxx_std_17)
set_target_properties(scara_vision PROPERTIES CXX_EXTENSIONS OFF)
target_include_directories(scara_vision PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(scara_vision PRIVATE ${OpenCV_LIBS})
