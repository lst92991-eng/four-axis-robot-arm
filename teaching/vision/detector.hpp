#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <vector>

// 模型类别固定为 kabnrcpKABNRCP；小写黑方，大写红方。
// 方框坐标属于传入的 RGB 图像，单位为像素。
struct Detection {
    int classId;
    float confidence;
    float x1, y1, x2, y2;
};

struct Detector;

// 模型须匹配已验证的14类/640RGB/三头INT8契约，来源见vision/SOURCES.md。
Detector* detector_load(const char* modelPath, std::string& error);
bool detector_detect(Detector* detector, const cv::Mat& rgb,
                     std::vector<Detection>& result, std::string& error);

// 释放所有 NPU 内存后再启动棋力引擎，避免小内存板同时驻留两套模型。
void detector_close(Detector* detector);
