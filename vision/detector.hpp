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

// 模型必须由 tools/model/export_rknn.py 导出，不接受 COCO 或任意 RKNN 文件。
Detector* detector_load(const char* modelPath, std::string& error);
bool detector_detect(Detector* detector, const cv::Mat& rgb,
                     std::vector<Detection>& result, std::string& error);

// 释放所有 NPU 内存后再启动棋力引擎，避免小内存板同时驻留两套模型。
void detector_close(Detector* detector);
