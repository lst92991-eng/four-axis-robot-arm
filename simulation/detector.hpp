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

// 桌面只接本机8767的已验证CUDA服务；RV1106另用已有RKNN适配。
Detector* detector_load(std::string& error);
bool detector_detect(Detector* detector, const cv::Mat& rgb,
                     std::vector<Detection>& result, std::string& error);

std::string detector_backend(Detector* detector);
void detector_close(Detector* detector);
