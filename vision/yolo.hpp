//约定模型参数、识别结果以及图像处理和NPU调用接口
#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace scara
{
//---------模型参数区---------
//模型输入图像的宽和高，单位为像素
constexpr int YoloSize = 640;
//红黑双方各7种棋子，共14个识别类别
constexpr int YoloClasses = 14;
//---------识别数据区---------
//单个棋子的识别结果，供棋盘归格和画面绘制使用
struct Detection
{
    //类别编号，0～6为黑方、7～13为红方，顺序与模型一致
    int classId = -1;
    // 目标分数乘以最高类别分数，得到该框的最终置信度。
    double confidence = 0;
    //对外结果中的原图左上角位置及宽高，单位为像素
    cv::Rect2d bbox;
};

//---------图像处理接口区---------
// 固定机位的缺省裁剪起点；安装文件可指定其他原图位置。
inline const cv::Point DefaultCropOrigin{375, 8};
// 教学机位输入为 1280×720 的 8 位 BGR 图，直裁 640×640 后转换为 RGB。
cv::Mat Vision_prepare(const cv::Mat &bgr, cv::Point origin = DefaultCropOrigin);
// 在原图副本上画出棋子的框、名称和得分，返回预览图。
cv::Mat Vision_draw(const cv::Mat &bgr, const std::vector<Detection> &pieces);

//---------NPU接口区---------
// 加载模型、准备输入输出缓冲。成功返回 true，准备失败时收回已申请的资源。
bool Yolo_open(std::string modelPath);
// 输入 RGB 图，执行 NPU 推理和模型后处理，返回原图坐标下的棋子列表。
std::vector<Detection> Yolo_run(const cv::Mat &rgb640, cv::Point origin = DefaultCropOrigin);
//释放已经分配的张量内存和模型上下文
void Yolo_close();
} // namespace scara
