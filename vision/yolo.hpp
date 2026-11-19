//约定模型参数、识别结果以及图像处理和NPU调用接口
#pragma once
#include <opencv2/core.hpp>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace scara
{
//---------模型参数区---------
//模型输入图像的宽和高，单位为像素
constexpr int YoloSize = 640;
//红黑双方各7种棋子，共14个识别类别
constexpr int YoloClasses = 14;
//每个网格点使用3个锚框，模型共有3路不同尺度的输出
constexpr int YoloAnchors = 3;
constexpr int YoloHeads = 3;
//每个候选框保存4个位置数、1个目标分数和14个类别分数
constexpr int YoloBoxValues = 5 + YoloClasses;
//一个网格点连续保存3个候选框，共57个通道值
constexpr int YoloChannels = YoloAnchors * YoloBoxValues;
//三路输出的网格边长，对应输入图中的8、16、32像素步长
constexpr int YoloGrid[YoloHeads] = {80, 40, 20};
//中文标签字图的宽度，以及单个类别文字占用的高度，单位为像素
constexpr int AtlasWidth = 64;
constexpr int LabelHeight = 24;

//---------识别数据区---------
//单个棋子的识别结果，供棋盘归格和画面绘制使用
struct Detection
{
    //类别编号，0～6为黑方、7～13为红方，顺序与模型一致
    int classId = -1;
    //最终得分，由目标分数乘以该类别分数得到
    double confidence = 0;
    //对外结果中的原图左上角位置及宽高，单位为像素
    cv::Rect2d bbox;
};

//一路NPU输出的读取信息，NHWC布局按行、列、通道排列
struct YoloHead
{
    //INT8输出数据首地址，内存由NPU模块管理
    const int8_t *data = nullptr;
    //相邻两行的实际字节间距，包含硬件对齐区域
    size_t rowBytes = 0;
    //量化零点，表示真实数值0对应的量化整数
    int zeroPoint = 0;
    //反量化系数，真实数值等于(量化整数-zeroPoint)乘以scale
    float scale = 0;
};

//---------图像处理接口区---------
//启动时保存原图尺寸与有效裁剪区域，计算缩放比例和居中位置
void Vision_setup(cv::Size frameSize = {1280, 720}, cv::Rect roi = {300, 70, 650, 600});
//将BGR原图裁剪、转RGB、等比例缩放并补边，写出640×640输入图
//输入帧格式或尺寸异常时，交给主入口的异常收尾
void Vision_prepare(const cv::Mat &bgr, cv::Mat &rgb640);
//读取已完成推理的三路输出，写出筛选后的棋子类别、得分和原图位置框
//离线回放由文件读取入口检查张量边界与格式
void Vision_decode(const YoloHead heads[YoloHeads], std::vector<Detection> &pieces);
//复制原图并画出检测框、类别和得分，生成原尺寸预览
//字图为随程序提供的64×336灰度图，按类别顺序竖向排列
void Vision_draw(const cv::Mat &bgr, const std::vector<Detection> &pieces, const cv::Mat &chineseLabels,
                 cv::Mat &preview);

//---------NPU接口区---------
//加载一个模型并准备输入输出缓冲，返回初始化结果
//主流程在正常退出或失败后统一调用Yolo_close释放资源
bool Yolo_open(std::string modelPath);
//模型打开成功后，传入Vision_prepare生成的图像并执行一次推理
//输出描述中的数据在下一次推理或关闭模型前有效
void Yolo_run(const cv::Mat &rgb640, YoloHead heads[YoloHeads]);
//释放已经分配的张量内存和模型上下文
void Yolo_close();
} // namespace scara
