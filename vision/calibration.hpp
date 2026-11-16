//定义平面标定的采点、误差报告及文件接口
#pragma once
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace scara
{
//固定相机的原图尺寸与配置标识
constexpr int ImageWidth = 1280, ImageHeight = 720;
constexpr char CameraId[] = "sc3336-1280x720-fixed-focus-v1";

struct PointPair
{
    //同一位置的原图像素坐标和基座XY毫米坐标
    cv::Point2d pixel, baseXY;
};
struct ErrorReport
{
    //均方根误差和最大误差，单位为毫米
    double rmsMm = 0, maxMm = 0;
};
struct PlaneMap
{
    //标定记录编号
    std::string id;
    //标定平面高度和允许误差，单位为毫米
    double planeZmm = 0, allowedErrorMm = 0;
    //像素坐标到基座XY毫米坐标的六个仿射系数
    cv::Matx23d pixelToBase{};
    //实测拟合点覆盖的图像范围
    std::vector<cv::Point2f> validPixelHull;
    //保存拟合点与独立检查点，供文件回读时复核
    std::vector<PointPair> fitPoints, checkPoints;
    //两组采点的误差报告和验收结果
    ErrorReport fitError, checkError;
    bool accepted = false;
};

//读取采点，计算并验收六个系数，将完整标定交给调用者
void loadPlane(const std::string &path, PlaneMap &map);
//输入拟合点和独立检查点，将系数、覆盖区和验误结果写入map
void calibratePlane(const std::string &id, double planeZmm, double allowedErrorMm, const std::vector<PointPair> &fit,
                    const std::vector<PointPair> &independentChecks, PlaneMap &map);
void savePlane(const std::string &path, const PlaneMap &);
void checkCamera(const cv::FileNode &node);
} // namespace scara
