#pragma once
#include "yolo.hpp"
#include "calibration.hpp"
#include <opencv2/core.hpp>
#include <array>
#include <string>
#include <vector>

//定义棋盘几何、格点观察和仿射坐标换算接口
namespace scara
{
constexpr int Columns = 9, Rows = 10, Cells = Columns * Rows;
constexpr int Unknown = -2, Empty = -1; // 未确认的格子与已确认的空格分开。
constexpr char PieceSymbols[] = "kabnrcpKABNRCP";

struct Board
{
    //行列坐标到原图像素的六个系数，第0行在红方底线
    cv::Matx23d gridToPixel{};
    //按行号乘9加列号保存90个交点
    std::array<cv::Point2d, Cells> intersections{};
    bool geometryValid = false, orientationValid = false;
};

// 四角按逻辑格点(0,0)、(8,0)、(8,9)、(0,9)给出，需对应红黑方向。
void boardFromCorners(const std::array<cv::Point2d, 4> &rawCorners, bool canonicalOrderConfirmed, Board &board);
// 固定棋盘的俯视展示。
void rectifiedBoard(const cv::Mat &geometricImage, const Board &board, cv::Mat &image);

struct Observation
{
    std::string frameId;
    std::array<int, Cells> pieces{};
    bool valid = false; // 本帧结果；提交棋局前再经过连续帧稳定确认。
};
void assignPieces(const Board &board, const std::string &detectionFrameId, const std::vector<Detection> &detections,
                  bool sceneClear, Observation &observation);
class Stabilizer
{
  public:
    void update(const Observation &input, Observation &result);
    void reset();

  private:
    int count_ = 0;
    Observation previous_;
};

//两组一一对应的点求出六个系数，输入由标定和棋盘初始化核对
void fitAffine(const std::vector<cv::Point2d> &source, const std::vector<cv::Point2d> &target, cv::Matx23d &matrix);
void transformPoint(const cv::Matx23d &matrix, cv::Point2d point, cv::Point2d &result);
//固定交点换算成基座XY，使用已确认方向及标定覆盖区
//关节逆解和Z层次由运动模块处理
bool gridTarget(const PlaneMap &map, const Board &board, int column, int row, cv::Point2d &baseMm);
void drawBoard(const cv::Mat &source, const Board &board, cv::Mat &image);
} // namespace scara
