//四角建立棋盘坐标，识别结果归格，最后生成预览
#include "geometry.hpp"
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scara
{
namespace
{
//俯视预览宽高、相邻交点间距和外边距，单位为像素
constexpr int RectWidth = 432, RectHeight = 480, Step = 48, Margin = 24;
//棋子中心和拟合角点允许偏离交点的最大格距
constexpr double MaxGridDistance = .35;
//条件未满足时，将输入错误交给调用者处理
void require(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}
} // namespace

//---------棋盘初始化区---------
void boardFromCorners(const std::array<cv::Point2d, 4> &raw, bool confirmed, Board &board)
{
    //按红方视角的左下、右下、右上、左上顺序保存原图四角
    std::vector<cv::Point2d> pixels(raw.begin(), raw.end());
    //四角对应的逻辑列、行，列为0～8，行为0～9
    std::vector<cv::Point2d> corners{{0, 0}, {8, 0}, {8, 9}, {0, 9}};
    //轮廓用于检查四角顺序，反向矩阵用于检查拟合格距误差
    std::vector<cv::Point2f> outline(raw.begin(), raw.end());
    cv::Matx23d pixelToGrid;
    cv::Rect2d frame(0, 0, ImageWidth, ImageHeight);
    //当前正在计算的棋盘列、行
    cv::Point2d cell;
    //先在局部对象中生成棋盘，全部完成后交给调用者
    Board prepared;

    //四个角点必须处于原图范围内
    for (cv::Point2d point : raw)
        require(frame.contains(point), "Invalid raw corner");
    //四角按边界顺序组成凸四边形，面积大于100平方像素
    require(cv::isContourConvex(outline) && std::abs(cv::contourArea(outline)) > 100,
            "Corners must form a nondegenerate ordered convex quad");
    //四角共同拟合六个系数，检查仿射模型在角点处的偏差
    fitAffine(corners, pixels, prepared.gridToPixel);
    cv::invertAffineTransform(prepared.gridToPixel, pixelToGrid);
    for (int i = 0; i < 4; ++i)
    {
        transformPoint(pixelToGrid, raw[i], cell);
        require(cv::norm(cell - corners[i]) <= MaxGridDistance, "Board corners exceed affine grid tolerance");
    }
    //依次遍历10行、9列，计算所有交点的像素位置
    for (int row = 0; row < Rows; ++row)
        for (int col = 0; col < Columns; ++col)
        {
            cell.x = col;
            cell.y = row;
            //数组下标为行号乘9再加列号，共保存90个交点
            transformPoint(prepared.gridToPixel, cell, prepared.intersections[row * Columns + col]);
        }
    //几何计算成功，红黑方向采用调用者给出的确认状态
    prepared.geometryValid = true;
    prepared.orientationValid = confirmed;
    //交付完整的棋盘坐标和有效状态
    board = prepared;
}

//---------棋子归格区---------
void assignPieces(const Board &board, const std::string &frameId, const std::vector<Detection> &detections, bool clear,
                  Observation &observation)
{
    constexpr double MinScore = .50;
    std::array<bool, Cells> occupied{};
    cv::Rect2d frame(0, 0, ImageWidth, ImageHeight);
    cv::Rect2d gridBounds(-.5, -.5, Columns, Rows);
    //原图像素到逻辑行列的六个反向系数
    cv::Matx23d pixelToGrid;

    observation = {};
    observation.frameId = frameId;
    observation.pieces.fill(Unknown);
    observation.valid = board.geometryValid && board.orientationValid && !frameId.empty();
    if (!observation.valid) return;
    if (clear)
        observation.pieces.fill(Empty);
    observation.valid = clear;
    cv::invertAffineTransform(board.gridToPixel, pixelToGrid);
    for (const Detection &piece : detections)
    {
        cv::Point2d center;        // 棋子检测框中心，原图像素。
        cv::Point2d grid, nearest; // 连续格点坐标和最近的整数交点。
        int index;
        bool valid, inside;

        center.x = piece.bbox.x + piece.bbox.width / 2;
        center.y = piece.bbox.y + piece.bbox.height / 2;
        valid = piece.classId >= 0 && piece.classId < YoloClasses &&
                piece.confidence >= MinScore && piece.confidence <= 1 &&
                !piece.bbox.empty() && frame.contains(center);
        if (!valid)
        {
            observation.valid = false;
            continue;
        }
        //棋子中心换算为连续行列，再寻找最近的整数交点
        transformPoint(pixelToGrid, center, grid);
        inside = gridBounds.contains(grid);
        if (inside)
        {
            nearest.x = std::round(grid.x);
            nearest.y = std::round(grid.y);
            inside = cv::norm(grid - nearest) <= MaxGridDistance;
        }
        if (!inside)
        {
            observation.valid = false;
            continue;
        }
        index = int(nearest.y) * Columns + int(nearest.x);
        observation.pieces[index] = occupied[index] ? Unknown : piece.classId;
        if (occupied[index]) observation.valid = false;
        occupied[index] = true;
    }
}

//---------连续帧确认区---------
void Stabilizer::reset()
{
    count_ = 0;
    previous_ = Observation{};
}

void Stabilizer::update(const Observation &input, Observation &result)
{
    result = input;
    if (!input.valid)
    {
        reset();
        return;
    }
    if (input.frameId == previous_.frameId)
    {
        result.valid = false;
        return;
    }
    bool same = count_ > 0 && input.pieces == previous_.pieces;
    count_ = same ? std::min(count_ + 1, 3) : 1;
    previous_ = input;
    result.valid = count_ >= 3;
}

//---------棋盘绘制区---------
void drawBoard(const cv::Mat &image, const Board &board, cv::Mat &out)
{
    out = image.clone();
    if (!board.geometryValid) return;
    for (int r = 0; r < Rows; ++r)
        for (int c = 0; c < Columns; ++c)
        {
            cv::Point2d pixel = board.intersections[r * Columns + c];
            cv::Point2d textOrigin = pixel + cv::Point2d(4, -4);
            std::string label = std::to_string(c) + "," + std::to_string(r);
            cv::circle(out, pixel, 3, {0, 255, 255}, -1);
            cv::putText(out, label, textOrigin, cv::FONT_HERSHEY_SIMPLEX, 0.32, {0, 0, 255}, 1, cv::LINE_AA);
        }
    cv::putText(out, board.orientationValid ? "ORIENTATION CONFIRMED" : "ORIENTATION UNKNOWN - NO ROBOT TARGETS",
                {10, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 0, 255}, 2);
}

//---------俯视预览区---------
void rectifiedBoard(const cv::Mat &image, const Board &board, cv::Mat &result)
{
    //原图像素先换算为逻辑行列，再缩放到预览像素
    cv::Matx23d pixelToRect;
    //俯视预览固定输出432×480
    cv::Size outputSize(RectWidth, RectHeight);

    //生成预览前确认棋盘几何已经建立
    require(board.geometryValid, "Grid required for rectification");
    //先由原图反算棋盘坐标，再映射到俯视预览
    cv::invertAffineTransform(board.gridToPixel, pixelToRect);
    //每格48像素，纵向翻转使红方位于下边
    for (int column = 0; column < 3; ++column)
    {
        pixelToRect(0, column) *= Step;
        pixelToRect(1, column) *= -Step;
    }
    pixelToRect(0, 2) += Margin;
    pixelToRect(1, 2) += Margin + 9 * Step;
    //按组合矩阵变换原图，结果写入输出图像
    cv::warpAffine(image, result, pixelToRect, outputSize);
}
} // namespace scara
