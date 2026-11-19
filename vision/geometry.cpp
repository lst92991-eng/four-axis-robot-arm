//一份配置完成标定和棋盘建系，运行时依次归格、确认盘面、查询目标
#include "geometry.hpp"
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scara
{
static void fitAffine(const cv::Mat &pixels, const cv::Mat &targets, cv::Matx23d &matrix);
static void transformPoint(const cv::Matx23d &matrix, cv::Point2d point, cv::Point2d &result);

//---------初始化区---------
//三点固定选a0、i0、a9；每行[u,v,X毫米,Y毫米]，教学采点保证不共线
void Geometry_init(const cv::FileNode &config, Geometry &geometry)
{
    geometry = {};
    //同一组三点同时建立像素、棋盘行列与机械臂XY的对应关系
    cv::Mat points;
    config["points"] >> points;
    cv::Mat pixels = points.colRange(0, 2), base = points.colRange(2, 4);
    double cells[3][2] = {{0, 0}, {8, 0}, {0, 9}};
    cv::Mat logical(3, 2, CV_64F, cells);
    fitAffine(pixels, base, geometry.pixelToBase);
    fitAffine(pixels, logical, geometry.pixelToGrid);
    //求一次反向关系，按行列顺序生成90个像素和毫米目标
    cv::Matx23d gridToPixel;
    cv::invertAffineTransform(geometry.pixelToGrid, gridToPixel);
    for (int row = 0; row < Rows; ++row)
        for (int col = 0; col < Columns; ++col)
        {
            int index = row * Columns + col;
            transformPoint(gridToPixel, {double(col), double(row)}, geometry.pixels[index]);
            transformPoint(geometry.pixelToBase, geometry.pixels[index], geometry.targets[index]);
        }
    geometry.ready = true;
}

//---------识别归格区---------
void Geometry_observe(Geometry &geometry, const std::string &frameId, const std::vector<Detection> &pieces,
                      bool sceneClear, Observation &result)
{
    result = {};
    result.frameId = frameId;
    //入口确定本帧能否参与归格，未准备好的画面保持全未知
    result.valid = geometry.ready && !frameId.empty() && sceneClear;
    result.pieces.fill(result.valid ? Empty : Unknown);
    cv::Rect2d frame(0, 0, ImageWidth, ImageHeight), bounds(-.5, -.5, Columns, Rows);
    if (result.valid)
        for (const Detection &piece : pieces)
        {
            cv::Point2d center(piece.bbox.x + piece.bbox.width / 2, piece.bbox.y + piece.bbox.height / 2);
            if (piece.classId < 0 || piece.classId >= YoloClasses || !(piece.confidence >= .5 && piece.confidence <= 1) ||
                piece.bbox.empty() || !frame.contains(center))
            {
                result.valid = false;
                continue;
            }
            //原图中心反算为连续行列，取最近交点并检查0.35格容差
            cv::Point2d cell;
            transformPoint(geometry.pixelToGrid, center, cell);
            cv::Point2d nearest(std::round(cell.x), std::round(cell.y));
            if (!bounds.contains(cell) || cv::norm(cell - nearest) > .35)
            {
                result.valid = false;
                continue;
            }
            int index = int(nearest.y) * Columns + int(nearest.x);
            bool empty = result.pieces[index] == Empty;
            result.pieces[index] = empty ? piece.classId : Unknown;
            result.valid = result.valid && empty;
        }
    //无效检测打断连续性，重复读取上一帧保持已有次数
    if (!result.valid)
    {
        geometry.previous = {};
        geometry.stableFrames = 0;
        return;
    }
    //只有编号更新的新帧才累计次数，重复读取保持原计数
    bool fresh = frameId != geometry.previous.frameId;
    if (fresh)
    {
        bool same = geometry.stableFrames > 0 && result.pieces == geometry.previous.pieces;
        //盘面变化从1开始，相同盘面累计至3，随后保持有效
        geometry.stableFrames = same ? std::min(geometry.stableFrames + 1, 3) : 1;
        geometry.previous = result;
    }
    result.valid = fresh && geometry.stableFrames >= 3;
}

//---------走棋目标区---------
bool Geometry_target(const Geometry &geometry, int column, int row, cv::Point2d &baseMm)
{
    //先核对布局和下标，通过后查缓存；失败输出清零并返回false
    bool valid = geometry.ready && column >= 0 && column < Columns && row >= 0 && row < Rows;
    baseMm = valid ? geometry.targets[row * Columns + column] : cv::Point2d();
    return valid;
}

//---------检查图绘制区---------
void Geometry_draw(const cv::Mat &source, const Geometry &geometry, cv::Mat &image)
{
    image = source.clone();
    //在原图副本上标出90个交点和行列，供现场检查坐标位置
    if (!geometry.ready) return;
    for (int i = 0; i < Cells; ++i)
    {
        cv::Point2d pixel = geometry.pixels[i];
        std::string label = std::to_string(i % Columns) + "," + std::to_string(i / Columns);
        cv::circle(image, pixel, 3, {0, 255, 255}, -1);
        cv::putText(image, label, pixel + cv::Point2d(4, -4), cv::FONT_HERSHEY_SIMPLEX, .32, {0, 0, 255}, 1, cv::LINE_AA);
    }
}

//---------数学换算区---------
//三个对应点组成线性方程，求出六个仿射系数
static void fitAffine(const cv::Mat &pixels, const cv::Mat &targets, cv::Matx23d &matrix)
{
    //输入N×3为[u,v,1]，输出N×2为[X,Y]
    cv::Mat inputs(pixels.rows, 3, CV_64F, cv::Scalar(1)), coefficients;
    pixels.copyTo(inputs.colRange(0, 2));
    if (!cv::solve(inputs, targets, coefficients, cv::DECOMP_LU) || !cv::checkRange(coefficients))
        throw std::runtime_error("采点无法求出仿射系数");
    //解方程后转置，两行分别保存X和Y的三个系数
    cv::transpose(coefficients, coefficients);
    matrix = coefficients;

}
static void transformPoint(const cv::Matx23d &matrix, cv::Point2d point, cv::Point2d &result)
{
    //按两行一次函数换算一点，输出单位由输入系数决定
    result.x = matrix(0, 0) * point.x + matrix(0, 1) * point.y + matrix(0, 2);
    result.y = matrix(1, 0) * point.x + matrix(1, 1) * point.y + matrix(1, 2);
}
} // namespace scara
