//拟合六个仿射系数，将二维点换算成目标坐标
#include "geometry.hpp"
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>

namespace scara
{
//---------仿射系数初始化区---------
void fitAffine(const std::vector<cv::Point2d> &source, const std::vector<cv::Point2d> &target, cv::Matx23d &matrix)
{
    //每行是一组输入[u,v,1]和对应输出[X,Y]
    cv::Mat inputs(int(source.size()), 3, CV_64F);
    cv::Mat outputs(int(target.size()), 2, CV_64F);
    cv::Mat coefficients;

    for (int i = 0; i < inputs.rows; ++i)
    {
        inputs.at<double>(i, 0) = source[i].x;
        inputs.at<double>(i, 1) = source[i].y;
        inputs.at<double>(i, 2) = 1;
        outputs.at<double>(i, 0) = target[i].x;
        outputs.at<double>(i, 1) = target[i].y;
    }
    //使用全部采点求最小二乘解，得到两组一次函数系数
    if (!cv::solve(inputs, outputs, coefficients, cv::DECOMP_QR))
        throw std::runtime_error("Affine samples must span a plane");
    //求解结果按列保存X、Y系数，转置为两行
    cv::transpose(coefficients, coefficients);
    matrix = coefficients;
    //系数必须有限，两个坐标方向不能退化成一条直线
    double determinant = matrix(0, 0) * matrix(1, 1) - matrix(0, 1) * matrix(1, 0);
    if (!cv::checkRange(cv::Mat(matrix)) || std::abs(determinant) <= 1e-14)
        throw std::runtime_error("Invalid affine coefficients");
}

//---------二维坐标换算区---------
//两行分别计算X=a*u+b*v+c、Y=d*u+e*v+f，单位由系数决定
void transformPoint(const cv::Matx23d &matrix, cv::Point2d point, cv::Point2d &result)
{
    result.x = matrix(0, 0) * point.x + matrix(0, 1) * point.y + matrix(0, 2);
    result.y = matrix(1, 0) * point.x + matrix(1, 1) * point.y + matrix(1, 2);
}

//---------棋盘目标换算区---------
bool gridTarget(const PlaneMap &map, const Board &board, int column, int row, cv::Point2d &baseMm)
{
    cv::Point2d pixel;

    baseMm = {};
    if (!map.accepted || !board.geometryValid || !board.orientationValid) return false;
    if (column < 0 || column >= Columns || row < 0 || row >= Rows) return false;
    pixel = board.intersections[row * Columns + column];
    if (map.validPixelHull.size() < 3 || cv::pointPolygonTest(map.validPixelHull, pixel, true) < -1e-4) return false;
    transformPoint(map.pixelToBase, pixel, baseMm);
    return true;
}
} // namespace scara
