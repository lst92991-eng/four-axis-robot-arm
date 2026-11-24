// 固定棋盘的三步流程是用三点标定交点、把本帧棋子归格、绘制格点预览。
#include "geometry.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <stdexcept>

namespace scara
{
//---------取三个标定点，生成90个交点---------
// 从红方看，依次取红方左角 (9,0)、红方右角 (9,8)、黑方左角 (0,0)。
void Geometry_calibrate(BoardPoint redLeft, BoardPoint redRight, BoardPoint blackLeft, Geometry &board)
{
    board = {};
    board.origin = blackLeft;
    // 红方底线横跨 8 格，从黑方底线到红方底线跨 9 格。
    board.rowStep.pixel = (redLeft.pixel - blackLeft.pixel) / (Rows - 1);
    board.rowStep.xy = (redLeft.xy - blackLeft.xy) / (Rows - 1);
    board.columnStep.pixel = (redRight.pixel - redLeft.pixel) / (Columns - 1);
    board.columnStep.xy = (redRight.xy - redLeft.xy) / (Columns - 1);
    // 在像素平面和基座平面分别求一格的有向面积。面积为零时无法从位移反求行列。
    double pixelArea = board.columnStep.pixel.cross(board.rowStep.pixel);
    double baseArea = board.columnStep.xy.cross(board.rowStep.xy);
    if (pixelArea == 0 || baseArea == 0 || !std::isfinite(pixelArea) || !std::isfinite(baseArea))
        throw std::runtime_error("标定需要三个有限且不共线的点");

    // 同一个行列在两个平面各算一次。pixel 给画面用，xy 给取放规划用。
    for (int row = 0; row < Rows; ++row)
        for (int col = 0; col < Columns; ++col)
        {
            BoardPoint &point = board.cell[row][col];
            point.pixel = blackLeft.pixel + row * board.rowStep.pixel + col * board.columnStep.pixel;
            point.xy = blackLeft.xy + row * board.rowStep.xy + col * board.columnStep.xy;
        }
}

//---------把相对原点的位移换算成行、列---------
// offset = row × rowStep + col × columnStep。叉乘可消掉其中一个未知数。
void Geometry_rowCol(cv::Point2d offset, cv::Point2d rowStep, cv::Point2d columnStep, double &row, double &col)
{
    double area = columnStep.cross(rowStep);
    row = columnStep.cross(offset) / area;
    col = offset.cross(rowStep) / area;
}

//---------将这一帧的棋子填进棋盘---------
bool Geometry_assign(const Geometry &board, const std::vector<Detection> &pieces, chess::Board &seen, bool visible)
{
    // 可读时先把各格置空，画面不可见时全盘记为未知。
    for (auto &rank : seen) rank.fill(visible ? '.' : '?');
    if (!visible) return false;

    bool valid = true;
    cv::Rect2d image(0, 0, ImageWidth, ImageHeight), grid(0, 0, Columns, Rows);
    for (const Detection &piece : pieces)
    {
        // 框中心在原图上。减掉黑方左角的像素位置，得到相对棋盘原点的位移。
        cv::Point2d center(piece.bbox.x + piece.bbox.width / 2, piece.bbox.y + piece.bbox.height / 2);
        double row, col;
        Geometry_rowCol(center - board.origin.pixel, board.rowStep.pixel, board.columnStep.pixel, row, col);
        // 先保留小数行列，再四舍五入到最近交点。图像坐标按 x、y 排列，对应列、行。
        cv::Point2d nearest(std::round(col), std::round(row));
        // 有效棋子需要已知类别、足够置信度和原图内的框中心。
        bool detected = piece.classId >= 0 && piece.classId < YoloClasses &&
                        piece.confidence >= .5 && piece.confidence <= 1 &&
                        !piece.bbox.empty() && image.contains(center);
        // 最近格点必须位于盘内。以格为单位比较检测中心与该格点的距离。
        double distance = std::hypot(row - nearest.y, col - nearest.x);
        if (!detected || !grid.contains(nearest) || distance > board.assignmentTolerance)
        {
            valid = false;
            continue;
        }
        // 一个交点只能放一枚棋子。两个检测落到同一格时，该格记为未知。
        char &square = seen[int(nearest.y)][int(nearest.x)];
        if (square == '.') square = chess::PieceSymbols[piece.classId];
        else
        {
            square = '?';
            valid = false;
        }
    }
    return valid;
}

//---------在原图上画出交点---------
void Geometry_draw(const cv::Mat &source, const Geometry &board, cv::Mat &image)
{
    // 在原图副本上画标记，识别所用的原图仍可继续使用。
    image = source.clone();
    for (int row = 0; row < Rows; ++row)
        for (int col = 0; col < Columns; ++col)
        {
            // 格号和点都取自标定表，便于逐格核对像素位置。
            cv::Point2d pixel = board.cell[row][col].pixel;
            std::string label = std::to_string(row) + "," + std::to_string(col);
            cv::circle(image, pixel, 3, {0, 255, 255}, -1);
            cv::putText(image, label, pixel + cv::Point2d(4, -4), cv::FONT_HERSHEY_SIMPLEX, .32, {0, 0, 255}, 1, cv::LINE_AA);
        }
}

} // namespace scara
