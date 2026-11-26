// 固定棋盘的三步流程是用三点标定交点、把本帧棋子归格、绘制格点预览。
#include "geometry.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>

namespace scara
{
//---------取三个标定点，生成90个交点---------
// 从红方看，依次取红方左角 (9,0)、红方右角 (9,8)、黑方左角 (0,0)。
Geometry Geometry_calibrate(BoardPoint redLeft, BoardPoint redRight, BoardPoint blackLeft)
{
    Geometry board;
    board.origin = blackLeft;
    // 红方底线横跨 8 格，从黑方底线到红方底线跨 9 格。
    board.rowStep.pixel = (redLeft.pixel - blackLeft.pixel) / (Rows - 1);
    board.rowStep.xy = (redLeft.xy - blackLeft.xy) / (Rows - 1);
    board.columnStep.pixel = (redRight.pixel - redLeft.pixel) / (Columns - 1);
    board.columnStep.xy = (redRight.xy - redLeft.xy) / (Columns - 1);

    // 同一个行列在两个平面各算一次。pixel 给画面用，xy 给取放规划用。
    for (int row = 0; row < Rows; ++row)
        for (int col = 0; col < Columns; ++col)
        {
            BoardPoint &point = board.cell[row][col];
            point.pixel = board.origin.pixel + row * board.rowStep.pixel + col * board.columnStep.pixel;
            point.xy = board.origin.xy + row * board.rowStep.xy + col * board.columnStep.xy;
        }
    return board;
}

//---------把相对原点的位移换算成行、列---------
// 两个步长分别放在矩阵的两列，OpenCV 从位移解出走了几列、几行。
cv::Point2d Geometry_gridPoint(cv::Point2d offset, cv::Point2d rowStep, cv::Point2d columnStep)
{
    cv::Matx22d steps(columnStep.x, rowStep.x, columnStep.y, rowStep.y);
    return steps.solve(cv::Vec2d(offset.x, offset.y), cv::DECOMP_LU);
}

//---------从一个检测框找到棋盘交点---------
static bool findSquare(const Geometry &board, const Detection &piece, cv::Point &square)
{
    // 棋子框的中心用原图像素表示。
    const cv::Rect2d &box = piece.bbox;
    cv::Point2d center(box.x + box.width / 2, box.y + box.height / 2);
    const cv::Rect2d image(0, 0, ImageWidth, ImageHeight);
    bool knownClass = piece.classId >= 0 && piece.classId < YoloClasses;
    bool validScore = piece.confidence >= 0.5 && piece.confidence <= 1;
    bool visibleBox = !box.empty() && image.contains(center);
    if (!knownClass || !validScore || !visibleBox) return false;

    // 先反算带小数的列、行，再四舍五入到最近的整数交点。
    cv::Point2d offset = center - board.origin.pixel;
    cv::Point2d cell = Geometry_gridPoint(offset, board.rowStep.pixel, board.columnStep.pixel);
    cv::Point2d nearest(std::round(cell.x), std::round(cell.y));

    // 交点须在盘内，棋子中心与交点的距离以格为单位。
    const cv::Rect2d grid(0, 0, Columns, Rows);
    double distance = std::hypot(cell.x - nearest.x, cell.y - nearest.y);
    if (!grid.contains(nearest) || distance > board.assignmentTolerance) return false;

    square = cv::Point(nearest);
    return true;
}

//---------将这一帧的棋子填进棋盘---------
BoardObservation Geometry_assign(const Geometry &board, const std::vector<Detection> &pieces, bool visible)
{
    BoardObservation frame{{}, visible};
    for (auto &rank : frame.cells) rank.fill(visible ? '.' : '?');
    if (!visible) return frame;

    for (const Detection &piece : pieces)
    {
        // 找到交点后再写棋盘。失败的检测让整帧继续等待确认。
        cv::Point cell;
        if (!findSquare(board, piece, cell))
        {
            frame.valid = false;
            continue;
        }
        // 同格只接受一枚棋子；第二枚落入同格时，标出冲突供操作者恢复。
        char &square = frame.cells[cell.y][cell.x];
        if (square == '.') square = chess::PieceSymbols[piece.classId];
        else
        {
            square = '?';
            frame.valid = false;
        }
    }
    return frame;
}

//---------在原图上画出交点---------
cv::Mat Geometry_draw(const cv::Mat &source, const Geometry &board)
{
    // 在原图副本上画标记，识别所用的原图仍可继续使用。
    cv::Mat image = source.clone();
    for (int row = 0; row < Rows; ++row)
        for (int col = 0; col < Columns; ++col)
        {
            // 格号和点都取自标定表，便于逐格核对像素位置。
            cv::Point2d pixel = board.cell[row][col].pixel;
            std::string label = cv::format("%d,%d", row, col);
            cv::circle(image, pixel, 3, {0, 255, 255}, -1);
            cv::putText(image, label, pixel + cv::Point2d(4, -4), cv::FONT_HERSHEY_SIMPLEX, 0.32, {0, 0, 255}, 1, cv::LINE_AA);
        }
    return image;
}

} // namespace scara
