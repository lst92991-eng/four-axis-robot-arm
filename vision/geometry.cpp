// 固定棋盘的三步流程是用三点标定交点、把本帧棋子归格、绘制格点预览。
#include "geometry.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <optional>

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
            point.pixel = blackLeft.pixel + row * board.rowStep.pixel + col * board.columnStep.pixel;
            point.xy = blackLeft.xy + row * board.rowStep.xy + col * board.columnStep.xy;
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

//---------从棋子中心找最近的交点---------
static std::optional<cv::Point> nearestSquare(const Geometry &board, cv::Point2d center)
{
    cv::Point2d offset = center - board.origin.pixel;
    cv::Point2d cell = Geometry_gridPoint(offset, board.rowStep.pixel, board.columnStep.pixel);

    // 最外一行或一列再往外半格，取整就会落到盘外。
    bool onColumn = cell.x > -0.5 && cell.x < Columns - 0.5;
    bool onRow = cell.y > -0.5 && cell.y < Rows - 0.5;
    if (!onColumn || !onRow)
        return std::nullopt;

    int column = static_cast<int>(cell.x + 0.5);
    int row = static_cast<int>(cell.y + 0.5);
    double distance = std::hypot(cell.x - column, cell.y - row);
    if (distance > board.assignmentTolerance)
        return std::nullopt;
    return cv::Point(column, row);
}

//---------将这一帧的棋子填进棋盘---------
BoardObservation Geometry_assign(const Geometry &board, const std::vector<Detection> &pieces, bool visible)
{
    BoardObservation frame{{}, visible};
    for (auto &rank : frame.cells) rank.fill(visible ? '.' : '?');
    if (!visible) return frame;

    const cv::Rect2d image(0, 0, ImageWidth, ImageHeight);
    for (const Detection &piece : pieces)
    {
        cv::Point2d center(piece.bbox.x + piece.bbox.width / 2, piece.bbox.y + piece.bbox.height / 2);
        bool knownClass = piece.classId >= 0 && piece.classId < YoloClasses;
        bool scoreInRange = piece.confidence >= 0.5 && piece.confidence <= 1;
        bool visibleBox = !piece.bbox.empty() && image.contains(center);
        std::optional<cv::Point> cell;
        if (knownClass && scoreInRange && visibleBox)
            cell = nearestSquare(board, center);

        // 检测无效或找不到交点，这一帧都不能确认走法。
        if (!cell)
        {
            frame.valid = false;
            continue;
        }
        // 同格只接受一枚棋子；第二枚落入同格时，标出冲突供操作者恢复。
        char &square = frame.cells[cell->y][cell->x];
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
            std::string label = std::to_string(row) + "," + std::to_string(col);
            cv::circle(image, pixel, 3, {0, 255, 255}, -1);
            cv::putText(image, label, pixel + cv::Point2d(4, -4), cv::FONT_HERSHEY_SIMPLEX, 0.32, {0, 0, 255}, 1, cv::LINE_AA);
        }
    return image;
}

} // namespace scara
