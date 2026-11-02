// 合成图和合成Detection只验证定位/映射逻辑，不代表NPU在实拍中的准确率。
#ifdef NDEBUG
#undef NDEBUG
#endif
#include "vision/board_vision.hpp"
#include "app/config.hpp"
#include <opencv2/imgproc.hpp>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <string>

namespace {

void paintMarker(cv::Mat& image, int id, cv::Point center, bool turned = false)
{
    const int side = int(project::MarkerSide);
    cv::Mat marker, colored;
    cv::aruco::generateImageMarker(
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), id, side, marker);
    if (turned) cv::rotate(marker, marker, cv::ROTATE_90_CLOCKWISE);
    cv::cvtColor(marker, colored, cv::COLOR_GRAY2BGR);
    const int white = side / 6;
    const cv::Rect black(center.x - side / 2, center.y - side / 2, side, side);
    image(cv::Rect(black.x - white, black.y - white, side + white * 2, side + white * 2)).setTo(cv::Scalar(255,255,255));
    colored.copyTo(image(black));
}

// 相机图中用1px/mm画一张完整棋盘；生产定位应自行恢复为2px/mm。
cv::Mat cameraBoard(int missing = -1, int turned = -1)
{
    cv::Mat image(project::ImageHeight, project::ImageWidth, CV_8UC3, cv::Scalar(40,40,40));
    const cv::Point origin(200, 100);
    image(cv::Rect(origin.x, origin.y, int(project::BoardWidth), int(project::BoardHeight)))
        .setTo(cv::Scalar(106,167,199));
    const int grid = int(project::GridStart), pitch = int(project::Pitch);
    for (int row = 0; row < 10; ++row)
        cv::line(image, origin + cv::Point(grid, grid + row * pitch),
                 origin + cv::Point(grid + 8 * pitch, grid + row * pitch), {20,20,20});
    for (int col = 0; col < 9; ++col) {
        const int x = grid + col * pitch;
        cv::line(image, origin + cv::Point(x, grid), origin + cv::Point(x, grid + 4 * pitch), {20,20,20});
        cv::line(image, origin + cv::Point(x, grid + 5 * pitch), origin + cv::Point(x, grid + 9 * pitch), {20,20,20});
        if (col == 0 || col == 8)
            cv::line(image, origin + cv::Point(x, grid + 4 * pitch), origin + cv::Point(x, grid + 5 * pitch), {20,20,20});
    }
    for (int base : {0, 7}) {
        cv::line(image, origin + cv::Point(grid + 3 * pitch, grid + base * pitch),
                 origin + cv::Point(grid + 5 * pitch, grid + (base + 2) * pitch), {20,20,20});
        cv::line(image, origin + cv::Point(grid + 5 * pitch, grid + base * pitch),
                 origin + cv::Point(grid + 3 * pitch, grid + (base + 2) * pitch), {20,20,20});
    }
    const int inset = int(project::MarkerInset);
    const cv::Point centers[] = {{inset,inset}, {int(project::BoardWidth)-inset,inset},
        {int(project::BoardWidth)-inset,int(project::BoardHeight)-inset}, {inset,int(project::BoardHeight)-inset}};
    for (int id = 0; id < 4; ++id) {
        if (id != missing) paintMarker(image, id, origin + centers[id], id == turned);
    }
    return image;
}

std::vector<Detection> initialDetections()
{
    std::vector<Detection> pieces;
    const auto board = chess::initialBoard();
    const std::string classes = project::Classes;
    const float half = float(project::PieceDiameter);
    std::array<bool, 14> seen{};
    for (int row = 0; row < 10; ++row) {
        for (int col = 0; col < 9; ++col) {
            if (board[row][col] == '.') continue;
            const int id = int(classes.find(board[row][col]));
            assert(id >= 0 && id < 14);
            seen[id] = true;
            const float x = float(2 * (project::GridStart + col * project::Pitch) - 120);
            const float y = float(2 * (project::GridStart + row * project::Pitch) - 120);
            pieces.push_back({id, 0.95f, x - half, y - half, x + half, y + half});
        }
    }
    for (bool value : seen) assert(value);
    return pieces;
}

void checkThreeFrames(BoardVision& vision, const cv::Mat& camera,
                      const std::vector<Detection>& pieces)
{
    cv::Mat rgb;
    std::string error;
    for (int frame = 1; frame <= project::StableFrames; ++frame) {
        assert(vision.locate(camera, rgb, error));
        assert(rgb.size() == cv::Size(720,800) && rgb.type() == CV_8UC3);
        // 检查输出确实从BGR换成RGB；这个位置是格子中心的黄褐底色。
        const auto color = rgb.at<cv::Vec3b>(80,80);
        assert(color[0] == 199 && color[1] == 167 && color[2] == 106);
        const auto observed = vision.interpret(pieces);
        assert(observed.complete && !observed.duplicate && observed.board == chess::initialBoard());
        assert(observed.stable == (frame >= project::StableFrames));
    }
}

} // namespace

void testVision()
{
    assert(CV_VERSION_MAJOR == 5 && cv::getVersionString() == "5.0.0");
    cv::setNumThreads(1);
    const cv::Mat camera = cameraBoard();
    const auto pieces = initialDetections();
    BoardVision vision;
    checkThreeFrames(vision, camera, pieces);
    cv::Mat rgb;
    std::string error;

    // 缺角后不能继续用旧H或旧的三帧稳定计数。
    assert(!vision.locate(cameraBoard(2), rgb, error));
    assert(rgb.empty() && vision.boardImage().empty());
    assert(!vision.interpret(pieces).complete);
    checkThreeFrames(vision, camera, pieces);

    cv::Mat duplicate = camera.clone();
    paintMarker(duplicate, 0, {1050,150});  // 原来的四个ID都还在，另外再加一个ID0。
    assert(!vision.locate(duplicate, rgb, error));
    assert(!vision.locate(cameraBoard(-1,0), rgb, error));
    assert(rgb.empty() && vision.boardImage().empty());

    // 整个画面旋转仍是同一块板；只旋转一个标签才是不一致布局。
    cv::Mat rotated;
    cv::rotate(camera, rotated, cv::ROTATE_90_CLOCKWISE);
    checkThreeFrames(vision, rotated, pieces);
    vision.reset();
    checkThreeFrames(vision, camera, pieces);

    // 突然移动棋盘时旧EKF映射不得继续被当成当前定位；重新定位后重新累计稳定帧。
    cv::Mat shifted;
    cv::warpAffine(camera, shifted, cv::Mat(cv::Matx23d(1,0,120, 0,1,80)), camera.size());
    assert(!vision.locate(shifted, rgb, error));
    assert(!vision.interpret(pieces).complete);
    checkThreeFrames(vision, shifted, pieces);

    // 下面只测试Detection到格点的解释，故意绕过NPU推理，不声称模型识别正确。
    auto low = pieces;
    low.front().confidence = 0.49f;
    assert(vision.locate(shifted, rgb, error));
    auto result = vision.interpret(low);
    assert(!result.complete && !result.stable && result.board[0][0] == '?');

    auto repeated = pieces;
    repeated.push_back(pieces.front());
    assert(vision.locate(shifted, rgb, error));
    result = vision.interpret(repeated);
    assert(result.duplicate && !result.complete && !result.stable && result.board[0][0] == '?');

    auto offGrid = pieces;
    offGrid.front().x1 += 35;
    offGrid.front().x2 += 35;
    assert(vision.locate(shifted, rgb, error));
    assert(!vision.interpret(offGrid).complete);
    auto invalid = pieces;
    invalid.front().classId = 14;
    assert(vision.locate(shifted, rgb, error));
    assert(!vision.interpret(invalid).complete);
    invalid = pieces;
    invalid.front().confidence = std::numeric_limits<float>::quiet_NaN();
    assert(vision.locate(shifted, rgb, error));
    assert(!vision.interpret(invalid).complete);
    assert(!vision.locate(cv::Mat(), rgb, error));
    assert(!vision.interpret(pieces).complete);
}
