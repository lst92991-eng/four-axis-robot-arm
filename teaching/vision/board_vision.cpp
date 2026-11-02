#include "board_vision.hpp"
#include <opencv2/geometry.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>

namespace {
cv::Point2d mapped(const cv::Matx33d& matrix, cv::Point2d point)
{
    const cv::Vec3d q = matrix * cv::Vec3d(point.x, point.y, 1);
    if (!std::isfinite(q[0] + q[1] + q[2]) || std::abs(q[2]) < 1e-9)
        throw std::runtime_error("标定映射无效");
    return {q[0] / q[2], q[1] / q[2]};
}
bool validMatrix(const cv::Matx33d& matrix)
{
    return cv::checkRange(cv::Mat(matrix)) && std::abs(cv::determinant(matrix)) > 1e-12;
}
}

BoardVision::BoardVision(const VisionConfig& c)
    : config_(c), detector_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50))
{
    if (!std::isfinite(c.boardWidthMm + c.boardHeightMm + c.gridXmm + c.gridYmm +
                       c.pitchMm + c.markerSideMm + c.pixelsPerMm) ||
        c.boardWidthMm <= 0 || c.boardHeightMm <= 0 || c.pitchMm <= 0 ||
        c.markerSideMm <= 0 || c.pixelsPerMm <= 0 || c.pixelsPerMm > 10 ||
        c.boardWidthMm * c.pixelsPerMm > 4096 || c.boardHeightMm * c.pixelsPerMm > 4096 ||
        c.gridXmm < c.pitchMm / 2 || c.gridYmm < c.pitchMm / 2 ||
        c.gridXmm + 8.5 * c.pitchMm > c.boardWidthMm ||
        c.gridYmm + 9.5 * c.pitchMm > c.boardHeightMm ||
        c.stableFrames < 2 || c.stableFrames > 30 ||
        !std::isfinite(c.confidence) || c.confidence < .5f || c.confidence > 1)
        throw std::invalid_argument("棋盘几何、稳定帧数或识别阈值未正确配置");
    const double half = c.markerSideMm / 2;
    for (const auto& p : c.markerCentersMm)
        if (!std::isfinite(p.x + p.y) || p.x < half || p.y < half ||
            p.x + half > c.boardWidthMm || p.y + half > c.boardHeightMm)
            throw std::invalid_argument("四角标记必须完整位于棋盘外包络内");
    if (c.boardCalibrated) {
        if (!validMatrix(c.boardToBase) || !std::isfinite(c.boardPoseTolerancePixels) ||
            c.boardPoseTolerancePixels <= 0)
            throw std::invalid_argument("棋盘到基座标定或参考图像阈值无效");
        for (const auto& p : c.referenceMarkerPixels)
            if (!std::isfinite(p.x + p.y) || p.x <= 0 || p.y <= 0)
                throw std::invalid_argument("已标定棋盘需要保存四个角标的参考像素中心");
    }
    if (c.tcpCalibrated && (!validMatrix(c.tcpImageToBase) || c.tcpMarkerId < 4 ||
        c.tcpMarkerId >= 50 || !std::isfinite(c.tcpPlaneZmm + c.tcpPlaneToleranceMm +
        c.tcpMarkerOffsetMm.x + c.tcpMarkerOffsetMm.y + c.maxObservationAgeSeconds) ||
        c.tcpPlaneToleranceMm <= 0 || c.maxObservationAgeSeconds <= 0 ||
        c.maxObservationAgeSeconds > .5))
        throw std::invalid_argument("末端标记平面、时间或坐标标定无效");
    const double scale = c.pixelsPerMm;
    const int x = int(std::lround((c.gridXmm - c.pitchMm / 2) * scale));
    const int y = int(std::lround((c.gridYmm - c.pitchMm / 2) * scale));
    area_ = {x, y, int(std::lround((c.gridXmm + 8.5 * c.pitchMm) * scale)) - x,
                   int(std::lround((c.gridYmm + 9.5 * c.pitchMm) * scale)) - y};
    cv::aruco::DetectorParameters p;
    p.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    p.adaptiveThreshConstant = 5;
    detector_.setDetectorParameters(p);
}

void BoardVision::reset()
{
    sameFrames_ = 0;
    located_ = false;
    rectified_.release();
}

bool BoardVision::locate(const cv::Mat& bgr, cv::Mat& rgb, std::string& error)
{
    error.clear();
    located_ = false;
    rgb.release();
    rectified_.release();
    markersFound_ = 0;
    auto fail = [&](const char* message) { reset(); error = message; return false; };
    if (bgr.empty() || bgr.type() != CV_8UC3 || bgr.cols < 3 || bgr.rows < 3)
        return fail("没有有效彩色帧");
    annotated_ = bgr.clone();
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector_.detectMarkers(bgr, corners, ids);
    cv::aruco::drawDetectedMarkers(annotated_, corners, ids);
    int index[4] = {-1, -1, -1, -1};
    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] < 0 || ids[i] > 3) continue;
        if (index[ids[i]] >= 0) return fail("四角标记ID重复");
        index[ids[i]] = int(i);
        ++markersFound_;
    }
    if (markersFound_ != 4) return fail("需要四个完整清晰的角标ID 0～3");
    const float half = float(config_.markerSideMm / 2);
    const cv::Point2f offsets[] = {{-half,-half}, {half,-half}, {half,half}, {-half,half}};
    std::vector<cv::Point2f> world, image;
    for (int id = 0; id < 4; ++id) {
        cv::Point2f center{};
        for (int corner = 0; corner < 4; ++corner) {
            world.push_back(config_.markerCentersMm[id] + offsets[corner]);
            image.push_back(corners[index[id]][corner]);
            center += corners[index[id]][corner] * .25f;
        }
        // boardToBase属于固定棋盘标定；棋盘或相机移动后必须重新核对，不能沿用旧落点。
        if (config_.boardCalibrated &&
            cv::norm(center - config_.referenceMarkerPixels[id]) > config_.boardPoseTolerancePixels)
            return fail("棋盘或相机偏离已标定参考位置，请重新标定");
    }
    const std::vector<cv::Point2f> outline = {image[0], image[5], image[10], image[15]};
    if (!cv::isContourConvex(outline) || cv::contourArea(outline, true) <= 0)
        return fail("角标顺序不符合棋盘坐标定义");
    const cv::Mat boardToImage = cv::findHomography(world, image, 0);
    if (boardToImage.empty() || !cv::checkRange(boardToImage) ||
        std::abs(cv::determinant(boardToImage)) < 1e-12) return fail("棋盘定位无效");
    std::vector<cv::Point2f> projected;
    cv::perspectiveTransform(world, projected, boardToImage);
    for (size_t i = 0; i < image.size(); ++i)
        if (!std::isfinite(projected[i].x + projected[i].y) || cv::norm(projected[i] - image[i]) > 5)
            return fail("四角尺寸、朝向或平面与配置不一致");
    const double scale = config_.pixelsPerMm;
    const cv::Mat imageToBoard = cv::Mat(cv::Matx33d(scale,0,0, 0,scale,0, 0,0,1)) * boardToImage.inv();
    const cv::Size size(int(std::lround(config_.boardWidthMm * scale)),
                        int(std::lround(config_.boardHeightMm * scale)));
    cv::warpPerspective(bgr, rectified_, imageToBoard, size);
    cv::Mat valid = cv::Mat::zeros(bgr.size(), CV_8UC1), warped;
    valid(cv::Rect(1,1,bgr.cols-2,bgr.rows-2)).setTo(255);
    cv::warpPerspective(valid, warped, imageToBoard, size, cv::INTER_NEAREST);
    if (cv::countNonZero(warped(area_)) != area_.area()) return fail("棋子区域没有完整入镜");
    cv::cvtColor(rectified_(area_), rgb, cv::COLOR_BGR2RGB);
    located_ = true;
    return true;
}

chess::Observation BoardVision::interpret(const std::vector<Detection>& pieces)
{
    chess::Observation result;
    for (auto& row : result.board) row.fill('.');
    if (!located_) { sameFrames_ = 0; return result; }
    located_ = false; // 同一帧不能重复调用interpret凑够稳定帧数。
    result.complete = true;
    const double scale = config_.pixelsPerMm, pitch = config_.pitchMm * scale;
    const double firstX = config_.gridXmm * scale - area_.x;
    const double firstY = config_.gridYmm * scale - area_.y;
    const char* classes = "kabnrcpKABNRCP";
    for (const auto& piece : pieces) {
        if (piece.classId < 0 || piece.classId >= 14 || !std::isfinite(piece.confidence) ||
            piece.confidence < 0 || piece.confidence > 1 ||
            !std::isfinite(piece.x1 + piece.y1 + piece.x2 + piece.y2) ||
            piece.x1 >= piece.x2 || piece.y1 >= piece.y2) { result.complete = false; continue; }
        const float x = (piece.x1 + piece.x2) / 2, y = (piece.y1 + piece.y2) / 2;
        if (x < 0 || x >= area_.width || y < 0 || y >= area_.height) {
            result.complete = false; continue;
        }
        const int col = int(std::lround((x - firstX) / pitch));
        const int row = int(std::lround((y - firstY) / pitch));
        if (row < 0 || row >= 10 || col < 0 || col >= 9 ||
            std::abs(x - (firstX + col * pitch)) > .3 * pitch ||
            std::abs(y - (firstY + row * pitch)) > .3 * pitch) { result.complete = false; continue; }
        char& square = result.board[row][col];
        if (square != '.') { result.duplicate = true; square = '?'; }
        else square = piece.confidence >= config_.confidence ? classes[piece.classId] : '?';
        cv::rectangle(rectified_, {int(piece.x1) + area_.x, int(piece.y1) + area_.y},
                      {int(piece.x2) + area_.x, int(piece.y2) + area_.y}, {0,180,0}, 2);
        cv::putText(rectified_, std::string(1,square), {int(x) + area_.x, int(y) + area_.y},
                    cv::FONT_HERSHEY_SIMPLEX, .7, {0,80,240}, 2, cv::LINE_AA);
    }
    std::string reason;
    result.complete = result.complete && !result.duplicate && chess::validBoard(result.board, reason);
    if (result.complete && result.board == previous_) sameFrames_ = std::min(sameFrames_ + 1, config_.stableFrames);
    else sameFrames_ = result.complete ? 1 : 0;
    previous_ = result.board;
    result.stable = result.complete && sameFrames_ >= config_.stableFrames;
    return result;
}

cv::Point2d BoardVision::squareXY(int row, int col) const
{
    if (!config_.boardCalibrated || row < 0 || row > 9 || col < 0 || col > 8)
        throw std::runtime_error("棋盘尚未标定或交点坐标越界");
    return mapped(config_.boardToBase, {config_.gridXmm + col * config_.pitchMm,
                                      config_.gridYmm + row * config_.pitchMm});
}

TcpObservation BoardVision::observeTcp(const cv::Mat& bgr, double measuredAt, double actualZmm)
{
    TcpObservation result;
    const double now = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (!config_.tcpCalibrated || bgr.empty() || bgr.type() != CV_8UC3 ||
        !std::isfinite(measuredAt + actualZmm) || measuredAt <= 0 || measuredAt > now ||
        now - measuredAt > config_.maxObservationAgeSeconds ||
        std::abs(actualZmm - config_.tcpPlaneZmm) > config_.tcpPlaneToleranceMm) return result;
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    detector_.detectMarkers(bgr, corners, ids);
    int index = -1;
    for (size_t i = 0; i < ids.size(); ++i) {
        if (ids[i] != config_.tcpMarkerId) continue;
        if (index >= 0) return result;
        index = int(i);
    }
    if (index < 0) return result;
    cv::Point2d center{};
    for (const auto& p : corners[index]) center += cv::Point2d(p) * .25;
    const cv::Point2d origin = mapped(config_.tcpImageToBase, corners[index][0]);
    cv::Point2d xAxis = mapped(config_.tcpImageToBase, corners[index][1]) - origin;
    cv::Point2d yAxis = mapped(config_.tcpImageToBase, corners[index][3]) - origin;
    if (cv::norm(xAxis) < 1e-6 || cv::norm(yAxis) < 1e-6) return result;
    xAxis *= 1.0 / cv::norm(xAxis);
    yAxis *= 1.0 / cv::norm(yAxis);
    result.xy = mapped(config_.tcpImageToBase, center) +
        xAxis * config_.tcpMarkerOffsetMm.x + yAxis * config_.tcpMarkerOffsetMm.y;
    const double finished = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
    if (finished - measuredAt > config_.maxObservationAgeSeconds) return {};
    result.stamp = measuredAt;
    result.valid = true;
    return result;
}
