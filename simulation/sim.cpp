// 教学仿真：参数 → 成像/识别 → CAN虚拟电机/EKF → 取放 → 棋局确认。
// 单位：世界坐标mm，关节角rad，电机协议角0.1度；只在协议边界换算。
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/geometry.hpp>
#include "cad_view.hpp"
#include <opencv2/highgui.hpp>
#include "detector.hpp"
#include "debug.hpp"
#include "x42s.hpp"
#include "motion_filter.hpp"
#include "chess/board.hpp"
#include "chess/pikafish.hpp"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <random>
#include <thread>
#include <future>
#include <mutex>

constexpr double pi = 3.141592653589793;
using Q = std::array<double, 4>; // 固定顺序：Z、肩、肘、腕，与四个CAN节点一致。
cv::FileStorage cfg;
cv::Point2d center, rightAxis, downAxis;
double pitch, diameter, pieceHeight, lift, focal, noise, timeNow = 0;
cv::Vec3d camera;
cv::Size frameSize;
cv::Mat boardImage, workspace;
std::array<cv::Mat, 14> sprites;
std::mt19937 randomSource;
std::normal_distribution<double> normal(0, 1);
const std::string names = "kabnrcpKABNRCP";
bool interactive = false, userQuit = false, guiPaused = false, guiStep = false, humanTurn = false,
     guiRobotRed = false, guiReady = false;
cv::Mat viewRaw, viewRectified, viewYolo, lastHomography;
Detector *guiDetector = nullptr;
std::string pendingHuman, guiMessage, previewFailure;
std::vector<std::string> guiLegal;
std::future<void> previewJob;
std::mutex viewMutex;
cv::Mat displayedYolo;
const char *yoloWindow = "YOLO - drag your pieces here";
void pumpGui();

// 1. 坐标与旧机械臂几何。板面坐标从左上角计量，不与关节坐标混用。
cv::Point2d fk(const Q &q) {
    return {195 * std::cos(q[1]) + 275 * std::cos(q[1] + q[2]),
            195 * std::sin(q[1]) + 275 * std::sin(q[1] + q[2])};
}
Q ik(cv::Point2d p, double z) {
    double c = (p.dot(p) - 195 * 195 - 275 * 275) / (2 * 195 * 275);
    if (std::abs(c) > 1)
        throw std::runtime_error("目标超出旧机几何可达范围");
    double b = std::acos(c),
           a = std::atan2(p.y, p.x) - std::atan2(275 * std::sin(b), 195 + 275 * c);
    return {z, a, b, -a - b}; // 电磁铁朝向保持不变。
}
bool freeJoint(const Q &q) {
    double a = (q[1] * 180 / pi + 90) / 4, b = (q[2] * 180 / pi + 180) / 4;
    int i = int(std::floor(a)), j = int(std::floor(b));
    if (i < 0 || i >= 90 || j < 0 || j >= 90)
        return false;
    return workspace.at<uchar>(i, j) && workspace.at<uchar>(i + 1, j) &&
           workspace.at<uchar>(i, j + 1) && workspace.at<uchar>(i + 1, j + 1);
}
bool freeLine(const Q &a, const Q &b) {
    int n = 1 + int(std::max(std::abs(a[1] - b[1]), std::abs(a[2] - b[2])) * 180 / pi);
    for (int i = 0; i <= n; ++i) {
        Q p;
        for (int k = 0; k < 4; ++k)
            p[k] = a[k] + (b[k] - a[k]) * i / n;
        if (!freeJoint(p))
            return false;
    }
    return true; // 每1度检查采样包络；精度受原4度CAD网格限制。
}
cv::Point2d boardWorld(double x, double y) {
    double w = (double)cfg["board_size_mm"][0], h = (double)cfg["board_size_mm"][1];
    // rightAxis沿棋盘行方向；downAxis沿棋盘负列方向，相机滚转90度以容纳整板。
    return center - downAxis * (x - w / 2) + rightAxis * (y - h / 2);
}
cv::Point2d square(int row, int col) {
    double w = (double)cfg["board_size_mm"][0], h = (double)cfg["board_size_mm"][1];
    return boardWorld(w / 2 + (col - 4) * pitch, h / 2 + (row - 4.5) * pitch);
}
cv::Point2f project(cv::Point2d p, double z) {
    cv::Point2d d = p - cv::Point2d(camera[0], camera[1]);
    double depth = camera[2] - z;
    return {float(frameSize.width / 2 + focal * d.dot(rightAxis) / depth),
            float(frameSize.height / 2 + focal * d.dot(downAxis) / depth)};
}

std::vector<Piece> pieces;
Q truth, encoder, estimate, command, motorScale, gap, zeroError;
int held = -1, captured = 0;
cv::Point2d attachment;

// 2. 生成真实像素输入。训练照片只作渲染贴图，检测器看不到pieces里的标签。
cv::Mat formatCamera(cv::Mat image, bool preview = false) {
    if (noise) {
        // 预览单独取图像噪声，不消耗运动传感器的随机序列。
        cv::GaussianBlur(image, image, {3, 3}, .35);
        double sigma = (double)cfg["camera_pixel_noise"];
        if (preview) {
            static cv::RNG rng(43);
            cv::Mat jitter(image.size(), CV_16SC3), wide;
            rng.fill(jitter, cv::RNG::NORMAL, 0, sigma);
            image.convertTo(wide, CV_16SC3);
            cv::add(wide, jitter, wide);
            wide.convertTo(image, CV_8UC3);
        } else
            for (int y = 0; y < image.rows; ++y)
                for (int x = 0; x < image.cols; ++x)
                    for (int c = 0; c < 3; ++c)
                        image.at<cv::Vec3b>(y, x)[c] = cv::saturate_cast<uchar>(
                            image.at<cv::Vec3b>(y, x)[c] + sigma * normal(randomSource));
    }
    // 模拟MIPI/ISP交付的NV12，然后调用与板端相同的转换接口。
    cv::Mat i420, nv12(frameSize.height * 3 / 2, frameSize.width, CV_8UC1), bgr;
    cv::cvtColor(image, i420, cv::COLOR_BGR2YUV_I420);
    int pixels = frameSize.area();
    std::copy_n(i420.data, pixels, nv12.data);
    for (int i = 0; i < pixels / 4; ++i) {
        nv12.data[pixels + 2 * i] = i420.data[pixels + i];
        nv12.data[pixels + 2 * i + 1] = i420.data[pixels + pixels / 4 + i];
    }
    cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
    return bgr;
}
chess::Observation analyseImage(Detector *detector, const cv::Mat &image, bool preview = false) {
    viewRaw = image;
    cv::imwrite("output/camera.jpg", image);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::ArucoDetector aruco(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50));
    aruco.detectMarkers(image, corners, ids);
    double w = boardImage.cols / 2., h = boardImage.rows / 2.;
    std::vector<cv::Point2f> source, dest;
    const std::array<cv::Point2f, 4> centers = {cv::Point2f(32, 32), cv::Point2f(w - 32, 32),
                                                cv::Point2f(w - 32, h - 32),
                                                cv::Point2f(32, h - 32)};
    bool fullMarkers = true;
    for (int id = 0; id < 4; ++id) {
        auto found = std::find(ids.begin(), ids.end(), id);
        if (found == ids.end()) {
            fullMarkers = false;
            break;
        }
        const auto &c = corners[found - ids.begin()];
        auto p = centers[id];
        for (int k = 0; k < 4; ++k) {
            source.push_back(c[k]);
            dest.push_back((p + cv::Point2f(k == 0 || k == 3 ? -24 : 24, k < 2 ? -24 : 24)) * 2);
        }
    }
    if (fullMarkers)
        lastHomography = cv::findHomography(source, dest);
    else if (!preview || lastHomography.empty())
        throw std::runtime_error("四角ArUco不完整");
    cv::Mat rectified, rgb;
    cv::warpPerspective(image, rectified, lastHomography, boardImage.size());
    viewRectified = rectified.clone();
    viewYolo = rectified;
    cv::imwrite("output/rectified.jpg", viewRectified);
    cv::Rect roi(int(w - 9 * pitch), int(h - 10 * pitch), int(18 * pitch), int(20 * pitch));
    cv::cvtColor(rectified(roi), rgb, cv::COLOR_BGR2RGB);
    std::vector<Detection> detections;
    std::string error;
    if (!detector_detect(detector, rgb, detections, error))
        throw std::runtime_error(error);
    chess::Observation observation;
    for (auto &row : observation.board)
        row.fill('.');
    for (const auto &d : detections) {
        float x = (d.x1 + d.x2) / 2, y = (d.y1 + d.y2) / 2;
        int col = std::lround(x / (2 * pitch) - .5), row = std::lround(y / (2 * pitch) - .5);
        if (row < 0 || row > 9 || col < 0 || col > 8)
            continue;
        if (std::abs(x - (col + .5) * 2 * pitch) > pitch * .65 ||
            std::abs(y - (row + .5) * 2 * pitch) > pitch * .65)
            throw std::runtime_error("棋子未靠近交点");
        if (observation.board[row][col] != '.')
            throw std::runtime_error("交点出现重复检测");
        observation.board[row][col] = names[d.classId];
        cv::rectangle(rectified, cv::Point(d.x1 + roi.x, d.y1 + roi.y),
                      cv::Point(d.x2 + roi.x, d.y2 + roi.y), {0, 180, 0}, 2);
        cv::putText(rectified,
                    std::string(1, names[d.classId]) + " " +
                        std::to_string(int(d.confidence * 100)),
                    cv::Point(d.x1 + roi.x, d.y1 + roi.y - 2), cv::FONT_HERSHEY_SIMPLEX, .45,
                    {0, 0, 220}, 1);
    }
    observation.complete = observation.stable = true; // 同一静止帧；未宣称完成实机连续帧稳定判定。
    cv::putText(rectified, detector_backend(detector), {140, 22},
                cv::FONT_HERSHEY_SIMPLEX, .48, {0, 110, 0}, 1);
    cv::imwrite("output/vision.jpg", rectified);
    if (debug.event && !preview) {
        std::string text;
        for (const auto &row : observation.board) {
            text.append(row.begin(), row.end());
            text += '\n';
        }
        debug.event("vision", text);
    }
    return observation;
}
void finishPreview() {
    if (previewJob.valid())
        previewJob.get();
}
chess::Observation observe(Detector *detector) {
    // 严格复拍前收回模型所有权；后台预览和棋局确认绝不并发调用同一个DNN。
    finishPreview();
    std::lock_guard<std::mutex> lock(viewMutex);
    auto result = analyseImage(detector, formatCamera(cadCamera(pieces, truth)), false);
    displayedYolo = viewYolo;
    return result;
}

// 3. X固件CAN最小子集。帧只进入本进程虚拟节点，没有SocketCAN或USB写操作。
struct Axis {
    double start = 0, target = 0, motor = 0, velocity = 0, accel = 0, ramp = 0, duration = 0,
           elapsed = 0;
    bool moving = false;
    std::vector<uint8_t> packet;
};
std::array<Axis, 4> axes;
std::array<bool, 4> arrived = {true, true, true, true};
std::array<X42Sample, 4> driverSamples;
void receive(uint32_t id, const std::vector<uint8_t> &p) {
    if (debug.can)
        debug.can(false, id, p.data(), p.size());
    x42Decode({id,p}, driverSamples, timeNow);
    int node = int(id >> 8) - 1;
    if (p[0] == 0x36) encoder[node] = driverSamples[node].angle / motorScale[node];
    if (p[0] == 0xFD && p[1] == 0x9F) arrived[node] = true;
}
void send(uint32_t id, const std::vector<uint8_t> &p) {
    if (debug.can)
        debug.can(true, id, p.data(), p.size());
    if (p.empty() || p.size() > 8)
        throw std::runtime_error("CAN DLC错误");
    int node = int(id >> 8) - 1;
    if (id == 0 && p == std::vector<uint8_t>({0xFF, 0x66, 0x6B})) {
        for (auto &a : axes) {
            a.start = a.motor;
            a.elapsed = 0;
            a.moving = true;
        }
        receive(0x100, {0xFF, 0x02, 0x6B});
        return;
    }
    if (node < 0 || node >= 4)
        throw std::runtime_error("CAN节点超界");
    auto &a = axes[node];
    if (p == std::vector<uint8_t>({0x36, 0x6B})) {
        double measured = a.motor + noise * .02 * normal(randomSource);
        uint32_t v = std::lround(std::abs(measured) * 10);
        receive((node + 1) << 8, {0x36, uint8_t(measured < 0), uint8_t(v >> 24), uint8_t(v >> 16),
                                  uint8_t(v >> 8), uint8_t(v), 0x6B});
        return;
    }
    // 本任务只有0xFD长指令：每包重复功能码，低8位依次为包号0、1。
    if (p[0] != 0xFD || p.size() != 8 || (id & 255) > 1)
        throw std::runtime_error("未支持的仿真命令");
    if ((id & 255) == 0)
        a.packet = p;
    else {
        if (a.packet.size() != 8)
            throw std::runtime_error("CAN分包顺序错误");
        a.packet.insert(a.packet.end(), p.begin() + 1, p.end());
        const auto &v = a.packet;
        if (v[14] != 0x6B || v[12] != 1 || v[13] != 1)
            throw std::runtime_error("只支持绝对位置同步模式");
        a.accel = ((v[2] << 8) | v[3]) * 6.;
        double decel = ((v[4] << 8) | v[5]) * 6.;
        a.velocity = ((v[6] << 8) | v[7]) * .6;
        uint32_t n = 0;
        for (int i = 8; i < 12; ++i)
            n = (n << 8) | v[i];
        a.target = (v[1] ? -1. : 1.) * n * .1;
        if (a.accel <= 0 || decel != a.accel || a.velocity <= 0)
            throw std::runtime_error("仿真梯形速度参数错误");
        double d = std::abs(a.target - a.motor);
        a.ramp = std::min(a.velocity / a.accel, std::sqrt(d / a.accel));
        a.velocity = a.ramp * a.accel;
        a.duration = a.velocity ? 2 * a.ramp + (d - a.velocity * a.ramp) / a.velocity : 0;
        arrived[node] = false;
        receive((node + 1) << 8, {0xFD, 0x02, 0x6B});
    }
}

// 4. EKF估计肩/肘及速度。非线性测量来自FK末端XY，编码器在减速器输入侧。
MotionFilter motionFilter(195,275);
void updateEkf(double dt) {
    auto measured = fk(truth); // 合成末端位置传感器，尚非真实相机跟踪。
    double sigma = (double)cfg["tcp_sensor_noise_mm"];
    measured +=
        cv::Point2d(noise * sigma * normal(randomSource), noise * sigma * normal(randomSource));
    auto angles = motionFilter.step({encoder[1],encoder[2]},dt,&measured,0,sigma);
    estimate = encoder;
    estimate[1] = angles[0]; estimate[2] = angles[1];
}
void tick() {
    if (interactive) {
        pumpGui();
        while (guiPaused && !guiStep && !userQuit)
            pumpGui();
        guiStep = false;
        if (userQuit)
            throw std::runtime_error("用户结束仿真");
    }
    constexpr double dt = .02;
    timeNow += dt;
    for (int k = 0; k < 4; ++k) {
        auto &a = axes[k];
        double desired = a.target;
        if (a.moving) {
            a.elapsed += dt;
            double t = std::min(a.elapsed, a.duration), d = std::abs(a.target - a.start), s;
            if (t < a.ramp)
                s = .5 * a.accel * t * t;
            else if (t > a.duration - a.ramp)
                s = d - .5 * a.accel * std::pow(a.duration - t, 2);
            else
                s = .5 * a.accel * a.ramp * a.ramp + a.velocity * (t - a.ramp);
            desired = a.start + std::copysign(s, a.target - a.start);
        }
        a.motor += (desired - a.motor) *
                   (1 - std::exp(-dt / std::max(.001, noise * (double)cfg["servo_tau_s"])));
        double input = a.motor / motorScale[k] + noise * zeroError[k], half = noise * gap[k] / 2;
        truth[k] = std::max(input - half,
                            std::min(truth[k], input + half)); // 背隙play算子：反向先走空程。
        if (a.moving && a.elapsed >= a.duration && std::abs(a.motor - a.target) < .05) {
            a.moving = false;
            receive((k + 1) << 8, {0xFD, 0x9F, 0x6B});
        }
        send((k + 1) << 8, {0x36, 0x6B});
    }
    if (!freeJoint(truth))
        throw std::runtime_error("虚拟轨迹进入CAD碰撞禁区");
    updateEkf(dt);
    if (held >= 0)
        pieces[held].p = fk(truth) + attachment;
    if (held >= 0 && truth[0] - pieceHeight < pieceHeight)
        for (size_t i = 0; i < pieces.size(); ++i)
            if (int(i) != held && cv::norm(pieces[i].p - pieces[held].p) < diameter)
                throw std::runtime_error("持子在低位碰到其他棋子");
    if (debug.motion)
        debug.motion(timeNow, command, truth, encoder, estimate);
}
void moveJoints(const Q &goal) {
    if (!freeLine(estimate, goal))
        throw std::runtime_error("路径进入CAD碰撞采样禁区");
    if (!cadValidZ(goal[0]))
        throw std::runtime_error("目标超出旧CAD的Z行程");
    command = goal;
    Q from{}, motorGoal{};
    for (int k=0; k<4; ++k) { from[k]=axes[k].motor; motorGoal[k]=goal[k]*motorScale[k]; }
    auto moves=x42Plan(from,motorGoal,(double)cfg["motor_max_rpm"],
                                     (double)cfg["motor_accel_rpm_s"]);
    double duration=.1;
    for (int k = 0; k < 4; ++k) {
        driverSamples[k].acknowledged=false;
        duration=std::max(duration,x42Duration(std::abs(motorGoal[k]-from[k]),moves[k]));
        for (const auto &frame:x42Split(k+1,x42Position(moves[k]))) send(frame.id,frame.data);
    }
    if (!std::all_of(driverSamples.begin(),driverSamples.end(),[](auto &s){return s.acknowledged;}))
        throw std::runtime_error("四轴位置命令尚未全部确认");
    send(0, {0xFF, 0x66, 0x6B});
    double deadline = timeNow + duration + 1;
    while (!std::all_of(arrived.begin(), arrived.end(), [](bool x) { return x; })) {
        if (timeNow > deadline)
            throw std::runtime_error("虚拟CAN电机到位超时");
        tick();
    }
    Q average{};
    for (int i = 0; i < 10; ++i) {
        tick();
        for (int k = 0; k < 4; ++k)
            average[k] += estimate[k] / 10;
    }
    estimate = average; // 停稳后平均200ms反馈，避免用单个有噪测量反复修正。
}
void go(cv::Point2d p, double z) {
    Q goal = ik(p, z);
    moveJoints(goal);
    // 基于EKF反馈小幅修正；没有读取truth来反算理想落点。
    for (int n = 0; n < 3 && cv::norm(fk(estimate) - p) > .35; ++n) {
        Q corrected = command;
        corrected[1] += goal[1] - estimate[1];
        corrected[2] += goal[2] - estimate[2];
        corrected[3] = -corrected[1] - corrected[2];
        moveJoints(corrected);
    }
    if (cv::norm(fk(estimate) - p) > 1)
        throw std::runtime_error("反馈校正后仍未靠近目标");
}
void magnet(bool on) {
    if (on) {
        for (size_t i = 0; i < pieces.size(); ++i)
            if (cv::norm(pieces[i].p - fk(truth)) < 4 && std::abs(truth[0] - pieceHeight) < .5) {
                if (held >= 0)
                    throw std::runtime_error("电磁铁同时接触两子");
                held = i;
            }
        if (held < 0)
            throw std::runtime_error("电磁铁未吸到棋子");
        attachment = pieces[held].p - fk(truth);
        pieces[held].held = true;
    } else {
        if (held < 0)
            throw std::runtime_error("电磁铁没有持子");
        if (std::abs(truth[0] - pieceHeight) > .5)
            throw std::runtime_error("放子高度不合适");
        pieces[held].held = false;
        held = -1;
    }
    for (int i = 0; i < 5; ++i)
        tick(); // 吸放等待100ms为仿真假设。
    if (debug.event)
        debug.event("magnet", on ? "on" : "off");
}
void transfer(cv::Point2d from, cv::Point2d to) {
    go(from, pieceHeight + lift);
    go(from, pieceHeight);
    magnet(true);
    go(from, pieceHeight + lift);
    go(to, pieceHeight + lift);
    go(to, pieceHeight);
    int placed = held;
    magnet(false);
    if (debug.placement)
        debug.placement({to.x, to.y}, {pieces[placed].p.x, pieces[placed].p.y});
    go(to, pieceHeight + lift);
}
void humanMove(const chess::Move &m) {
    auto from = square(m.fromRow, m.fromCol), to = square(m.toRow, m.toCol);
    // 模拟对手手动摆子，仅改变环境；随后仍必须通过相机识别确认。
    pieces.erase(std::remove_if(pieces.begin(), pieces.end(),
                                [&](const Piece &p) { return cv::norm(p.p - to) < diameter / 2; }),
                 pieces.end());
    auto p = std::find_if(pieces.begin(), pieces.end(),
                          [&](const Piece &p) { return cv::norm(p.p - from) < diameter / 2; });
    if (p == pieces.end())
        throw std::runtime_error("模拟对手找不到起点棋子");
    p->p = to;
}
void drawScene() { cv::imwrite("output/scene.jpg", cadScene(pieces, truth)); }

// 同一三维场景接收拖棋；鼠标射线只决定起终交点，不代替图像识别。
int dragging = -1;
chess::Move dragMove;
cv::Point2d dragWorld;
bool worldSquare(cv::Point2d world, int &row, int &col) {
    auto d = world - center;
    col = std::lround(-d.dot(downAxis) / pitch + 4);
    row = std::lround(d.dot(rightAxis) / pitch + 4.5);
    return row >= 0 && row < 10 && col >= 0 && col < 9 &&
           cv::norm(world - square(row, col)) < pitch * .48;
}
void mouseInput(const CadInput &input) {
    if (!humanTurn || guiPaused)
        return;
    if (input.mouse == 1 && input.overBoard) {
        int r, c;
        if (!worldSquare(input.world, r, c))
            return;
        for (size_t i = 0; i < pieces.size(); ++i)
            if (cv::norm(pieces[i].p - square(r, c)) < diameter / 2 &&
                bool(std::isupper(pieces[i].kind)) != guiRobotRed) {
                dragging = i;
                dragMove = {r, c, r, c};
                dragWorld = pieces[i].p;
            }
        if (dragging < 0)
            guiMessage = "Select your own piece";
    } else if (input.mouse == 2 && dragging >= 0 && input.overBoard)
        dragWorld = input.world;
    else if (input.mouse == 3 && dragging >= 0) {
        int r, c;
        if (input.overBoard && worldSquare(input.world, r, c)) {
            dragMove.toRow = r;
            dragMove.toCol = c;
            auto move = chess::uci(dragMove);
            if (std::find(guiLegal.begin(), guiLegal.end(), move) != guiLegal.end()) {
                pendingHuman = move;
                guiMessage = "Checking camera: " + move;
            } else {
                guiMessage = "Illegal " + move + " (Pikafish legal list)";
                if (debug.event)
                    debug.event("rejected", move);
            }
        }
        dragging = -1;
    }
}
void yoloMouse(int event, int x, int y, int, void *) {
    CadInput input;
    input.mouse = event == cv::EVENT_LBUTTONDOWN ? 1
                  : event == cv::EVENT_LBUTTONUP ? 3
                  : event == cv::EVENT_MOUSEMOVE ? 2
                                                 : 0;
    input.overBoard = x >= 0 && y >= 0 && x < boardImage.cols && y < boardImage.rows;
    input.world = boardWorld(x / 2., y / 2.);
    mouseInput(input);
}
void pumpGui() {
    if (!interactive)
        return;
    static auto last = std::chrono::steady_clock::time_point{};
    if (previewJob.valid() &&
        previewJob.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
        finishPreview();
    if (guiDetector && !previewJob.valid() &&
        std::chrono::steady_clock::now() - last > std::chrono::milliseconds(150)) {
        // OpenGL取图只在主线程；格式转换/ArUco/YOLO在唯一的后台作业中完成。
        auto image = cadCamera(pieces, truth);
        auto *model = guiDetector;
        previewJob = std::async(std::launch::async, [image = std::move(image), model]() mutable {
            auto bgr = formatCamera(std::move(image), true);
            std::lock_guard<std::mutex> lock(viewMutex);
            try {
                analyseImage(model, bgr, true);
            } catch (const std::exception &e) {
                if (std::string(e.what()).find("GPU") != std::string::npos)
                    previewFailure = e.what();
            } // 遮挡不提交棋局。
        });
        last = std::chrono::steady_clock::now();
    }
    if (viewMutex.try_lock()) {
        if (!viewYolo.empty())
            displayedYolo = viewYolo;
        std::string failure = previewFailure;
        viewMutex.unlock();
        if (!failure.empty())
            throw std::runtime_error(failure);
    }
    if (!displayedYolo.empty()) {
        cv::Mat display = displayedYolo.clone();
        if (dragging >= 0) {
            auto d = dragWorld - center;
            cv::Point pixel(boardImage.cols / 2 - 2 * d.dot(downAxis),
                            boardImage.rows / 2 + 2 * d.dot(rightAxis));
            cv::circle(display, pixel, diameter, {0, 160, 255}, 2);
        }
        cv::imshow(yoloWindow, display);
    }
    std::string caption = std::string("Robot: ") + (guiRobotRed ? "RED" : "BLACK") +
                          " | Human: " + (guiRobotRed ? "BLACK" : "RED") + " | " +
                          (guiPaused   ? "PAUSED"
                           : humanTurn ? "Drag in YOLO window"
                                       : "Robot / Pikafish");
    auto input = cadShow(pieces, truth, caption, guiMessage, dragging, dragWorld);
    mouseInput(input);
    int key = cv::waitKey(1) & 255;
    if (input.pause || key == ' ')
        guiPaused = !guiPaused;
    if (input.step || key == 'n' || key == 'N')
        guiStep = true;
    if (input.quit || key == 27 || cv::getWindowProperty(yoloWindow, cv::WND_PROP_AUTOSIZE) < 0)
        userQuit = true;
}

int main(int argc, char **argv) {
    Detector *detector = nullptr;
    try {
        std::filesystem::create_directories("output");
        std::ofstream("output/result.json") << "{\"result\":\"running\"}";
        // 5. 一份配置驱动所有模块。CLI只允许覆盖机器人执红/执黑。
        cfg.open("config.json", cv::FileStorage::READ);
        if (!cfg.isOpened())
            throw std::runtime_error("找不到config.json");
        std::string side = argc > 1 ? argv[1] : (std::string)cfg["robot_side"];
        if (side != "red" && side != "black")
            throw std::runtime_error("机器人颜色应为red或black");
        bool robotRed = side == "red";
        interactive = argc > 2 && std::string(argv[2]) == "--interactive";
        guiRobotRed = robotRed;
        noise = (double)cfg["noise"];
        randomSource.seed((int)cfg["seed"]);
        pitch = (double)cfg["pitch_mm"];
        diameter = (double)cfg["piece_diameter_mm"];
        pieceHeight = (double)cfg["piece_height_mm"];
        lift = (double)cfg["lift_mm"];
        if (diameter >= pitch || lift <= pieceHeight)
            throw std::runtime_error("棋子间距或10mm抬升净空不满足仿真假设");
        center = {(double)cfg["board_center_mm"][0], (double)cfg["board_center_mm"][1]};
        double a = (double)cfg["board_angle_deg"] * pi / 180;
        rightAxis = {std::sin(a), -std::cos(a)};
        downAxis = {-std::cos(a), -std::sin(a)};
        for (int k = 0; k < 3; ++k)
            camera[k] = (double)cfg["camera_position_mm"][k];
        frameSize = {(int)cfg["image_size_px"][0], (int)cfg["image_size_px"][1]};
        if ((frameSize.width | frameSize.height) & 1 || camera[2] <= 150)
            throw std::runtime_error("NV12尺寸应为偶数，相机应位于臂杆上方");
        focal = frameSize.height / (2 * std::tan((double)cfg["vertical_fov_deg"] * pi / 360));
        int w = 2 * (int)cfg["board_size_mm"][0], h = 2 * (int)cfg["board_size_mm"][1];
        boardImage = cv::Mat(h, w, CV_8UC3, cv::Scalar(185, 215, 230));
        for (auto p : {cv::Point2d(0, 0), cv::Point2d(w / 2, 0), cv::Point2d(w / 2, h / 2),
                       cv::Point2d(0, h / 2)}) {
            auto uv = project(boardWorld(p.x, p.y), 0);
            if (uv.x < 20 || uv.x > frameSize.width - 20 || uv.y < 20 ||
                uv.y > frameSize.height - 20)
                throw std::runtime_error("相机无法完整覆盖棋盘和白边");
        }
        double x0 = w / 2. - 8 * pitch, y0 = h / 2. - 9 * pitch;
        for (int r = 0; r < 10; ++r)
            cv::line(boardImage, {int(x0), int(y0 + 2 * r * pitch)},
                     {int(x0 + 16 * pitch), int(y0 + 2 * r * pitch)}, {40, 40, 40}, 2);
        for (int c = 0; c < 9; ++c)
            cv::line(boardImage, {int(x0 + 2 * c * pitch), int(y0)},
                     {int(x0 + 2 * c * pitch), int(y0 + 18 * pitch)}, {40, 40, 40}, 2);
        for (int id = 0; id < 4; ++id) {
            cv::Mat marker, bgr;
            cv::aruco::generateImageMarker(
                cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50), id, 96, marker);
            int x = id == 0 || id == 3 ? 16 : w - 112, y = id < 2 ? 16 : h - 112;
            cv::rectangle(boardImage, {x - 12, y - 12, 120, 120}, {255, 255, 255}, -1);
            cv::cvtColor(marker, bgr, cv::COLOR_GRAY2BGR);
            bgr.copyTo(boardImage(cv::Rect(x, y, 96, 96)));
        }
        workspace = cv::Mat(91, 91, CV_8UC1);
        std::ifstream mask("assets/workspace.bin", std::ios::binary);
        mask.read((char *)workspace.data, 8281);
        if (!mask)
            throw std::runtime_error("缺少旧机CAD碰撞采样");
        for (int r = 0; r < 10; ++r)
            for (int c = 0; c < 9; ++c)
                if (!freeJoint(ik(square(r, c), pieceHeight + lift)))
                    throw std::runtime_error("棋盘交点进入CAD禁区");
        for (int i = 0; i < 14; ++i) {
            sprites[i] = cv::imread("assets/" + std::string(1, names[i]) +
                                        std::to_string((int)cfg["sprite_variant"]) + ".png",
                                    cv::IMREAD_UNCHANGED);
            if (sprites[i].empty())
                throw std::runtime_error("缺少棋子渲染贴图");
        }
        auto initial = chess::initialBoard();
        for (int r = 0; r < 10; ++r)
            for (int c = 0; c < 9; ++c)
                if (initial[r][c] != '.')
                    pieces.push_back({initial[r][c], square(r, c)});
        cv::Point2d park((double)cfg["park_xy_mm"][0], (double)cfg["park_xy_mm"][1]);
        truth = ik(park, pieceHeight + lift);
        if (!freeJoint(truth))
            throw std::runtime_error("停车点进入CAD禁区");
        encoder = estimate = command = truth;
        for (int k = 0; k < 4; ++k) {
            motorScale[k] = (double)cfg["motor_ratio"][k] * (double)cfg["motor_sign"][k] *
                            (k ? 180 / pi : 360. / (double)cfg["screw_lead_mm"]);
            gap[k] = (double)cfg["backlash_mm_deg"][k] * (k ? pi / 180 : 1);
            zeroError[k] = (double)cfg["zero_error_mm_deg"][k] * (k ? pi / 180 : 1);
            axes[k].motor = axes[k].target = truth[k] * motorScale[k];
        }
        motionFilter.reset({truth[1],truth[2]});
        std::filesystem::create_directories("output");
#ifdef SIM_DEBUG
        setupDebug("output/debug");
#endif
        cv::setNumThreads(4);
        std::string error;
        detector = detector_load(error);
        if (!detector)
            throw std::runtime_error(error);
        guiDetector = detector;
        cadOpen(cfg, boardImage, sprites, interactive);
        if (!cadValidZ(pieceHeight) || !cadValidZ(pieceHeight + lift))
            throw std::runtime_error("棋盘/末端假设不满足旧CAD的Z行程");
        auto observation = observe(detector);
        if (observation.board != initial)
            throw std::runtime_error("开局实像素识别未匹配32枚棋子，见output/vision.jpg");
        drawScene(); // 新对局的三维记录立即更新，不沿用上一局的scene.jpg。
        if (interactive) {
            cv::namedWindow(yoloWindow, cv::WINDOW_NORMAL);
            cv::resizeWindow(yoloWindow, 720, 774);
            cv::moveWindow(yoloWindow, 1080, 80);
            cv::setMouseCallback(yoloWindow, yoloMouse);
            guiReady = true;
            pumpGui();
        }
        Pikafish engine;
        if (!engine.start("engine"))
            throw std::runtime_error(engine.error);
        chess::Game game;
        game.reset(observation.board, true, !robotRed);
        {
            cv::FileStorage start("output/status.json",
                                  cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
            start << "opencv" << CV_VERSION << "robot_side" << side << "ply" << 0 << "move" << ""
                  << "fen" << chess::fen(game.board(), true) << "virtual_motion_seconds" << 0.
                  << "last_motion_seconds" << 0. << "captured_by_arm" << 0;
        }
        int limit = interactive ? 400 : (int)cfg["max_plies"], completed = 0;
        if (interactive)
            engine.cancelled = []() {
                static auto nextFrame = std::chrono::steady_clock::time_point{};
                if (std::chrono::steady_clock::now() >= nextFrame) {
                    pumpGui();
                    nextFrame = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
                }
                return userQuit;
            };
        auto wallStart = std::chrono::steady_clock::now();
        for (int ply = 0; ply < limit; ++ply) {
            if (game.result() != chess::Result::Playing)
                break;
            std::vector<std::string> legal;
            if (!engine.legalMoves(game.baseFen(), game.history(), legal))
                throw std::runtime_error(engine.error);
            if (!game.judge(legal))
                throw std::runtime_error(game.error);
            if (game.result() != chess::Result::Playing)
                break;
            std::string choice;
            bool robotTurn = game.redToMove() == robotRed;
            humanTurn = interactive && !robotTurn;
            guiLegal = legal;
            auto scripts = cfg["scripted_moves"];
            if (humanTurn) {
                pendingHuman.clear();
                guiMessage = "Your turn - drag a piece to an intersection";
                while (pendingHuman.empty() && !userQuit)
                    pumpGui();
                if (userQuit)
                    throw std::runtime_error("用户结束仿真");
                choice = pendingHuman;
                humanTurn = false;
            } else if (!interactive && ply < (int)scripts.size())
                choice = (std::string)scripts[ply];
            else if (!engine.think(game.baseFen(), game.history(), (int)cfg["think_ms"], choice))
                throw std::runtime_error(engine.error);
            if (std::find(legal.begin(), legal.end(), choice) == legal.end())
                throw std::runtime_error("引擎或示例走法不合法：" + choice);
            chess::Move move;
            chess::parseMove(choice, move);
            double before = timeNow;
            if (robotTurn) {
                if (!game.prepareRobot(observation, choice, legal, move))
                    throw std::runtime_error(game.error);
                auto to = square(move.toRow, move.toCol);
                if (game.board()[move.toRow][move.toCol] != '.') {
                    cv::Point2d tray(100 + (captured % 5) * 30,
                                     270 + (captured / 5) * 30); // 棋盘之外的虚拟收纳区。
                    if (!freeJoint(ik(tray, pieceHeight)))
                        throw std::runtime_error("收纳点不可达");
                    transfer(to, tray);
                    ++captured;
                }
                transfer(square(move.fromRow, move.fromCol), to);
                go(park, pieceHeight + lift);
                observation = observe(detector);
                if (!game.confirmRobot(observation, true))
                    throw std::runtime_error(game.error);
            } else {
                humanMove(move);
                observation = observe(detector);
                if (!game.acceptHuman(observation, legal, move))
                    throw std::runtime_error(game.error);
            }
            completed = ply + 1;
            drawScene();
            cv::FileStorage status("output/status.json",
                                   cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
            status << "opencv" << CV_VERSION << "robot_side" << side << "ply" << ply + 1 << "move"
                   << choice << "fen" << chess::fen(game.board(), game.redToMove())
                   << "virtual_motion_seconds" << timeNow << "last_motion_seconds"
                   << timeNow - before << "captured_by_arm" << captured;
            if (debug.event)
                debug.event("confirmed", choice);
            guiMessage = "Confirmed: " + choice;
        }
        std::vector<std::string> legal;
        if (!engine.legalMoves(game.baseFen(), game.history(), legal))
            throw std::runtime_error(engine.error);
        if (game.result() == chess::Result::Playing && !game.judge(legal))
            throw std::runtime_error(game.error);
        const char *results[] = {"playing", "red_won", "black_won", "draw", "review_required"};
        cv::FileStorage result("output/result.json",
                               cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
        result << "result" << results[int(game.result())] << "stop_reason"
               << (game.result() == chess::Result::Playing ? "ply_limit" : "chess_rule")
               << "completed_plies" << completed << "history" << game.history();
        result.release();
        engine.close();
        if (interactive) {
            humanTurn = false;
            guiMessage = results[int(game.result())];
            while (!userQuit)
                pumpGui();
        }
        finishPreview();
        detector_close(detector);
        detector = nullptr;
        double wall =
            std::chrono::duration<double>(std::chrono::steady_clock::now() - wallStart).count();
        std::cout << "OpenCV " << CV_VERSION << " | robot=" << side << " | wall=" << wall
                  << "s | virtual=" << timeNow << "s\n";
#ifdef SIM_DEBUG
        finishDebug();
#endif
        if (interactive)
            cv::destroyAllWindows();
        cadClose();
        return 0;
    } catch (const std::exception &e) {
        finishPreview();
        previewFailure.clear();
        detector_close(detector);
        guiDetector = nullptr;
        std::cerr << e.what() << '\n';
        cv::FileStorage failure("output/result.json",
                                cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
        failure << "result" << (userQuit ? "stopped" : "error") << "message" << e.what();
        failure.release();
#ifdef SIM_DEBUG
        finishDebug();
#endif
        if (interactive && guiReady) {
            humanTurn = false;
            guiMessage = "Stopped - see terminal. ESC to close.";
            while (!userQuit)
                pumpGui();
            cadClose();
        }
        if (interactive)
            cv::destroyAllWindows();
        cadClose();
        return userQuit ? 0 : 1;
    }
}
