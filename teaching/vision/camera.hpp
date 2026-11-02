#pragma once
#include <opencv2/core.hpp>
#include <functional>
#include <string>

// 复用本板已验证的rkisp_demo和ISP配置，不在课程代码里重写厂商ISP。
class Camera {
public:
    explicit Camera(int width = 1280, int height = 960);
    ~Camera() { stop(); }
    Camera(const Camera&) = delete;
    Camera& operator=(const Camera&) = delete;
    bool start(const std::string& moduleScript, std::string& error);
    bool read(cv::Mat& bgr, std::string& error);
    // false时保留PID和锁，必须确认回收后才能启动下一资源阶段。
    bool stop();
    std::function<bool()> cancelled;
private:
    int width_ = 1280, height_ = 960;
    int fd_ = -1, pid_ = -1, loaderPid_ = -1;
    bool locked_ = false;
};
