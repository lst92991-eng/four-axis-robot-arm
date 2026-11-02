// 桌面GPU适配：OpenCV准备RGB，主机CUDA服务推理；服务不可用就停止，不转CPU。
#include "detector.hpp"
#include <opencv2/imgproc.hpp>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <algorithm>
#include <stdexcept>

struct Detector {
    int socket = -1;
    std::string backend;
    bool proofSaved = false;
};
namespace {
void transfer(int socket, void *data, size_t bytes, bool sending) {
    auto *p = static_cast<char *>(data);
    while (bytes) {
        ssize_t n = sending ? send(socket, p, bytes, MSG_NOSIGNAL) : recv(socket, p, bytes, 0);
        if (n <= 0)
            throw std::runtime_error("GPU连接中断/超时；请启动主机start-gpu.ps1，不回退CPU");
        p += n;
        bytes -= n;
    }
}
cv::FileStorage request(Detector *d, const cv::Mat &input) {
    uint32_t bytes = htonl(input.total() * input.elemSize());
    transfer(d->socket, &bytes, 4, true);
    if (!input.empty())
        transfer(d->socket, input.data, input.total() * input.elemSize(), true);
    transfer(d->socket, &bytes, 4, false);
    bytes = ntohl(bytes);
    if (!bytes || bytes > 65536)
        throw std::runtime_error("GPU返回长度异常");
    std::string json(bytes, ' ');
    transfer(d->socket, json.data(), bytes, false);
    return cv::FileStorage(json, cv::FileStorage::READ | cv::FileStorage::MEMORY |
                                     cv::FileStorage::FORMAT_JSON);
}
} // namespace
Detector *detector_load(std::string &error) {
    auto *d = new Detector;
    try {
        d->socket = socket(AF_INET, SOCK_STREAM, 0);
        int noDelay = 1;
        setsockopt(d->socket, IPPROTO_TCP, TCP_NODELAY, &noDelay, sizeof(noDelay));
        timeval timeout{10, 0};
        setsockopt(d->socket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        setsockopt(d->socket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_port = htons(8767);
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        if (connect(d->socket, reinterpret_cast<sockaddr *>(&address), sizeof(address)))
            throw std::runtime_error("GPU未连接；请先在Windows运行start-gpu.ps1，不回退CPU");
        auto info = request(d, {});
        if ((std::string)info["backend"] != "cuda:0" ||
            (std::string)info["classes"] != "kabnrcpKABNRCP" ||
            (std::string)info["checkpoint_sha256"] !=
                "b18cabfe0202b05ad4b127cbe52632aa7d20026af2f5b9d5c9186ab476050a61")
            throw std::runtime_error("GPU后端/模型不匹配");
        d->backend = "CUDA:0 | " + (std::string)info["gpu"];
        return d;
    } catch (const std::exception &e) {
        error = e.what();
        detector_close(d);
        return nullptr;
    }
}
bool detector_detect(Detector *d, const cv::Mat &rgb, std::vector<Detection> &result,
                     std::string &error) {
    try {
        result.clear();
        const float ratio = std::min(640.f / rgb.cols, 640.f / rgb.rows);
        const int w = cvRound(rgb.cols * ratio), h = cvRound(rgb.rows * ratio),
                  left = (640 - w) / 2, top = (640 - h) / 2;
        cv::Mat input(640, 640, CV_8UC3, cv::Scalar(114, 114, 114)), small;
        cv::resize(rgb, small, {w, h});
        small.copyTo(input(cv::Rect(left, top, w, h)));
        auto answer = request(d, input);
        if (!(int)answer["model_cuda"] || !(int)answer["input_cuda"] || !(int)answer["nms_cuda"])
            throw std::runtime_error("GPU执行验证失败");
        d->backend = "CUDA:0 | " + (std::string)answer["gpu"] + " | " +
                     std::to_string(cvRound((double)answer["ms"])) + " ms";
        if (!d->proofSaved) {
            cv::FileStorage proof("output/gpu.json",
                                  cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON);
            proof << "backend" << d->backend << "model_cuda" << 1 << "input_cuda" << 1 << "nms_cuda"
                  << 1 << "first_frame_ms" << (double)answer["ms"] << "allocated_mb"
                  << (double)answer["allocated_mb"] << "checkpoint_sha256"
                  << (std::string)answer["checkpoint_sha256"];
            d->proofSaved = true;
        }
        for (const auto &box : answer["detections"])
            result.push_back({(int)box[0], (float)box[1], ((float)box[2] - left) / ratio,
                              ((float)box[3] - top) / ratio, ((float)box[4] - left) / ratio,
                              ((float)box[5] - top) / ratio});
        return true;
    } catch (const std::exception &e) {
        error = e.what();
        return false;
    }
}
std::string detector_backend(Detector *d) {
    return d->backend;
}
void detector_close(Detector *d) {
    if (!d)
        return;
    if (d->socket >= 0)
        close(d->socket);
    delete d;
}
