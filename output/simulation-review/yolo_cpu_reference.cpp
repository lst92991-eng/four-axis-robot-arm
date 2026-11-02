// 历史CPU基准，仅供GPU切换的同帧对比；不参与当前仿真构建。
#include "detector.hpp"
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

struct Detector {
    cv::dnn::Net net;
};
Detector *detector_load(const char *path, std::string &error) {
    try {
        return new Detector{cv::dnn::readNetFromONNX(path)};
    } catch (const std::exception &e) {
        error = e.what();
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
        d->net.setInput(cv::dnn::blobFromImage(input, 1. / 255));
        std::vector<cv::Mat> heads;
        d->net.forward(heads, d->net.getUnconnectedOutLayersNames());
        const int anchors[3][6] = {
            {10, 13, 16, 30, 33, 23}, {30, 61, 62, 45, 59, 119}, {116, 90, 156, 198, 373, 326}};
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> labels;
        for (int layer = 0; layer < 3; ++layer) {
            const auto &head = heads.at(layer);
            const int n = 80 >> layer, stride = 8 << layer;
            if (head.dims != 4 || head.size[1] != 57 || head.size[2] != n || head.size[3] != n)
                throw std::runtime_error("Unexpected YOLO probability-head shape");
            const float *p = head.ptr<float>();
            for (int a = 0; a < 3; ++a)
                for (int y = 0; y < n; ++y)
                    for (int x = 0; x < n; ++x) {
                        auto at = [&](int c) { return p[(a * 19 + c) * n * n + y * n + x]; };
                        if (at(4) < .5f)
                            continue;
                        int cls = 0;
                        for (int c = 1; c < 14; ++c)
                            if (at(5 + c) > at(5 + cls))
                                cls = c;
                        const float score = at(4) * at(5 + cls);
                        if (score < .5f)
                            continue;
                        const float cx = (at(0) * 2 - .5f + x) * stride,
                                    cy = (at(1) * 2 - .5f + y) * stride;
                        const float bw = std::pow(at(2) * 2, 2) * anchors[layer][2 * a];
                        const float bh = std::pow(at(3) * 2, 2) * anchors[layer][2 * a + 1];
                        boxes.emplace_back(cvRound((cx - bw / 2 - left) / ratio),
                                           cvRound((cy - bh / 2 - top) / ratio),
                                           cvRound(bw / ratio), cvRound(bh / ratio));
                        scores.push_back(score);
                        labels.push_back(cls);
                    }
        }
        std::vector<int> keep;
        cv::dnn::NMSBoxes(boxes, scores, .5f, .45f, keep);
        for (int i : keep) {
            const auto &b = boxes[i];
            result.push_back({labels[i], scores[i], float(b.x), float(b.y), float(b.x + b.width),
                              float(b.y + b.height)});
        }
        return true;
    } catch (const std::exception &e) {
        error = e.what();
        return false;
    }
}
void detector_close(Detector *d) {
    delete d;
}
