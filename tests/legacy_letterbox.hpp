// 旧录像张量按录制时的矩形 ROI、缩放与补边解释，只用于离线回放。
#pragma once
#include "models/xiangqi14-local-v3/postprocess.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>

struct LegacyLetterbox
{
    cv::Rect roi, content;
    double scale;

    explicit LegacyLetterbox(cv::Rect region) : roi(region)
    {
        const int side = scara::YoloSize;
        scale = std::min(double(side) / roi.width, double(side) / roi.height);
        cv::Size size(cvRound(roi.width * scale), cvRound(roi.height * scale));
        content = {(side - size.width) / 2, (side - size.height) / 2, size.width, size.height};
    }

    cv::Mat input(const cv::Mat &source) const
    {
        cv::Mat rgb;
        cv::cvtColor(source(roi), rgb, cv::COLOR_BGR2RGB);
        cv::Mat canvas(scara::YoloSize, scara::YoloSize, CV_8UC3, cv::Scalar::all(114));
        cv::resize(rgb, canvas(content), content.size());
        return canvas;
    }

    std::vector<scara::Detection> decode(const scara::YoloHead heads[scara::YoloHeads]) const
    {
        // 先按模型画布解码，再去掉历史补边、撤销历史比例并还原原图坐标。
        std::vector<scara::Detection> result;
        for (auto piece : scara::Vision_decode(heads, {0, 0}))
        {
            const auto &b = piece.bbox;
            double left = std::clamp((b.x - content.x) / scale, 0.0, double(roi.width));
            double top = std::clamp((b.y - content.y) / scale, 0.0, double(roi.height));
            double right = std::clamp((b.x + b.width - content.x) / scale, 0.0, double(roi.width));
            double bottom = std::clamp((b.y + b.height - content.y) / scale, 0.0, double(roi.height));
            piece.bbox = {left + roi.x, top + roi.y, right - left, bottom - top};
            if (!piece.bbox.empty()) result.push_back(piece);
        }
        return result;
    }
};
