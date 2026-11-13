#include "yolo.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <stdexcept>

namespace scara
{
static cv::Size FrameSize;
static cv::Rect Roi, InputArea;
static double Scale;
constexpr float ScoreThreshold = .50f, NmsThreshold = .45f;
constexpr int LabelWidth = 44, ScoreHeight = 16;
static const cv::Size2f Anchors[YoloHeads][YoloAnchors] = {
    {{10, 13}, {16, 30}, {33, 23}},
    {{30, 61}, {62, 45}, {59, 119}},
    {{116, 90}, {156, 198}, {373, 326}},
};

static float dequantize(const YoloHead &head, int8_t value)
{
    return (int(value) - head.zeroPoint) * head.scale;
}

static bool higherScore(const Detection &a, const Detection &b)
{
    return a.confidence > b.confidence;
}

static void readCandidates(const YoloHead &head, int index, std::vector<Detection> &candidates)
{
    int size = YoloGrid[index];
    int stride = YoloSize / size;
    // INT8 最大值对应类别分数上界，不把量化后的上界假定为恰好 1。
    float maximumClassScore = dequantize(head, 127);
    // 每个网格点有 3 个候选框，每个框由 4 个位置数、1 个目标分数和 14 个类别分数组成。
    for (int y = 0; y < size; ++y)
    {
        for (int x = 0; x < size; ++x)
        {
            for (int anchor = 0; anchor < YoloAnchors; ++anchor)
            {
                const int8_t *prediction = head.data + y * head.rowBytes + x * YoloChannels + anchor * YoloBoxValues;
                float objectScore = dequantize(head, prediction[4]);
                if (objectScore >= 0 && objectScore * maximumClassScore < ScoreThreshold) continue;
                const int8_t *classScores = prediction + 5;
                const int8_t *best = std::max_element(classScores, classScores + YoloClasses);
                int classId = int(best - classScores);
                float score = objectScore * dequantize(head, prediction[5 + classId]);
                if (score < ScoreThreshold) continue;
                // 先得到格内偏移，再加网格位置；与原实现保持相同的 float/double 顺序。
                float offsetX = dequantize(head, prediction[0]) * 2 - .5f;
                float offsetY = dequantize(head, prediction[1]) * 2 - .5f;
                double centerX = (double(offsetX) + x) * stride;
                double centerY = (double(offsetY) + y) * stride;
                float widthFactor = dequantize(head, prediction[2]) * 2;
                float heightFactor = dequantize(head, prediction[3]) * 2;
                cv::Size2f anchorSize = Anchors[index][anchor];
                float width = widthFactor * widthFactor * anchorSize.width;
                float height = heightFactor * heightFactor * anchorSize.height;
                candidates.push_back(
                    {classId, score, cv::Rect2d(centerX - width / 2, centerY - height / 2, width, height)});
            }
        }
    }
}

// 固定流程先设置一次参数，取帧时不再重新计算几何。
void Vision_setup(cv::Size frameSize, cv::Rect roi)
{
    FrameSize = frameSize;
    Roi = roi;
    Scale = std::min(double(YoloSize) / roi.width, double(YoloSize) / roi.height);
    cv::Size resized(cvRound(roi.width * Scale), cvRound(roi.height * Scale));
    InputArea = {(YoloSize - resized.width) / 2, (YoloSize - resized.height) / 2, resized.width, resized.height};
}

cv::Mat Vision_prepare(const cv::Mat &bgr)
{
    if (bgr.type() != CV_8UC3 || bgr.size() != FrameSize) throw std::runtime_error("Unexpected camera frame");
    cv::Mat rgb, input(YoloSize, YoloSize, CV_8UC3, cv::Scalar::all(114));
    cv::cvtColor(bgr(Roi), rgb, cv::COLOR_BGR2RGB);
    cv::Mat picture = input(InputArea);
    cv::resize(rgb, picture, InputArea.size(), 0, 0, cv::INTER_LINEAR);
    return input;
}

std::vector<Detection> Vision_decode(const YoloHead heads[YoloHeads])
{
    std::vector<Detection> boxes;
    for (int i = 0; i < YoloHeads; ++i)
        readCandidates(heads[i], i, boxes);
    std::stable_sort(boxes.begin(), boxes.end(), higherScore);
    // 保留高分框，删除后面与它重叠的框；同分维持原顺序。
    for (size_t i = 0; i < boxes.size(); ++i)
        for (size_t j = i + 1; j < boxes.size();)
        {
            cv::Rect2d a = boxes[i].bbox, b = boxes[j].bbox;
            double overlap = (a & b).area();
            double total = a.area() + b.area() - overlap;
            if (overlap / std::max(total, 1e-9) > NmsThreshold)
                boxes.erase(boxes.begin() + j);
            else
                ++j;
        }
    // 原地写回有效结果，先去重，再裁切并恢复原图坐标。
    size_t count = 0;
    for (Detection piece : boxes)
    {
        cv::Rect2d b = piece.bbox;
        double x1 = std::clamp((b.x - InputArea.x) / Scale, 0.0, double(Roi.width));
        double y1 = std::clamp((b.y - InputArea.y) / Scale, 0.0, double(Roi.height));
        double x2 = std::clamp((b.x + b.width - InputArea.x) / Scale, 0.0, double(Roi.width));
        double y2 = std::clamp((b.y + b.height - InputArea.y) / Scale, 0.0, double(Roi.height));
        if (x2 > x1 && y2 > y1)
        {
            piece.bbox = {x1 + Roi.x, y1 + Roi.y, x2 - x1, y2 - y1};
            boxes[count++] = piece;
        }
    }
    boxes.resize(count);
    return boxes;
}

cv::Mat Vision_draw(const cv::Mat &bgr, const std::vector<Detection> &detections, const cv::Mat &labels)
{
    cv::Mat result = bgr.clone();
    cv::Rect bounds(0, 0, result.cols, result.rows);
    for (const Detection &piece : detections)
    {
        cv::Rect box = cv::Rect(piece.bbox) & bounds;
        if (box.empty()) continue;
        cv::Scalar color = piece.classId < 7 ? cv::Scalar(255, 180, 65) : cv::Scalar(65, 85, 245);
        cv::rectangle(result, box, color, 2);
        int x = std::clamp(box.x, 0, std::max(0, result.cols - LabelWidth));
        int y = std::clamp(box.y - LabelHeight - 1, 0, std::max(0, result.rows - LabelHeight));
        cv::Rect label = cv::Rect(x, y, LabelWidth, LabelHeight) & bounds;
        result(label).setTo(color);
        result(label).setTo(cv::Scalar::all(255),
                            labels(cv::Rect(0, piece.classId * LabelHeight, label.width, label.height)));
        int scoreY = std::clamp(box.y + box.height - ScoreHeight, 0, std::max(0, result.rows - ScoreHeight));
        cv::rectangle(result, {x, scoreY, LabelWidth, ScoreHeight}, color, cv::FILLED);
        cv::putText(result, cv::format("%.2f", piece.confidence), {x + 3, scoreY + 12}, cv::FONT_HERSHEY_SIMPLEX, .4,
                    cv::Scalar::all(255), 1, cv::LINE_AA);
    }
    return result;
}
} // namespace scara
