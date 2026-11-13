// 按已验证的固定 ROI、RGB letterbox、三路概率输出和跨类别 NMS 处理。
#include "yolo.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <stdexcept>

namespace scara
{
namespace
{
// 中文字图每行占 64×24；其中 44 像素用于显示两个字。
constexpr int LabelWidth = 44;
constexpr int ScoreHeight = 16;

// 每一对分别是锚框的宽、高，单位：640 输入图像素。
const cv::Size2f Anchors[YoloHeads][YoloAnchors] = {
    {{10, 13}, {16, 30}, {33, 23}},
    {{30, 61}, {62, 45}, {59, 119}},
    {{116, 90}, {156, 198}, {373, 326}},
};

} // namespace

static void readCandidates(const YoloHead &head, int index, float scoreThreshold, std::vector<Detection> &candidates);
static float dequantize(const YoloHead &head, int8_t value);
static bool higherScore(const Detection &a, const Detection &b);
static double intersectionOverUnion(const cv::Rect2d &a, const cv::Rect2d &b);
static bool overlapsKept(const Detection &box, const std::vector<Detection> &kept, float threshold);

//---------模型输入区---------
cv::Mat Vision_prepare(const cv::Mat &bgr, const VisionConfig &config, ImageMapping &mapping)
{
    cv::Rect roi = config.inferenceRoi;
    cv::Rect frameBounds(0, 0, bgr.cols, bgr.rows);
    if (bgr.type() != CV_8UC3 || bgr.size() != config.frameSize || roi.width <= 0 || roi.height <= 0 ||
        (roi & frameBounds) != roi)
        throw std::runtime_error("Camera size or inference ROI differs from configuration");

    // Python 参考实现使用同一个缩放比恢复 X、Y，边缘留白按整数尺寸计算。
    double scaleX = double(YoloSize) / roi.width;
    double scaleY = double(YoloSize) / roi.height;
    double scale = std::min(scaleX, scaleY);
    cv::Size resized{cvRound(roi.width * scale), cvRound(roi.height * scale)};
    mapping.roi = roi;
    mapping.scale = scale;
    mapping.left = (YoloSize - resized.width) / 2;
    mapping.top = (YoloSize - resized.height) / 2;
    cv::Mat cropped = bgr(roi);
    cv::Mat rgb;
    cv::cvtColor(cropped, rgb, cv::COLOR_BGR2RGB);
    cv::Mat input(YoloSize, YoloSize, CV_8UC3, cv::Scalar(114, 114, 114));
    cv::Rect inputArea(mapping.left, mapping.top, resized.width, resized.height);
    cv::Mat picture = input(inputArea);
    cv::resize(rgb, picture, resized, 0, 0, cv::INTER_LINEAR);
    return input;
}

//---------检测输出区---------
std::vector<Detection> Vision_decode(const YoloHead heads[YoloHeads], const ImageMapping &mapping,
                                     const VisionConfig &config)
{
    // 候选和去重阶段均为640输入图坐标，返回前统一换成原图坐标。
    std::vector<Detection> candidates, kept;
    // 1. 逐路、逐格读取候选框及其最高分类分数。
    for (int index = 0; index < YoloHeads; ++index)
        readCandidates(heads[index], index, config.scoreThreshold, candidates);
    // 2. 按分数去重；同分保留遍历顺序，不因类别不同重复保留同一枚棋子。
    std::stable_sort(candidates.begin(), candidates.end(), higherScore);
    for (const Detection &candidate : candidates)
    {
        if (!overlapsKept(candidate, kept, config.nmsThreshold)) kept.push_back(candidate);
    }
    // 3. 去掉留白、还原缩放，最后加回裁剪区域在原图中的偏移。
    std::vector<Detection> detections;
    for (Detection piece : kept)
    {
        cv::Rect2d box = piece.bbox;
        double x1 = std::clamp((box.x - mapping.left) / mapping.scale, 0.0, double(mapping.roi.width));
        double y1 = std::clamp((box.y - mapping.top) / mapping.scale, 0.0, double(mapping.roi.height));
        double x2 = std::clamp((box.x + box.width - mapping.left) / mapping.scale, 0.0, double(mapping.roi.width));
        double y2 = std::clamp((box.y + box.height - mapping.top) / mapping.scale, 0.0, double(mapping.roi.height));
        if (x2 > x1 && y2 > y1)
        {
            piece.bbox = cv::Rect2d(x1 + mapping.roi.x, y1 + mapping.roi.y, x2 - x1, y2 - y1);
            detections.push_back(piece);
        }
    }
    return detections;
}

//---------识别画面区---------
cv::Mat Vision_draw(const cv::Mat &bgr, const std::vector<Detection> &detections, const cv::Mat &labels)
{
    cv::Mat result = bgr.clone();
    cv::Rect bounds(0, 0, result.cols, result.rows);
    for (const Detection &detection : detections)
    {
        cv::Rect box = cv::Rect(detection.bbox) & bounds;
        if (box.empty()) continue;
        cv::Scalar color = detection.classId < 7 ? cv::Scalar(255, 180, 65) : cv::Scalar(65, 85, 245);
        cv::rectangle(result, box, color, 2);
        int x = std::clamp(box.x, 0, std::max(0, result.cols - LabelWidth));
        int y = std::clamp(box.y - LabelHeight - 1, 0, std::max(0, result.rows - LabelHeight));
        cv::Rect label = cv::Rect(x, y, LabelWidth, LabelHeight) & bounds;
        cv::Rect symbolArea(0, detection.classId * LabelHeight, label.width, label.height);
        cv::Mat textMask = labels(symbolArea);
        cv::Mat caption = result(label);
        caption.setTo(color);
        caption.setTo(cv::Scalar::all(255), textMask);
        int scoreY = std::clamp(box.y + box.height - ScoreHeight, 0, std::max(0, result.rows - ScoreHeight));
        cv::rectangle(result, {x, scoreY, LabelWidth, ScoreHeight}, color, cv::FILLED);
        std::string score = cv::format("%.2f", detection.confidence);
        cv::putText(result, score, {x + 3, scoreY + 12}, cv::FONT_HERSHEY_SIMPLEX, .4, cv::Scalar::all(255), 1,
                    cv::LINE_AA);
    }
    return result;
}

//---------内部计算区---------
// 一次处理一路输出；候选框保持原来的行、列、锚框遍历顺序。
static void readCandidates(const YoloHead &head, int index, float scoreThreshold, std::vector<Detection> &candidates)
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
                if (objectScore >= 0 && objectScore * maximumClassScore < scoreThreshold) continue;
                const int8_t *classScores = prediction + 5;
                const int8_t *best = std::max_element(classScores, classScores + YoloClasses);
                int classId = int(best - classScores);
                float score = objectScore * dequantize(head, prediction[5 + classId]);
                if (score < scoreThreshold) continue;
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
                Detection box;
                box.classId = classId;
                box.confidence = score;
                box.bbox = cv::Rect2d(centerX - width / 2, centerY - height / 2, width, height);
                candidates.push_back(box);
            }
        }
    }
}

static float dequantize(const YoloHead &head, int8_t value)
{
    return (int(value) - head.zeroPoint) * head.scale;
}

static bool higherScore(const Detection &a, const Detection &b)
{
    return a.confidence > b.confidence;
}

static bool overlapsKept(const Detection &box, const std::vector<Detection> &kept, float threshold)
{
    for (const Detection &old : kept)
        if (intersectionOverUnion(box.bbox, old.bbox) > threshold) return true;
    return false;
}

static double intersectionOverUnion(const cv::Rect2d &a, const cv::Rect2d &b)
{
    double intersection = (a & b).area();
    double combined = a.area() + b.area() - intersection;
    return intersection / std::max(combined, 1e-9);
}
} // namespace scara
