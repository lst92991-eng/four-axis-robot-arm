// 按已验证的固定 ROI、RGB letterbox、三路概率输出和跨类别 NMS 处理。
#include "yolo.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace scara {
namespace {
constexpr float Anchors[YoloHeads][YoloAnchors*2] = {{10,13,16,30,33,23}, {30,61,62,45,59,119},
                                {116,90,156,198,373,326}};

struct Candidate
{
  int classId;
  float score;
  double x1, y1, x2, y2;
};

// 把模型输出的 INT8 数值还原成概率或位置系数。
float dequantize(const YoloHead& head, int8_t value)
{
  return (int(value)-head.zeroPoint)*head.scale;
}

bool higherScore(const Candidate& a, const Candidate& b)
{
  return a.score>b.score;
}

double overlap(const Candidate& a, const Candidate& b)
{
  double width = std::max(0.0, std::min(a.x2,b.x2)-std::max(a.x1,b.x1));
  double height = std::max(0.0, std::min(a.y2,b.y2)-std::max(a.y1,b.y1));
  double intersection = width * height;
  double area = (a.x2-a.x1)*(a.y2-a.y1) + (b.x2-b.x1)*(b.y2-b.y1);
  return intersection / std::max(area-intersection, 1e-9);
}
}

//---------模型输入区---------
cv::Mat Vision_prepare(const cv::Mat& bgr, const VisionConfig& config, ImageMapping& mapping)
{
  const auto roi = config.inferenceRoi;
  if (bgr.type()!=CV_8UC3 || bgr.size()!=config.frameSize || roi.width<=0 || roi.height<=0 ||
      (roi & cv::Rect({}, bgr.size()))!=roi)
    throw std::runtime_error("Camera size or inference ROI differs from configuration");

  // Python 参考实现使用同一个缩放比恢复 X、Y，边缘留白按整数尺寸计算。
  double scale = std::min(double(YoloSize)/roi.width, double(YoloSize)/roi.height);
  cv::Size resized{cvRound(roi.width*scale), cvRound(roi.height*scale)};
  mapping = {roi, scale, (YoloSize-resized.width)/2, (YoloSize-resized.height)/2};
  cv::Mat rgb;
  cv::cvtColor(bgr(roi), rgb, cv::COLOR_BGR2RGB);
  cv::Mat input(YoloSize, YoloSize, CV_8UC3, cv::Scalar(114,114,114));
  cv::Mat picture = input(cv::Rect(mapping.left, mapping.top, resized.width, resized.height));
  cv::resize(rgb, picture, resized, 0, 0, cv::INTER_LINEAR);
  return input;
}

//---------检测输出区---------
std::vector<Detection> Vision_decode(const YoloHead heads[YoloHeads],
                                    const ImageMapping& mapping, const VisionConfig& config)
{
  std::vector<Candidate> candidates, kept;
  for (int index=0; index<YoloHeads; ++index)
  {
    const auto& head = heads[index];
    int size = YoloGrid[index];
    int stride = YoloSize / size;
    if (!head.data || head.rowBytes<size_t(size)*YoloChannels ||
        head.bytes<size_t(size-1)*head.rowBytes+size*YoloChannels ||
        !std::isfinite(head.scale) || head.scale<=0)
      throw std::runtime_error("Invalid native YOLO output layout");
    // 每个网格点有 3 个候选框，每个框由 4 个位置数、1 个目标分数和 14 个类别分数组成。
    for (int y=0; y<size; ++y)
    {
      for (int x=0; x<size; ++x)
      {
        for (int anchor=0; anchor<YoloAnchors; ++anchor)
        {
          const int8_t *prediction = head.data+y*head.rowBytes+x*YoloChannels+anchor*YoloBoxValues;
          int classId = 0;
          for (int candidateId=1; candidateId<YoloClasses; ++candidateId)
            if (prediction[5+candidateId]>prediction[5+classId]) classId = candidateId;
          float score = dequantize(head,prediction[4])*dequantize(head,prediction[5+classId]);
          if (score<config.scoreThreshold) continue;
          double centerX = (double(dequantize(head,prediction[0])*2-.5f)+x)*stride;
          double centerY = (double(dequantize(head,prediction[1])*2-.5f)+y)*stride;
          float width = dequantize(head,prediction[2])*2;
          float height = dequantize(head,prediction[3])*2;
          width = width*width*Anchors[index][anchor*2];
          height = height*height*Anchors[index][anchor*2+1];
          candidates.push_back({classId, score, centerX-width/2, centerY-height/2,
                               centerX+width/2, centerY+height/2});
        }
      }
    }
  }
  // 同分框保留原来的遍历次序，NMS 在网络坐标中执行，再还原到原始帧。
  std::stable_sort(candidates.begin(), candidates.end(), higherScore);
  for (const auto& candidate : candidates)
  {
    bool duplicate = false;
    for (const auto& old : kept)
    {
      if (overlap(candidate, old)>config.nmsThreshold)
      {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) kept.push_back(candidate);
  }
  std::vector<Detection> detections;
  for (const auto& box : kept)
  {
    double x1 = std::clamp((box.x1-mapping.left)/mapping.scale, 0.0, double(mapping.roi.width));
    double y1 = std::clamp((box.y1-mapping.top)/mapping.scale, 0.0, double(mapping.roi.height));
    double x2 = std::clamp((box.x2-mapping.left)/mapping.scale, 0.0, double(mapping.roi.width));
    double y2 = std::clamp((box.y2-mapping.top)/mapping.scale, 0.0, double(mapping.roi.height));
    if (x2>x1 && y2>y1)
      detections.push_back({box.classId, box.score, {x1+mapping.roi.x, y1+mapping.roi.y, x2-x1, y2-y1}});
  }
  return detections;
}

//---------识别画面区---------
cv::Mat Vision_draw(const cv::Mat& bgr, const std::vector<Detection>& detections,
                    const cv::Mat& labels)
{
  if (labels.type()!=CV_8UC1 || labels.rows!=YoloClasses*24 || labels.cols!=64)
    throw std::runtime_error("Expected the 14-class Chinese label atlas");
  cv::Mat result = bgr.clone();
  for (const auto& detection : detections)
  {
    if (detection.classId<0 || detection.classId>=YoloClasses) continue;
    cv::Rect box = cv::Rect(detection.bbox) & cv::Rect({}, result.size());
    cv::Scalar color = detection.classId<7 ? cv::Scalar(255,180,65) : cv::Scalar(65,85,245);
    cv::rectangle(result, box, color, 2);
    int x = std::clamp(box.x, 0, std::max(0, result.cols-44));
    int y = std::clamp(box.y-25, 0, std::max(0, result.rows-24));
    cv::rectangle(result, {x,y,44,24}, color, cv::FILLED);
    result(cv::Rect(x,y,44,24)).setTo(cv::Scalar::all(255), labels(cv::Rect(0,detection.classId*24,44,24)));
    int scoreY = std::clamp(box.y+box.height-16, 0, std::max(0, result.rows-16));
    cv::rectangle(result, {x,scoreY,44,16}, color, cv::FILLED);
    cv::putText(result, cv::format("%.2f", detection.confidence), {x+3,scoreY+12},
                cv::FONT_HERSHEY_SIMPLEX, .4, cv::Scalar::all(255), 1, cv::LINE_AA);
  }
  return result;
}
}
