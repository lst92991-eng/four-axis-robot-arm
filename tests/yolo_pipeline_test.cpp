#include "vision/yolo.hpp"
#include "vision/capture.hpp"
#include "fixture_bytes.hpp"
#include "legacy_letterbox.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
void check(bool passed, const char *message)
{
  if (!passed) throw std::runtime_error(message);
}

cv::Mat image(const std::string& path, int mode=cv::IMREAD_COLOR)
{
  std::string bytes;
  scara::readBytes(path, bytes);
  return cv::imdecode(std::vector<uint8_t>(bytes.begin(),bytes.end()), mode);
}

void sameInput(const cv::Mat& source, cv::Rect roi, const std::string& expected)
{
  cv::Mat input = LegacyLetterbox(roi).input(source);
  cv::Mat rgb;
  cv::cvtColor(image(expected), rgb, cv::COLOR_BGR2RGB);
  check(cv::norm(input, rgb, cv::NORM_INF)==0, "Preprocessing differs from the verified Python path");
}

void decodeEdges()
{
  std::vector<int8_t> data[3];
  scara::YoloHead heads[3];
  for (int i=0; i<3; ++i)
  {
    int size = 80 >> i;
    data[i].resize(size*size*57, 0);
    heads[i] = {data[i].data(), size_t(size)*57, 0, .01f};
  }
  auto *p = data[0].data()+(20*80+20)*57;
  p[0] = p[1] = p[2] = p[3] = 50;
  p[4] = 40; p[5+8] = 126; // 0.40 * 1.26 = 0.504, above the 0.50 threshold.
  auto found = scara::Vision_decode(heads, {0,0});
  check(found.size()==1 && found[0].classId==8, "Early rejection assumed class scores cannot exceed one");
  auto modelBox = found[0].bbox;
  auto translated = scara::Vision_decode(heads, {375,8});
  check(translated.size()==1 && translated[0].bbox == modelBox + cv::Point2d(375,8),
        "Direct crop must translate boxes into the source image");
  p[5+2] = 126;
  found = scara::Vision_decode(heads, {0,0});
  check(found.size()==1 && found[0].classId==2, "Equal class scores did not keep the first class");
  p[5+2] = 0;
  p[4] = 39;
  found = scara::Vision_decode(heads, {0,0});
  check(found.empty(), "Below-threshold candidate survived");
  p[2] = p[3] = p[4] = 100; p[5+8] = 0; p[5+2] = 90;
  auto *q = p+19;
  q[0] = q[1] = 50; q[2] = 80; q[3] = 66; q[4] = 100; q[5+5] = 90;
  found = scara::Vision_decode(heads, {0,0});
  check(found.size()==1 && found[0].classId==2, "Cross-class NMS or equal-score traversal order changed");
  q[5+5] = 91;
  found = scara::Vision_decode(heads, {0,0});
  check(found.size()==1 && found[0].classId==5, "NMS did not keep the highest scoring overlapping box");
  q[4] = 0; p[5+2] = 50;
  found = scara::Vision_decode(heads, {0,0});
  check(found.size()==1, "Score exactly 0.50 was rejected");

  // 左边缘的一对重叠框：高分框完全在图外，低分框只有右边一小段可见。
  std::fill(data[0].begin(),data[0].end(),0);
  p = data[0].data() + 20*80*57;
  q = p + 19;
  p[0] = -10; p[1] = p[2] = p[3] = 50; p[4] = 100; p[5] = 90;
  q[0] = 5; q[1] = 50; q[2] = 40; q[3] = 33; q[4] = 100; q[5] = 80;
  check(scara::Vision_decode(heads, {375,8}).empty(), "Edge boxes must be suppressed before clipping");
  p[4] = 0;
  found = scara::Vision_decode(heads, {375,8});
  check(found.size()==1 && found[0].bbox.x==375 && found[0].bbox.width<2,
        "Visible part of the edge box was not clipped to the crop");

  // 三个框依次重叠，第二个被第一个排除后，第三个仍应保留。
  std::fill(data[0].begin(),data[0].end(),0);
  for (int i=0; i<3; ++i)
  {
    auto *candidate = data[0].data()+(20*80+20+i*2)*57;
    candidate[0] = candidate[1] = candidate[3] = 50;
    candidate[2] = 112;
    candidate[4] = 100;
    candidate[5+2] = int8_t(90-i*10);
  }
  found = scara::Vision_decode(heads, {0,0});
  check(found.size()==2 && found[1].bbox.x>found[0].bbox.x+25,
        "A discarded middle box suppressed the final candidate");

  // 每行填入额外的高分字节，验证只读取有效列并正确跨过对齐区。
  std::vector<scara::Detection> expected = found;
  size_t rowBytes = size_t(81)*scara::YoloChannels;
  std::vector<int8_t> padded(80*rowBytes,127);
  for (int row=0; row<80; ++row)
    std::memcpy(padded.data()+row*rowBytes,data[0].data()+row*80*scara::YoloChannels,80*scara::YoloChannels);
  heads[0].data = padded.data();
  heads[0].rowBytes = rowBytes;
  found = scara::Vision_decode(heads, {0,0});
  check(found.size()==expected.size(), "Padded tensor changed the detection count");
  for (size_t i=0; i<found.size(); ++i)
    check(found[i].classId==expected[i].classId && found[i].confidence==expected[i].confidence &&
          found[i].bbox==expected[i].bbox, "Padded tensor changed a decoded value");
}
}

int main(int argc, char **argv)
{
  try
  {
    decodeEdges();
    const std::string fixture = "tests/fixtures/vision/";
    auto source = image(fixture+"source.png");
    cv::FileStorage expected;
    expected.open(fixture+"expected.json", cv::FileStorage::READ);
    auto roi = expected["roi"];
    cv::Rect historical = {int(roi[0]),int(roi[1]),int(roi[2]),int(roi[3])};
    sameInput(source, historical, fixture+"historical-input.png");
    sameInput(source, {300,70,650,600}, fixture+"fixed-input.png");
    cv::Mat expectedRgb;
    cv::cvtColor(source(cv::Rect(375,8,640,640)), expectedRgb, cv::COLOR_BGR2RGB);
    auto direct = scara::Vision_prepare(source);
    check(direct.size()==cv::Size(640,640) && cv::norm(direct,expectedRgb,cv::NORM_INF)==0,
          "Direct crop changed pixels or channel order");
    const cv::Rect corner(source.cols - 640, source.rows - 640, 640, 640);
    cv::cvtColor(source(corner), expectedRgb, cv::COLOR_BGR2RGB);
    check(cv::norm(scara::Vision_prepare(source, corner.tl()), expectedRgb, cv::NORM_INF)==0,
          "Direct crop at the bottom-right corner changed pixels");

    std::string packet;

    scara::readBytes(fixture+"rknn-output.bin", packet);
    uint32_t headerSize;
    std::memcpy(&headerSize, packet.data(), sizeof(headerSize));
    auto meta = cv::FileStorage(packet.substr(4,headerSize), cv::FileStorage::READ|cv::FileStorage::MEMORY);
    scara::YoloHead heads[3];
    size_t offset = 4+headerSize;
    for (int i=0; i<3; ++i)
    {
      int size = 80 >> i;
      size_t bytes = size_t(size)*size*57;
      auto attr = meta["outputs"][i];
      check(offset+bytes<=packet.size(), "Recorded tensor is incomplete");
      heads[i] = {reinterpret_cast<const int8_t *>(packet.data()+offset), size_t(size)*57,
                  int(attr["zero_point"]), float(attr["scale"])};
      offset += bytes;
    }
    auto pieces = LegacyLetterbox(historical).decode(heads);
    check(pieces.size()==expected["detections"].size(), "Detection count changed");
    double maximum = 0;
    for (size_t i=0; i<pieces.size(); ++i)
    {
      auto old = expected["detections"][int(i)];
      auto box = old["bbox"];
      check(pieces[i].classId==int(old["class_id"]), "Class or equal-score order changed");
      check(std::abs(pieces[i].confidence-double(old["score"]))<1e-7, "Score changed");
      const auto& b = pieces[i].bbox;
      maximum = std::max({maximum,std::abs(b.x-double(box[0])),std::abs(b.y-double(box[1])),
                          std::abs(b.x+b.width-double(box[2])),std::abs(b.y+b.height-double(box[3]))});
    }
    check(maximum<.0001, "Original-image bounding boxes changed");
    auto originalSource = source.clone();
    cv::Mat preview = scara::Vision_draw(source,pieces);
    check(cv::norm(source,originalSource,cv::NORM_INF)==0, "Drawing modified the source image");
    check(preview.size()==source.size(), "Drawing changed the display field of view");
    cv::Mat drawing = scara::Vision_draw(source,{});
    check(cv::norm(drawing, source, cv::NORM_INF)==0, "Empty detection altered frame");
    drawing = scara::Vision_draw(source,{{0,.9,{-100,-100,10,10}}});
    check(cv::norm(drawing,source,cv::NORM_INF)==0,
          "Off-image detection drew a false label at the image origin");
    cv::Mat tiny(12,20,CV_8UC3,cv::Scalar::all(0));
    drawing = scara::Vision_draw(tiny,{{12,.9,{0,0,5,5}}});
    check(drawing.size()==tiny.size(),
          "Clipped Chinese label changed the image size");
    const char *names[] = {"黑将","黑士","黑象","黑马","黑车","黑炮","黑卒","红帅","红仕","红相","红马","红车","红炮","红兵"};
    cv::FontFace font("uni");
    cv::Mat blank(120,160,CV_8UC3,cv::Scalar::all(0));
    std::vector<cv::Mat> masks;
    for (int id=0; id<scara::YoloClasses; ++id)
    {
        drawing = scara::Vision_draw(blank,{{id,.93,{30,40,50,45}}});
        cv::Mat label=drawing(cv::Rect(30,15,44,24)), expected=label.clone();
        expected.setTo(id<7 ? cv::Scalar(255,180,65) : cv::Scalar(65,85,245));
        cv::putText(expected,names[id],{2,19},cv::Scalar::all(255),font,18);
        check(cv::norm(label,expected,cv::NORM_INF)==0,"Incorrect Chinese class name");
        cv::Mat white;
        cv::inRange(label,cv::Scalar::all(255),cv::Scalar::all(255),white);
        check(cv::countNonZero(white)>0,"Missing Chinese class text");
        for (const auto &mask : masks)
            check(cv::norm(white,mask,cv::NORM_INF)>0,"Two Chinese class names rendered as the same glyphs");
        masks.push_back(white);
    }
    std::filesystem::create_directories("output/vision-stage-verify");
    std::vector<uint8_t> encoded;
    cv::imencode(".png", preview, encoded);
    scara::writeBytes("output/vision-stage-verify/preview.png", {encoded.begin(),encoded.end()});

    check(argc==2, "Camera test needs the local stream fixture");
    for (int i=0; i<2; ++i)
    {
      scara::Camera_open(argv[1]);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      cv::Mat first = scara::Camera_read();
      auto saved = first.clone();
      cv::Mat next = scara::Camera_read();
      check(first.size()==source.size() && next.size()==source.size(), "Camera frame size changed");
      check(cv::norm(first,next,cv::NORM_INF)>10, "Camera returned a cached frame again");
      std::this_thread::sleep_for(std::chrono::milliseconds(1200));
      cv::Mat resumed = scara::Camera_read();
      check(resumed.size()==source.size(), "Camera did not resume with a new frame");
      check(cv::norm(first,saved,cv::NORM_INF)==0, "Another read changed a frame still held by the caller");
      scara::Camera_close();
    }
    cv::Mat frame;
    // 两个源都在 12 帧后结束，/stall 在断开前再等待 15 秒。
    for (const char *path : {"/finite", "/stall"})
    {
      scara::Camera_open(std::string(argv[1]) + path);
      int frames = 0;
      while (frames < 20)
      {
        frame = scara::Camera_read();
        if (frame.empty()) break;
        ++frames;
      }
      check(frames > 0 && frame.empty(), "End of stream returned a stale frame");
      scara::Camera_close();
    }
    scara::Camera_open(argv[1]);
    frame = scara::Camera_read();
    check(frame.size()==source.size(), "Camera retained an old failure after reopening");
    scara::Camera_close();
    std::cout << "PASS preprocessing byte equality, " << pieces.size()
              << " recorded NPU detections, max box difference " << maximum
              << " px, display size, camera reopen\n";
    return 0;
  }
  catch (const std::exception& e)
  {
    scara::Camera_close();
    std::cerr << e.what() << '\n';
    return 1;
  }
}
