#include "vision/yolo.hpp"
#include "vision/capture.hpp"
#include "vision/io.hpp"
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
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
  auto bytes = scara::readBytes(path);
  return cv::imdecode(std::vector<uint8_t>(bytes.begin(),bytes.end()), mode);
}

void sameInput(const cv::Mat& source, cv::Rect roi, const std::string& expected)
{
  scara::Vision_setup(source.size(),roi);
  cv::Mat input;
  scara::Vision_prepare(source,input);
  cv::Mat rgb;
  cv::cvtColor(image(expected), rgb, cv::COLOR_BGR2RGB);
  check(cv::norm(input, rgb, cv::NORM_INF)==0, "Preprocessing differs from the verified Python path");
}

void decodeEdges()
{
  scara::Vision_setup({640,640},{0,0,640,640});
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
  std::vector<scara::Detection> found;
  scara::Vision_decode(heads,found);
  check(found.size()==1 && found[0].classId==8, "Early rejection assumed class scores cannot exceed one");
  p[5+2] = 126;
  scara::Vision_decode(heads,found);
  check(found.size()==1 && found[0].classId==2, "Equal class scores did not keep the first class");
  p[5+2] = 0;
  p[4] = 39;
  scara::Vision_decode(heads,found);
  check(found.empty(), "Below-threshold candidate survived");
  p[2] = p[3] = p[4] = 100; p[5+8] = 0; p[5+2] = 90;
  auto *q = p+19;
  q[0] = q[1] = 50; q[2] = 80; q[3] = 66; q[4] = 100; q[5+5] = 90;
  scara::Vision_decode(heads,found);
  check(found.size()==1 && found[0].classId==2, "Cross-class NMS or equal-score traversal order changed");
  q[5+5] = 91;
  scara::Vision_decode(heads,found);
  check(found.size()==1 && found[0].classId==5, "NMS did not keep the highest scoring overlapping box");
  q[4] = 0; p[5+2] = 50;
  scara::Vision_decode(heads,found);
  check(found.size()==1, "Score exactly 0.50 was rejected");
}
}

int main(int argc, char **argv)
{
  try
  {
    decodeEdges();
    const std::string fixture = "tests/fixtures/vision/";
    auto source = image(fixture+"source.png");
    auto expected = scara::readStorage(fixture+"expected.json");
    auto roi = expected["roi"];
    cv::Rect historical = {int(roi[0]),int(roi[1]),int(roi[2]),int(roi[3])};
    sameInput(source, historical, fixture+"historical-input.png");
    sameInput(source, {300,70,650,600}, fixture+"fixed-input.png");

    auto packet = scara::readBytes(fixture+"rknn-output.bin");
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
    scara::Vision_setup(source.size(),historical);
    std::vector<scara::Detection> pieces;
    scara::Vision_decode(heads,pieces);
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
    auto labels = image("config/vision/chinese-labels.png", cv::IMREAD_GRAYSCALE);
    cv::Mat preview, drawing;
    auto originalSource = source.clone();
    scara::Vision_draw(source,pieces,labels,preview);
    check(cv::norm(source,originalSource,cv::NORM_INF)==0, "Drawing modified the source image");
    check(preview.size()==source.size(), "Drawing changed the display field of view");
    scara::Vision_draw(source,{},labels,drawing);
    check(cv::norm(drawing, source, cv::NORM_INF)==0, "Empty detection altered frame");
    scara::Vision_draw(source,{{0,.9,{-100,-100,10,10}}},labels,drawing);
    check(cv::norm(drawing,source,cv::NORM_INF)==0,
          "Off-image detection drew a false label at the image origin");
    cv::Mat tiny(12,20,CV_8UC3,cv::Scalar::all(0));
    scara::Vision_draw(tiny,{{12,.9,{0,0,5,5}}},labels,drawing);
    check(drawing.size()==tiny.size(),
          "Clipped Chinese label changed the image size");
    std::filesystem::create_directories("output/vision-stage-verify");
    std::vector<uint8_t> encoded;
    cv::imencode(".png", preview, encoded);
    scara::writeBytes("output/vision-stage-verify/preview.png", {encoded.begin(),encoded.end()});

    check(argc==2, "Camera test needs the local stream fixture");
    for (int i=0; i<2; ++i)
    {
      scara::Camera_open(argv[1]);
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
      cv::Mat first, next, resumed;
      scara::Camera_read(first);
      auto saved = first.clone();
      scara::Camera_read(next);
      check(first.size()==source.size() && next.size()==source.size(), "Camera frame size changed");
      check(cv::norm(first,next,cv::NORM_INF)>10, "Camera returned a cached frame again");
      std::this_thread::sleep_for(std::chrono::milliseconds(1200));
      scara::Camera_read(resumed);
      check(resumed.size()==source.size(), "Camera did not resume with a new frame");
      check(cv::norm(first,saved,cv::NORM_INF)==0, "Background capture modified a frame still held by the caller");
      scara::Camera_close();
    }
    scara::Camera_open(std::string(argv[1])+"/finite");
    cv::Mat frame;
    bool ended = false;
    try
    {
      for (int i=0; i<20; ++i) scara::Camera_read(frame);
    }
    catch (const std::runtime_error&) { ended = true; }
    scara::Camera_close();
    check(ended && frame.empty(), "End of stream did not clear the output frame");
    scara::Camera_open(std::string(argv[1])+"/stall");
    auto waiting = std::chrono::steady_clock::now();
    ended = false;
    try
    {
      for (int i=0; i<20; ++i) scara::Camera_read(frame);
    }
    catch (const std::runtime_error&) { ended = true; }
    double waitSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now()-waiting).count();
    scara::Camera_close();
    check(ended && frame.empty() && waitSeconds<8, "Stalled stream left the consumer waiting indefinitely");
    std::cout << "camera_stall_wait_s=" << waitSeconds << '\n';
    scara::Camera_open(argv[1]);
    scara::Camera_read(frame);
    check(frame.size()==source.size(), "Camera retained an old failure after reopening");
    scara::Camera_close();
    cv::Mat badInput;
    bool rejected = false;
    try { scara::Vision_prepare(source(cv::Rect(0,0,320,240)),badInput); }
    catch (const std::runtime_error&) { rejected = true; }
    check(rejected, "Unexpected camera size was accepted");
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
