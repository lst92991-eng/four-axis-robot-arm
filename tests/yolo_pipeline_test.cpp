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

void sameInput(const cv::Mat& source, const scara::VisionConfig& config, const std::string& expected)
{
  scara::ImageMapping mapping;
  auto input = scara::Vision_prepare(source, config, mapping);
  cv::Mat rgb;
  cv::cvtColor(image(expected), rgb, cv::COLOR_BGR2RGB);
  check(cv::norm(input, rgb, cv::NORM_INF)==0, "Preprocessing differs from the verified Python path");
}

void scoreUpperBound()
{
  std::vector<int8_t> data[3];
  scara::YoloHead heads[3];
  for (int i=0; i<3; ++i)
  {
    int size = 80 >> i;
    data[i].resize(size*size*57, 0);
    heads[i] = {data[i].data(), data[i].size(), size_t(size)*57, 0, .01f};
  }
  auto *p = data[0].data()+(20*80+20)*57;
  p[0] = p[1] = p[2] = p[3] = 50;
  p[4] = 40; p[5+8] = 126; // 0.40 * 1.26 = 0.504, above the 0.50 threshold.
  auto found = scara::Vision_decode(heads, {{0,0,640,640},1,0,0}, {});
  check(found.size()==1 && found[0].classId==8, "Early rejection assumed class scores cannot exceed one");
  p[4] = 39;
  check(scara::Vision_decode(heads, {{0,0,640,640},1,0,0}, {}).empty(), "Below-threshold candidate survived");
}
}

int main(int argc, char **argv)
{
  try
  {
    scoreUpperBound();
    const std::string fixture = "tests/fixtures/vision/";
    auto source = image(fixture+"source.png");
    auto expected = scara::readStorage(fixture+"expected.json");
    auto roi = expected["roi"];
    scara::VisionConfig historical;
    historical.inferenceRoi = {int(roi[0]),int(roi[1]),int(roi[2]),int(roi[3])};
    sameInput(source, historical, fixture+"historical-input.png");
    sameInput(source, scara::VisionConfig{}, fixture+"fixed-input.png");

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
      heads[i] = {reinterpret_cast<const int8_t *>(packet.data()+offset), bytes, size_t(size)*57,
                  int(attr["zero_point"]), float(attr["scale"])};
      offset += bytes;
    }
    scara::ImageMapping mapping;
    scara::Vision_prepare(source, historical, mapping);
    auto pieces = scara::Vision_decode(heads, mapping, historical);
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
    auto preview = scara::Vision_draw(source, pieces, labels);
    check(preview.size()==source.size(), "Drawing changed the display field of view");
    check(cv::norm(scara::Vision_draw(source, {}, labels), source, cv::NORM_INF)==0, "Empty detection altered frame");
    check(cv::norm(scara::Vision_draw(source, {{0,.9,{-100,-100,10,10}}}, labels),source,cv::NORM_INF)==0,
          "Off-image detection drew a false label at the image origin");
    cv::Mat tiny(12,20,CV_8UC3,cv::Scalar::all(0));
    check(scara::Vision_draw(tiny, {{12,.9,{0,0,5,5}}}, labels).size()==tiny.size(),
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
      auto first = scara::Camera_read();
      auto saved = first.clone();
      auto next = scara::Camera_read();
      check(first.size()==source.size() && next.size()==source.size(), "Camera frame size changed");
      check(cv::norm(first,next,cv::NORM_INF)>10, "Camera returned a cached frame again");
      std::this_thread::sleep_for(std::chrono::milliseconds(1200));
      check(scara::Camera_read().size()==source.size(), "Camera did not resume with a new frame");
      check(cv::norm(first,saved,cv::NORM_INF)==0, "Background capture modified a frame still held by the caller");
      scara::Camera_close();
    }
    scara::Camera_open(std::string(argv[1])+"/finite");
    bool ended = false;
    for (int i=0; i<20 && !ended; ++i)
    {
      try { scara::Camera_read(); }
      catch (const std::exception& e) { ended = std::string(e.what())=="Camera stream ended"; }
    }
    scara::Camera_close();
    check(ended, "End of stream did not stop fresh-frame reads");
    scara::Camera_open(argv[1]);
    check(scara::Camera_read().size()==source.size(), "Camera retained an old failure after reopening");
    scara::Camera_close();
    bool rejected = false;
    try { scara::Vision_prepare(source(cv::Rect(0,0,320,240)), historical, mapping); }
    catch (const std::exception&) { rejected = true; }
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
