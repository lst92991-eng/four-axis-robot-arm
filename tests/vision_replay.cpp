// Offline comparison tool. Recorded tensors are replayed; no NPU or motor is opened.
#include "vision/yolo.hpp"
#include "vision/io.hpp"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cstring>
#include <iostream>

int main(int argc, char **argv)
{
  try
  {
    if (argc!=3) throw std::runtime_error("Usage: replay manifest.json output-directory");
    auto manifest = scara::readStorage(argv[1]);
    std::filesystem::create_directories(std::filesystem::u8path(argv[2]));
    cv::setNumThreads(1);
    for (auto item : manifest["cases"])
    {
      std::string id = item["id"], imagePath = item["image"], packetPath = item["packet"];
      std::string prefix = std::string(argv[2])+"/"+id;
      auto bytes = scara::readBytes(imagePath);
      auto source = cv::imdecode(std::vector<uint8_t>(bytes.begin(),bytes.end()), cv::IMREAD_COLOR);
      if (source.empty()) throw std::runtime_error("Cannot decode replay image");
      scara::VisionConfig config;
      config.frameSize = source.size();
      auto roi = item["roi"];
      config.inferenceRoi = {int(roi[0]),int(roi[1]),int(roi[2]),int(roi[3])};
      scara::ImageMapping mapping;
      auto input = scara::Vision_prepare(source, config, mapping);
      scara::writeBytes(prefix+".rgb", {reinterpret_cast<const char *>(input.data), input.total()*input.elemSize()});
      std::vector<scara::Detection> detections;
      double decodeMs = 0;
      if (!packetPath.empty())
      {
        auto packet = scara::readBytes(packetPath);
        if (packet.size()<4) throw std::runtime_error("Missing tensor header");
        uint32_t headerSize;
        std::memcpy(&headerSize, packet.data(), 4);
        if (headerSize>packet.size()-4) throw std::runtime_error("Truncated tensor header");
        cv::FileStorage meta(packet.substr(4,headerSize), cv::FileStorage::READ|cv::FileStorage::MEMORY);
        scara::YoloHead heads[3];
        size_t offset = 4+headerSize;
        for (int i=0; i<3; ++i)
        {
          int size = 80 >> i;
          auto attr = meta["outputs"][i];
          size_t rowBytes = size*57, length = rowBytes*size;
          if (offset+length>packet.size()) throw std::runtime_error("Truncated tensor data");
          heads[i] = {reinterpret_cast<const int8_t *>(packet.data()+offset), length, rowBytes,
                      int(attr["zero_point"]), float(attr["scale"])};
          offset += length;
        }
        if (offset!=packet.size()) throw std::runtime_error("Unexpected tensor tail");
        detections = scara::Vision_decode(heads, mapping, config);
        auto start = std::chrono::steady_clock::now();
        for (int i=0; i<100; ++i) scara::Vision_decode(heads, mapping, config);
        decodeMs = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/100;
      }
      auto result = scara::memoryStorage(true);
      result << "id" << id << "decode_ms" << decodeMs << "detections" << "[";
      for (const auto& d : detections)
        result << "{" << "class_id" << d.classId << "score" << d.confidence << "bbox" << "["
               << d.bbox.x << d.bbox.y << d.bbox.x+d.bbox.width << d.bbox.y+d.bbox.height << "]" << "}";
      result << "]";
      scara::writeBytes(prefix+".json", result.releaseAndGetString());
      std::cout << id << ": " << detections.size() << " detections\n";
    }
    return 0;
  }
  catch (const std::exception& e)
  {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
