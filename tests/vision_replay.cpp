// Offline comparison tool. Recorded tensors are replayed; no NPU or motor is opened.
#include "vision/yolo.hpp"
#include "vision/io.hpp"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

int main(int argc, char **argv)
{
  try
  {
    if (argc!=3) throw std::runtime_error("Usage: replay manifest.json output-directory");
    cv::FileStorage manifest;
    scara::readStorage(argv[1], manifest);
    std::filesystem::create_directories(std::filesystem::u8path(argv[2]));
    cv::setNumThreads(1);
    std::string labelBytes;
    scara::readBytes("config/vision/chinese-labels.png", labelBytes);
    auto labels = cv::imdecode(std::vector<uint8_t>(labelBytes.begin(),labelBytes.end()), cv::IMREAD_GRAYSCALE);
    if (labels.cols!=scara::AtlasWidth || labels.rows!=scara::YoloClasses*scara::LabelHeight)
      throw std::runtime_error("Invalid label atlas");
    for (auto item : manifest["cases"])
    {
      std::string id = item["id"], imagePath = item["image"], packetPath = item["packet"];
      std::string prefix = std::string(argv[2])+"/"+id;
      std::string bytes;
      scara::readBytes(imagePath, bytes);
      auto source = cv::imdecode(std::vector<uint8_t>(bytes.begin(),bytes.end()), cv::IMREAD_COLOR);
      if (source.empty()) throw std::runtime_error("Cannot decode replay image");
      auto roi = item["roi"];
      cv::Rect region = {int(roi[0]),int(roi[1]),int(roi[2]),int(roi[3])};
      scara::Vision_setup(source.size(),region);
      cv::Mat input;
      scara::Vision_prepare(source,input);
      scara::writeBytes(prefix+".rgb", {reinterpret_cast<const char *>(input.data), input.total()*input.elemSize()});
      std::vector<scara::Detection> detections;
      double decodeMs = 0;
      if (!packetPath.empty())
      {
        std::string packet;
        scara::readBytes(packetPath, packet);
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
          auto dims = attr["dims"];
          float scale = attr["scale"];
          int widthStride = attr["w_stride"];
          if (dims.size()!=4 || int(dims[0])!=1 || int(dims[1])!=size || int(dims[2])!=size || int(dims[3])!=57 ||
              int(attr["type"])!=2 || int(attr["fmt"])!=1 || (widthStride!=0 && widthStride!=size) ||
              !std::isfinite(scale) || scale<=0)
            throw std::runtime_error("Unexpected recorded tensor layout");
          size_t rowBytes = size*57, length = rowBytes*size;
          if (offset+length>packet.size()) throw std::runtime_error("Truncated tensor data");
          heads[i] = {reinterpret_cast<const int8_t *>(packet.data()+offset), rowBytes,
                      int(attr["zero_point"]), scale};
          offset += length;
        }
        if (offset!=packet.size()) throw std::runtime_error("Unexpected tensor tail");
        scara::Vision_decode(heads,detections);
        auto start = std::chrono::steady_clock::now();
        for (int i=0; i<100; ++i) scara::Vision_decode(heads,detections);
        decodeMs = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/100;
      }
      cv::FileStorage result;
      scara::memoryStorage(true, result);
      result << "id" << id << "decode_ms" << decodeMs << "detections" << "[";
      for (const auto& d : detections)
        result << "{" << "class_id" << d.classId << "score" << d.confidence << "bbox" << "["
               << d.bbox.x << d.bbox.y << d.bbox.x+d.bbox.width << d.bbox.y+d.bbox.height << "]" << "}";
      result << "]";
      scara::writeBytes(prefix+".json", result.releaseAndGetString());
      cv::Mat preview;
      scara::Vision_draw(source,detections,labels,preview);
      std::vector<uint8_t> png;
      cv::imencode(".png", preview, png);
      scara::writeBytes(prefix+".png", {png.begin(),png.end()});
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
