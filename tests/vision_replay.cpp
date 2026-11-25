// 离线回放保存的原图和 NPU 张量，用正式视觉代码对照 RGB 输入、检测结果与预览图。
#include "vision/yolo.hpp"
#include "fixture_bytes.hpp"
#include "legacy_letterbox.hpp"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

int main(int argc, char **argv)
{
  try
  {
    //---------加载回放清单区---------
    if (argc!=3) throw std::runtime_error("Usage: replay manifest.json output-directory");
    cv::FileStorage manifest;
    manifest.open(argv[1], cv::FileStorage::READ);
    std::filesystem::create_directories(std::filesystem::u8path(argv[2]));
    cv::setNumThreads(1);
    for (auto item : manifest["cases"])
    {
      std::string id = item["id"], imagePath = item["image"], packetPath = item["packet"];
      std::string prefix = std::string(argv[2])+"/"+id;
      std::string bytes;
      // 按每个样本记录的原图尺寸和 ROI 重建模型输入，再保存 RGB 字节供逐像素对照。
      scara::readBytes(imagePath, bytes);
      auto source = cv::imdecode(std::vector<uint8_t>(bytes.begin(),bytes.end()), cv::IMREAD_COLOR);
      if (source.empty()) throw std::runtime_error("Cannot decode replay image");
      auto roi = item["roi"];
      cv::Rect region = {int(roi[0]),int(roi[1]),int(roi[2]),int(roi[3])};
      bool direct = region.size() == cv::Size(scara::YoloSize, scara::YoloSize);
      cv::Mat input = direct ? scara::Vision_prepare(source, region.tl()) : LegacyLetterbox(region).input(source);
      scara::writeBytes(prefix+".rgb", {reinterpret_cast<const char *>(input.data), input.total()*input.elemSize()});
      std::vector<scara::Detection> detections;
      double decodeMs = 0;
      // 带张量包的样本继续解码，只有原图的样本仍可核对预处理结果。
      if (!packetPath.empty())
      {
        std::string packet;
        scara::readBytes(packetPath, packet);
        if (packet.size()<4) throw std::runtime_error("Missing tensor header");
        // 包首的 4 字节给出元数据长度，随后连续存放三个尺度的输出。
        uint32_t headerSize;
        std::memcpy(&headerSize, packet.data(), 4);
        if (headerSize>packet.size()-4) throw std::runtime_error("Truncated tensor header");
        cv::FileStorage meta(packet.substr(4,headerSize), cv::FileStorage::READ|cv::FileStorage::MEMORY);
        scara::YoloHead heads[3];
        size_t offset = 4+headerSize;
        //---------三路张量还原区---------
        for (int i=0; i<3; ++i)
        {
          int size = 80 >> i;
          auto attr = meta["outputs"][i];
          auto dims = attr["dims"];
          float scale = attr["scale"];
          int widthStride = attr["w_stride"];
          // 每路应为单批 NHWC、57 通道、INT8，并带有正的量化比例。
          if (dims.size()!=4 || int(dims[0])!=1 || int(dims[1])!=size || int(dims[2])!=size || int(dims[3])!=57 ||
              int(attr["type"])!=2 || int(attr["fmt"])!=1 || (widthStride!=0 && widthStride!=size) ||
              !std::isfinite(scale) || scale<=0)
            throw std::runtime_error("Unexpected recorded tensor layout");
          // 按行字节数切出这一段数据，heads 直接引用当前样本的 packet。
          size_t rowBytes = size*57, length = rowBytes*size;
          if (offset+length>packet.size()) throw std::runtime_error("Truncated tensor data");
          heads[i] = {reinterpret_cast<const int8_t *>(packet.data()+offset), rowBytes,
                      int(attr["zero_point"]), scale};
          offset += length;
        }
        if (offset!=packet.size()) throw std::runtime_error("Unexpected tensor tail");
        auto decode = [&] { return direct ? scara::Vision_decode(heads, region.tl()) : LegacyLetterbox(region).decode(heads); };
        detections = decode();
        // 在同一份数据上重复解码 100 次，只用来计算平均耗时。
        auto start = std::chrono::steady_clock::now();
        for (int i=0; i<100; ++i) detections = decode();
        decodeMs = std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/100;
      }
      //---------结果与预览导出区---------
      // 按样本保存类别、得分和原图框四条边，再用同一检测列表绘制预览。
      cv::FileStorage result;
      result.open("memory.json", cv::FileStorage::WRITE | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
      result << "id" << id << "decode_ms" << decodeMs << "detections" << "[";
      for (const auto& d : detections)
        result << "{" << "class_id" << d.classId << "score" << d.confidence << "bbox" << "["
               << d.bbox.x << d.bbox.y << d.bbox.x+d.bbox.width << d.bbox.y+d.bbox.height << "]" << "}";
      result << "]";
      scara::writeBytes(prefix+".json", result.releaseAndGetString());
      cv::Mat preview = scara::Vision_draw(source,detections);
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
