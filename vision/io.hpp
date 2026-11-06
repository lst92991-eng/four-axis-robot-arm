#pragma once
#include <opencv2/core.hpp>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

namespace scara {
// std::filesystem handles Windows Unicode paths; OpenCV's narrow fopen does not always do so.
inline std::string readBytes(const std::string& path) {
    std::ifstream file(std::filesystem::u8path(path),std::ios::binary);
    if(!file) throw std::runtime_error("Cannot read: "+path);
    return {std::istreambuf_iterator<char>(file),std::istreambuf_iterator<char>()};
}
inline void writeBytes(const std::string& path,const std::string& bytes) {
    std::ofstream file(std::filesystem::u8path(path),std::ios::binary);
    if(!file || !file.write(bytes.data(),static_cast<std::streamsize>(bytes.size())))
        throw std::runtime_error("Cannot write: "+path);
}
inline cv::FileStorage readStorage(const std::string& path) {
    cv::FileStorage fs(readBytes(path),cv::FileStorage::READ|cv::FileStorage::MEMORY);
    if(!fs.isOpened()) throw std::runtime_error("Invalid configuration: "+path); return fs;
}
inline cv::FileStorage memoryStorage(bool json=false) {
    return cv::FileStorage("memory",cv::FileStorage::WRITE|cv::FileStorage::MEMORY|
        (json?cv::FileStorage::FORMAT_JSON:cv::FileStorage::FORMAT_YAML));
}
}
