#pragma once
#include <opencv2/core.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace scara
{
//读取二进制文件，结果写入字节字符串
inline void readBytes(const std::string &path, std::string &bytes)
{
    std::ostringstream buffer;
    std::ifstream file(std::filesystem::u8path(path), std::ios::binary);
    if (!file) throw std::runtime_error("Cannot read: " + path);
    buffer << file.rdbuf();
    bytes = buffer.str();
}
inline void writeBytes(const std::string &path, const std::string &bytes)
{
    std::ofstream file(std::filesystem::u8path(path), std::ios::binary);
    file.write(bytes.data(), std::streamsize(bytes.size()));
    if (!file) throw std::runtime_error("Cannot write: " + path);
}
inline void readStorage(const std::string &path, cv::FileStorage &file)
{
    file.open(path, cv::FileStorage::READ);
    if (!file.isOpened()) throw std::runtime_error("Invalid configuration: " + path);
}
inline void memoryStorage(bool json, cv::FileStorage &file)
{
    int mode = cv::FileStorage::WRITE | cv::FileStorage::MEMORY;
    if (json)
        mode |= cv::FileStorage::FORMAT_JSON;
    else
        mode |= cv::FileStorage::FORMAT_YAML;
    file.open("memory", mode);
}
} // namespace scara
