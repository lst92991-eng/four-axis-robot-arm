#pragma once
#include <opencv2/core.hpp>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace scara
{
//---------原始文件读写区---------
//读取二进制文件，结果写入字节字符串
inline void readBytes(const std::string &path, std::string &bytes)
{
    //字符串流收集文件原始字节，二进制模式保持文件内容
    std::ostringstream buffer;
    std::ifstream file(std::filesystem::u8path(path), std::ios::binary);
    if (!file) throw std::runtime_error("Cannot read: " + path);
    //读完文件后，一次交付调用者的bytes
    buffer << file.rdbuf();
    bytes = buffer.str();
}
//将序列化好的字节字符串写入指定文件，打开时覆盖原内容
inline void writeBytes(const std::string &path, const std::string &bytes)
{
    //按实际字节数写入，允许数据中含有0字节
    std::ofstream file(std::filesystem::u8path(path), std::ios::binary);
    file.write(bytes.data(), std::streamsize(bytes.size()));
    //打开或写入失败时向外报告，避免把失败写入当成保存成功
    if (!file) throw std::runtime_error("Cannot write: " + path);
}
//---------结构化配置区---------
//打开JSON或YAML，后续通过file[字段名]读取内容
inline void readStorage(const std::string &path, cv::FileStorage &file)
{
    file.open(path, cv::FileStorage::READ);
    if (!file.isOpened()) throw std::runtime_error("Invalid configuration: " + path);
}
//在内存中组织JSON或YAML，调用者用releaseAndGetString取得完整文本
inline void memoryStorage(bool json, cv::FileStorage &file)
{
    //WRITE允许写字段，MEMORY表示本次先写到内存
    int mode = cv::FileStorage::WRITE | cv::FileStorage::MEMORY;
    //true用于坐标JSON报告，false用于YAML标定文件
    if (json)
        mode |= cv::FileStorage::FORMAT_JSON;
    else
        mode |= cv::FileStorage::FORMAT_YAML;
    //memory是内存存储的名称，磁盘写入由writeBytes完成
    file.open("memory", mode);
}
} // namespace scara
