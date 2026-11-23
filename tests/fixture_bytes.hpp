#pragma once
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

namespace scara
{
inline void readBytes(const std::string &path, std::string &out)
{
    std::ifstream file(std::filesystem::u8path(path), std::ios::binary);
    if (!file) throw std::runtime_error("Cannot read " + path);
    out.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
}
inline void writeBytes(const std::string &path, const std::string &bytes)
{
    std::ofstream file(std::filesystem::u8path(path), std::ios::binary);
    file.write(bytes.data(), std::streamsize(bytes.size()));
    file.close();
    if (!file) throw std::runtime_error("Cannot write " + path);
}
}
