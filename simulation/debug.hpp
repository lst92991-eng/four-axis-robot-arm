#pragma once
#include <array>
#include <cstdint>
#include <string>

// 单纯观察的回调。默认全为空，核心代码不依赖debug.cpp才能运行。
struct DebugHooks {
    void (*can)(bool, uint32_t, const uint8_t *, int) = nullptr;
    void (*motion)(double, const std::array<double, 4> &, const std::array<double, 4> &,
                   const std::array<double, 4> &, const std::array<double, 4> &) = nullptr;
    void (*event)(const char *, const std::string &) = nullptr;
    void (*placement)(const std::array<double, 2> &, const std::array<double, 2> &) = nullptr;
};
inline DebugHooks debug;
#ifdef SIM_DEBUG
void setupDebug(const std::string &directory);
void finishDebug();
#endif
