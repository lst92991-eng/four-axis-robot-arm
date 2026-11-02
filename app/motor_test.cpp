#include "robot/motor.hpp"
#include <chrono>
#include <csignal>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; } // 信号处理器不访问CAN，主循环负责停止。
int integer(const char* text) {
    std::size_t used = 0;
    int value = std::stoi(text, &used);
    if (text[used]) throw std::runtime_error("参数必须是整数");
    return value;
}
void print(const robot::MotorFeedback& state) {
    std::cout << state.angleDeg << " deg, " << state.rpm << " RPM, "
              << state.currentMa << " mA, status=" << unsigned(state.status) << '\n';
}
} // namespace

int main(int argc, char** argv) {
    if (argc != 2 && argc != 4) {
        std::cout << "用法: scara_motor_test can0 [有符号RPM 秒数]\n"
                     "仅接口参数: prepare并读取，不发运动命令。\n"
                     "指定RPM及1～34秒: 节点1正值上升/负值下降，Ctrl+C停止。\n";
        return argc == 1 ? 0 : 1;
    }
    try {
        const int rpm = argc == 4 ? integer(argv[2]) : 0;
        const int seconds = argc == 4 ? integer(argv[3]) : 0;
        if (argc == 4 && (!rpm || rpm < -3000 || rpm > 3000 || seconds < 1 || seconds > 34))
            throw std::runtime_error("RPM幅值1～3000，运行时间1～34秒；协议范围不等于已验证机械能力");
        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);
        robot::Can can(argv[1]);        // 1. 打开已配置好的500kbit/s接口。
        robot::Motor motor(can, 1);     // 2. 当前只验证过节点1。
        motor.prepare();               // 3. 静止检查，临时设置500ms失联停机。
        print(motor.read());
        if (argc == 2 || interrupted) return 0;

        try {
            motor.start(rpm);           // 4. F6只发送一次。
            const auto end = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
            while (!interrupted && std::chrono::steady_clock::now() < end) {
                print(motor.read());    // 5. 读取反馈同时维持驱动心跳。
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } catch (...) {
            try { motor.stop(); }
            catch (const std::exception& error) { std::cerr << "停止未确认: " << error.what() << '\n'; }
            throw;
        }
        motor.stop();                  // 6. FE停止并确认连续零转速，不松轴。
        std::cout << "已确认停止\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
