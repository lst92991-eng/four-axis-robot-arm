#include "chess/pikafish.hpp"
#include <chrono>
#include <iostream>
#include <thread>
int main() {
    Pikafish engine;
    if (!engine.start("engine")) { std::cerr << engine.error; return 2; }
    int renders=0;
    engine.cancelled=[&] {
        static auto nextFrame=std::chrono::steady_clock::time_point{};
        if (std::chrono::steady_clock::now()>=nextFrame) {
            ++renders;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            nextFrame=std::chrono::steady_clock::now()+std::chrono::milliseconds(50);
        }
        return false;
    };
    auto start=std::chrono::steady_clock::now();
    std::vector<std::string> moves;
    bool ok=engine.legalMoves("1rbakabr1/9/n5n2/p3p1C1p/9/2P2CP2/P3c3P/2c1B4/2R1A4/4KABNR b - - 0 1","",moves);
    std::cout << "ok=" << ok << " legal=" << moves.size() << " renders=" << renders
              << " seconds=" << std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count()
              << " error=" << engine.error << '\n';
    if (ok) {
        std::string best;
        ok=engine.think("1rbakabr1/9/n5n2/p3p1C1p/9/2P2CP2/P3c3P/2c1B4/2R1A4/4KABNR b - - 0 1","",100,best);
        std::cout << "think=" << ok << " best=" << best << " renders=" << renders
                  << " error=" << engine.error << '\n';
    }
    return ok ? 0 : 1;
}
