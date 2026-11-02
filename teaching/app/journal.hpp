#pragma once
#include "chess/board.hpp"
#include <array>
#include <string>

namespace app {
struct Journal {
    std::string baseFen, history, phase = "ready", pendingMove;
    std::array<int, 2> captured{{0, 0}};
    int nextStep = 0, result = 0;
    bool carrying = false;
};
Journal loadJournal(const std::string& path);
void saveJournal(const std::string& path, const Journal& journal);
void committed(Journal& journal, const chess::Game& game);
} // namespace app
