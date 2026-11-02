#include "app/config.hpp"
#include "app/journal.hpp"
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <unistd.h>

namespace {
void checkApp(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
template<class F> void rejects(F action) {
    bool failed = false;
    try { action(); } catch (const std::exception&) { failed = true; }
    checkApp(failed,"invalid application input was accepted");
}
}
void testApp() {
    const auto example = app::loadConfig(std::string(SCARA_SOURCE_DIR) + "/config/example.json");
    checkApp(!example.hardwareVerified && !example.motion.calibrated &&
             !example.vision.boardCalibrated,"example must never claim real calibration");
    rejects([&] { app::validateConfig(example); });
    char pattern[] = "/tmp/scara-teaching-app-XXXXXX";
    const char* folder = mkdtemp(pattern);
    checkApp(folder != nullptr,"create journal test directory");
    const std::filesystem::path directory(folder);
    struct Cleanup { std::filesystem::path directory; ~Cleanup() { std::filesystem::remove_all(directory); } } cleanup{directory};
    const auto file = (directory / "state.yml").string();
    checkApp(app::loadJournal(file).baseFen.empty(),"first launch has no fabricated history");
    chess::Game game;
    app::Journal journal; app::committed(journal,game);
    app::saveJournal(file,journal);
    auto restored = app::loadJournal(file);
    checkApp(restored.baseFen == game.baseFen() && restored.history.empty() &&
             restored.phase == "ready", "committed checkpoint round trip");
    journal.phase = "moving"; journal.pendingMove = "e3e4"; journal.nextStep = 4; journal.carrying = true;
    app::saveJournal(file,journal);
    restored = app::loadJournal(file);
    checkApp(restored.phase == "moving" && restored.nextStep == 4 && restored.carrying &&
             restored.history.empty(),"in-flight intent must not become a committed move");
    checkApp(game.agreeDraw(),"end test game"); app::committed(journal,game); app::saveJournal(file,journal);
    checkApp(app::loadJournal(file).result == int(chess::Result::Draw),"agreed draw survives restart");
    { std::ofstream corrupt(file); corrupt << "not a checkpoint"; }
    rejects([&] { app::loadJournal(file); });
}
