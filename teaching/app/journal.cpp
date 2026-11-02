#include "journal.hpp"
#include <opencv2/core.hpp>
#include <filesystem>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>

namespace app {
Journal loadJournal(const std::string& path) {
    Journal j;
    if (!std::filesystem::exists(path)) return j;
    if (std::filesystem::file_size(path) > 65536) throw std::runtime_error("恢复记录超过大小限制");
    cv::FileStorage f(path, cv::FileStorage::READ);
    if (!f.isOpened()) throw std::runtime_error("恢复记录无法读取");
    f["base_fen"] >> j.baseFen; f["history"] >> j.history;
    f["phase"] >> j.phase; f["pending_move"] >> j.pendingMove;
    f["next_step"] >> j.nextStep; j.carrying = int(f["carrying"]) != 0;
    f["result"] >> j.result;
    f["captured_red"] >> j.captured[0]; f["captured_black"] >> j.captured[1];
    if (j.baseFen.empty() || j.history.size() > 10000 || j.nextStep < 0 || j.result < 0 || j.result > 4 ||
        j.captured[0] < 0 || j.captured[0] > 15 || j.captured[1] < 0 || j.captured[1] > 15 ||
        (j.phase != "ready" && j.phase != "moving" && j.phase != "verify" && j.phase != "fault"))
        throw std::runtime_error("恢复记录字段损坏；保留原文件供核对");
    return j;
}
void saveJournal(const std::string& path, const Journal& j) {
    const std::filesystem::path destination(path);
    if (!destination.parent_path().empty()) std::filesystem::create_directories(destination.parent_path());
    const std::string temporary = path + ".tmp";
    cv::FileStorage f(temporary, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);
    if (!f.isOpened()) throw std::runtime_error("无法写入对局检查点，停止推进");
    f << "base_fen" << j.baseFen << "history" << j.history << "phase" << j.phase
      << "pending_move" << j.pendingMove << "next_step" << j.nextStep << "carrying" << int(j.carrying)
      << "captured_red" << j.captured[0] << "captured_black" << j.captured[1] << "result" << j.result;
    f.release();
    const int fd = ::open(temporary.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) throw std::runtime_error("检查点落盘失败");
    const int synced = ::fsync(fd); ::close(fd);
    if (synced < 0 || ::rename(temporary.c_str(), path.c_str()) != 0)
        throw std::runtime_error("检查点提交失败，旧记录保留");
    const std::string parent = destination.parent_path().empty() ? "." : destination.parent_path().string();
    const int dir = ::open(parent.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC);
    if (dir < 0) throw std::runtime_error("检查点目录无法同步");
    const int directorySynced = ::fsync(dir); ::close(dir);
    if (directorySynced < 0) throw std::runtime_error("检查点目录同步失败");
}
void committed(Journal& j, const chess::Game& game) {
    j.baseFen = game.baseFen(); j.history = game.history();
    j.result = int(game.result());
    j.phase = "ready"; j.pendingMove.clear(); j.nextStep = 0; j.carrying = false;
}
} // namespace app
