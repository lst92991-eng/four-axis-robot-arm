#pragma once
#include <cerrno>
#include <deque>
#include <fcntl.h>
#include <string>
#include <unistd.h>

namespace app {

// 单线程命令泵：等待相机/引擎时也可收集控制命令，不在回调里执行走棋。
// fd是借用的输入描述符；退出时恢复原flags，不关闭调用者的描述符。
class InputCommands {
public:
    explicit InputCommands(int fd = STDIN_FILENO) : fd_(fd), flags_(fcntl(fd, F_GETFL))
    {
        if (flags_ < 0 || fcntl(fd_, F_SETFL, flags_ | O_NONBLOCK) < 0) ended_ = true;
    }
    ~InputCommands() { if (flags_ >= 0) fcntl(fd_, F_SETFL, flags_); }
    InputCommands(const InputCommands&) = delete;
    InputCommands& operator=(const InputCommands&) = delete;

    void poll()
    {
        if (ended_) return;
        char character;
        while (1) {
            const ssize_t count = read(fd_, &character, 1);
            if (count < 0 && errno == EINTR) continue;
            if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return;
            if (count <= 0) {
                ended_ = true;
                break;
            }
            if (character == '\r') continue;
            if (character == '\n') {
                if (lines_.size() == 8) { ended_ = true; break; }
                lines_.push_back(partial_);
                partial_.clear();
            } else {
                if (partial_.size() == 80) { ended_ = true; break; }
                partial_ += character;
            }
        }
        // EOF、读错和超限都停止输入，不能在连接失效后继续执行积压走法。
        lines_.clear();
        partial_.clear();
    }

    bool pop(std::string& line)
    {
        if (lines_.empty()) return false;
        // 控制命令优先，取消它之前尚未执行的普通命令；后面的新输入仍保留。
        for (auto next = lines_.begin(); next != lines_.end(); ++next) {
            if (*next == "pause" || *next == "new" || *next == "quit") {
                lines_.erase(lines_.begin(), next);
                break;
            }
        }
        line = lines_.front();
        lines_.pop_front();
        return true;
    }
    bool eof() const { return ended_; }
    bool cancelled() const
    {
        if (ended_) return true;
        for (const auto& line : lines_) {
            if (line == "pause" || line == "new" || line == "quit") return true;
        }
        return false;
    }

private:
    int fd_, flags_;
    bool ended_ = false;
    std::string partial_;
    std::deque<std::string> lines_;
};

} // namespace app
