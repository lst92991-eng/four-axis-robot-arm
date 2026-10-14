#pragma once

namespace robot {

// 本阶段仅建立接口；关闭、发送、接收在后续提交逐项加入。
class Can {
public:
    explicit Can(const char* interface);

private:
    int fd_ = -1;
};

} // namespace robot
