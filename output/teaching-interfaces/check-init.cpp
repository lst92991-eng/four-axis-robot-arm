// 本次构造接口的离线验证材料，不是教学项目的第二个运行入口。
#include "teaching/robot/can.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/socket.h>
#include <dirent.h>
#include <poll.h>
#include <unistd.h>
#include <cassert>
#include <iostream>
#include <stdexcept>

int descriptors()
{
    DIR* directory = opendir("/proc/self/fd");
    assert(directory);
    int count = 0;
    while (auto* entry = readdir(directory))
        if (entry->d_name[0] != '.') ++count;
    closedir(directory);
    return count;
}

int main()
{
    int before = descriptors();
    for (const char* name : {"missing_can", "lo"}) {
        for (int i = 0; i < 20; ++i) {
            bool failed = false;
            try { robot::Can can(name); }
            catch (const std::runtime_error&) { failed = true; }
            assert(failed && descriptors() == before);
        }
    }

    int observer = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    assert(observer >= 0);
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = if_nametoindex("vcan0");
    assert(address.can_ifindex != 0);
    assert(bind(observer, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
    before = descriptors();
    robot::Can can("vcan0");
    assert(descriptors() == before + 1);
    pollfd frame{observer, POLLIN, 0};
    assert(poll(&frame, 1, 30) == 0);
    close(observer);
    std::cout << "PASS: init, lookup/bind failure cleanup, no transmitted frame\n";
    // Can关闭接口尚未进入本教学提交；成功资源由本测试进程退出回收。
}
