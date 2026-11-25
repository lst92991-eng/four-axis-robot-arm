#include "site.hpp"
#include <iostream>
#include <stdexcept>

//---------对局启动区---------
int main(int argc, char **argv)
{
    using namespace scara;
    if (argc != 2) return 2; // 唯一参数是 RKNN 模型路径。
    GameConfig config;
    Session session;

    // 安装参数完整后才开始连接设备。
    try
    {
        Site_loadFile(config, session);
        if (config.cameraUrl.empty()) throw std::runtime_error("缺少 camera.url");
    }
    catch (const std::exception &error)
    {
        std::cerr << "安装配置 " << error.what() << '\n';
        return 2;
    }
    return Match_run(config, session, argv[1]);
}
