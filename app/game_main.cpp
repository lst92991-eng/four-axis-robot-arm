// RV1106 对局入口，从编入程序的安装表建立本局坐标。
#include <installation.hpp>

//---------对局启动区---------
int main(int argc, char **argv)
{
    using namespace scara;
    if (argc != 2) return 2; // 唯一参数是 RKNN 模型。
    const char *modelPath = argv[1];
    GameConfig config;
    Session session;

    // 先装入本机三点、机械参考和取放高度，再进入整局循环。
    Setup_game(config, session);
    return Match_run(config, session, modelPath);
}
