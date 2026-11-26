#include "site.hpp"


//---------对局启动区---------
int main(int, char **argv)
{
    using namespace scara;
    StaticParams staticParams;

    // 从本机 TOML 读取安装参数。
    Site_loadFile(staticParams);
    DynamicParams dynamicParams;

    int exitcode = Match_run(staticParams, dynamicParams, argv[1]);

    return exitcode;
}
