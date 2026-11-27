#include "rv1106.hpp"
#include "site.hpp"
#include <exception>
#include <iostream>

//---------对局启动区---------
int main(int argc, char **argv)
{
    using namespace scara;
    if (argc < 2)
    {
        std::cerr << "请提供 RKNN 模型路径。\n";
        return 2;
    }

    StaticParams site;
    DynamicParams match;
    try
    {
        Site_loadFile(site);
        RV1106_configure(argv[1], site.visionCropOrigin, site.engineDir,
                         site.skillLevel, site.thinkMs, site.motion);
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return Match_run(site, match);
}
