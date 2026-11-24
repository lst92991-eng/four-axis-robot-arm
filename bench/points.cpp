// 供三点示教和 90 点巡检调用，输出坐标与关节角。
#include "bench/legacy_site.hpp"
#include "app/site.hpp"
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <iterator>

//---------正解采点与整盘逆解入口---------
int main(int argc, char **argv)
{
    if (argc != 2 && argc != 5) return 2;
    try
    {
        std::cout << std::setprecision(15);
        bool toml = std::filesystem::u8path(argv[1]).extension() == ".toml";
        // 示教模式输入 J2 反馈角和 J3 电机角，输出吸头中心的基座 XY 毫米。
        if (argc == 5)
        {
            if (std::string(argv[2]) != "--forward") return 2;
            scara::ArmGeometry arm;
            if (toml) scara::Site_loadArm(argv[1], arm);
            else if (!scara::Arm_load(argv[1], arm)) throw std::runtime_error("缺少机械参考");
            scara::JointTarget joints{std::stod(argv[3]), std::stod(argv[4])};
            cv::Point2d xy;
            scara::Arm_forward(arm, joints, xy);
            std::cout << xy.x << '\t' << xy.y << '\n';
            return 0;
        }
        // 巡点模式加载整张标定表，从第 0 行第 0 列开始逐格求逆解。
        scara::GameConfig config;
        scara::Session session;
        if (std::string_view(argv[1]) == "-")
        {
            std::string text{std::istreambuf_iterator<char>(std::cin), {}};
            scara::Site_parse(text, config, session);
        }
        else if (toml) scara::Site_loadFile(argv[1], config, session);
        else scara::Bench_load(argv[1], config, session);
        for (int row = 0; row < scara::Rows; ++row)
            for (int col = 0; col < scara::Columns; ++col)
            {
                cv::Point2d xy = session.layout.cell[row][col].xy;
                scara::JointTarget joints;
                if (!scara::Arm_inverse(session.arm, xy, joints))
                {
                    std::cerr << "格点不可达 " << row << '\t' << col << '\n';
                    return 1;
                }
                // 每个格子输出一行，依次是行、列、X、Y、J2 角和 J3 电机角。
                std::cout << row << '\t' << col << '\t' << xy.x << '\t' << xy.y
                          << '\t' << joints.j2Deg << '\t' << joints.j3MotorDeg << '\n';
            }
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
    return 0;
}
