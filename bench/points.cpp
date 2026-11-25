// 供三点示教和 90 点巡检调用，输出坐标与关节角。
#include "bench/legacy_site.hpp"
#include "app/site.hpp"
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <iterator>

// 14/20/21 读取和对局相同的配置快照，台架直接接收换算后的角度与限幅。
static void printSetup(const scara::GameConfig &config, const scara::Session &s)
{
    const auto &arm = s.arm;
    const auto &limits = config.motion;
    cv::FileStorage out("site.json", cv::FileStorage::WRITE | cv::FileStorage::MEMORY | cv::FileStorage::FORMAT_JSON);
    out << "arm_lengths_mm" << std::vector<double>{arm.upperMm, arm.forearmMm}
        << "j2_reference_deg" << arm.shoulder.driveRefDeg << "j3_reference_motor_deg" << arm.elbow.driveRefDeg
        << "shoulder_ref_rad" << arm.shoulder.jointRefRad << "elbow_ref_rad" << arm.elbow.jointRefRad
        << "joint_per_drive" << std::vector<double>{arm.shoulder.jointPerDrive, arm.elbow.jointPerDrive}
        << "park_joints_deg" << std::vector<double>{s.park.j2Deg, s.park.j3MotorDeg}
        << "joint_velocity_deg_s" << std::vector<double>(limits.velocity.begin(), limits.velocity.end())
        << "joint_acceleration_deg_s2" << std::vector<double>(limits.acceleration.begin(), limits.acceleration.end())
        << "z" << "{" << "pickup_deg" << s.heights.pickupDeg << "travel_deg" << s.heights.travelDeg
        << "release_deg" << s.heights.releaseDeg << "clearance_deg" << s.heights.clearanceDeg << "}"
        << "pickup_at_rest_ms" << s.heights.gripDwellMs << "camera_url" << config.cameraUrl;
    std::cout << out.releaseAndGetString();
}

//---------正解采点与整盘逆解入口---------
int main(int argc, char **argv)
{
    try
    {
        std::cout << std::setprecision(15);
        if (argc == 3 && std::string_view(argv[1]) == "--travel-from-contact")
        {
            std::cout << scara::WorkHeights_fromContact(std::stod(argv[2])).travelDeg << '\n';
            return 0;
        }
        if (argc != 2 && argc != 5) return 2;
        // 命令行文件名按本机编码读入，传给 TOML 库时统一使用 UTF-8。
        std::filesystem::path path(argv[1]);
        bool toml = path.extension() == ".toml";
        // 示教模式输入 J2 反馈角和 J3 电机角，输出吸头中心的基座 XY 毫米。
        if (argc == 5)
        {
            if (std::string(argv[2]) != "--forward") return 2;
            scara::ArmGeometry arm;
            if (toml) arm = scara::Site_loadArm(path.u8string());
            else if (!scara::Arm_load(argv[1], arm)) throw std::runtime_error("缺少机械参考");
            scara::JointTarget joints{std::stod(argv[3]), std::stod(argv[4])};
            cv::Point2d xy = scara::Arm_forward(arm, joints);
            std::cout << xy.x << '\t' << xy.y << '\n';
            return 0;
        }
        // 巡点模式加载整张标定表，从第 0 行第 0 列开始逐格求逆解。
        scara::GameConfig config;
        scara::Session session;
        bool setupOnly = std::string_view(argv[1]) == "--settings";
        if (setupOnly || std::string_view(argv[1]) == "-")
        {
            std::string text{std::istreambuf_iterator<char>(std::cin), {}};
            scara::Site_parse(text, config, session);
        }
        else if (toml) scara::Site_loadFile(config, session, path.u8string());
        else scara::Bench_load(argv[1], config, session);
        if (setupOnly)
        {
            printSetup(config, session);
            return 0;
        }
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
