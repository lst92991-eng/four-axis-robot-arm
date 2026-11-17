//给定一对格位，检查计划或用原CAN和GPIO40完成一次搬棋
#include "motion/execute.hpp"
#include "vision/geometry.hpp"
#include "vision/io.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string_view>

namespace
{
volatile std::sig_atomic_t running = 1;

//---------配置与格位区---------
cv::Point2d savedTarget(const cv::FileNode &target)
{
    cv::FileNode xy = target["base_xy_mm"];
    if (xy.size() != 2 || !(xy[0].isReal() || xy[0].isInt()) || !(xy[1].isReal() || xy[1].isInt()))
        throw std::runtime_error("取放计划需要两个基座XY坐标");
    return {double(xy[0]), double(xy[1])};
}

//行号以红方底线为0，与Geometry_target一致
cv::Point2d gridTarget(const scara::Geometry &layout, std::string_view grid)
{
    cv::Point2d target;
    if (!scara::Geometry_target(layout, grid[0] - 'a', grid[1] - '0', target))
        throw std::runtime_error("格位必须为a0到i9");
    return target;
}

//---------停止回调区---------
void requestStop(int)
{
    running = 0;
}
}

int main(int argc, char **argv)
{
    using namespace scara;
    std::string_view mode = argc > 4 ? argv[4] : "--plan";
    if ((argc != 4 && argc != 5 && argc != 7) ||
        (mode != "--plan" && mode != "--check" && mode != "--run"))
    {
        std::cerr << "用法  scara_pick 取放计划.json arm-registration.json RV1106序列号 "
                     "[--plan|--check|--run] [layout.json c3c4]\n"
                     "只有一个ADB设备时序列号可填-，Linux板端执行可填local。\n";
        return 2;
    }
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    try
    {
        //沿用旧取放计划中的工作高度、释放参数和机械参考
        PickSetup setup;
        cv::FileStorage file;
        readStorage(argv[1], file);
        if (!Arm_load(argv[2], setup.arm) || !WorkHeights_load(argv[1], setup.heights))
            throw std::runtime_error("机械参考或工作高度无效");
        cv::FileNode z = file["z"];
        double rise = z["operator_estimated_rise_mm"].empty() ? 20 : double(z["operator_estimated_rise_mm"]);
        setup.zDegreesPerMm = std::abs(setup.heights.travelDeg - setup.heights.pickupDeg) / rise;
        if (!z["speed_deg_s"].empty()) setup.zSpeed = double(z["speed_deg_s"]);
        setup.heights.clearanceDeg = setup.heights.travelDeg;
        setup.adbSerial = argv[3];
        if (const char *serial = std::getenv("SCARA_CAN_SERIAL")) setup.canSerial = serial;

        cv::Point2d source, destination;
        if (argc == 7)
        {
            std::string_view move = argv[6];
            if (move.size() != 4) throw std::runtime_error("输入格式为c3c4");
            Geometry layout;
            cv::FileStorage points;
            readStorage(argv[5], points);
            Geometry_init(points.root(), layout);
            source = gridTarget(layout, move.substr(0, 2));
            destination = gridTarget(layout, move.substr(2, 2));
        }
        else
        {
            //同一机位首次复验可直接读取原c3/c4示教记录
            source = savedTarget(file["source"]);
            destination = savedTarget(file["destination"]);
        }
        std::vector<PickStep> steps;
        if (cv::norm(source - destination) < 1 || !Pick_plan(setup.arm, setup.heights, source, destination, false, steps))
            throw std::runtime_error("源格与目标格相同或不可达");

        std::cout << "基座XY(mm)  " << source << " -> " << destination << '\n';
        for (const PickStep &step : steps)
        {
            if (step.action == PickAction::Move)
                std::cout << "J1/J2/J3(°)  " << step.target[0] << ' ' << step.target[1] << ' ' << step.target[2] << '\n';
            if (step.action == PickAction::ReleasePulse)
                std::cout << "GPIO40释放脉冲  " << step.durationMs << " ms\n";
        }
        std::cout << std::flush;
        if (mode != "--plan") Motion_pick(steps, setup, mode == "--run", running);
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
