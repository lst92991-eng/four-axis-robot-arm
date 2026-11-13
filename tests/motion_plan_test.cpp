#include "motion/plan.hpp"
#include "vision/io.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

static void check(bool value, const char *message)
{
    if (!value) throw std::runtime_error(message);
}

int main()
{
    try
    {
        scara::ArmGeometry arm;
        scara::WorkHeights heights;
        check(scara::Arm_load("tests/fixtures/upper-app/arm-registration.json", arm), "Arm configuration failed");
        check(scara::WorkHeights_load("tests/fixtures/upper-app/pick-plan.json", heights), "Work heights failed");
        check(arm.upperMm == 275 && arm.forearmMm == 275 && arm.j2Scale == .8 && arm.j3Ratio == 3,
              "Saved arm scales were replaced");
        auto reference = scara::readStorage("tests/fixtures/upper-app/kinematic-reference.json");
        int count = 0;
        for (const auto &item : reference["cases"])
        {
            auto xy = item["xy_mm"];
            cv::Point2d point{double(xy[0]), double(xy[1])};
            scara::JointTarget joints;
            bool reachable = scara::Arm_inverse(arm, point, joints);
            check(reachable == (int(item["reachable"]) != 0), "Reachability differs from demonstrated Python model");
            if (reachable)
            {
                check(std::abs(joints.j2Deg - double(item["j2_deg"])) < 1e-9 &&
                          std::abs(joints.j3MotorDeg - double(item["j3_motor_deg"])) < 1e-9,
                      "Joint targets differ from demonstrated Python model");
                cv::Point2d reconstructed;
                scara::Arm_forward(arm, joints, reconstructed);
                check(cv::norm(reconstructed - point) < 1e-8, "Forward/inverse coordinates disagree");
            }
            ++count;
        }
        auto plan = scara::readStorage("tests/fixtures/upper-app/pick-plan.json");
        auto from = plan["source"]["base_xy_mm"], to = plan["destination"]["base_xy_mm"];
        cv::Point2d source{double(from[0]), double(from[1])};
        cv::Point2d destination{double(to[0]), double(to[1])};
        std::vector<scara::PickStep> steps;
        check(scara::Pick_plan(arm, heights, source, destination, false, steps) && steps.size() == 8,
              "Pick/place plan is incomplete");
        using Action = scara::PickAction;
        check(steps[0].action == Action::Hold && steps[1].action == Action::MoveXY &&
                  steps[2].action == Action::MoveZ && steps[2].zDeg == heights.pickupDeg &&
                  steps[3].zDeg == heights.travelDeg && steps[4].action == Action::MoveXY &&
                  steps[5].zDeg == heights.releaseDeg && steps[6].action == Action::ReleasePulse &&
                  steps[6].durationMs == heights.releasePulseMs && steps[7].zDeg == heights.travelDeg,
              "Release must be at placement height and complete before lifting");
        check(std::abs(heights.releaseDeg - (-20053.1425)) < 1e-7 && heights.releasePulseMs == 20,
              "Taught height or stored pulse was changed");
        check(scara::Pick_plan(arm, heights, {1000, 1000}, destination, true, steps) && steps.size() == 5,
              "Held-piece resume tried to pick from a source again");
        check(!scara::Pick_plan(arm, heights, source, {1000, 1000}, false, steps) && steps.empty(),
              "Unreachable destination retained an old plan");
        std::cout << "PASS " << count
                  << " saved kinematic cases, taught heights, release order and held-piece resume; no I/O\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
