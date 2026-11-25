#include "bench/legacy_site.hpp"
#include "motion/plan.hpp"
#include <opencv2/core.hpp>
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
        check(arm.upperMm == 275 && arm.forearmMm == 275 && arm.j2JointPerDrive == .8 && arm.j3JointPerDrive == -1.0/3,
              "Saved arm scales were replaced");
        cv::FileStorage reference;
        reference.open("tests/fixtures/upper-app/kinematic-reference.json", cv::FileStorage::READ);
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
                cv::Point2d reconstructed = scara::Arm_forward(arm, joints);
                check(cv::norm(reconstructed - point) < 1e-8, "Forward/inverse coordinates disagree");
            }
            ++count;
        }
        cv::FileStorage plan;
        plan.open("tests/fixtures/upper-app/pick-plan.json", cv::FileStorage::READ);
        auto from = plan["source"]["base_xy_mm"], to = plan["destination"]["base_xy_mm"];
        cv::Point2d source{double(from[0]), double(from[1])};
        cv::Point2d destination{double(to[0]), double(to[1])};
        const scara::JointTarget park{12, 34};
        const cv::Point2d discard{300, 0};
        std::vector<scara::PickStep> steps;
        using Action = scara::PickAction;
        scara::Plan_turn(arm, heights, source, destination, nullptr, park, steps);
        check(steps.size() == 9, "Pick/place/park plan is incomplete");
        check(steps[0].action == Action::Move && steps[0].target[0] == heights.travelDeg &&
                  std::abs(steps[0].target[1] - double(plan["source"]["j2_deg"])) < 1e-9 &&
                  std::abs(steps[0].target[2] - double(plan["source"]["j3_motor_deg"])) < 1e-9 &&
                  steps[1].action == Action::Move && steps[1].target[0] == heights.pickupDeg &&
                  steps[2].action == Action::Grip && steps[2].durationMs == heights.gripDwellMs &&
                  steps[3].target[0] == heights.clearanceDeg,
              "Pick-up must approach, lower, hold and lift");
        // 棋子已在吸头上，放子段从搬运层直接移到目标格，不再次取子。
        check(steps[4].action == Action::Move && steps[4].target[0] == heights.travelDeg &&
                  std::abs(steps[4].target[1] - double(plan["destination"]["j2_deg"])) < 1e-9 &&
                  std::abs(steps[4].target[2] - double(plan["destination"]["j3_motor_deg"])) < 1e-9 &&
                  steps[5].action == Action::Move && steps[5].target[0] == heights.releaseDeg &&
                  steps[6].action == Action::Release && steps[6].durationMs == 0 &&
                  steps[7].action == Action::Move && steps[7].target[0] == heights.clearanceDeg,
              "Release must be at placement height and complete before lifting");
        const scara::AxisValues parkAtTravel{heights.travelDeg, park.j2Deg, park.j3MotorDeg};
        check(steps[8].action == Action::Move && steps[8].target == parkAtTravel,
              "Ordinary move did not finish at the observation pose");

        const auto ordinary = steps;
        scara::Plan_turn(arm, heights, source, destination, &discard, park, steps);
        check(steps.size() == 15, "Capture did not rebuild a complete plan");
        check(steps[0].target == steps[10].target && steps[1].target[0] == heights.pickupDeg &&
                  steps[2].action == Action::Grip && steps[2].durationMs == heights.gripDwellMs &&
                  steps[3].target[0] == heights.clearanceDeg && steps[4].target[0] == heights.travelDeg &&
                  steps[5].action == Action::ReleaseOffBoard && steps[5].durationMs == 0,
              "Capture must pick on the destination pose and release off board at transport height");
        cv::Point2d discarding = scara::Arm_forward(arm, {steps[4].target[1], steps[4].target[2]});
        check(cv::norm(discarding - discard) < 1e-8, "Discard used the wrong XY waypoint");
        for (size_t i = 0; i < ordinary.size(); ++i)
        {
            const auto &normal = ordinary[i];
            const auto &capture = steps[6 + i];
            check(capture.action == normal.action && capture.target == normal.target &&
                      capture.durationMs == normal.durationMs,
                  "Capture changed the attacking piece transfer or observation park");
        }

        check(heights.clearanceDeg == heights.travelDeg, "Untaught clearance was guessed");
        auto taught = scara::WorkHeights_fromContact(-20225.7605);
        check(taught.pickupDeg == taught.releaseDeg && taught.travelDeg == taught.clearanceDeg &&
                  std::abs(taught.travelDeg - (-18219.66)) < 1e-8,
              "Historical J1 lift was not relative to the contact reading");
        auto shifted = scara::WorkHeights_fromContact(123, 2.15);
        check(std::abs(shifted.travelDeg - (123 + 1003.05025)) < 1e-8, "Lift in cm lost its origin or scale");
        scara::WorkHeights overlap;
        check(scara::WorkHeights_load("tests/fixtures/game-app/work-heights.json", overlap), "Overlap fixture failed");
        scara::Plan_turn(arm, overlap, source, destination, nullptr, park, steps);
        check(steps[3].target[0] == overlap.clearanceDeg && steps[4].target[0] == overlap.travelDeg &&
                  steps[3].target[0] != steps[4].target[0] && steps[4].action == Action::Move &&
                  steps[3].target[1] != steps[4].target[1] && steps[3].target[2] != steps[4].target[2],
              "Travel segment does not move all three axes together");
        check(std::abs(heights.releaseDeg - (-20053.1425)) < 1e-7 && heights.gripDwellMs == 420,
              "Taught height or stored pulse was changed");

        auto requireEmptyOnError = [&](auto plan, const char *message)
        {
            steps = ordinary;
            bool failed = false;
            try { plan(); } catch (const std::runtime_error &) { failed = true; }
            check(failed && steps.empty(), message);
        };
        const cv::Point2d unreachable{1000, 1000};
        requireEmptyOnError([&] { scara::Plan_turn(arm, heights, unreachable, destination, &discard, park, steps); },
                            "Unreachable attacker retained a capture plan");
        requireEmptyOnError([&] { scara::Plan_turn(arm, heights, source, unreachable, nullptr, park, steps); },
                            "Unreachable destination retained an old plan");
        requireEmptyOnError([&] { scara::Plan_turn(arm, heights, source, destination, &unreachable, park, steps); },
                            "Unreachable discard retained the pick-up steps");
        scara::WorkHeights missingHeights = heights;
        check(!scara::WorkHeights_load("tests/fixtures/upper-app/missing-heights.json", missingHeights),
              "Missing taught pickup height was silently replaced by zero");
        requireEmptyOnError([&] { scara::Plan_turn(arm, missingHeights, source, destination, nullptr, park, steps); },
                            "Unconfigured work heights generated a movement plan");
        scara::ArmGeometry missingArm = arm;
        check(!scara::Arm_load("tests/fixtures/upper-app/missing-arm-reference.json", missingArm),
              "Missing mechanical reference was silently replaced by zero");
        scara::JointTarget invalidTarget{12, 34};
        check(!scara::Arm_inverse(missingArm, source, invalidTarget) && invalidTarget.j2Deg == 0 &&
                  invalidTarget.j3MotorDeg == 0,
              "Failed reload reused old joint targets");

        //速度受限与加速度受限两种情况，检查公共进度、实际导数上限和精确端点
        scara::AxisValues start{0,10,-20}, goal{300,-14,40};
        for (const scara::MotionLimits &limits : std::vector<scara::MotionLimits>{
                 {{180,8,12},{60,30,40}}, {{100000,100000,100000},{60,30,40}}})
        {
            scara::JointTrajectory group;
            check(scara::Trajectory_plan(start, goal, limits, group), "Group trajectory failed");
            double longest = 0;
            for (int axis = 0; axis < 3; ++axis)
            {
                auto singleGoal = start;
                singleGoal[axis] = goal[axis];
                scara::JointTrajectory single;
                check(scara::Trajectory_plan(start, singleGoal, limits, single), "Single-axis bound failed");
                longest = std::max(longest, single.seconds);
            }
            check(std::abs(group.seconds - longest) < 1e-12, "Group time is not the slowest axis time");
            auto previous = scara::Trajectory_sample(group, 0);
            check(previous.position == start && previous.velocity == scara::AxisValues{}, "Nonzero start speed");
            double dt = group.seconds / 1000;
            for (int sample = 1; sample <= 1000; ++sample)
            {
                auto reference = scara::Trajectory_sample(group, sample * dt);
                double progress = (reference.position[0] - start[0]) / (goal[0] - start[0]);
                for (int axis = 0; axis < 3; ++axis)
                {
                    double fraction = (reference.position[axis] - start[axis]) / (goal[axis] - start[axis]);
                    check(std::abs(fraction - progress) < 1e-12, "Axes do not share progress");
                    check(std::abs(reference.velocity[axis]) <= limits.velocity[axis] + 1e-8, "Speed cap exceeded");
                    check(std::abs(reference.velocity[axis] - previous.velocity[axis]) / dt <=
                              limits.acceleration[axis] + 1e-6, "Acceleration cap exceeded");
                }
                previous = reference;
            }
            auto reference = scara::Trajectory_sample(group, group.seconds + 1);
            check(reference.position == goal && reference.velocity == scara::AxisValues{}, "Endpoint not exact/resting");
        }
        scara::JointTrajectory zero;
        scara::MotionLimits limits{{60,60,60},{100,100,100}};
        check(scara::Trajectory_plan(start, start, limits, zero) && zero.seconds == 0, "Zero-distance path failed");
        auto rest = scara::Trajectory_sample(zero, 0);
        check(rest.position == start && rest.velocity == scara::AxisValues{}, "Zero-distance path must hold position");
        limits.velocity[1] = 0;
        check(!scara::Trajectory_plan(start, goal, limits, zero), "Zero speed cap accepted");
        std::cout << "PASS " << count
                  << " saved kinematic cases, taught heights, ordinary/capture plans and unreachable targets\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
