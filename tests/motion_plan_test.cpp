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
                cv::Point2d reconstructed;
                scara::Arm_forward(arm, joints, reconstructed);
                check(cv::norm(reconstructed - point) < 1e-8, "Forward/inverse coordinates disagree");
            }
            ++count;
        }
        cv::FileStorage plan;
        plan.open("tests/fixtures/upper-app/pick-plan.json", cv::FileStorage::READ);
        auto from = plan["source"]["base_xy_mm"], to = plan["destination"]["base_xy_mm"];
        cv::Point2d source{double(from[0]), double(from[1])};
        cv::Point2d destination{double(to[0]), double(to[1])};
        std::vector<scara::PickStep> steps;
        scara::Plan_pick(arm, heights, source, steps);
        scara::Plan_place(arm, heights, destination, steps);
        check(steps.size() == 8,
              "Pick/place plan is incomplete");
        using Action = scara::PickAction;
        check(steps[0].action == Action::Move && steps[1].action == Action::Move && steps[1].target[0] == heights.pickupDeg &&
                  steps[2].action == Action::Grip && steps[2].durationMs == heights.gripDwellMs &&
                  steps[3].target[0] == heights.clearanceDeg && steps[4].action == Action::Move &&
                  steps[5].target[0] == heights.releaseDeg && steps[6].action == Action::Release &&
                  steps[6].durationMs == 0 && steps[7].target[0] == heights.clearanceDeg,
              "Release must be at placement height and complete before lifting");
        steps.clear();
        scara::Plan_pick(arm, heights, source, steps);
        scara::Plan_discard(arm, heights, destination, steps);
        check(steps.size() == 6 && steps[4].target[0] == heights.travelDeg &&
                  steps[5].action == Action::ReleaseOffBoard && steps[5].durationMs == 0,
              "The captured piece must be expelled at the transport height");
        check(heights.clearanceDeg == heights.travelDeg, "Untaught clearance was guessed");
        scara::WorkHeights overlap;
        check(scara::WorkHeights_load("tests/fixtures/game-app/work-heights.json", overlap), "Overlap fixture failed");
        steps.clear();
        scara::Plan_pick(arm, overlap, source, steps);
        scara::Plan_place(arm, overlap, destination, steps);
        check(steps[3].target[0] == overlap.clearanceDeg && steps[4].target[0] == overlap.travelDeg &&
                  steps[3].target[0] != steps[4].target[0] && steps[4].axes == scara::AllAxes &&
                  steps[3].target[1] != steps[4].target[1] && steps[3].target[2] != steps[4].target[2],
              "Travel segment does not move all three axes together");
        check(std::abs(heights.releaseDeg - (-20053.1425)) < 1e-7 && heights.gripDwellMs == 420,
              "Taught height or stored pulse was changed");
        // 手里已有棋子时，维持吸合后直接放，计划中没有取子点。
        steps = {{Action::Grip}};
        scara::Plan_place(arm, heights, destination, steps);
        check(steps.size() == 5 &&
              steps[0].durationMs == 0 && steps[1].target[0] == heights.travelDeg && steps[3].action == Action::Release,
              "Held-piece resume tried to pick from a source again");
        auto requireEmptyOnError = [&](auto plan, const char *message)
        {
            bool failed = false;
            try { plan(); } catch (const std::runtime_error &) { failed = true; }
            check(failed && steps.empty(), message);
        };
        scara::Plan_pick(arm, heights, source, steps);
        requireEmptyOnError([&] { scara::Plan_place(arm, heights, {1000, 1000}, steps); },
                            "Unreachable destination retained an old plan");
        scara::Plan_pick(arm, heights, source, steps);
        requireEmptyOnError([&] { scara::Plan_discard(arm, heights, {1000, 1000}, steps); },
                            "Unreachable discard retained the pick-up steps");
        scara::WorkHeights missingHeights = heights;
        check(!scara::WorkHeights_load("tests/fixtures/upper-app/missing-heights.json", missingHeights),
              "Missing taught pickup height was silently replaced by zero");
        requireEmptyOnError([&] { scara::Plan_pick(arm, missingHeights, source, steps); },
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
            scara::JointReference previous, reference;
            scara::Trajectory_sample(group, 0, previous);
            check(previous.position == start && previous.velocity == scara::AxisValues{}, "Nonzero start speed");
            double dt = group.seconds / 1000;
            for (int sample = 1; sample <= 1000; ++sample)
            {
                scara::Trajectory_sample(group, sample * dt, reference);
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
            scara::Trajectory_sample(group, group.seconds + 1, reference);
            check(reference.position == goal && reference.velocity == scara::AxisValues{}, "Endpoint not exact/resting");
        }
        scara::JointTrajectory zero;
        scara::MotionLimits limits{{60,60,60},{100,100,100}};
        check(scara::Trajectory_plan(start, start, limits, zero) && zero.seconds == 0, "Zero-distance path failed");
        scara::JointReference rest;
        scara::Trajectory_sample(zero, 0, rest);
        check(rest.position == start && rest.velocity == scara::AxisValues{}, "Zero-distance path must hold position");
        limits.velocity[1] = 0;
        check(!scara::Trajectory_plan(start, goal, limits, zero), "Zero speed cap accepted");
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
