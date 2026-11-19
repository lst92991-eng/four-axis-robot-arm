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
        cv::FileStorage reference;
        scara::readStorage("tests/fixtures/upper-app/kinematic-reference.json", reference);
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
        scara::readStorage("tests/fixtures/upper-app/pick-plan.json", plan);
        auto from = plan["source"]["base_xy_mm"], to = plan["destination"]["base_xy_mm"];
        cv::Point2d source{double(from[0]), double(from[1])};
        cv::Point2d destination{double(to[0]), double(to[1])};
        std::vector<scara::PickStep> steps;
        check(scara::Pick_plan(arm, heights, source, destination, false, steps) && steps.size() == 8,
              "Pick/place plan is incomplete");
        using Action = scara::PickAction;
        check(steps[0].action == Action::Hold && steps[1].action == Action::Move &&
                  steps[2].action == Action::Move && steps[2].target[0] == heights.pickupDeg &&
                  steps[3].target[0] == heights.clearanceDeg && steps[4].action == Action::Move &&
                  steps[5].target[0] == heights.releaseDeg && steps[6].action == Action::ReleasePulse &&
                  steps[6].durationMs == heights.releasePulseMs && steps[7].target[0] == heights.clearanceDeg,
              "Release must be at placement height and complete before lifting");
        check(heights.clearanceDeg == heights.travelDeg, "Untaught clearance was guessed");
        scara::WorkHeights overlap;
        check(scara::WorkHeights_load("tests/fixtures/game-app/work-heights.json", overlap), "Overlap fixture failed");
        check(scara::Pick_plan(arm, overlap, source, destination, false, steps), "Overlap plan failed");
        check(steps[3].target[0] == overlap.clearanceDeg && steps[4].target[0] == overlap.travelDeg &&
                  steps[3].target[0] != steps[4].target[0] && steps[4].axes == scara::AllAxes &&
                  steps[3].target[1] != steps[4].target[1] && steps[3].target[2] != steps[4].target[2],
              "Travel segment does not move all three axes together");
        check(std::abs(heights.releaseDeg - (-20053.1425)) < 1e-7 && heights.releasePulseMs == 20,
              "Taught height or stored pulse was changed");
        check(scara::Pick_plan(arm, heights, {1000, 1000}, destination, true, steps) && steps.size() == 5,
              "Held-piece resume tried to pick from a source again");
        check(!scara::Pick_plan(arm, heights, source, {1000, 1000}, false, steps) && steps.empty(),
              "Unreachable destination retained an old plan");
        scara::WorkHeights missingHeights = heights;
        check(!scara::WorkHeights_load("tests/fixtures/upper-app/missing-heights.json", missingHeights),
              "Missing taught pickup height was silently replaced by zero");
        check(!scara::Pick_plan(arm, missingHeights, source, destination, false, steps) && steps.empty(),
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
