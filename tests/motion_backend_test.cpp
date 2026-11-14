#include "motion/execute.hpp"
#include <iostream>

int main()
{
    volatile std::sig_atomic_t running = 1;
    scara::PickStep step;
    std::vector<scara::PickStep> steps;
    bool success = true;

    step.action = scara::PickAction::MoveZ;
    step.zDeg = 123;
    step.speedDegS = 180;
    steps.push_back(step);
    scara::Motion_execute(steps, running, success);
    if (success || scara::Motion_stop()) return 1;
    running = 0;
    success = true;
    scara::Motion_execute(steps, running, success);
    if (success) return 1;
    std::cout << "PASS unconnected hardware backend never claims completed or stopped; no hardware calls\n";
    return 0;
}
