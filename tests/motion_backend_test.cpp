#include "motion/execute.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace fake
{
bool connected=false, healthy=true, frozen=false, stale=false, stopWorks=true;
int reads=0, writes=0, failWrite=0, failRead=0, stops=0, cancelWrite=0;
uint64_t clockMs=0;
scara::JointReference command;
std::vector<scara::JointReference> samples;
std::vector<std::pair<uint64_t,int>> pulses;
volatile std::sig_atomic_t running=1;
void reset()
{
    connected=true; healthy=true; frozen=false; stale=false; stopWorks=true;
    reads=writes=failWrite=failRead=stops=cancelWrite=0;
    clockMs=0; command={}; samples.clear(); pulses.clear(); running=1;
}
void check(bool value,const char *message)
{
    if(!value) throw std::runtime_error(message);
}
}
namespace scara
{
//测试在链接时提供驱动函数，同一个Motion_execute保持原样执行
bool Drive_read(DriveFeedback &feedback)
{
    ++fake::reads;
    if(!fake::connected || !fake::healthy || (fake::stale && fake::reads>1) ||
       (fake::failRead && fake::reads>=fake::failRead)) return false;
    if(!fake::stale) fake::clockMs+=10;
    feedback.position=fake::frozen ? AxisValues{} : fake::command.position;
    feedback.velocity=fake::frozen ? AxisValues{} : fake::command.velocity;
    feedback.timeMs=fake::clockMs;
    return true;
}
bool Drive_write(const JointReference &reference)
{
    ++fake::writes;
    if(fake::failWrite && fake::writes>=fake::failWrite) return false;
    fake::command=reference;
    fake::samples.push_back(reference);
    if(fake::cancelWrite && fake::writes==fake::cancelWrite) fake::running=0;
    return fake::connected;
}
bool Drive_release(int milliseconds)
{
    fake::pulses.push_back({fake::clockMs,milliseconds});
    return fake::connected;
}
bool Drive_stop()
{
    ++fake::stops;
    fake::command.velocity={};
    return fake::connected && fake::stopWorks;
}
}

int main()
{
    using namespace scara;
    using fake::check;
    try
    {
        MotionLimits limits{{60,50,80},{200,200,200}};
        PickStep move;
        move.action=PickAction::Move;
        move.target={12,-8,24};
        bool success=true;
        fake::reset(); fake::connected=false;
        Motion_execute({move},limits,fake::running,success);
        check(!success && !Motion_stop() && fake::samples.empty(),"Disconnected drive claimed success");

        fake::reset();
        Motion_execute({move},limits,fake::running,success);
        check(success && fake::stops==0 && fake::samples.size()>20,"Group move failed");
        bool intermediate=false;
        for(const auto &sample:fake::samples)
        {
            double progress=sample.position[0]/12;
            check(std::abs(sample.position[1]/-8-progress)<1e-12 &&
                  std::abs(sample.position[2]/24-progress)<1e-12,"Axes advanced at different times");
            intermediate=intermediate || (progress>.1 && progress<.9);
            for(int axis=0;axis<3;++axis)
                check(std::abs(sample.velocity[axis])<=limits.velocity[axis]+1e-8,"Written speed exceeded cap");
        }
        check(intermediate && fake::command.position==move.target && fake::command.velocity==AxisValues{},
              "Final reference or coordinated intermediate samples missing");

        fake::reset(); fake::command.position={3,4,5};
        auto vertical=move; vertical.axes=ZAxis; vertical.target={13,999,999};
        Motion_execute({vertical},limits,fake::running,success);
        check(success,"Selected-axis move failed");
        for(const auto &sample:fake::samples)
            check(sample.position[1]==4 && sample.position[2]==5,"Unselected axes were moved");

        fake::reset(); fake::failWrite=4;
        Motion_execute({move},limits,fake::running,success);
        check(!success && fake::stops==1 && fake::writes==4,"Group write failure was retried or not stopped");
        fake::reset(); fake::failRead=5;
        Motion_execute({move},limits,fake::running,success);
        check(!success && fake::stops==1,"Feedback loss did not stop whole group");
        fake::reset(); fake::stale=true;
        Motion_execute({move},limits,fake::running,success);
        check(!success && fake::stops==1 && fake::samples.empty(),"Repeated feedback clock was accepted");
        fake::reset(); fake::frozen=true;
        Motion_execute({move},limits,fake::running,success);
        check(!success && fake::stops==1,"Elapsed trajectory time was mistaken for physical arrival");

        fake::reset(); fake::cancelWrite=3;
        Motion_execute({move},limits,fake::running,success);
        check(!success && fake::stops==1 && fake::writes==3,"Cancellation sent more trajectory references");
        fake::reset(); fake::running=0;
        Motion_execute({move},limits,fake::running,success);
        check(!success && fake::writes==0 && fake::reads==0,"Stopped input started a move");

        PickStep pulse;
        pulse.action=PickAction::ReleasePulse; pulse.durationMs=40;
        fake::reset();
        Motion_execute({pulse,move},limits,fake::running,success);
        check(success && fake::pulses.size()==2 && fake::pulses[0].second==40 &&
              fake::pulses[1].second==0 && fake::pulses[1].first-fake::pulses[0].first>=40,
              "Release did not finish before the next movement");
        fake::reset(); fake::cancelWrite=2;
        Motion_execute({pulse,move},limits,fake::running,success);
        check(!success && fake::stops==1 && fake::pulses.back().second==0 && fake::writes==2,
              "Cancelled release continued into movement or left pulse active");

        fake::reset(); fake::failWrite=2; fake::stopWorks=false;
        bool threw=false;
        try { Motion_execute({move},limits,fake::running,success); }
        catch(const std::exception &) { threw=true; }
        check(threw && !success && fake::stops==1,"Unconfirmed stop was hidden");
        fake::reset(); move.target={};
        Motion_execute({move},limits,fake::running,success);
        check(success && fake::samples.empty(),"No-op target issued needless movement");
        std::cout<<"PASS shared progress, limits, selective hold, group stop, feedback loss, cancellation, release ordering and unconnected state; fake drives only\n";
        return 0;
    }
    catch(const std::exception &error)
    {
        std::cerr<<error.what()<<'\n';
        return 1;
    }
}
