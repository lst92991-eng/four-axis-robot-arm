#include "motion/execute.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <utility>

namespace fake
{
bool connected=false, healthy=true, frozen=false, stale=false, stopWorks=true;
bool magnetOffWorks=true, throwOnMagnetOff=false, throwOnStop=false;
int reads=0, writes=0, failWrite=0, failRead=0, stops=0, cancelWrite=0;
int cancelRead=0, readsAtCancel=0;
uint64_t clockMs=0;
uint64_t magnetAckDelayMs=0;
bool invalidFeedback=false;
uint64_t disturbedAt=0;
int holdFrames=0, disturbMode=0;
scara::JointReference command;
std::vector<scara::JointReference> samples;
std::vector<std::pair<uint64_t,int>> pulses;
volatile std::sig_atomic_t running=1;
void reset()
{
    connected=true; healthy=true; frozen=false; stale=false; stopWorks=true;
    magnetOffWorks=true; throwOnMagnetOff=false; throwOnStop=false;
    reads=writes=failWrite=failRead=stops=cancelWrite=0;
    cancelRead=readsAtCancel=0;
    magnetAckDelayMs=0; invalidFeedback=false;
    clockMs=0; disturbedAt=0; holdFrames=disturbMode=0; command={}; samples.clear(); pulses.clear(); running=1;
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
    if(fake::reads==fake::cancelRead) fake::running=0;
    if(!fake::connected || !fake::healthy || (fake::invalidFeedback && fake::reads>1) || (fake::stale && fake::reads>1) ||
       (fake::failRead && fake::reads>=fake::failRead)) return false;
    if(!fake::stale) fake::clockMs+=10;
    feedback.position=fake::frozen ? AxisValues{} : fake::command.position;
    feedback.timeMs=fake::clockMs;
    feedback.stationary=fake::command.velocity==AxisValues{};
    feedback.reached.reset();
    if(feedback.stationary && !fake::frozen) feedback.reached=fake::command.position;
    if (fake::disturbMode && fake::command.position==AxisValues{12,-8,24} && fake::command.velocity==AxisValues{})
    {
        // 模拟板端先回旧目标，或仍在运动，到第十帧才确认本段。
        ++fake::holdFrames;
        if (fake::holdFrames < 10)
        {
            if (fake::disturbMode==1) feedback.reached=AxisValues{};
            else feedback.stationary=false;
        }
        else fake::disturbedAt=feedback.timeMs;
    }
    return true;
}
bool Drive_write(const JointReference &reference)
{
    ++fake::writes;
    if(fake::failWrite && fake::writes>=fake::failWrite) return false;
    fake::command=reference;
    fake::samples.push_back(reference);
    if(fake::writes==fake::cancelWrite) { fake::running=0; fake::readsAtCancel=fake::reads; }
    return fake::connected;
}
bool Drive_magnet(bool enabled)
{
    if(enabled) fake::clockMs+=fake::magnetAckDelayMs;
    fake::pulses.push_back({fake::clockMs,int(enabled)});
    if(!enabled && fake::throwOnMagnetOff) throw std::runtime_error("Magnet IO failed");
    if(!enabled && !fake::magnetOffWorks) return false;
    return fake::connected;
}
bool Drive_stop()
{
    ++fake::stops;
    if(fake::throwOnStop) throw std::runtime_error("Motor IO failed");
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
        bool success;
        fake::reset(); fake::connected=false;
        success = Motion_execute({move},limits,fake::running);
        check(!success && !Motion_stop() && fake::samples.empty(),"Disconnected drive claimed success");

        fake::reset();
        success = Motion_execute({move},limits,fake::running);
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
        for (int mode : {1, 2})
        {
            fake::reset();
            fake::disturbMode = mode;
            check(Motion_execute({move}, limits, fake::running), "Did not settle again after the perturbation");
            check(fake::disturbedAt && fake::clockMs == fake::disturbedAt && fake::holdFrames==10 && fake::stops == 0,
                  "Only a fresh, stationary confirmation for this target may finish the move");
        }

        fake::reset(); fake::command.position={3,4,5};
        auto vertical=move; vertical.action=PickAction::LiftZ; vertical.target={13,999,999};
        success = Motion_execute({vertical},limits,fake::running);
        check(success,"Selected-axis move failed");
        for(const auto &sample:fake::samples)
            check(sample.position[1]==4 && sample.position[2]==5,"Unselected axes were moved");

        fake::reset(); fake::failWrite=4;
        success = Motion_execute({move},limits,fake::running);
        check(!success && fake::stops==1 && fake::writes==4,"Group write failure was retried or not stopped");
        fake::reset(); fake::failRead=5;
        success = Motion_execute({move},limits,fake::running);
        check(!success && fake::stops==1,"Feedback loss did not stop whole group");
        fake::reset(); fake::stale=true;
        success = Motion_execute({move},limits,fake::running);
        check(!success && fake::stops==1 && fake::samples.empty(),"Repeated feedback clock was accepted");
        fake::reset(); fake::frozen=true;
        success = Motion_execute({move},limits,fake::running);
        check(!success && fake::stops==1,"Elapsed trajectory time was mistaken for physical arrival");

        fake::reset(); fake::cancelWrite=3;
        success = Motion_execute({move},limits,fake::running);
        check(!success && fake::stops==1 && fake::writes==3,"Cancellation sent more trajectory references");
        check(fake::reads==fake::readsAtCancel,"Move waited for another sample after cancellation");
        fake::reset(); fake::running=0;
        success = Motion_execute({move},limits,fake::running);
        check(!success && fake::writes==0 && fake::reads==0,"Stopped input started a move");

        PickStep grip, release;
        grip.action=PickAction::Grip; grip.durationMs=40;
        release.action=PickAction::Release;
        fake::reset(); fake::cancelRead=2;
        success = Motion_execute({grip},limits,fake::running);
        check(!success && fake::stops==1 && fake::pulses.size()==1 && fake::pulses.front().second==0,
              "Cancellation during the pre-action read energized the magnet");
        fake::reset(); fake::magnetAckDelayMs=90;
        success = Motion_execute({grip},limits,fake::running);
        check(success && fake::pulses.size()==1 && fake::clockMs-fake::pulses.front().first>=40,
              "Waiting for the magnet acknowledgement consumed the grip dwell");
        fake::reset(); fake::invalidFeedback=true;
        success = Motion_execute({move},limits,fake::running);
        check(!success && fake::stops==1 && fake::samples.empty(),"Invalid feedback issued a move");
        fake::reset();
        success = Motion_execute({grip,move,release},limits,fake::running);
        check(success && fake::pulses.size()==2 && fake::pulses[0].second==1 &&
              fake::pulses[1].second==0 && fake::pulses[1].first-fake::pulses[0].first>=40,
              "Grip was not held through transport and switched off at the destination");
        fake::reset(); fake::cancelWrite=2;
        success = Motion_execute({grip,move},limits,fake::running);
        check(!success && fake::stops==1 && fake::pulses.back().second==0 && fake::writes==2,
              "Cancelled grip continued into movement or left the magnet on");
        check(fake::reads==fake::readsAtCancel,"Grip waited for another sample after cancellation");

        fake::reset(); fake::failWrite=2; fake::stopWorks=false;
        bool threw=false;
        try { success = Motion_execute({move},limits,fake::running); }
        catch(const std::exception &) { threw=true; }
        check(threw && fake::stops==1,"Unconfirmed stop was hidden");
        // 运动写失败后，同时核对关磁铁和停车的结果。
        struct StopCase { bool magnetOff, motorStop; const char *message; };
        const StopCase stopCases[] = {
            {false, true, "磁铁关断未确认"}, {true, false, "机械臂停稳未确认"},
            {false, false, "磁铁关断和机械臂停稳均未确认"}
        };
        for (const auto &item : stopCases)
        {
            fake::reset(); fake::failWrite=1;
            fake::magnetOffWorks=item.magnetOff;
            fake::stopWorks=item.motorStop;
            bool reported=false;
            try { Motion_execute({move},limits,fake::running); }
            catch(const std::runtime_error &e) { reported=std::string_view(e.what())==item.message; }
            check(reported && fake::stops==1,"Cleanup did not identify the failed device");
        }
        fake::reset(); fake::throwOnMagnetOff=true;
        check(!Motion_stop() && fake::stops==1,"Magnet exception skipped the motor stop");
        fake::reset(); fake::throwOnStop=true;
        check(!Motion_stop() && fake::pulses.size()==1 && fake::pulses[0].second==0,
              "Motor exception hid the magnet-off request");
        fake::reset(); move.target={};
        success = Motion_execute({move},limits,fake::running);
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
