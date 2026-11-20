// Verify held 1 kHz additions, inter-stage guards and total voltage limiter on the native core.
#include "foc_control_core.hpp"
#include <cassert>
#include <cmath>
using namespace libecu;
using Core=FocControlCore<FixedProfile,16000>;
int main() {
    Core foc;Core::Input in{};Core::Params p{};foc.setParams(p);foc.setCurrentPiGains({},{});
    const auto one=number<Core::Current>(1);
    in.electric_mode=ElectricMode::CURRENT_MODE;in.drive_mode=DriveMode::FORWARD;
    in.target_current=one;in.bus_voltage=number<Core::Voltage>(39);
    for(int rpm:{16,20,24,27,28,30,33,34,36,38,45,48,58,300}) {
        const auto v=number<FixedProfile::PllSpeed>(2*rpm);
        const auto g=Motor1BenchPoint::fromSpeed(v);in.speed_steps_s=v;foc.applyBenchPoint(g);
        double high=0,low=0,dc=0,sn=0,cs=0;
        for(int i=0;i<768;i++) {
            in.angle_steps=number<Core::Angle>(i,768);
            const auto trim=Motor1BenchTrims::prepare(in.angle_steps,v,one);
            high=std::max(high,std::abs(double(trim.voltage.toFloat())));
            low=std::max(low,std::abs(double(trim.current.toFloat())));
            dc+=trim.voltage.toFloat();
            const double phi=i*(6*M_PI/768.);
            sn+=trim.voltage.toFloat()*std::sin(phi);cs+=trim.voltage.toFloat()*std::cos(phi);
            foc.applyBenchTrims(trim);
            assert(foc.update(in).v_q.raw()==numberCast<Core::Voltage>(trim.voltage).raw());
        }
        const double g18=std::max(0.,std::min({1.,(rpm-20)/8.,(38-rpm)/8.}));
        const double g6=std::max(0.,std::min({1.,(rpm-30)/8.,(58-rpm)/8.}));
        assert(std::abs(high-g18/32)<=1./4096);
        assert(std::abs(low-g6/16)<.00002);
        assert(std::abs(dc/768)<.00002);
        if(high>0)assert(std::abs(std::remainder(std::atan2(cs,sn)-6*M_PI*rpm/1000.,2*M_PI))<.016);
    }
    const auto v=number<FixedProfile::PllSpeed>(60);
    auto t=Motor1BenchTrims::prepare(number<Core::Angle>(1,12),v,one);
    assert(t.voltage.raw()>0);foc.applyBenchTrims(t);in.speed_steps_s=v;
    for(int target:{0,-1,3}) {
        in.target_current=number<Core::Current>(target);assert(foc.update(in).v_q.raw()==0);
    }
    in.target_current=one;in.drive_mode=DriveMode::REVERSE;assert(foc.update(in).v_q.raw()==0);
    in.drive_mode=DriveMode::FORWARD;in.speed_steps_s=-v;assert(foc.update(in).v_q.raw()==0);
    in.speed_steps_s=v;in.bus_voltage={};assert(foc.update(in).v_q.raw()==0);
}
