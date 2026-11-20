#include "order_feedback.hpp"
#include <cassert>
#include <cmath>
#include <cstdio>
using libecu::Order144Feedback;

int main() {
    constexpr double tau=6.283185307179586;
    constexpr int32_t amp=1<<20, sector=1<<24, vel=1<<14;
    for(int sign:{-1,0,1}) for(int initial:{0,1,2,3,4}) for(int rpm:{34,39,42,44}){
        Order144Feedback feedback;
        double cross=0,power=0,signal_power=0,mean=0;
        for(int i=0;i<2500;i++){
            const double step=6*initial+.27+2*rpm*i*.001;
            const double phi=1.2*tau*step+.81;
            auto angle=int32_t(std::fmod(step,6.)*sector);
            const auto iq=int32_t(amp*(.84+.07*std::sin(phi)));
            const int32_t correction=feedback.step(angle,2*rpm*vel,amp,iq,32*sign);
            assert(std::abs(correction)<=amp/8);
            if(sign==0)assert(correction==0);
            if(i>500){
                const double ref=std::sin(phi+tau*1.2*rpm/1000),res=correction/double(amp);
                cross+=ref*res;power+=res*res;signal_power+=ref*ref;mean+=res;
            }
        }
        if(sign) {
            assert(sign*cross/std::sqrt(power*signal_power)>.97);
            assert(std::abs(mean/1999)<.001);
            if(rpm==39 || rpm==42)assert(sign*cross/signal_power>.028 && sign*cross/signal_power<.043);
        }
        for(int32_t demand:{0,-amp,3*amp,INT32_MAX})
            assert(feedback.step(0,2*rpm*vel,demand,INT32_MIN,64)==0);
        for(int32_t speed:{INT32_MIN,-2*rpm*vel,0,60*vel,92*vel,1100*vel,INT32_MAX})
            assert(feedback.step(0,speed,amp,INT32_MAX,64)==0);
        assert(feedback.step(0,2*rpm*vel,amp,INT32_MAX,64)==0);
    }
}
