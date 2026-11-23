/**
 * @file model.cpp
 * @brief Non-drive A/B bench. Real fixed scooter speed PI, rolling I bound and Hall PLL.
 * Only the plant, measurement transport and scene are synthetic. Model truth never enters MTTE.
 */
#include "fixture.hpp"
#include "control_calibration.hpp"
#include "motor_pll_core.hpp"
#include "pi_float_interface.hpp"
#include "traction_acceleration.hpp"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {
using namespace fixture;
using namespace libecu;
using P = FixedProfile;
using Pi = PiFloatInterface<P::SpeedPiTypes, MtteObserver::RATE_HZ>;
using Pll = MotorPLLCore<P, 16000>;
using State = MtteObserver::State;
constexpr double TURN = 6.283185307179586476925;
constexpr unsigned FAST_HZ = 16000, STEPS = FAST_HZ / MtteObserver::RATE_HZ, HALL = 3*BLDC_NUM_POLES;
constexpr double DT = 1.0 / FAST_HZ;
constexpr float HARD_A = 16; // Additional independent positive limit for BOTH arms; not a controller setting.
constexpr unsigned SWITCH = 150, REGRIP = 230, EARLY_END = 450, LONG_END = 2000;

void require(bool ok, const char* what) {
    if (!ok) { std::fprintf(stderr,"FAIL: %s\n",what); std::exit(1); }
}

enum Scene { Dry, Ice, Lift, Air, AirWithLoss, Stale, CoastBrake, Mismatch, SharedDrop, SmallStep };
const char* const scene_names[] = {
    "dry","ice","zero-grip","air","air+drag","stale","coast-brake","ice-mismatch","matched-drop","incipient"
};
struct Policy { const char* name; bool legacy, mtte; };
constexpr Policy POLICIES[] = {
    {"PI",false,false}, {"PI+MTTE",false,true}, {"bench-1",true,false}, {"bench-1+MTTE",true,true}
};

double mu(Scene s, unsigned k) {
    if (s == Air || s == AirWithLoss) return 0;
    if (k >= SWITCH && k < REGRIP) {
        if (s == Ice || s == Mismatch || s == SharedDrop) return 0.012;
        if (s == Lift) return 0;
    }
    return 0.65;
}
const char* stateName(State s) {
    return s == State::Tracking ? "tracking" : s == State::Settling ? "settling" : "unavailable";
}

struct Interval {
    float mean_iq{}, mean_force{}, peak_acc_rps2{}, peak_slip_mps{};
    bool hall_contiguous = false;
};

// Two independent momenta. Enforce rolling with a static contact impulse if |F| <= mu*N;
// otherwise kinetic friction saturates in the direction opposing relative contact motion.
// This is a rigid Coulomb contact, not a fitted tyre curve. Force on chassis is +F, on wheel -rF.
struct Plant {
    explicit Plant(Scene s) : j(J*(s==Mismatch ? 1.5 : 1)), kt(KT*(s==Mismatch ? 0.8 : 1)),
        mass(s==Mismatch ? 100 : M), drag_n(s==Air || s==AirWithLoss ? 0 : 0.007*mass*9.80665),
        bearing_nm(s==AirWithLoss ? 0.08 : 0), iq(cruiseCurrent()) {}
    double cruiseCurrent() const { return (R*drag_n + bearing_nm)/kt; } // Pre-existing steady rolling point.
    const double j, kt, mass, drag_n, bearing_nm;
    double omega = TURN*2.0, v = R*TURN*2.0, angle_turns = 0.13/HALL, iq;
    const double current_fraction = -std::expm1(-TURN*250*DT);
    const double avg_factor = current_fraction/(TURN*250*DT); // exact average of first-order ZOH current
    uint32_t edges = 0, edge_age = 0;
    uint8_t sector() const {
        const int h = static_cast<int>(std::floor(angle_turns*HALL + 0.5)) % 6;
        return static_cast<uint8_t>(h < 0 ? h+6 : h);
    }
    Interval advance(float request, double friction, Pll& pll) {
        double iq_sum=0, force_sum=0, max_acc=0, max_slip=0;
        bool hall_contiguous = pll.getInfo().is_sync;
        uint8_t old_hall = pll.getInfo().hall_state_raw;
        const double max_force = friction*0.5*mass*9.80665; // half of the static weight on the driven tyre
        for (unsigned i=0; i<STEPS; ++i) {
            const double mean_iq = request + (iq-request)*avg_factor;
            iq += (request-iq)*current_fraction;
            const double load = std::clamp(v*mass/DT, -drag_n, drag_n);
            const double bearing = std::clamp(omega*j/DT, -bearing_nm, bearing_nm);
            const double torque = kt*mean_iq - bearing;
            const double stick = ((R*omega-v)/DT + R*torque/j + load/mass)/(R*R/j + 1.0/mass);
            const double force = std::clamp(stick,-max_force,max_force);
            const double accel = (torque-R*force)/j;
            angle_turns += (omega+0.5*accel*DT)*DT/TURN;
            omega += DT*accel;
            v += DT*(force-load)/mass;
            iq_sum += numberCast<P::Current>(float(iq)).toFloat(); // actual 16-kHz sample endpoint
            force_sum += force;
            max_acc = std::max(max_acc,accel/TURN);
            max_slip = std::max(max_slip,R*omega-v);
            const uint8_t hall = sector();
            if (hall != old_hall) {
                const unsigned step = (hall+6-old_hall)%6;
                hall_contiguous &= step==1 || step==5;
                ++edges; edge_age = 0;
                pll.updateHall(hall);
                old_hall = hall;
            } else ++edge_age;
            pll.updateTick();
            hall_contiguous &= pll.getInfo().is_sync;
        }
        return {float(iq_sum/STEPS),float(force_sum/STEPS),float(max_acc),float(max_slip),hall_contiguous};
    }
};

struct Window {
    double iq_sum{}, positive_iq_sum{}, command_sum{}, force_sum{}, estimate_sum{}, force_error_sq{};
    double wheel_sum{}, chassis_sum{}, max_slip{}, max_acc{}, max_tick_acc{};
    unsigned n{}, estimated{}, limited{};
    void add(Interval in, float request, float wheel_rps, float body_mps, float tick_acc,
             const MtteObserver::Result& o, float previous_force, bool clipped) {
        ++n;
        iq_sum+=in.mean_iq; positive_iq_sum+=std::max(0.0f,in.mean_iq);
        command_sum+=request; force_sum+=in.mean_force; wheel_sum+=wheel_rps; chassis_sum+=body_mps;
        max_slip=std::max<double>(max_slip,in.peak_slip_mps);
        max_acc=std::max<double>(max_acc,in.peak_acc_rps2);
        max_tick_acc=std::max<double>(max_tick_acc,tick_acc);
        limited += clipped;
        if (o.state == State::Tracking) {
            ++estimated; estimate_sum+=o.force_n;
            const double error=o.force_n-previous_force;
            force_error_sq+=error*error;
        }
    }
};
struct Result {
    Window rolling, event, recovery, tail, final;
    float entry_n{}, entry_a{}, end_n{};
    unsigned stale_block{}, coast_ticks{}, brake_ticks{};
};

void writeSummary(FILE* file, Scene s, Policy policy, const char* name, const Window& w) {
    std::fprintf(file,"%s,%s,%s,%u,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,",
                 scene_names[s],policy.name,name,w.n,w.iq_sum/w.n,w.positive_iq_sum/w.n,w.command_sum/w.n,
                 w.max_acc,w.max_tick_acc,w.max_slip,w.wheel_sum/w.n,w.chassis_sum/w.n,w.force_sum/w.n,w.estimated);
    if (w.estimated) std::fprintf(file,"%.6f,%.6f",w.estimate_sum/w.estimated,std::sqrt(w.force_error_sq/w.estimated));
    else std::fputc(',',file);
    std::fprintf(file,",%u\n",w.limited);
}

Result simulate(Scene scene, Policy policy, FILE* trace, FILE* summary) {
    Plant plant(scene);
    Pi pi;
    require(pi.setParameters(COMS3_PI),"fixed COMS3 PI accepted");
    Pll pll(constant<P::PllSpeed>(BLDC_MAX_SPEED*HALL));
    pll.setGains(constant<Pll::Gain>(BLDC_PLL_KP),constant<Pll::Gain>(BLDC_PLL_KI));
    pll.setUsePLL(true);
    pll.updateHall(plant.sector());
    pll.reset();
    MtteObserver observer;
    require(observer.configure(PARAMS),"MTTE SI parameters accepted");
    TractionAcceleration<P> legacy;
    P::Speed feedback = number<P::Speed>(2);
    P::Accumulator target = number<P::Accumulator>(2);
    constexpr auto ramp = number_v<P::Accumulator, 2, 100>;
    const auto initial_a = numberCast<P::Current>(float(plant.cruiseCurrent()));
    // Handoff the unchanged speed PI at the existing cruise torque, not an inferred friction floor.
    // Both arms share the same initial state, and the observer warms from sensor data alone.
    Pi::Gains gains{};
    require(Pi::prepareGains(COMS3_PI.kp,COMS3_PI.ki,gains),"COMS3 gains representable");
    Pi::Kernel::Accumulator initial_i{};
    require(Pi::Kernel::rebaseIntegral({P::Speed{},initial_a},gains.kp,number<P::Current>(-4),
            number<P::Current>(45),initial_i),"steady rolling point representable");
    pi.applyGains(COMS3_PI.kp,COMS3_PI.ki,gains,&initial_i);
    P::Current last = initial_a;
    Interval previous{};
    MtteObserver::Result o;
    Result result;
    double patch_mu=0;
    const unsigned end = scene==CoastBrake ? EARLY_END : LONG_END;
    // Negative indices are only the test schedule. k is the monotonic 100-Hz clock.
    for (int tick=-100; tick<int(end); ++tick) {
        const auto k=uint32_t(tick+100);
        const auto t=unsigned(std::max(0,tick));
        const bool coast = scene==CoastBrake && tick>=int(SWITCH) && tick<180;
        const bool brake = scene==CoastBrake && tick>=180;
        const bool dropout = scene==Stale && tick>=int(SWITCH) && tick<185;
        const P::Speed hall = divideByInteger<P::Speed>(pll.getSpeedStepsSec(),HALL);
        feedback += multiplyTo<P::Speed>(number_v<P::Fraction,1,2>,hall-feedback);
        auto frame = sample(k,hall.toFloat(),previous.mean_iq,coast ? DriveMode::NEUTRAL : DriveMode::FORWARD);
        frame.hall_valid = previous.hall_contiguous && plant.edge_age < FAST_HZ/50 &&
                           std::abs(hall.toFloat()) < BLDC_MAX_SPEED;
        frame.iq_complete = tick != -100;
        if (dropout) {
            if (tick<170) --frame.iq_tick; // delayed Iq transport, speed PLL still valid
            else frame.hall_valid=false;  // stale Hall qualification even though PLL still reports a number
        }
        if (tick<0) {
            o = observer.update(k,frame,initial_a.toFloat(),HARD_A); // rolling run-in, shadow only
            previous = plant.advance(initial_a.toFloat(),mu(scene,0),pll);
            continue;
        }
        if (tick==0) require(o.state==State::Tracking,"qualified rolling handoff in every arm");
        const double before = plant.omega/TURN;
        if (t==SWITCH) {
            result.entry_n=float(before); result.entry_a=previous.mean_iq;
            // A second matched-entry test asks if the ESTIMATOR acts while PLL lock survives.
            // In that scene the patch supports 95% of the pair's common pre-step contact force.
            // This value sets only the synthetic road, never an input or tuning of either controller.
            patch_mu=0.95*previous.mean_force/(0.5*plant.mass*9.80665);
        }
        const float command = brake || coast ? 0.0f : t<50 ? 2.0f : 5.0f;
        // A neutral interval clears command/integral; the driver brake test after it uses the real negative PI bound.
        if (coast) { pi.reset(); legacy.reset(); target=numberCast<P::Accumulator>(feedback); }
        else target+=numericClamp(numberCast<P::Accumulator>(command)-target,-ramp,ramp);
        const auto ref=numberCast<P::Speed>(target);
        const auto integral_max=rollingIntegralCeiling<P, motor_config::rolling_integral>(
            feedback, constant<P::Current>(COMS3_PI.integral_max));
        auto maximum=numberCast<P::Current>(coast ? 0.0f : HARD_A);
        if (policy.legacy && !coast)
            maximum=legacy.ceiling(feedback,number_v<P::Speed,1,100>,last,maximum,
                                   constant<P::Speed>(motor_config::traction_gain_a_per_rps));
        // Side-effect-free preview of THIS tick's pre-MTTE PI demand (E17's T*). Only the actual PI is committed.
        auto preview=pi;
        const auto ordinary=coast ? P::Current{} : preview.updateNativeCapped(ref,feedback,0,integral_max,maximum);
        o=observer.update(k,frame,ordinary.toFloat(),maximum.toFloat());
        // Normally active from t=0, before any disturbance. An extra matched-entry control isolates causality
        // at the torque step: both arms first drive the same PI in shadow, then bifurcate at the event.
        const bool matched=scene==SharedDrop || scene==SmallStep;
        const bool active=policy.mtte && (!matched || t>=SWITCH);
        const auto cap=active ? std::min(maximum,numberCast<P::Current>(o.positive_ceiling_a)) : maximum;
        last=coast ? P::Current{} : pi.updateNativeCapped(ref,feedback,0,integral_max,cap);
        require(last<=maximum && last<=cap && last>=numberCast<P::Current>(BLDC_MIN_CURRENT),
                "independent current bounds");
        require(pi.getIntegral()>=-4.0f && pi.getIntegral()<=integral_max.toFloat(),"rolling integral bound");
        if (ordinary<=P::Current{}) require(last==ordinary,"zero and every negative request preserved at the PI");
        if (active && o.state!=State::Tracking) require(last<=P::Current{},"unknown cannot authorize drive");
        if (dropout && active) {
            require(o.state==State::Unavailable,"stale observation rejected");
            ++result.stale_block;
        }
        if (coast) { require(last==P::Current{},"neutral torque release"); ++result.coast_ticks; }
        if (brake && ordinary < number_v<P::Current,-1,2>) ++result.brake_ticks;

        const double v = plant.v;
        const unsigned edges=plant.edges, age=plant.edge_age;
        const double road=scene==SmallStep && t>=SWITCH && t<REGRIP ? patch_mu : mu(scene,t);
        const auto next=plant.advance(last.toFloat(),road,pll);
        const float tick_acc=float((plant.omega/TURN-before)*MtteObserver::RATE_HZ);
        require(std::isfinite(plant.omega) && std::isfinite(plant.v),"finite mechanical response");
        if (road==0.0) require(next.mean_force==0.0f,"lifted wheel transmits exactly zero force");
        const bool clipped=active && last<ordinary;
        auto add=[&](Window& w) {
            w.add(next,last.toFloat(),float(before),float(v),tick_acc,o,previous.mean_force,clipped);
        };
        if (t>=100 && t<SWITCH) add(result.rolling);
        if (t>=SWITCH && t<REGRIP) add(result.event);
        if (t>=REGRIP && t<EARLY_END) add(result.recovery);
        if (t>=EARLY_END-50 && t<EARLY_END) add(result.tail);
        if (t>=end-50) add(result.final);
        std::fprintf(trace,"%s,%s,%.2f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%u,%u,%d,%d,%d,%s,",
                     scene_names[scene],policy.name,t*0.01,command,ref.toFloat(),before,hall.toFloat(),
                     feedback.toFloat(),v,edges,age,frame.hall_valid,frame.iq_complete && frame.iq_tick==k,
                     active,stateName(o.state));
        if (o.state==State::Tracking)
            std::fprintf(trace,"%.6f,%.6f,%.6f,%.6f",o.acceleration_rad_s2,o.motor_nm,o.force_n,o.tmax_nm);
        else std::fputs(",,,",trace); // unknown is not a measured zero
        std::fprintf(trace,",%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%.6f,%.6f,%.6f,%.6f\n",
                     o.rise_a,o.probe_a,previous.mean_iq,maximum.toFloat(),ordinary.toFloat(),cap.toFloat(),
                     last.toFloat(),pi.getIntegral(),pi.saturationSign(),previous.mean_force,next.peak_acc_rps2,
                     tick_acc,next.peak_slip_mps);
        previous=next;
    }
    result.end_n=float(plant.omega/TURN);
    writeSummary(summary,scene,policy,"rolling",result.rolling);
    writeSummary(summary,scene,policy,"event",result.event);
    writeSummary(summary,scene,policy,"recovery",result.recovery);
    writeSummary(summary,scene,policy,"tail",result.tail);
    if (end==LONG_END) writeSummary(summary,scene,policy,"late",result.final);
    return result;
}
} // namespace

int main(int argc, char** argv) {
    if (argc!=2) { std::fprintf(stderr,"Usage: %s output-directory\n",argv[0]); return 2; }
    FILE* trace=std::fopen((std::string(argv[1])+"/trace.csv").c_str(),"w");
    FILE* summary=std::fopen((std::string(argv[1])+"/summary.csv").c_str(),"w");
    require(trace && summary,"open CSV outputs");
    std::fputs("scene,policy,t_s,driver_rps,ref_rps,n_true_rps,n_pll_rps,n_pi_rps,body_mps,hall_edges,age_16k_ticks,"
               "hall_valid,iq_valid,mtte_active,state,alpha_hat_rad_s2,motor_hat_nm,force_hat_n,tmax_hat_nm,"
               "e17_a,probe_a,iq_meas_prev_a,existing_max_a,ordinary_a,ceiling_a,applied_a,integral_a,pi_sat,"
               "force_true_prev_n,max_acc_next_rps2,acc_10ms_next_rps2,max_slip_next_mps\n",trace);
    std::fputs("scene,policy,window,n,mean_iq_a,mean_positive_interval_iq_a,mean_request_a,peak_positive_acc_rps2,"
               "peak_positive_10ms_acc_rps2,peak_positive_slip_mps,mean_wheel_rps,mean_body_mps,mean_true_force_n,"
               "estimate_count,mean_hat_force_n,rms_force_error_n,mtte_clipped_ticks\n",summary);
    std::puts("scene        policy          entry A | event: mean Iq A / peak wheel rev/s^2 / peak slip m/s | end RPS");
    for (Scene s: {Dry,Ice,Lift,Air,AirWithLoss,Stale,CoastBrake,Mismatch,SharedDrop,SmallStep}) {
        std::array<Result,4> r;
        for (unsigned i=0; i<4; ++i) {
            auto& v=r[i];
            const auto policy=POLICIES[i];
            v=simulate(s,policy,trace,summary);
            std::printf("%-12s %-15s %7.3f | %9.3f / %9.3f / %9.4f | %7.3f\n",
                scene_names[s],policy.name,v.entry_a,v.event.iq_sum/v.event.n,v.event.max_acc,v.event.max_slip,v.end_n);
            if (s==Stale && policy.mtte) {
                require(v.stale_block==35 && v.tail.estimated==50,"stale window and qualified re-entry");
                require(v.tail.command_sum/v.tail.n>0.5,"positive torque resumes after stale hold");
            }
            if (s==CoastBrake) require(v.coast_ticks==30 && v.brake_ticks>10,"exercise physical coast and negative PI");
        }
        if (s==SharedDrop || s==SmallStep) {
            for (unsigned i: {0u,2u})
                require(r[i].entry_a==r[i+1].entry_a && r[i].entry_n==r[i+1].entry_n,"identical loss entry state");
        }
        if (s==Ice || s==Lift || s==SharedDrop || s==SmallStep) {
            require(r[1].event.max_slip<r[0].event.max_slip && r[1].event.limited>0,"A/B reduces positive wheel slip");
            require(r[1].tail.command_sum/r[1].tail.n>0.5 && r[1].tail.estimated==50,
                    "observer reacquired, positive torque resumes after contact returns");
        }
        if (s==SharedDrop)
            require(r[1].event.max_acc==r[0].event.max_acc,"an unannounced equal-torque loss has the same first peak");
        if (s==SmallStep) {
            const Window& a=r[2].event;
            const Window& b=r[3].event;
            require(a.estimated==a.n && b.estimated==b.n && b.limited>0,"MTTE can act without losing Hall lock");
            require(b.max_slip<a.max_slip && std::abs(b.force_sum-a.force_sum)<0.01*b.n,
                    "lower slip at equal transmitted force on the locked matched-entry patch");
        }
    }
    require(std::fclose(trace)==0 && std::fclose(summary)==0,"completed trace and summary");
    std::puts("PASS: 10 scenes x 4 policies, physical/contact/current, freshness, braking and pairwise checks");
}
