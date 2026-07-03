/**
 * @file bldc_controller.cpp
 * @brief Implementation of high-level BLDC motor controller
 */

#include "../include/bldc_controller.hpp"
#include "../include/platform/critical_section.hpp"
#include <algorithm>
#include <stdio.h>
#include <cmath>

// Speed measurement configuration
#define SPEED_TIMEOUT_US          500000  ///< Timeout for speed measurement (1 second)
#define BLDC_NUM_PHASES           3        ///< Number of phases in BLDC motor

// Platform specific time function
uint32_t time_us(void);

namespace libecu {

BldcController::BldcController(
    PwmInterface& pwm_interface,
    HallInterface& hall_interface,
    CommutationController& commutation_controller,
    const MotorControlParams& params,
    AdcInterface* adc_interface) noexcept
    : pwm_interface_(pwm_interface)
    , hall_interface_(hall_interface)
    , commutation_controller_(commutation_controller)
    , adc_interface_(adc_interface)
    , bemf_observer_(nullptr)
    , bemf_divider_direct_mode_(false)
    , motor_pll_(pwm_interface_.getFrequency(), params.max_speed_rps * commutation_controller.getNumPoles() * 6, params.useInverseCommTable)
    , pid_speed_controller_()
    , current_controller_()
    , params_(params)
    , dmode_(DriveMode::NEUTRAL)
    , initialized_(false)
    , prev_position_(0xFF)
    , open_loop_step_(0)
    , open_loop_last_step_time_us_(0)
    , last_pid_update_time_us_(0)
    , filtered_target_speed_(0.0f)
    , filtered_measured_speed_(0.0f)
    , limited_target_speed_(0.0f)

{
    // Initialize status
    status_.current_speed_rps = 0.0f;
    status_.target_speed_rps = 0.0f;
    status_.duty_cycle = 0.0f;
    status_.target_current = 0.0f;
    status_.measured_current = 0.0f;
    status_.bus_voltage = 0.0f;
    status_.bemf_voltage = 0.0f;
    status_.bemf_active = false;
    status_.target_position = 0xFF;
    status_.measured_position = 0xFF;
    status_.is_running = false;
    status_.control_mode = ControlMode::CLOSED_LOOP_VELOCITY;
    status_.electric_mode = ElectricMode::CURRENT_MODE;
    // PID controller settings
    params_.pid_voltage_mode.sample_time_s = 1.0f / float(pwm_interface_.getFrequency());
    params_.pid_voltage_mode.min_output = 0.0f;
    params_.pid_voltage_mode.max_output = 1.0f;

    params_.pid_current_mode.sample_time_s = 1.0f / float(pwm_interface_.getFrequency());
    params_.pid_current_mode.min_output = params_.min_current;
    params_.pid_current_mode.max_output = params_.max_current;

    params_.pid_current_regulator.sample_time_s = 1.0f / float(pwm_interface_.getFrequency());
    params_.pid_current_regulator.min_output = 0.0f;
    params_.pid_current_regulator.max_output = 1.0f;

    current_controller_.setParameters(params_.pid_current_regulator);
    pid_speed_controller_.setParameters(params_.pid_current_mode);

    motor_pll_.setUsePLL(true);
}

bool BldcController::initialize() noexcept
{
    pwm_interface_.setNeutral();

    // Reset PID controller
    pid_speed_controller_.reset();

    initialized_ = true;
    return true;
}

void BldcController::update() noexcept
{
    float speed_rps = motor_pll_.getSpeedStepsSec() / (commutation_controller_.getNumPoles() * 6.0f);

    float alpha = params_.measured_speed_lpf_alpha;
    if (alpha > 0.0f && alpha < 1.0f) {
        filtered_measured_speed_ = alpha * speed_rps + (1.0f - alpha) * filtered_measured_speed_;
    } else {
        filtered_measured_speed_ = speed_rps;
    }

    MotorStatus status;
    {
        CriticalSection cs;
        status_.current_speed_rps = filtered_measured_speed_;
        status = status_;
    }

    if (status.is_running) {
        switch (status.control_mode) {
            case ControlMode::OPEN_LOOP: {
                uint32_t current_time_us = time_us();
                uint32_t step_interval_us = calculateOpenLoopStepInterval(status.target_speed_rps);
                if (step_interval_us > 0 && (current_time_us - open_loop_last_step_time_us_) >= step_interval_us) {
                    open_loop_step_ = (open_loop_step_ + 1) % 6;
                    open_loop_last_step_time_us_ = current_time_us;
                }
                motor_pll_.updateHall(open_loop_step_);
                break;
            }

            case ControlMode::CLOSED_LOOP_VELOCITY: {
                // Velocity control with speed PID
                // Calculate real time difference since last PID update
                uint32_t current_time_us = time_us();
                float dt;

                if (last_pid_update_time_us_ == 0) {
                    // First PID update - use nominal period
                    dt = 1.0f / params_.control_frequency;
                } else {
                    // Calculate actual time difference in seconds
                    uint32_t dt_us = current_time_us - last_pid_update_time_us_;
                    dt = static_cast<float>(dt_us) / 1000000.0f;
                }

                // Update timestamp for next iteration
                last_pid_update_time_us_ = current_time_us;

                // Apply acceleration limiting (slew rate limiter on target_speed)
                float limited_target = applyAccelerationLimit(
                    status.target_speed_rps,
                    dt
                );

                // Run speed PID (already configured for correct mode)
                float pid_output = pid_speed_controller_.update(
                    limited_target,
                    status.current_speed_rps,
                    dt
                );

                // Electric mode determines how to use PID output
                if (status.electric_mode == ElectricMode::VOLTAGE_MODE) {
                    // VOLTAGE_MODE: PID outputs duty cycle directly
                    {
                        CriticalSection cs;
                        status_.duty_cycle = pid_output;
                    }
                } else {
                    // CURRENT_MODE: PID outputs target current
                    // Store for inner current loop (runs in pwmInterruptHandler)
                    {
                        CriticalSection cs;
                        status_.target_current = pid_output;
                    }

                    // duty_cycle is calculated by pwmInterruptHandler() (atomic read)
                }
                break;
            }

            case ControlMode::CLOSED_LOOP_TORQUE:
                // Torque control: fixed duty cycle or current + Hall sensor commutation
                // No speed PID - user sets fixed torque/current command
                break;
        }
    }
}

void BldcController::setTargetSpeed(float speed_rps) noexcept
{
    CriticalSection cs;

    if (dmode_ == DriveMode::NEUTRAL)
        return;
    if (speed_rps < 0.0f)
        return;
    // Clamp to maximum speed
    status_.target_speed_rps = std::min(std::abs(speed_rps), params_.max_speed_rps);
}

void BldcController::setDutyCycle(float duty_cycle) noexcept
{
    CriticalSection cs;
    if (dmode_ == DriveMode::NEUTRAL)
        return;
    if (duty_cycle < 0.0f)
        return;
    status_.duty_cycle = std::max(0.0f, std::min(duty_cycle, params_.max_duty_cycle));
}

void BldcController::setCurrent(float current_a) noexcept
{
    CriticalSection cs;
    if (dmode_ == DriveMode::NEUTRAL)
        return;
    status_.target_current = std::max(params_.min_current, std::min(current_a, params_.max_current));
}

void BldcController::setControlMode(ControlMode mode) noexcept
{
    CriticalSection cs;
    if (status_.control_mode != mode) {
        status_.control_mode = mode;

        // Reset PID when switching to velocity control
        if (mode == ControlMode::CLOSED_LOOP_VELOCITY) {
            pid_speed_controller_.reset();
            last_pid_update_time_us_ = 0;  // Reset timing for fresh start
        }
    }
}

void BldcController::setElectricMode(ElectricMode mode) noexcept
{
    CriticalSection cs;
    status_.electric_mode = mode;

    // Reconfigure PID controller with appropriate parameters for the mode
    if (mode == ElectricMode::VOLTAGE_MODE) {
        // VOLTAGE_MODE: PID outputs duty cycle (0.0-1.0)
        pid_speed_controller_.setParameters(params_.pid_voltage_mode);
    } else {
        // CURRENT_MODE: PID outputs current (Amperes)
        pid_speed_controller_.setParameters(params_.pid_current_mode);
    }
    pid_speed_controller_.reset();
}

void BldcController::setDriveMode(DriveMode mode) noexcept
{
    CriticalSection cs;
    dmode_ = mode;
    if (dmode_ == DriveMode::NEUTRAL) {
        pwm_interface_.enable(false);
        status_.target_speed_rps = 0.0f;
        status_.duty_cycle = 0.0f;
        status_.target_current = 0.0f;
        pid_speed_controller_.reset();
        current_controller_.reset();
    } else {
        pwm_interface_.enable(true);
    }
}

void BldcController::setSpeedPid(float kp, float ki, float kd) noexcept
{
    PidParameters p = pid_speed_controller_.getParameters();
    p.kp = kp;
    p.ki = ki;
    p.kd = kd;
    pid_speed_controller_.setParameters(p);
    pid_speed_controller_.reset();
}

void BldcController::setCurrentPid(float kp, float ki, float kd) noexcept
{
    PidParameters p = current_controller_.getParameters();
    p.kp = kp;
    p.ki = ki;
    p.kd = kd;
    current_controller_.setParameters(p);
    current_controller_.reset();
}

MotorPLL::PllInfo BldcController::getPllInfo() const noexcept {
    CriticalSection cs;
    return motor_pll_.getInfo();
}

void BldcController::setPllGains(float kp, float ki) noexcept {
    CriticalSection cs;
    motor_pll_.setGains(kp, ki);
}

DriveMode BldcController::getDriveMode() const noexcept {
    CriticalSection cs;
    return dmode_;
}

void BldcController::getSpeedPidGains(float& kp, float& ki, float& kd) const noexcept {
    CriticalSection cs;
    const PidParameters& p = pid_speed_controller_.getParameters();
    kp = p.kp;
    ki = p.ki;
    kd = p.kd;
}

void BldcController::getCurrentPidGains(float& kp, float& ki, float& kd) const noexcept {
    CriticalSection cs;
    const PidParameters& p = current_controller_.getParameters();
    kp = p.kp;
    ki = p.ki;
    kd = p.kd;
}

void BldcController::getPllBaseGains(float& kp, float& ki) const noexcept {
    CriticalSection cs;
    motor_pll_.getBaseGains(kp, ki);
}

void BldcController::start() noexcept
{
    status_.is_running = true;
    pwm_interface_.enable(true);

    CriticalSection cs;
    open_loop_last_step_time_us_ = time_us();
    uint8_t current_hall = commutation_controller_.getCurrentPosition();
    status_.measured_position = current_hall;
    motor_pll_.updateHall(current_hall);

    // Reset PID timing and target speed filters
    last_pid_update_time_us_ = 0;
    filtered_target_speed_ = 0.0f;
    filtered_measured_speed_ = 0.0f;
    limited_target_speed_ = 0.0f;
}

void BldcController::stop() noexcept
{
    CriticalSection cs;
    status_.is_running = false;
    status_.duty_cycle = 0.0f;
    open_loop_step_ = 0;
    commutation_controller_.updateDutyCycle(0.0f);

    status_.current_speed_rps = 0.0f;

    // Reset PID timing
    last_pid_update_time_us_ = 0;
}

MotorStatus BldcController::getStatus() const noexcept
{
    return status_;
}

float BldcController::applyAccelerationLimit(float target_speed, float dt) noexcept
{
    // Stage 1: LPF to smooth noisy analog input
    float alpha = params_.target_speed_lpf_alpha;
    if (alpha > 0.0f && alpha < 1.0f) {
        filtered_target_speed_ = alpha * target_speed + (1.0f - alpha) * filtered_target_speed_;
    } else {
        filtered_target_speed_ = target_speed;
    }

    // Stage 2: Slew rate limiter
    if (params_.acceleration_rate == 0.0f) {
        limited_target_speed_ = filtered_target_speed_;
        return limited_target_speed_;
    }

    float speed_diff = filtered_target_speed_ - limited_target_speed_;
    float max_change = params_.acceleration_rate * dt;

    if (std::abs(speed_diff) <= max_change) {
        limited_target_speed_ = filtered_target_speed_;
    } else if (speed_diff > 0.0f) {
        limited_target_speed_ += max_change;
    } else {
        limited_target_speed_ -= max_change;
    }

    return limited_target_speed_;
}

uint32_t BldcController::calculateOpenLoopStepInterval(float speed_rps) noexcept
{
    // step_interval_us = 1,000,000 / (speed_rps / 60 * num_poles * 6)
    // = 1,000,000 * 60 / (speed_rps * num_poles * 6)
    // = 10,000,000 / (speed_rps * num_poles)
    if (speed_rps <= 0.0f) {
        return 0;
    }
    uint8_t num_poles = commutation_controller_.getNumPoles();
    return static_cast<uint32_t>(10000000.0f / (speed_rps * num_poles * BLDC_NUM_PHASES));
}

void BldcController::setBemfObserver(BemfObserver* observer) noexcept {
    bemf_observer_ = observer;
}

bool BldcController::findFloatingPhase(PwmChannel& channel) noexcept {
    PwmState state_u = commutation_controller_.getPhaseState(PwmChannel::PHASE_U);
    PwmState state_v = commutation_controller_.getPhaseState(PwmChannel::PHASE_V);
    PwmState state_w = commutation_controller_.getPhaseState(PwmChannel::PHASE_W);

    if (state_u == PwmState::OFF) { channel = PwmChannel::PHASE_U; return true; }
    if (state_v == PwmState::OFF) { channel = PwmChannel::PHASE_V; return true; }
    if (state_w == PwmState::OFF) { channel = PwmChannel::PHASE_W; return true; }
    return false;
}

void BldcController::hallSensorInterruptHandler() noexcept
{
    if (status_.control_mode == ControlMode::OPEN_LOOP)
        return;

    uint8_t hall_state = commutation_controller_.getCurrentPosition();

    if (hall_state > 5) {
        return;
    }

    {
        CriticalSection cs;
        status_.measured_position = hall_state;
    }

    // If BEMF observer is active and speed is above threshold, don't feed Hall to PLL
    if (bemf_observer_ &&
        bemf_observer_->shouldIgnoreHall(motor_pll_.getSpeedStepsSec())) {
        return;
    }

    motor_pll_.updateHall(hall_state);
}

void BldcController::pwmInterruptHandler() noexcept {
    // Read shared data atomically (avoid torn reads from SysTick interrupt)
    ElectricMode electric_mode;
    uint8_t new_position;
    float target_current;
    {
        CriticalSection cs;
        electric_mode = status_.electric_mode;
        target_current = status_.target_current;
        motor_pll_.updateTick();
        new_position = motor_pll_.getNextHall(dmode_);
    }

    if (electric_mode == ElectricMode::VOLTAGE_MODE) {
        if (status_.target_position != new_position) {
            CriticalSection cs;
            status_.target_position = new_position;
            commutation_controller_.update(new_position, status_.duty_cycle);
            if (bemf_observer_) bemf_observer_->onCommutation(new_position);
        }
        // BEMF observer update in VOLTAGE_MODE
        if (bemf_observer_ && adc_interface_ &&
            bemf_observer_->isBemfModeActive(motor_pll_.getSpeedStepsSec())) {
            PwmChannel floating_phase;
            if (findFloatingPhase(floating_phase)) {
                float bemf_v = adc_interface_->readPhaseVoltage(floating_phase, bemf_divider_direct_mode_);
                float bus_v = adc_interface_->readBusVoltage();
                status_.bemf_voltage = bemf_v;
                status_.bemf_active = true;
                if (bemf_observer_->update(bemf_v, bus_v, status_.target_position,
                                            motor_pll_.getSpeedStepsSec())) {
                    motor_pll_.updateHall(bemf_observer_->getSyntheticHallStep());
                }
            }
        } else {
            status_.bemf_active = false;
        }
        return;
    }

    // Check if current controller and ADC are available
    if (!adc_interface_) {
        return;
    }

    // Read current from active conducting phase
    float measured_current = getCurrentFromActivePhase();

    // Read bus voltage
    float bus_voltage = adc_interface_->readBusVoltage();

    if (bus_voltage > params_.max_voltage)
        setDriveMode(DriveMode::NEUTRAL);

    // Run current controller
    float duty_cycle = current_controller_.update(target_current, measured_current);

    {
        CriticalSection cs;
        status_.measured_current = measured_current;
        status_.duty_cycle = duty_cycle;
        status_.bus_voltage = bus_voltage;
        status_.pll_angle = motor_pll_.getAngle();
        if (status_.target_position != new_position) {
            commutation_controller_.update(new_position, duty_cycle);
            status_.target_position = new_position;
            if (bemf_observer_) bemf_observer_->onCommutation(new_position);
        } else {
            commutation_controller_.updateDutyCycle(duty_cycle);
        }
    }

    // BEMF observer update in CURRENT_MODE
    if (bemf_observer_ && adc_interface_ &&
        bemf_observer_->isBemfModeActive(motor_pll_.getSpeedStepsSec())) {
        PwmChannel floating_phase;
        if (findFloatingPhase(floating_phase)) {
            float bemf_v = adc_interface_->readPhaseVoltage(floating_phase, bemf_divider_direct_mode_);
            status_.bemf_voltage = bemf_v;
            status_.bemf_active = true;
            if (bemf_observer_->update(bemf_v, bus_voltage, status_.target_position,
                                        motor_pll_.getSpeedStepsSec())) {
                motor_pll_.updateHall(bemf_observer_->getSyntheticHallStep());
            }
        }
    } else {
        status_.bemf_active = false;
    }
}

float BldcController::getCurrentFromActivePhase() noexcept {
    if (!adc_interface_) {
        return 0.0f;
    }

    // Determine which phase is actively conducting based on cached phase states
    // In 6-step commutation, we want to read current from the DOWN phase (low-side conducting)

    // Get cached phase states from CommutationController
    PwmState state_u = commutation_controller_.getPhaseState(PwmChannel::PHASE_U);
    PwmState state_v = commutation_controller_.getPhaseState(PwmChannel::PHASE_V);
    PwmState state_w = commutation_controller_.getPhaseState(PwmChannel::PHASE_W);

    // Find the DOWN phase (low-side conducting)
    if (state_u == PwmState::DOWN) {
        return adc_interface_->readPhaseCurrent(PwmChannel::PHASE_U);
    } else if (state_v == PwmState::DOWN) {
        return adc_interface_->readPhaseCurrent(PwmChannel::PHASE_V);
    } else if (state_w == PwmState::DOWN) {
        return adc_interface_->readPhaseCurrent(PwmChannel::PHASE_W);
    } else {
        // No DOWN phase found (e.g., all phases OFF), return 0
        return 0.0f;
    }
}



} // namespace libecu
