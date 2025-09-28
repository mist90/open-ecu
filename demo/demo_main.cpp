/**
 * @file demo_main.cpp
 * @brief Demo application showing libecu library usage
 * 
 * This demo shows how to use the libecu library with mock implementations
 * for development and testing without actual hardware.
 */

#include "../libecu/include/libecu.hpp"
#include "../libecu/include/bldc_controller.hpp"
#include <iostream>
#include <chrono>
#include <thread>

using namespace libecu;

/**
 * @brief Mock PWM implementation for testing
 */
class MockPwm : public PwmInterface {
public:
    bool initialize(uint32_t frequency) override {
        frequency_ = frequency;
        std::cout << "MockPwm: Initialized at " << frequency << " Hz\n";
        return true;
    }
    
    void setDutyCycle(PwmChannel channel, float duty_cycle) override {
        duty_cycles_[static_cast<int>(channel)] = duty_cycle;
        std::cout << "MockPwm: Channel " << static_cast<int>(channel) 
                  << " duty cycle: " << duty_cycle * 100.0f << "%\n";
    }
    
    void setState(PwmChannel channel, PwmState state) override {
        states_[static_cast<int>(channel)] = state;
        std::cout << "MockPwm: Channel " << static_cast<int>(channel) 
                  << " state: " << static_cast<int>(state) << "\n";
    }
    
    void enable(bool enable) override {
        enabled_ = enable;
        std::cout << "MockPwm: " << (enable ? "Enabled" : "Disabled") << "\n";
    }
    
    void emergencyStop() override {
        std::cout << "MockPwm: EMERGENCY STOP!\n";
        enabled_ = false;
    }
    
    uint32_t getFrequency() const override {
        return frequency_;
    }

private:
    uint32_t frequency_ = 0;
    float duty_cycles_[3] = {0.0f, 0.0f, 0.0f};
    PwmState states_[3] = {PwmState::OFF, PwmState::OFF, PwmState::OFF};
    bool enabled_ = false;
};

/**
 * @brief Mock Hall sensor implementation for testing
 */
class MockHallSensor : public HallInterface {
public:
    MockHallSensor() : position_counter_(0) {}
    
    bool initialize() override {
        std::cout << "MockHallSensor: Initialized\n";
        return true;
    }
    
    HallState readState() override {
        // Simulate rotating Hall sensor pattern
        uint8_t patterns[] = {0b001, 0b011, 0b010, 0b110, 0b100, 0b101};
        uint8_t pattern = patterns[position_counter_ % 6];
        
        HallState state;
        state.hall_a = (pattern & 0b001) != 0;
        state.hall_b = (pattern & 0b010) != 0;
        state.hall_c = (pattern & 0b100) != 0;
        
        return state;
    }
    
    MotorPosition getPosition(const HallState& state) override {
        uint8_t value = state.getValue();
        
        switch (value) {
            case 1: return MotorPosition::POSITION_1;
            case 3: return MotorPosition::POSITION_2;
            case 2: return MotorPosition::POSITION_3;
            case 6: return MotorPosition::POSITION_4;
            case 4: return MotorPosition::POSITION_5;
            case 5: return MotorPosition::POSITION_6;
            default: return MotorPosition::INVALID;
        }
    }
    
    bool isValidState(const HallState& state) override {
        uint8_t value = state.getValue();
        return (value != 0) && (value != 7);
    }
    
    // Simulate motor rotation
    void simulateRotation() {
        position_counter_++;
    }

private:
    int position_counter_;
};

int main() {
    std::cout << "=== libecu BLDC Motor Control Demo ===\n\n";
    
    // Create mock hardware interfaces
    MockPwm pwm_interface;
    MockHallSensor hall_interface;
    
    // Create controllers
    CommutationController commutation_controller(pwm_interface, hall_interface);
    
    // Configure PID controller
    PidParameters pid_params;
    pid_params.kp = 0.1f;
    pid_params.ki = 0.01f;
    pid_params.kd = 0.001f;
    pid_params.max_output = 1.0f;
    pid_params.min_output = 0.0f;
    pid_params.max_integral = 10.0f;
    
    PidController pid_controller(pid_params);
    
    // Configure safety monitoring
    SafetyLimits safety_limits;
    safety_limits.max_current = 10.0f;      // 10A max
    safety_limits.max_temperature = 85.0f;  // 85°C max
    safety_limits.min_voltage = 10.0f;      // 10V min
    safety_limits.max_voltage = 50.0f;      // 50V max
    safety_limits.fault_timeout = 1000;     // 1s timeout
    
    SafetyMonitor safety_monitor(safety_limits);
    
    // Configure motor parameters
    MotorControlParams motor_params;
    motor_params.max_duty_cycle = 0.8f;     // 80% max duty cycle
    motor_params.max_speed_rpm = 3000.0f;   // 3000 RPM max
    motor_params.acceleration_rate = 500.0f; // 500 RPM/s
    motor_params.control_frequency = 100;    // 100 Hz control loop
    
    // Create high-level controller
    BldcController bldc_controller(
        pwm_interface,
        hall_interface,
        commutation_controller,
        pid_controller,
        safety_monitor,
        motor_params
    );
    
    // Initialize system
    std::cout << "Initializing BLDC controller...\n";
    if (!bldc_controller.initialize()) {
        std::cerr << "Failed to initialize BLDC controller!\n";
        return -1;
    }
    std::cout << "Initialization successful!\n\n";
    
    // Demo scenario 1: Open-loop control
    std::cout << "=== Demo 1: Open-loop control ===\n";
    bldc_controller.setControlMode(ControlMode::OPEN_LOOP);
    bldc_controller.setDutyCycle(0.3f);  // 30% duty cycle
    bldc_controller.start();
    
    // Simulate running for a few cycles
    SafetyData safety_data = {0};
    safety_data.phase_u_current = 2.0f;
    safety_data.phase_v_current = 2.1f;
    safety_data.phase_w_current = 1.9f;
    safety_data.temperature = 45.0f;
    safety_data.bus_voltage = 24.0f;
    safety_data.emergency_stop = false;
    safety_data.hall_fault = false;
    
    for (int i = 0; i < 10; i++) {
        hall_interface.simulateRotation();
        bldc_controller.update(safety_data);
        
        MotorStatus status = bldc_controller.getStatus();
        std::cout << "Cycle " << i << ": Position=" << static_cast<int>(status.position)
                  << ", Duty=" << status.duty_cycle * 100.0f << "%\n";
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    bldc_controller.stop();
    std::cout << "\n";
    
    // Demo scenario 2: Closed-loop speed control
    std::cout << "=== Demo 2: Closed-loop speed control ===\n";
    bldc_controller.setControlMode(ControlMode::CLOSED_LOOP);
    bldc_controller.setTargetSpeed(1000.0f);  // 1000 RPM target
    bldc_controller.start();
    
    for (int i = 0; i < 10; i++) {
        hall_interface.simulateRotation();
        bldc_controller.update(safety_data);
        
        MotorStatus status = bldc_controller.getStatus();
        std::cout << "Cycle " << i << ": Target=" << status.target_speed_rpm 
                  << " RPM, Current=" << status.current_speed_rpm 
                  << " RPM, Duty=" << status.duty_cycle * 100.0f << "%\n";
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    bldc_controller.stop();
    std::cout << "\n";
    
    // Demo scenario 3: Safety fault handling
    std::cout << "=== Demo 3: Safety fault simulation ===\n";
    safety_data.phase_u_current = 15.0f;  // Overcurrent!
    
    bldc_controller.start();
    bldc_controller.update(safety_data);
    
    MotorStatus status = bldc_controller.getStatus();
    std::cout << "Safety fault detected: " << static_cast<int>(status.active_fault) << "\n";
    std::cout << "Motor running: " << (status.is_running ? "Yes" : "No") << "\n";
    
    // Clear fault and normal operation
    safety_data.phase_u_current = 2.0f;  // Normal current
    bldc_controller.clearFault(SafetyFault::OVERCURRENT);
    bldc_controller.update(safety_data);
    
    status = bldc_controller.getStatus();
    std::cout << "After fault clear - Active fault: " << static_cast<int>(status.active_fault) << "\n";
    
    std::cout << "\n=== Demo completed successfully! ===\n";
    
    return 0;
}