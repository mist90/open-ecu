# BLDC ECU
This is software for BLDC ECU project based on b-g431b-esc1 board.
The software realizes 6-step commutation algorithm with Hall sensors as velocity feedback.

Pinout:
PA8 - TIM1CH1 (HIN PWM output for phase U)
PC13 - TIM1CH1N (LIN PWM output for phase U)
PA9 - TIM1CH2 (HIN PWM output for phase V)
PA12 - TIM1CH2N (LIN PWM output for phase V)
PA10 - TIM1CH3 (HIN PWM output for phase W)
PB15 - TIM1CH3N (LIN PWM output for phase W)

PB6 - A+/H1 GPIO input
PB7 - B+/H2 GPIO input
PB8 - Z+/H3 GPIO input

PA1 - OPAMP1+ - connected to + pole of shunt (0.03 Ohm) for current sensing in U phase
PA3 - OPAMP1- - not connected
PA7 - OPAMP2+ - connected to + pole of shunt (0.03 Ohm) for current sensing in V phase
PA5 - OPAMP2- - not connected
PB0 - OPAMP3+ - connected to + pole of shunt (0.03 Ohm) for current sensing in W phase
PB2 - OPAMP3- - not connected

All shunts are low side current senses and connected between ground and low side MOSFET.

## Firmware Architecture

The BLDC ECU firmware is implemented as a modular C++ system with the following components:

### libecu Library (Platform-Independent Core)
- **CommutationController**: 6-step commutation algorithm with Hall sensor feedback
- **PidController**: Digital PID controller for speed control with anti-windup
- **SafetyMonitor**: Real-time safety monitoring with fault detection
- **HallSensor**: Hall sensor interface (platform-independent)
- **PwmInterface**: PWM abstraction for motor control

### HAL Layer (STM32G4-Specific)
- **Stm32Pwm**: TIM1-based 3-phase PWM with complementary outputs
- **HallSensor**: GPIO-based Hall sensor reading
- **BldcController**: High-level BLDC motor control

### Key Features
- **6-step trapezoidal control** with Hall sensor position feedback
- **Closed-loop speed control** using PID algorithm
- **Safety monitoring** with overcurrent, overtemperature, and undervoltage protection
- **Modular architecture** allowing easy porting to other MCUs
- **Real-time performance** optimized for embedded systems

### Control Loop
The firmware runs a 100Hz control loop that:
1. Reads Hall sensor states
2. Updates commutation sequence
3. Calculates motor speed
4. Runs PID speed control
5. Monitors safety parameters
6. Updates PWM duty cycles

### Safety Features
- Overcurrent protection (120% of rated current)
- Overtemperature protection (95% of max temperature)
- Undervoltage protection (minimum bus voltage monitoring)
- Emergency stop functionality
- Fault counters and diagnostic information

## Implementation Phases

### Phase 1: Core Infrastructure (Week 1-2)
- Define platform-independent interfaces (PWM, GPIO, ADC, Timer)
- Implement basic data structures and types
- Create libecu library structure
- Set up build system and testing framework

### Phase 2: BLDC Algorithms (Week 3-4)
- Implement 6-step commutation algorithm
- Develop PID controller with anti-windup
- Create Hall sensor processing logic
- Add safety monitoring and fault detection

### Phase 3: STM32 HAL Implementation (Week 5-6)
- Implement STM32G4-specific PWM driver
- Create GPIO interface for Hall sensors
- Add ADC interface for current sensing
- Integrate with STM32 HAL libraries

### Phase 4: Integration & Testing (Week 7-8)
- System integration and testing
- Performance optimization
- Safety system validation
- Documentation and code review

## Development Team Structure

The project uses specialized subagents for different aspects:

- **BLDC-Expert**: Motor control algorithms and commutation logic
- **C++ Embedded Programmer**: Core library implementation and optimization
- **HAL Adapter**: MCU-specific hardware abstraction
- **Safety Expert**: Fault detection and emergency handling

## Build Instructions

This project uses STM32CubeIDE for development:

1. Open project in STM32CubeIDE
2. Build Debug configuration for development
3. Build Release configuration for production
4. Use built-in debugger for testing

## Library Structure

```
libecu/
├── include/
│   ├── interfaces/     # Platform-independent interfaces
│   ├── algorithms/     # BLDC control algorithms
│   └── safety/        # Safety monitoring
├── src/
│   ├── commutation.cpp
│   ├── pid_controller.cpp
│   └── safety_monitor.cpp
└── hal/
    └── stm32g4/       # STM32G4-specific implementation
```
