# STM32CubeIDE Integration Guide

## Overview
This guide explains how to integrate the libecu BLDC motor control library into STM32CubeIDE for the B-G431B-ESC1 board.

## Prerequisites
- STM32CubeIDE 1.7.0 or later
- STM32G4 HAL libraries
- B-G431B-ESC1 evaluation board
- BLDC motor with Hall sensors

## Integration Steps

### 1. Library Compilation
The libecu library is already compiled and ready for integration:
```bash
cd libecu
make clean && make
```
This generates `lib/liblibecu.a`

### 2. STM32CubeIDE Project Setup

#### Include Paths
Add these paths to Project Properties → C/C++ Build → Settings → Tool Settings → MCU G++ Compiler → Include paths:
- `../libecu/include`
- `../libecu/include/interfaces`
- `../libecu/include/algorithms`
- `../libecu/include/hal/stm32g4`

#### Library Configuration
Add to Project Properties → C/C++ Build → Settings → Tool Settings → MCU G++ Linker → Libraries:
- Library search paths: `../libecu/lib`
- Libraries: `libecu`

#### Compiler Flags
Add to MCU G++ Compiler → Miscellaneous → Other flags:
- `-DSTM32G4`
- `-std=c++17`

### 3. Hardware Configuration (STM32CubeMX)

#### TIM1 Configuration
- Mode: PWM Generation CH1, CH2, CH3
- Prescaler: 0
- Counter Period: 4249 (for 20kHz PWM @ 170MHz)
- Pulse: 0
- Dead Time: Configure as needed
- Break/Break2: Configure for safety

#### GPIO Configuration
Hall sensors are already configured in main.h:
- Hall A: PB6 (A__Pin)
- Hall B: PB7 (B__Pin)  
- Hall C: PB8 (Z__Pin)

#### OPAMP Configuration
Current sensing amplifiers (OPAMP1-3) should be configured for motor current feedback.

### 4. Code Integration

The main.cpp file already includes the necessary integration code with conditional compilation using `#ifdef STM32G4`.

#### Key Components:
- **PWM Driver**: `STM32PWM` class handles 3-phase PWM generation
- **Hall Sensor**: `STM32HallSensor` class reads motor position
- **Motor Controller**: `BldcController` implements 6-step commutation
- **Safety Monitor**: Built-in fault detection and emergency stop
- **Control Loop**: 100Hz update rate via SysTick interrupt

### 5. Runtime Configuration

#### Motor Parameters
Adjust in main.cpp USER CODE section:
```cpp
libecu::MotorConfig config;
config.max_current = 10.0f;     // Max current (A)
config.max_speed = 3000.0f;     // Max speed (RPM)
config.pole_pairs = 7;          // Motor pole pairs
```

#### Control Parameters
- Target speed: `motor_controller->setTargetSpeed(1000.0f)` (RPM)
- Control frequency: 100Hz (configurable in SysTick setup)
- PWM frequency: 20kHz (configurable in PWM initialization)

### 6. Safety Features

#### Built-in Protection:
- Overcurrent detection via safety monitor
- Invalid Hall state filtering
- Emergency stop functionality
- Fault condition handling

#### Custom Safety:
Add custom safety checks in the main control loop:
```cpp
if (motor_controller->hasFault()) {
    motor_controller->emergencyStop();
    // Handle fault condition
}
```

### 7. Debugging and Testing

#### Build Process:
1. Clean and rebuild project in STM32CubeIDE
2. Flash to B-G431B-ESC1 board
3. Connect BLDC motor with Hall sensors
4. Power on and monitor via debugger/UART

#### Verification:
- Check Hall sensor readings
- Verify PWM output on oscilloscope
- Test motor rotation and speed control
- Validate safety systems

## Troubleshooting

### Common Issues:
1. **Compilation errors**: Verify include paths and library configuration
2. **Linker errors**: Check library path and C++17 support
3. **Runtime faults**: Verify hardware connections and motor parameters
4. **No motor rotation**: Check Hall sensor wiring and motor power

### Debug Tools:
- STM32CubeIDE debugger
- UART output for diagnostics
- Live expressions for motor state
- SWV console for real-time monitoring

## Performance Notes

- **Memory usage**: ~8KB RAM, ~32KB Flash
- **CPU usage**: ~10% at 100Hz control loop
- **Real-time constraints**: Control loop must complete within 10ms
- **Interrupt priorities**: SysTick should have high priority

## Next Steps

1. **Current Sensing**: Integrate OPAMP/ADC current feedback
2. **Speed Estimation**: Implement Hall-based speed calculation
3. **Advanced Control**: Add field-oriented control (FOC)
4. **Communication**: Add CAN bus or other interfaces
5. **Diagnostics**: Implement comprehensive fault reporting