# Hall Sensor Migration: GPIO EXTI → TIM4 Hall Sensor Interface

## TL;DR

> **Quick Summary**: Replace existing `Stm32HallSensor` class with `Stm32TimHallSensor` in-place within `libecu/hal/stm32g4/stm32_hall_sensor.hpp` and `.cpp`, switching from GPIO EXTI to TIM4 Hall Sensor Interface for hardware debounce (~752ns). Full replacement — no fallback. All libecu core logic preserved.
>
> **Deliverables**:
> - `Stm32TimHallSensor` class replaces `Stm32HallSensor` in `stm32_hall_sensor.hpp` / `.cpp` (same files)
> - PB6/PB7/PB8 remapped from GPIO EXTI to AF2_TIM4
> - TIM4 ISR replaces EXTI9_5 Hall interrupt chain in `stm32g4xx_it.c`
> - `main.cpp` wiring updated: `Stm32HallSensor` → `Stm32TimHallSensor`
> - Old EXTI9_5 Hall wiring fully removed
>
> **Estimated Effort**: Short
> **Parallel Execution**: YES — 3 waves
> **Critical Path**: Task 2 (TIM4 HAL impl) → Task 4 (main.cpp wiring) → Task 6 (build)

---

## Context

### Original Request
> "Хочу переделать реализацию чтения датчиков Холла с GPIO на специальные таймерные функции STM32G431 для чтения с датчиков Холла. Это нужно, чтобы фильтровать дребезг."

### Interview Summary
**Key Discussions**:
- **Timer choice**: TIM4 selected because PB6=TIM4_CH1, PB7=TIM4_CH2, PB8=TIM4_CH3 — perfect pin match, no PCB changes needed
- **Filter aggressiveness**: User chose IC1F = 0xC (FDIV16_N8, ~752ns debounce at 170MHz CKD=DIV1)
- **Speed measurement**: Keep current software timestamp approach — NO change to BldcController speed logic
- **Replacement strategy**: Full replacement — no EXTI9_5 fallback, no preprocessor guards
- **Old code**: Delete `Stm32HallSensor` entirely

**Research Findings**:
- STM32G431 TIM1/2/3/4/15 support Hall mode; TIM4 confirmed via CMSIS defines
- HAL provides `HAL_TIMEx_HallSensor_Init/Start/Start_IT` with `TIM_HallSensor_InitTypeDef`
- Hall mode: XORs 3 CH1/CH2/CH3 inputs into TI1 (`TIM_CR2_TI1S`), resets counter on edge, captures in CCR1
- GPIO IDR remains readable in AF mode, so `getPosition()` can still read pin state + POSITION_TABLE lookup
- EXTI9_5 ISR chain: `EXTI9_5_IRQHandler` → `HAL_GPIO_EXTI_Callback` → `motor_controller_hall_interrupt_handler()` → `BldcController::hallSensorInterruptHandler()`
- TIM4 ISR chain will be: `TIM4_IRQHandler` → `HAL_TIM_IC_CaptureCallback` → same `motor_controller_hall_interrupt_handler()`

### Metis Review
**Identified Gaps** (addressed):
- **In-place replacement**: User chose to replace class in existing `stm32_hall_sensor.hpp`/`.cpp` — no new files, no deletion
- **Flash/RAM budget**: New impl must not exceed old impl size significantly; will verify with build size check
- **EXTI9_5 NVIC cleanup**: Must fully disable old EXTI9_5 NVIC for Hall pins, verify no other EXTI users
- **Delta validation preserved**: ±2 step validation in `BldcController::hallSensorInterruptHandler()` stays untouched
- **Invalid state handling**: 0x00 and 0x07 (all low/all high) still return 0xFF via existing POSITION_TABLE

---

## Work Objectives

### Core Objective
Replace GPIO EXTI-based Hall sensor reading with TIM4 Hall Sensor Interface on STM32G431 to add hardware debounce filtering (~752ns), fully replacing the existing `Stm32HallSensor` implementation.

### Concrete Deliverables
- `libecu/hal/stm32g4/stm32_hall_sensor.hpp` — Class renamed `Stm32HallSensor` → `Stm32TimHallSensor`; TIM4 HAL types added
- `libecu/hal/stm32g4/stm32_hall_sensor.cpp` — TIM4 Hall mode impl replaces GPIO reading; `initialize()` + `getPosition()` updated
- `STM32G431/Core/Src/main.cpp` — Type changed `Stm32HallSensor` → `Stm32TimHallSensor`
- `STM32G431/Core/Src/stm32g4xx_it.c` — TIM4_IRQHandler replaces EXTI9_5 Hall chain
- `STM32G431/Core/Src/stm32g4xx_hal_msp.c` — TIM4 HAL MSP initialization added; old EXTI Hall config removed
- Old EXTI9_5 Hall interrupt wiring removed from `stm32g4xx_it.c`

### Definition of Done
- [ ] `./build.sh` succeeds (Debug) with `-Os`, zero new warnings
- [ ] `./build.sh --release` succeeds
- [ ] Flash size increase < 1KB compared to baseline
- [ ] Old `Stm32HallSensor` files fully deleted, no remaining references
- [ ] EXTI9_5 no longer calls any Hall-related handler
- [ ] TIM4 ISR correctly triggers `BldcController::hallSensorInterruptHandler()` on Hall transitions

### Must Have
- Hardware debounce via TIM4 IC1F = 0xC (FDIV16_N8, ~752ns)
- `HallInterface` contract preserved: `initialize()` + `getPosition()` returns 0-5 or 0xFF
- `BldcController` and `CommutationController` unchanged — only HAL impl changes
- Static allocation only, no heap usage
- Existing delta validation (±2 steps) preserved
- Existing POSITION_TABLE mapping preserved

### Must NOT Have (Guardrails)
- NO changes to `BldcController::hallSensorInterruptHandler()` or speed measurement logic
- NO changes to `CommutationController::update()` or commutation table
- NO hardware commutation via TIM1/TIM8 TRGO (software only, as before)
- NO fallback to GPIO EXTI — clean full replacement
- NO `#ifdef` guards switching between implementations
- NO change to `HallInterface` abstract interface signature
- NO new timers affected (TIM1=PWM, TIM2=timing must stay untouched)
- NO EXTI9_5 Hall-related code remaining after migration

---

## Verification Strategy

### Test Decision
- **Infrastructure exists**: NO (no libecu/tests/ directory, confirmed in AGENTS.md)
- **Automated tests**: None (project has Python simulations only, not firmware tests)
- **Framework**: None
- **Agent-Executed QA**: MANDATORY for all tasks (see QA scenarios below)

### QA Policy
Every task includes agent-executed QA scenarios. Evidence saved to `.sisyphus/evidence/task-{N}-{scenario-slug}.{ext}`.

- **Build verification**: Bash — `./build.sh`, check exit code + output
- **Code inspection**: Bash — grep for forbidden patterns, check file existence
- **Size check**: Bash — `arm-none-eabi-size` on .elf files, compare before/after

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Start Immediately — new TIM4 HAL implementation):
├── Task 1: Rename Stm32HallSensor → Stm32TimHallSensor in header [quick]
├── Task 2: Rewrite .cpp impl with TIM4 Hall mode [unspecified-high]
└── Task 3: TIM4 HAL MSP + pin AF config [quick]

Wave 2 (After Wave 1 — integrate into application):
├── Task 4: main.cpp type swap [quick]
├── Task 5: stm32g4xx_it.c — replace EXTI9_5 with TIM4 ISR [quick]
└── Task 6: Search + confirm no old references remain [quick]

Wave 3 (After Wave 2 — build + verify):
└── Task 7: Build verification (Debug + Release) + size check [quick]

Wave FINAL (After ALL tasks — parallel reviews):
├── Task F1: Plan compliance audit (oracle)
├── Task F2: Code quality review (unspecified-high)
├── Task F3: Manual QA verification (unspecified-high)
└── Task F4: Scope fidelity check (deep)
-> Present results -> Get explicit user okay

Critical Path: Task 2 → Task 4 → Task 7 → F1-F4
Parallel Speedup: ~55% faster than sequential
Max Concurrent: 3 (Wave 1, Wave 2)
```

### Dependency Matrix

| Task | Blocked By | Blocks |
|------|-----------|--------|
| 1 | None | 2, 4 |
| 2 | 1 | 4, 6 |
| 3 | None | 4, 5 |
| 4 | 1, 2, 3 | 6, 7 |
| 5 | 3 | 6 |
| 6 | 2, 4, 5 | 7 |
| 7 | 6 | F1-F4 |

### Agent Dispatch Summary

- **Wave 1 (3 tasks)**: T1 → `quick`, T2 → `unspecified-high`, T3 → `quick`
- **Wave 2 (3 tasks)**: T4 → `quick`, T5 → `quick`, T6 → `quick`
- **Wave 3 (1 task)**: T7 → `quick`
- **FINAL (4 tasks)**: F1 → `oracle`, F2 → `unspecified-high`, F3 → `unspecified-high`, F4 → `deep`

---

## TODOs

- [x] 1. Rename `Stm32HallSensor` → `Stm32TimHallSensor` in header file

  **What to do**:
  - In `libecu/hal/stm32g4/stm32_hall_sensor.hpp` (IN-PLACE):
    - Update `@brief`: "STM32G4 TIM4 Hall Sensor Interface implementation" (was "GPIO-based")
    - Rename class `Stm32HallSensor` → `Stm32TimHallSensor`
    - Add private member: `TIM_HandleTypeDef tim_handle_` (not pointer — static allocation)
    - Add forward declare `TIM_HandleTypeDef` before class or include `stm32g4xx_hal.h`
    - Keep `HallGpioConfig` struct unchanged
    - Keep include guard (update if includes new header)
    - Keep `readGpioPin()` private method (still needed for getPosition AF-mode GPIO read)
    - Add `static constexpr uint8_t POSITION_TABLE[8]` inline (move from .cpp if desired, or keep in .cpp)
    - Update Doxygen comments to reflect TIM4 Hall mode, not GPIO

  **Must NOT do**:
  - Do NOT rename the file (keep `stm32_hall_sensor.hpp`)
  - Do NOT change `HallInterface` base class
  - Do NOT change `HallGpioConfig` struct
  - Do NOT add `virtual` methods or change interface contract

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: In-place rename + header update, single file
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: YES (Wave 1, with Tasks 2, 3)
  - **Parallel Group**: Wave 1
  - **Blocks**: Task 2 (impl needs header), Task 4 (main.cpp type swap)
  - **Blocked By**: None

  **References**:
  - `libecu/hal/stm32g4/stm32_hall_sensor.hpp` — current file to modify (see user context above)
  - `libecu/include/interfaces/hall_interface.hpp` — base class contract
  - `STM32G431/Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal_tim_ex.h` — `TIM_HandleTypeDef`, `HAL_TIMEx_HallSensor_Init`

  **Acceptance Criteria**:
  - Class name is `Stm32TimHallSensor : public HallInterface`
  - File still named `stm32_hall_sensor.hpp` (no rename)
  - `HallGpioConfig` struct unchanged
  - `initialize()` and `getPosition()` override declarations present
  - `TIM_HandleTypeDef tim_handle_` private member added
  - `HallInterface` NOT modified

  **QA Scenarios**:
  ```
  Scenario: Header compiles after changes
    Tool: Bash
    Steps:
      1. arm-none-eabi-g++ -Os -std=c++17 -fno-exceptions -fno-rtti -fsyntax-only -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Ilibecu/include -ISTM32G431/Drivers/STM32G4xx_HAL_Driver/Inc -ISTM32G431/Drivers/CMSIS/Include -ISTM32G431/Drivers/CMSIS/Device/ST/STM32G4xx/Include libecu/hal/stm32g4/stm32_hall_sensor.hpp 2>&1 | tee .sisyphus/evidence/task-1-header-compiles.txt
    Expected Result: Exit code 0, no errors
    Evidence: .sisyphus/evidence/task-1-header-compiles.txt
  ```

  **Commit**: YES (groups 1-3)
  - Message: `refactor(hall): rename Stm32HallSensor → Stm32TimHallSensor for TIM4 mode`
  - Files: `libecu/hal/stm32g4/stm32_hall_sensor.hpp`

- [x] 2. Rewrite `.cpp` implementation with TIM4 Hall mode

  **What to do**:
  - In `libecu/hal/stm32g4/stm32_hall_sensor.cpp` (IN-PLACE):
    - Update `@file` brief: "STM32G4 TIM4 Hall Sensor Interface implementation"
    - Update constructor: `Stm32TimHallSensor::Stm32TimHallSensor(...)`
    - **Rewrite `initialize()`**:
      - Zero-init `tim_handle_`: `tim_handle_ = {0}`
      - Set `tim_handle_.Instance = TIM4`
      - Base config: `Prescaler=0`, `CounterMode=UP`, `Period=0xFFFF`, `ClockDivision=DIV1`
      - Hall config: `IC1Polarity=RISING`, `IC1Prescaler=DIV1`, `IC1Filter=0x0C`, `Commutation_Delay=0`
      - Call `__HAL_RCC_TIM4_CLK_ENABLE()` and `HAL_TIMEx_HallSensor_Init(&tim_handle_, &hall_config)`
      - Call `HAL_TIMEx_HallSensor_Start_IT(&tim_handle_)`
      - Enable NVIC: `HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0)`, `HAL_NVIC_EnableIRQ(TIM4_IRQn)`
      - Keep `return true/false` pattern (no GPIO EXTI setup)
    - **Rewrite `getPosition()`**:
      - Remove old GPIO polling via `HAL_GPIO_ReadPin()`
      - Keep `readGpioPin()` usage OR rewrite to read `GPIOB->IDR` directly
      - 3-bit state: bit0=A_Pin, bit1=B_Pin, bit2=C_Pin (still works in AF mode)
      - POSITION_TABLE lookup: `{0xFF, 0, 2, 1, 4, 5, 3, 0xFF}` — unchanged
    - **Remove** old EXTI GPIO init code from `initialize()` (GPIO_MODE_IT_RISING_FALLING, pull-up, etc.)
    - **Keep** `readGpioPin()` if still used, or remove if inline GPIOB->IDR used

  **Must NOT do**:
  - Do NOT rename the file (keep `stm32_hall_sensor.cpp`)
  - Do NOT implement speed measurement
  - Do NOT use `double` or heap allocation
  - Do NOT change POSITION_TABLE values

  **Recommended Agent Profile**:
  - **Category**: `unspecified-high`
    - Reason: Rewrite of TIM4 HAL impl, register-level configuration
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: YES (Wave 1, with Tasks 1, 3)
  - **Parallel Group**: Wave 1
  - **Blocks**: Task 4 (main.cpp), Task 6 (grep verification)
  - **Blocked By**: Task 1 (needs header class name)

  **References**:
  - `libecu/hal/stm32g4/stm32_hall_sensor.cpp` — current GPIO impl to replace
  - `libecu/hal/stm32g4/stm32_hall_sensor.hpp` — updated header (Task 1)
  - `STM32G431/Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal_tim_ex.h` — `HAL_TIMEx_HallSensor_Init/Start_IT`
  - `STM32G431/Drivers/STM32G4xx_HAL_Driver/Inc/stm32g4xx_hal_tim.h` — `TIM_HandleTypeDef` structure
  - `stm32g4xx_ll_tim.h` lines 844-859 — IC filter table (value 0xC = FDIV16_N8)

  **Acceptance Criteria**:
  - Position table preserved: `{0xFF, 0, 2, 1, 4, 5, 3, 0xFF}`
  - `IC1Filter = 0x0C` in Hall config
  - `HAL_TIMEx_HallSensor_Init()` called (no GPIO EXTI setup)
  - `HAL_TIMEx_HallSensor_Start_IT()` called
  - `getPosition()` reads pin state via IDF + POSITION_TABLE
  - Constructor name: `Stm32TimHallSensor::Stm32TimHallSensor`
  - No `double`, no `new`/`malloc`

  **QA Scenarios**:
  ```
  Scenario: .cpp compiles with updated header
    Tool: Bash
    Steps:
      1. arm-none-eabi-g++ -Os -std=c++17 -fno-exceptions -fno-rtti -fsyntax-only -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Ilibecu/include -ISTM32G431/Drivers/STM32G4xx_HAL_Driver/Inc -ISTM32G431/Drivers/CMSIS/Include -ISTM32G431/Drivers/CMSIS/Device/ST/STM32G4xx/Include -ISTM32G431/Core/Inc libecu/hal/stm32g4/stm32_hall_sensor.cpp 2>&1 | tee .sisyphus/evidence/task-2-cpp-compiles.txt
    Expected Result: Exit code 0, no errors
    Evidence: .sisyphus/evidence/task-2-cpp-compiles.txt
  ```

  **Commit**: YES (groups 1-3)
  - Message: `refactor(hall): implement TIM4 Hall Sensor Interface with hardware debounce`
  - Files: `libecu/hal/stm32g4/stm32_hall_sensor.cpp`

- [x] 3. TIM4 HAL MSP init + GPIO AF pin remap

  **What to do**:
  - In `STM32G431/Core/Src/stm32g4xx_hal_msp.c`:
    - Add `HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef* htim)`:
      - Check `if (htim->Instance == TIM4)`
      - Enable `__HAL_RCC_TIM4_CLK_ENABLE()`
      - Configure PB6, PB7, PB8: `GPIO_MODE_AF_PP`, `GPIO_NOPULL`, `GPIO_SPEED_LOW`, `GPIO_AF2_TIM4`
      - Use `A__GPIO_Port` and pin macros from main.h
    - Add `HAL_TIMEx_MSPDeInit()` stub for TIM4 if `HAL_TIMEx_HallSensor_MspDeInit` pattern needed
  - Remove old EXTI Hall MSP config from `HAL_GPIO_MspInit()` or `MX_GPIO_Init()`:
    - Remove PB6/PB7/PB8 setup as `GPIO_MODE_IT_RISING_FALLING`
    - Remove EXTI9_5 NVIC enable for Hall pins if no other EXTI9_5 users exist
  - Keep `HallGpioConfig` struct (unchanged by this task)

  **Must NOT do**:
  - Do NOT touch TIM1 MSP
  - Do NOT touch TIM2 MSP
  - Do NOT remove non-Hall EXTI config

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Standard STM32CubeMX MSP pattern
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: YES (Wave 1)
  - **Parallel Group**: Wave 1 (with Tasks 1, 2)
  - **Blocks**: Task 4, Task 5
  - **Blocked By**: None

  **References**:
  - `STM32G431/Core/Src/stm32g4xx_hal_msp.c` — existing MSPs for TIM1, TIM2
  - `STM32G431/Core/Inc/main.h` — `A__Pin`, `B__Pin`, `Z__Pin`, `A__GPIO_Port`

  **Acceptance Criteria**:
  - `HAL_TIMEx_HallSensor_MspInit()` exists and handles TIM4
  - TIM4 clock enabled
  - PB6/PB7/PB8 as AF2_TIM4 with NOPULL
  - TIM4_IRQn enabled at priority 1
  - Old EXTI Hall MSP config removed

  **QA Scenarios**:
  ```
  Scenario: MSP file compiles
    Tool: Bash
    Steps:
      1. arm-none-eabi-gcc -Os -std=c17 -fsyntax-only -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -DSTM32G431xx -ISTM32G431/Drivers/STM32G4xx_HAL_Driver/Inc -ISTM32G431/Drivers/CMSIS/Include -ISTM32G431/Drivers/CMSIS/Device/ST/STM32G4xx/Include -ISTM32G431/Core/Inc STM32G431/Core/Src/stm32g4xx_hal_msp.c 2>&1 | tee .sisyphus/evidence/task-3-msp-compiles.txt
    Expected Result: Exit code 0
    Evidence: .sisyphus/evidence/task-3-msp-compiles.txt
  ```

  **Commit**: YES (groups 1-3)
  - Message: `refactor(hall): add TIM4 HAL MSP init, remove EXTI9_5 Hall MSP`
  - Files: `STM32G431/Core/Src/stm32g4xx_hal_msp.c`

- [x] 4. Update `main.cpp` type swap

  **What to do**:
  - In `STM32G431/Core/Src/main.cpp`:
    - Replace `Stm32HallSensor` → `Stm32TimHallSensor` in static allocation:
      ```cpp
      // Before: static libecu::Stm32HallSensor hall_sensor(hall_config);
      // After:  static libecu::Stm32TimHallSensor hall_sensor(hall_config);
      ```
    - Include path stays the same: `#include "../../libecu/hal/stm32g4/stm32_hall_sensor.hpp"`
    - `hall_config` struct unchanged
    - `hall_sensor.initialize()` call unchanged
    - Update comments if referencing GPIO → TIM4

  **Must NOT do**:
  - Do NOT change include path (file name unchanged)
  - Do NOT change initialization order
  - Do NOT change `BldcController` setup

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Single type rename, one file
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: YES (Wave 2)
  - **Parallel Group**: Wave 2
  - **Blocks**: Task 6, Task 7
  - **Blocked By**: Tasks 1, 2, 3

  **References**:
  - `STM32G431/Core/Src/main.cpp:39-44` — static allocation line
  - `libecu/hal/stm32g4/stm32_hall_sensor.hpp` — new class name

  **Acceptance Criteria**:
  - Zero references to `Stm32HallSensor` in main.cpp
  - `Stm32TimHallSensor` used in static allocation
  - Include path unchanged
  - All method calls unchanged

  **QA Scenarios**:
  ```
  Scenario: No old class name references in main.cpp
    Tool: Bash
    Steps:
      1. grep -n 'Stm32HallSensor[^T]' STM32G431/Core/Src/main.cpp | tee .sisyphus/evidence/task-4-no-old-refs.txt || echo "No matches" >> .sisyphus/evidence/task-4-no-old-refs.txt
    Expected Result: No matches
    Evidence: .sisyphus/evidence/task-4-no-old-refs.txt
  ```

  **Commit**: YES (groups 4-5)
  - Message: `refactor(hall): wire Stm32TimHallSensor in main.cpp`
  - Files: `STM32G431/Core/Src/main.cpp`

- [x] 5. Replace EXTI9_5 Hall ISR with TIM4 ISR

  **What to do**:
  - In `STM32G431/Core/Src/stm32g4xx_it.c`:
    - Add `TIM4_IRQHandler()` → calls `HAL_TIM_IRQHandler()` or directly forwards
    - Add `HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef*)` override: if `htim->Instance == TIM4`, call `motor_controller_hall_interrupt_handler()`
    - Remove PB6/PB7/PB8 from `HAL_GPIO_EXTI_Callback()` check list
    - Remove PB6/PB7/PB8 from `EXTI9_5_IRQHandler()` (or entire handler if no other users)

  **Must NOT do**:
  - Do NOT change TIM1/TIM2 IRQHandlers
  - Do NOT remove non-Hall EXTI handlers

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Standard ISR replacement pattern
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: YES (Wave 2)
  - **Parallel Group**: Wave 2
  - **Blocks**: Task 6, Task 7
  - **Blocked By**: Task 3

  **References**:
  - `STM32G431/Core/Src/stm32g4xx_it.c` — current `EXTI9_5_IRQHandler`, `HAL_GPIO_EXTI_Callback()`
  - `STM32G431/Core/Src/main.cpp` — `extern "C" void motor_controller_hall_interrupt_handler(void)`
  - Existing `TIM1_UP_TIM16_IRQHandler()` — TIM ISR pattern

  **Acceptance Criteria**:
  - `TIM4_IRQHandler()` exists
  - `HAL_TIM_IC_CaptureCallback()` handles TIM4
  - PB6/PB7/PB8 removed from EXTI callbacks

  **QA Scenarios**:
  ```
  Scenario: No Hall pins in EXTI callbacks
    Tool: Bash
    Steps:
      1. grep -n 'A__Pin\|B__Pin\|Z__Pin\|GPIO_PIN_6\|GPIO_PIN_7\|GPIO_PIN_8' STM32G431/Core/Src/stm32g4xx_it.c | tee .sisyphus/evidence/task-5-no-hall-exti.txt || echo "No matches" >> .sisyphus/evidence/task-5-no-hall-exti.txt
    Expected Result: No matches in EXTI9_5_IRQHandler or HAL_GPIO_EXTI_Callback
    Evidence: .sisyphus/evidence/task-5-no-hall-exti.txt

  Scenario: TIM4_IRQHandler exists
    Tool: Bash
    Steps:
      1. grep -n 'TIM4_IRQHandler' STM32G431/Core/Src/stm32g4xx_it.c | tee .sisyphus/evidence/task-5-tim4-isr.txt
    Expected Result: Function exists
    Evidence: .sisyphus/evidence/task-5-tim4-isr.txt
  ```

  **Commit**: YES (groups 4-5)
  - Message: `refactor(hall): replace EXTI9_5 Hall ISR with TIM4 handler`
  - Files: `STM32G431/Core/Src/stm32g4xx_it.c`

- [x] 6. Scan codebase for orphaned old references

  **What to do**:
  - Search entire codebase for `Stm32HallSensor` (note: `Stm32TimHallSensor` should exist as the new name):
    ```bash
    grep -rn 'Stm32HallSensor[^T]' --include='*.cpp' --include='*.hpp' --include='*.c' --include='*.h' --include='CMakeLists.txt' --include='*.md' .
    ```
  - Search for `stm32_hall_sensor` references in comments/docs (file name unchanged, so most references should stay valid)
  - If any remaining references found, fix or explain them
  - Verify `libecu` builds standalone: `cd libecu && make clean && make`

  **Must NOT do**:
  - Do NOT modify source beyond fixing orphaned references

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: grep search + build verification
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: YES (Wave 2)
  - **Parallel Group**: Wave 2
  - **Blocks**: Task 7
  - **Blocked By**: Tasks 2, 4, 5

  **References**:
  - Entire codebase

  **Acceptance Criteria**:
  - Zero references to `Stm32HallSensor` (without `Tim` prefix)
  - `libecu` builds: `cd libecu && make clean && make` succeeds

  **QA Scenarios**:
  ```
  Scenario: No orphaned Stm32HallSensor references
    Tool: Bash
    Steps:
      1. grep -rn 'Stm32HallSensor[^T]' --include='*.cpp' --include='*.hpp' --include='*.c' --include='*.h' . 2>&1 | tee .sisyphus/evidence/task-6-no-orphans.txt || echo "Clean" >> .sisyphus/evidence/task-6-no-orphans.txt
    Expected Result: Clean (no matches, or only plan files referencing old name)
    Evidence: .sisyphus/evidence/task-6-no-orphans.txt
  ```

  **Commit**: YES (groups 4-5)
  - Message: `refactor(hall): clean up orphaned Stm32HallSensor references`
  - Files: (any files with fixes, or empty if clean)

- [x] 7. Build verification + size check

  **What to do**:
  - Record baseline: `./build.sh` then `arm-none-eabi-size STM32G431/build/open-ecu.elf`
  - `./build.sh --clean` for fresh build
  - `./build.sh` (Debug) — capture output, verify exit 0
  - `./build.sh --release` — capture output, verify exit 0
  - `arm-none-eabi-size` on both debug and release ELF
  - Verify Flash increase < 1KB, RAM increase < 2KB vs baseline
  - Verify zero NEW warnings
  - All evidence in `.sisyphus/evidence/task-7-*`

  **Must NOT do**:
  - Do NOT modify source code

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Build + size check
  - **Skills**: `[]`

  **Parallelization**:
  - **Can Run In Parallel**: NO (Wave 3, after Wave 2 complete)
  - **Parallel Group**: Wave 3
  - **Blocks**: F1-F4
  - **Blocked By**: Task 6

  **Acceptance Criteria**:
  - Debug build: exit 0
  - Release build: exit 0
  - Flash delta < 1KB
  - RAM delta < 2KB
  - No new warnings
  - All evidence files present

  **QA Scenarios**:
  ```
  Scenario: Baseline size snapshot before clean build
    Tool: Bash
    Steps:
      1. ./build.sh 2>/dev/null && arm-none-eabi-size STM32G431/build/open-ecu.elf 2>&1 | tee .sisyphus/evidence/task-7-baseline-size.txt
    Expected Result: Size output captured
    Evidence: .sisyphus/evidence/task-7-baseline-size.txt

  Scenario: Fresh Debug build succeeds
    Tool: Bash
    Steps:
      1. ./build.sh --clean 2>&1 | tee .sisyphus/evidence/task-7-clean.txt
      2. ./build.sh 2>&1 | tee .sisyphus/evidence/task-7-debug.txt
    Expected Result: Exit 0, ELF exists
    Evidence: .sisyphus/evidence/task-7-debug.txt

  Scenario: Release build succeeds
    Tool: Bash
    Steps:
      1. ./build.sh --release 2>&1 | tee .sisyphus/evidence/task-7-release.txt
    Expected Result: Exit 0, ELF exists
    Evidence: .sisyphus/evidence/task-7-release.txt

  Scenario: Size within limits
    Tool: Bash
    Steps:
      1. arm-none-eabi-size STM32G431/build/open-ecu.elf 2>&1 | tee .sisyphus/evidence/task-7-size-debug.txt
      2. arm-none-eabi-size STM32G431/build-release/open-ecu.elf 2>&1 | tee .sisyphus/evidence/task-7-size-release.txt
    Expected Result: Flash delta < 1KB, RAM delta < 2KB
    Evidence: .sisyphus/evidence/task-7-size-delta.txt
  ```

  **Commit**: NO (verification only)

## Final Verification Wave (MANDATORY — after ALL implementation tasks)

> 4 review agents run in PARALLEL. ALL must APPROVE.

- [ ] F1. **Plan Compliance Audit** — `oracle`
  For each "Must Have": verify `Stm32TimHallSensor` in same files, IC1Filter=0xC in code, `HallInterface` unchanged, `BldcController` untouched, `POSITION_TABLE` preserved, `HallGpioConfig` unchanged. For each "Must NOT Have": grep for forbidden patterns. Check evidence files exist for all tasks 1-7.
  Output: `Must Have [N/N] | Must NOT Have [N/N] | Tasks [N/N] | VERDICT: APPROVE/REJECT`

- [ ] F2. **Code Quality Review** — `unspecified-high`
  Run `./build.sh` + `cd libecu && make` + `./build.sh --release`. Review all changed files for: `double` usage (forbidden), heap allocation (forbidden), unused imports. Check IC1Filter=0x0C not accidentally 0xC0.
  Output: `Build [PASS/FAIL] | Files [N clean/N issues] | VERDICT`

- [ ] F3. **Real Manual QA** — `unspecified-high`
  Execute all QA scenarios from tasks 1-7 — follow exact steps, capture evidence. Verify compile checks pass, grep verifications clean, build succeeds, size within limits. Test cross-task integration.
  Output: `Scenarios [N/N pass] | Integration [N/N] | VERDICT`

- [ ] F4. **Scope Fidelity Check** — `deep`
  For each task: read "What to do", read diff. Verify 1:1 — everything in spec, no creep. Check "Must NOT do" compliance: no BldcController changes, no speed measurement changes, no HallInterface changes.
  Output: `Tasks [N/N compliant] | Contamination [CLEAN/N issues] | VERDICT`

## Commit Strategy

- **1**: `refactor(hall): rename Stm32HallSensor → Stm32TimHallSensor for TIM4 mode` — `stm32_hall_sensor.hpp`
- **2**: `refactor(hall): implement TIM4 Hall Sensor Interface with hardware debounce` — `stm32_hall_sensor.cpp`
- **3**: `refactor(hall): add TIM4 HAL MSP init, remove EXTI9_5 Hall MSP` — `stm32g4xx_hal_msp.c`
- **4**: `refactor(hall): wire Stm32TimHallSensor in main.cpp` — `main.cpp`
- **5**: `refactor(hall): replace EXTI9_5 Hall ISR with TIM4 handler` — `stm32g4xx_it.c`
- **6**: `refactor(hall): clean up orphaned Stm32HallSensor references` — (if any fixes needed)

All: pre-commit = `./build.sh` passes

---

## Success Criteria

### Verification Commands
```bash
./build.sh                        # Expected: exit 0, ELF in STM32G431/build/
./build.sh --release              # Expected: exit 0, ELF in STM32G431/build-release/
cd libecu && make                 # Expected: exit 0
arm-none-eabi-size STM32G431/build/open-ecu.elf   # Expected: Flash delta < 1KB
grep -rn 'Stm32HallSensor[^T]' .  # Expected: no matches (except git plan files)
```

### Final Checklist
- [ ] Class renamed to `Stm32TimHallSensor` (same files)
- [ ] TIM4 Hall mode configured with IC1Filter = 0x0C (~752ns debounce)
- [ ] PB6/PB7/PB8 remapped to AF2_TIM4 (no longer EXTI)
- [ ] TIM4 ISR calls `motor_controller_hall_interrupt_handler()`
- [ ] `BldcController` and `CommutationController` unchanged
- [ ] `POSITION_TABLE` preserved: `{0xFF, 0, 2, 1, 4, 5, 3, 0xFF}`
- [ ] Zero orphaned `Stm32HallSensor` references
- [ ] EXTI9_5 no longer handles Hall pins
- [ ] `HallInterface` abstract class unchanged
- [ ] `HallGpioConfig` struct unchanged
- [ ] Build succeeds (Debug + Release), no new warnings
