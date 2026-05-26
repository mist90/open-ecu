
## T2: Rewrite stm32_hall_sensor.cpp to TIM4 Hall Mode

### Approach
- Completely replaced GPIO EXTI-based implementation with TIM4 Hall Sensor Interface
- Class name changed from `Stm32HallSensor` to `Stm32TimHallSensor` (header already updated in T1)
- `initialize()` now calls `HAL_TIMEx_HallSensor_Init()` + `HAL_TIMEx_HallSensor_Start_IT()`
- `getPosition()` reads `GPIOB->IDR` directly (faster than HAL_GPIO_ReadPin)
- IC1Filter=0x0C provides ~752ns digital debounce at fDTS=170MHz

### Key Details
- `tim_handle_` is a value member (not pointer), initialized via `tim_handle_()` in constructor
- `POSITION_TABLE` defined in .cpp since header declares `static const uint8_t POSITION_TABLE[8]` without inline values
- `__HAL_RCC_TIM4_CLK_ENABLE()` must be called BEFORE `HAL_TIMEx_HallSensor_Init()` — MSP needs the clock
- `readGpioPin()` kept for API stability though unused in TIM4 mode

### main.cpp Fix Required
- Updated `main.cpp` line 41: `Stm32HallSensor` → `Stm32TimHallSensor`
- This was needed because the class name change (T1) broke the application instantiation

### Build Verification
- ARM GCC 13.2.1 clean build passes: 74944 text + 480 data + 2768 bss = 78192 bytes total
- Clangd LSP shows false errors (HAL headers not in include path) — ignore, trust ARM GCC build

### Still TODO in Other Tasks
- TIM4_IRQHandler in `stm32g4xx_it.c` to handle commutation interrupts
- Remove EXTI9_5 setup from `MX_GPIO_Init()` (no longer needed)
- MSP init (`HAL_TIMEx_HallSensor_MspInit`) already exists in `stm32g4xx_hal_msp.c`

## T7: Build Verification Results

### Architecture Fix Applied (during verification)
- **Problem**: `Stm32TimHallSensor::tim_handle_` (class member) was separate from `htim4` (global for IRQ handler)
- **Fix**: Removed `tim_handle_` member, use global `htim4` directly (same pattern as `htim1`)
- **Files modified**: `stm32_hall_sensor.hpp` (-1 member), `stm32_hall_sensor.cpp` (use `extern htim4`)

### Final Sizes
- Debug: text=74848, data=480, bss=2768, total=78096 bytes
- Release: text=53048, data=480, bss=2768, total=56296 bytes
- vs baseline: Flash -96 bytes, RAM 0 bytes — both smaller!

### Key Lesson
- When a HAL handle needs to be used by both init code AND an IRQ handler in a different compilation unit, use a single global `TIM_HandleTypeDef` rather than class members. This matches the project's existing pattern for `htim1` and `htim2`.

## F3 QA Verification Results ($(date +%Y-%m-%d))

### Compile command gotcha
- Plan's Task 1 compile command is missing `-DSTM32G431xx` and `-ISTM32G431/Core/Inc`. Without these the HAL chain fails on `stm32g4xx_hal_conf.h` and `stm32g4xx.h #error`. The corrected flags work fine.
- Task 2 compile command also needs `-DSTM32G431xx` (same reason). Plan command as written would show spurious errors.
- Task 3 (gcc -DSTM32G431xx was correct in plan) — passes cleanly.

### htim4 handle pattern
- Declared as `TIM_HandleTypeDef htim4;` in main.cpp (line 36).
- `extern TIM_HandleTypeDef htim4;` in both stm32g4xx_it.c (line 18) and stm32_hall_sensor.cpp (line 11) — consistent shared handle pattern, same as htim1/htim2.

### Callback chain confirmed
- `TIM4_IRQHandler` → `HAL_TIM_IRQHandler(&htim4)` (stm32g4xx_it.c:109-111)
- `HAL_TIM_IC_CaptureCallback` handles TIM4 → `motor_controller_hall_interrupt_handler()` (stm32g4xx_it.c:118-122)
- `motor_controller_hall_interrupt_handler` defined in main.cpp:99 → calls `bldc_controller.hallSensorInterruptHandler()`

### EXTI9_5 fully removed
- Zero references to `EXTI9_5_IRQHandler` or `HAL_GPIO_EXTI_Callback` in stm32g4xx_it.c
- Zero GPIO EXTI config for Hall pins anywhere in the codebase

### Build health
- Debug: text=74848, data=480, bss=2768 (57.5% Flash used) — healthy
- Release: text=53048, data=480, bss=2768 (40.8% Flash used) — healthy
- libecu standalone build: clean pass
- Zero warnings in both builds
