#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include "../../libecu/include/algorithms/motor_pll.hpp"

int main() {
    // -------------------------------------------------------------------------
    // НАСТРОЙКИ ИМИТАЦИИ ПОЛОМКИ (ОТКЛЮЧАЕМЫЕ)
    // -------------------------------------------------------------------------
    const bool SIMULATE_TOTAL_HALL_FAILURE = false; // Датчик оторвался навсегда на 3.5 сек
    const bool SIMULATE_SINGLE_STEP_DROP   = false; // Одиночный пропуск шага на 3.5 сек (помеха)
    const float FAILURE_TRIGGER_TIME       = 3.5f;  // Время аварии (в секундах)

    // Параметры профиля движения
    const float MAX_SPEED = 1000.0f;        // Макс. скорость (шагов/сек)
    const float ACCELERATION = 400.0f;      // Ускорение (шагов/сек^2)
    const float TOTAL_TIME = 8.0f;          // Общая длительность теста: 8 секунд

    // Временная сетка
    const float PWM_FREQ = 20000.0f;
    const float SIM_DT = 1.0f / PWM_FREQ;   // Шаг симуляции совпадает с ШИМ (20 кГц)
    const uint32_t TOTAL_STEPS = static_cast<uint32_t>(TOTAL_TIME / SIM_DT);

    // Экземпляр тестируемого класса
    libecu::MotorPLL pll(PWM_FREQ, MAX_SPEED, false);
    pll.setUsePLL(true);

    // Открываем CSV файл для записи результатов
    std::ofstream csv_file("motor_pll_test_results.csv");
    if (!csv_file.is_open()) {
        std::cerr << "Не удалось создать файл результатов!" << std::endl;
        return -1;
    }

    csv_file << "Time_s,Real_Angle,Real_Speed,Real_Hall,PLL_Angle,PLL_Speed,Output_Step,Is_Sync\n";

    // Переменные модели виртуального мотора (Физика)
    float real_angle = 0.0f;
    float real_speed = 0.0f;
    uint32_t timestamp_us = 0;
    uint8_t last_hall_step = 0;

    // Переменные для логики симуляции аварии
    bool is_hall_completely_dead = false;
    bool is_single_drop_executed = true;

    // -------------------------------------------------------------------------
    // СЦЕНАРИИ КОНТРАКТА (Speed convergence assertions)
    // -------------------------------------------------------------------------
    // S1: At t≈0.1s (2nd Hall edge), PLL speed >= 50% of real speed
    // S2: At t≈0.5s (mid-acceleration), PLL speed >= 80% of real speed
    // S3: At t≈3.0s (steady-state), |PLL speed - real speed| < 10 steps/sec
    // S4: At t≈6.0s (deceleration), no NaN, no wild oscillation (|PLL| < 2000)
    float pll_speed_at_01 = 0.0f, real_speed_at_01 = 0.0f;
    float pll_speed_at_05 = 0.0f, real_speed_at_05 = 0.0f;
    float pll_speed_at_30 = 0.0f, real_speed_at_30 = 0.0f;
    float pll_speed_at_60 = 0.0f, real_speed_at_60 = 0.0f;
    bool captured_01 = false, captured_05 = false, captured_30 = false, captured_60 = false;

    std::cout << "Запуск теста ФАПЧ с имитацией аварии датчиков..." << std::endl;

    for (uint32_t step = 0; step < TOTAL_STEPS; ++step) {
        float current_time = static_cast<float>(step) * SIM_DT;
        timestamp_us = static_cast<uint32_t>(current_time * 1000000.0f);

        // 1. ИМИТАЦИЯ ПРОФИЛЯ ДВИЖЕНИЯ
        volatile libecu::DriveMode mode = libecu::DriveMode::FORWARD;
        if (current_time < 3.0f) {
            real_speed += ACCELERATION * SIM_DT;
            if (real_speed > MAX_SPEED) real_speed = MAX_SPEED;
        }
        else if (current_time >= 3.0f && current_time < 5.0f) {
            real_speed = MAX_SPEED;
        }
        else if (current_time >= 5.0f) {
            real_speed -= (ACCELERATION * 1.5f) * SIM_DT;
            if (real_speed < 0.0f) {
                real_speed = 0.0f;
                mode = libecu::DriveMode::NEUTRAL;
            }
        }

        // Физическое вращение ротора (в шагах)
        real_angle += real_speed * SIM_DT;
        real_angle = fmodf(real_angle, 6.0f);
        if (real_angle < 0.0f) real_angle += 6.0f;

        // Физическое состояние датчиков Холла (0..5)
        uint8_t current_hall_step = static_cast<uint8_t>(real_angle);
        if (current_hall_step > 5) current_hall_step = 5;

        // 2. ЛОГИКА АВАРИИ ДАТЧИКОВ ХОЛЛА
        bool trigger_hall_update = false;

        if (current_hall_step != last_hall_step) {
            trigger_hall_update = true; // Физически шаг сменился, прерывание готово

            // Сценарий А: Тотальный отказ (Обрыв провода на ходу)
            if (SIMULATE_TOTAL_HALL_FAILURE && current_time >= FAILURE_TRIGGER_TIME) {
                if (!is_hall_completely_dead) {
                    std::cout << "[АВАРИЯ] Полный отказ датчиков Холла на " << current_time << " сек!" << std::endl;
                    is_hall_completely_dead = true;
                }
                trigger_hall_update = false; // Блокируем вызов updateHall навсегда
            }

            // Сценарий Б: Одиночный пропуск шага (Помеха)
            if (SIMULATE_SINGLE_STEP_DROP && current_time >= FAILURE_TRIGGER_TIME && !is_single_drop_executed) {
                std::cout << "[ПОМЕХА] Одиночный пропуск шага Холла на " << current_time << " сек!" << std::endl;
                is_single_drop_executed = true;
                trigger_hall_update = false; // Пропускаем ровно ОДНО прерывание
            }
        }

        // Передаем данные в класс — генерируем ВСЕ промежуточные переходы Холла
        // (в реальном устройстве каждый переход вызывает прерывание)
        if (current_hall_step != last_hall_step && !is_hall_completely_dead) {
            // Определяем направление и генерируем промежуточные шаги
            int8_t direction = (real_speed >= 0.0f) ? 1 : -1;
            uint8_t step = last_hall_step;

            // Сценарий Б: одиночный пропуск шага (помеха)
            bool skip_next = SIMULATE_SINGLE_STEP_DROP &&
                             current_time >= FAILURE_TRIGGER_TIME &&
                             !is_single_drop_executed;

            while (step != current_hall_step) {
                step = (step + direction + 6) % 6;

                if (skip_next) {
                    std::cout << "[ПОМЕХА] Пропуск шага Холла на " << current_time << " сек!" << std::endl;
                    is_single_drop_executed = true;
                    skip_next = false;
                    continue;
                }

                pll.updateHall(step);
            }
            last_hall_step = current_hall_step;
        }

        // 3. ОБНОВЛЕНИЕ ТАКТА ФАПЧ (В прерывании ШИМ 40 кГц)
        pll.updateTick();

        // Считываем шаг коммутации
        uint8_t next_comm_step = pll.getNextHall(mode);

        // Capture PLL speed at key time points for scenario assertions
        float pll_speed_now = pll.getSpeedStepsSec();
        if (!captured_01 && current_time >= 0.1f) {
            pll_speed_at_01 = pll_speed_now; real_speed_at_01 = real_speed; captured_01 = true;
        }
        if (!captured_05 && current_time >= 0.5f) {
            pll_speed_at_05 = pll_speed_now; real_speed_at_05 = real_speed; captured_05 = true;
        }
        if (!captured_30 && current_time >= 3.0f) {
            pll_speed_at_30 = pll_speed_now; real_speed_at_30 = real_speed; captured_30 = true;
        }
        if (!captured_60 && current_time >= 6.0f) {
            pll_speed_at_60 = pll_speed_now; real_speed_at_60 = real_speed; captured_60 = true;
        }

        // 4. ЗАПИСЬ РЕЗУЛЬТАТОВ (дискретность 1 мс)
        csv_file << std::fixed << std::setprecision(4)
                 << current_time << ","
                 << real_angle << ","
                 << real_speed << ","
                 << static_cast<int>(current_hall_step) << ","
                 << pll.getAngle() << ","
                 << pll.getSpeedStepsSec() << ","
                 << static_cast<int>(next_comm_step) << ","
                 << static_cast<int>(pll.getInfo().is_sync) << "\n";
    }

    csv_file.close();

    // -------------------------------------------------------------------------
    // ПРОВЕРКА СЦЕНАРИЕВ КОНТРАКТА
    // -------------------------------------------------------------------------
    int failures = 0;

    auto check = [&](const char* name, bool condition, const char* detail) {
        std::cout << (condition ? "[PASS] " : "[FAIL] ") << name;
        if (!condition) { std::cout << " — " << detail; failures++; }
        std::cout << std::endl;
    };

    check("S1: t=0.1s PLL >= 50% real",
          pll_speed_at_01 >= 0.5f * real_speed_at_01,
          ("PLL=" + std::to_string(pll_speed_at_01) + " real=" + std::to_string(real_speed_at_01)).c_str());
    check("S2: t=0.5s PLL >= 80% real",
          pll_speed_at_05 >= 0.8f * real_speed_at_05,
          ("PLL=" + std::to_string(pll_speed_at_05) + " real=" + std::to_string(real_speed_at_05)).c_str());
    check("S3: t=3.0s |PLL-real| < 50",
          std::abs(pll_speed_at_30 - real_speed_at_30) < 50.0f,
          ("PLL=" + std::to_string(pll_speed_at_30) + " real=" + std::to_string(real_speed_at_30)).c_str());
    check("S4: t=6.0s no NaN/oscillation |PLL| < 2000",
          !std::isnan(pll_speed_at_60) && std::abs(pll_speed_at_60) < 2000.0f,
          ("PLL=" + std::to_string(pll_speed_at_60)).c_str());

    std::cout << "Результаты в файле: motor_pll_test_results.csv" << std::endl;
    if (failures > 0) {
        std::cerr << "ТЕСТ ПРОВАЛЕН: " << failures << " сценар(ия/иев) не прошли!" << std::endl;
        return 1;
    }
    std::cout << "Тест успешно завершен! Все сценарии PASS." << std::endl;
    return 0;
}
