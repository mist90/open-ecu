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
    const float MAX_SPEED = 2400.0f;        // Макс. скорость (шагов/сек)
    const float ACCELERATION = 800.0f;      // Ускорение (шагов/сек^2)
    const float TOTAL_TIME = 8.0f;          // Общая длительность теста: 8 секунд

    // Временная сетка
    const float SIM_DT = 1.0f / 40000.0f;   // Шаг симуляции совпадает с ШИМ (40 кГц)
    const uint32_t TOTAL_STEPS = static_cast<uint32_t>(TOTAL_TIME / SIM_DT);

    // Экземпляр тестируемого класса
    const float PWM_FREQ = 40000.0f;
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
    std::cout << "Тест успешно завершен! Результаты в файле: motor_pll_test_results.csv" << std::endl;
    return 0;
}
