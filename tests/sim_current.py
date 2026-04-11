#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt

# --- Параметры железа ---
R_motor = 0.3        # Сопротивление фазы мотора, Ом
L_motor = 0.0004      # Индуктивность, Гн
R_ds_on = 0.005      # Сопротивление одного транзистора (5 мОм)
V_bus = 30.0         # Напряжение питания, В
E_back = 0.0         # Противо-ЭДС, В (тормозит рост, ускоряет спад)

# --- Параметры управления ---
f_pwm = 40000        # Частота ШИМ, Гц
dt = 0.1e-6          # Шаг 0.1 мкс для лучшего разрешения фронтов
T_total = 0.06       # Время симуляции

# --- ПИД Коэффициенты ---
Kp, Ki, Kd = 0.1, 50.0, 0.0

# Настройка меандра
f_step = 40          # Частота смены задания
I_target_high, I_target_low = 6.0, 2.0

# Подготовка массивов
time = np.arange(0, T_total, dt)
current = np.zeros_like(time)
target_history = np.zeros_like(time)

# Переменные состояния
i_integral = 0.0
prev_error = 0.0
current_val = 0.0
duty_cycle = 0.0

# Суммарное сопротивление цепи (2 транзистора + мотор)
R_total = R_motor + 2 * R_ds_on

for i, t in enumerate(time):
    # 1. Задание тока (Меандр)
    target = I_target_high if (t * f_step * 2) % 2 < 1 else I_target_low
    target_history[i] = target

    # 2. ПИД-регулятор (вызов раз в период ШИМ)
    if i % int(1 / (f_pwm * dt)) == 0:
        error = target - current_val
        
        potential_integral = i_integral + Ki * (error + prev_error) * 0.5 * (1.0 / f_pwm)
        
        derivative = Kd * (error - prev_error) * f_pwm
        
        duty_cycle = Kp * error + i_integral + derivative
        if duty_cycle > 1.0:
            duty_cycle = 1.0
            if error < 0.0:
                i_integral = potential_integral
        elif duty_cycle < 0.0:
            duty_cycle = 0.0
            if error > 0.0:
                i_integral = potential_integral
        else:
            i_integral = potential_integral
        prev_error = error
        #print(f"{target} {current_val} {error} {duty_cycle} {i_integral}")

    # 3. Физика: Режим Slow Decay
    pwm_step = (t * f_pwm) % 1.0
    if pwm_step < duty_cycle:
        # Ток растет: приложено V_bus
        u_applied = V_bus
    else:
        # Ток спадает: фаза замкнута через нижние ключи
        u_applied = 0.0

    # Моделируем падение на R_total и влияние противо-ЭДС
    # dI = (U_applied - I*R_total - E_back) / L * dt
    di = (u_applied - (current_val * R_total) - E_back) / L_motor * dt
    current_val += di

    # Защита от отрицательного тока (диоды не пустят в обратку в этой схеме)
    if current_val < 0: current_val = 0
    current[i] = current_val

# --- Графики ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

ax1.plot(time * 1000, target_history, 'r--', label='Target')
ax1.plot(time * 1000, current, 'b', label='Current')
ax1.set_title('Динамика тока (Slow Decay)')
ax1.set_ylabel('Ток, А')
ax1.grid(True); ax1.legend()

# Zoom для анализа пульсаций
zoom_range = (time > 0.025) & (time < 0.026)
ax2.plot(time[zoom_range] * 1000, current[zoom_range], 'b')
ax2.set_title('Пульсации тока внутри тактов ШИМ (Zoom)')
ax2.set_xlabel('Время, мс'); ax2.set_ylabel('Ток, А')
ax2.grid(True)

plt.tight_layout()
plt.show()
