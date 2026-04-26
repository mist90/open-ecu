#!/usr/bin/env python3
"""
Симуляция 3-фазного тока БДКТ мотора (звезда)
- 4 связанных ОДУ: 3 фазных тока + потенциал точки звезды Vc
- 6-шаговая коммутация из commutation_controller.cpp
- Метод Эйлера (без scipy)
- Те же ПИД-параметры, что в sim_current.py
"""
import numpy as np
import matplotlib.pyplot as plt

# --- ПИД-параметры (идентично sim_current.py) ---
Kp, Ki, Kd = 0.1, 50.0, 0.0

# --- Параметры железа (идентично sim_current.py) ---
R_motor = 0.3        # Сопротивление фазы мотора, Ом
L_motor = 0.0004     # Индуктивность, Гн
R_ds_on = 0.005      # Сопротивление одного транзистора (5 мОм)
V_bus = 30.0         # Напряжение питания, В
V_diode = 1.0        # Прямое падение на теле диода, В

# --- Параметры управления ---
f_pwm = 40000        # Частота ШИМ, Гц
dt = 0.1e-6          # Шаг 0.1 мкс для лучшего разрешения фронтов
T_total = 0.15       # Время симуляции (6 шагов при 20 Гц = 0.3 с/цикл)

# --- Параметры коммутации ---
f_step = 20          # Частота смены шага коммутации, Гц
I_target = 15.0       # Целевой ток (постоянный)

# --- Состояния фаз (совпадает с PwmState enum в pwm_interface.hpp) ---
OFF = 0              # Высокий импеданс - оба ключа выключены
UP = 1               # PWM на верхнем ключе, нижний комплементарно
DOWN = 2             # Нижний ключ всегда включён, верхний выключен

# Таблица коммутации (ТОЧНО из commutation_controller.cpp)
# Шаг  U     V     W
COMMUTATION_TABLE = [
    [UP,   DOWN, OFF],  # 0: A=UP,   B=DOWN,  C=OFF
    [UP,   OFF,  DOWN], # 1: A=UP,   B=OFF,   C=DOWN
    [OFF,  UP,   DOWN], # 2: A=OFF,  B=UP,    C=DOWN
    [DOWN, UP,   OFF],  # 3: A=DOWN, B=UP,    C=OFF
    [DOWN, OFF,  UP],   # 4: A=DOWN, B=OFF,   C=UP
    [OFF,  DOWN, UP],   # 5: A=OFF,  B=DOWN,  C=UP
]

# Подготовка массивов
time = np.arange(0, T_total, dt)
current_A = np.zeros_like(time)
current_B = np.zeros_like(time)
current_C = np.zeros_like(time)
star_point_v = np.zeros_like(time)
target_history = np.zeros_like(time)
duty_cycle_history = np.zeros_like(time)

# Переменные состояния
i_A = 0.0
i_B = 0.0
i_C = 0.0
Vc = 0.0  # Потенциал точки звезды

i_integral = 0.0
prev_error = 0.0
current_down = 0.0  # Ток фазы в состоянии DOWN (для ПИД)
duty_cycle = 0.0

# Текущий шаг коммутации
comm_step = 0

for i, t in enumerate(time):
    # 1. Определение текущего шага коммутации
    step_index = int(t * f_step) % 6
    if step_index != comm_step:
        comm_step = step_index
        # If you enable this you see harm of PID reset  
        #i_integral = 0.0
        #prev_error = 0.0
        #duty_cycle = 0.0

    states = COMMUTATION_TABLE[comm_step]
    state_A, state_B, state_C = states[0], states[1], states[2]

    # 2. Целевой ток (постоянный)
    target = I_target
    target_history[i] = target

    # 3. ПИД-регулятор (вызов раз в период ШИМ) - ИДЕНТИЧНО sim_current.py
    if i % int(1 / (f_pwm * dt)) == 0:
        error = target - current_down

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
    # 4. Определение напряжений и сопротивлений фаз
    pwm_step = (t * f_pwm) % 1.0  # Положение внутри периода ШИМ
    pwm_on = pwm_step < duty_cycle

    # UP и DOWN: напряжение определено однозначно
    def get_active_VR(state, pwm_on):
        """V(n) и R для UP и DOWN. Для OFF возвращает None."""
        if state == DOWN:
            return 0.0, R_motor + R_ds_on
        elif state == UP:
            return (V_bus if pwm_on else 0.0), R_motor + R_ds_on
        return None, R_motor  # OFF

    states = [state_A, state_B, state_C]
    currents = [i_A, i_B, i_C]

    # Находим OFF-фазу
    off_idx = None
    for j in range(3):
        if states[j] == OFF:
            off_idx = j
            break

    # Шаг 1: V и R для активных фаз (UP, DOWN)
    V = [0.0, 0.0, 0.0]
    R = [0.0, 0.0, 0.0]
    for j in range(3):
        V[j], R[j] = get_active_VR(states[j], pwm_on)

    # Шаг 2: Для OFF-фазы определяем напряжение через состояние диодов
    R[off_idx] = R_motor  # OFF-фаза: только сопротивление обмотки

    sum_V_other = sum(V[j] for j in range(3) if j != off_idx)
    sum_Ri = sum(R[j] * currents[j] for j in range(3))
    # Из диодных уравнений: V(n) - Vc = R·i + L·di/dt → di/dt = (V(n) - Vc - R·i) / L
    # KCL: i_A + i_B + i_C = 0 → дифференцируем: di_A/dt + di_B/dt + di_C/dt = 0
    # Подставляем → (V_A + V_B + V_C - 3·Vc - ΣR·i) / L = 0
    # → 3·Vc = V_A + V_B + V_C - ΣR·i (все 3 фазы активны)
    #
    # OFF-фаза, диоды закрыты → V_off = Vc (плавающая, нет тока)
    # Подставляем: 3·Vc = V_UP + V_DOWN + Vc - ΣR·i
    # → 2·Vc = V_UP + V_DOWN - ΣR·i
    # → Vc_trial = (sum_V_other - sum_Ri) / 2
    Vc_trial = (sum_V_other - sum_Ri) / 2.0

    # Проверяем, открывается ли диод на OFF-фазе
    off_floating = False
    if Vc_trial > V_bus - V_diode:
        V[off_idx] = V_bus - V_diode
    elif Vc_trial < V_diode:
        V[off_idx] = V_diode
    else:
        off_floating = True
        V[off_idx] = Vc_trial

    # Шаг 3: Финальный расчёт Vc с учётом диода
    V_A, V_B, V_C = V[0], V[1], V[2]
    R_A, R_B, R_C = R[0], R[1], R[2]
    Vc = (V_A + V_B + V_C - R_A * i_A - R_B * i_B - R_C * i_C) / 3.0

    # Шаг 4: Производные токов
    di_A_dt = (V_A - Vc - R_A * i_A) / L_motor
    di_B_dt = (V_B - Vc - R_B * i_B) / L_motor
    di_C_dt = (V_C - Vc - R_C * i_C) / L_motor

    # 7. Интегрирование Эйлера
    i_A += di_A_dt * dt
    i_B += di_B_dt * dt
    i_C += di_C_dt * dt

    # 8. Коррекция Кирхгофа
    if off_floating:
        # OFF-фаза обесточена — строгий 0, ошибка только между активными
        active_sum = sum(c for j, c in enumerate([i_A, i_B, i_C]) if j != off_idx)
        corr = active_sum / 2.0
        currents_post = [i_A, i_B, i_C]
        currents_post[off_idx] = 0.0
        for j in range(3):
            if j != off_idx:
                currents_post[j] -= corr
        i_A, i_B, i_C = currents_post
    else:
        current_sum = i_A + i_B + i_C
        corr = current_sum / 3.0
        i_A -= corr
        i_B -= corr
        i_C -= corr

    # 9. Определение тока фазы в состоянии DOWN (для ПИД)
    if state_A == DOWN:
        current_down = -i_A
    elif state_B == DOWN:
        current_down = -i_B
    elif state_C == DOWN:
        current_down = -i_C
    else:
        # Нет фазы в DOWN (теоретически невозможно в 6-шаговой коммутации)
        current_down = 0.0

    # Сохранение результатов
    current_A[i] = i_A
    current_B[i] = i_B
    current_C[i] = i_C
    star_point_v[i] = Vc
    duty_cycle_history[i] = duty_cycle

# --- Графики ---
fig, axes = plt.subplots(5, 1, figsize=(12, 10), sharex=False)

# Фаза A
axes[0].plot(time * 1000, current_A, 'b', label='Phase A', linewidth=0.5)
axes[0].set_title('3-Phase BLDC Current Simulation (6-Step Commutation) — Phase A')
axes[0].set_ylabel('Phase A Current, A')
axes[0].grid(True)
axes[0].legend()

# Фаза B
axes[1].plot(time * 1000, current_B, 'g', label='Phase B', linewidth=0.5)
axes[1].set_title('Phase B')
axes[1].set_ylabel('Phase B Current, A')
axes[1].grid(True)
axes[1].legend()

# Фаза C
axes[2].plot(time * 1000, current_C, 'r', label='Phase C', linewidth=0.5)
axes[2].set_title('Phase C')
axes[2].set_ylabel('Phase C Current, A')
axes[2].grid(True)
axes[2].legend()

# Все 3 фазы вместе
axes[3].plot(time * 1000, current_A, 'b', label='Phase A', linewidth=0.5)
axes[3].plot(time * 1000, current_B, 'g', label='Phase B', linewidth=0.5)
axes[3].plot(time * 1000, current_C, 'r', label='Phase C', linewidth=0.5)
axes[3].set_title('All 3 Phases Overlay')
axes[3].set_ylabel('Current, A')
axes[3].grid(True)
axes[3].legend()

axes[4].plot(time * 1000, duty_cycle_history, 'k', linewidth=0.8)
axes[4].set_title('PID Duty Cycle')
axes[4].set_ylabel('Duty Cycle')
axes[4].set_xlabel('Time, ms')
axes[4].set_ylim(-0.05, 1.05)
axes[4].grid(True)

zoom_start = 0.0
zoom_end = 0.001
zoom_mask = (time >= zoom_start) & (time < zoom_end)

fig_zoom, ax_cur = plt.subplots(figsize=(10, 5))

ax_cur.plot(time[zoom_mask] * 1000, current_A[zoom_mask], 'b', label='Phase A', linewidth=0.8)
ax_cur.plot(time[zoom_mask] * 1000, current_B[zoom_mask], 'g', label='Phase B', linewidth=0.8)
ax_cur.plot(time[zoom_mask] * 1000, current_C[zoom_mask], 'r', label='Phase C', linewidth=0.8)
ax_cur.set_title(f'Zoom ({zoom_start*1000:.1f}–{zoom_end*1000:.1f} ms) — PWM Ripple & Duty Cycle')
ax_cur.set_xlabel('Time, ms')
ax_cur.set_ylabel('Current, A')
ax_cur.grid(True)

ax_dc = ax_cur.twinx()
ax_dc.plot(time[zoom_mask] * 1000, duty_cycle_history[zoom_mask], 'k--', label='Duty Cycle', linewidth=1.2)
ax_dc.set_ylabel('Duty Cycle', color='k')
ax_dc.set_ylim(-0.1, 1.1)
ax_dc.tick_params(axis='y', labelcolor='k')

lines_cur, labels_cur = ax_cur.get_legend_handles_labels()
lines_dc, labels_dc = ax_dc.get_legend_handles_labels()
ax_cur.legend(lines_cur + lines_dc, labels_cur + labels_dc, loc='upper right')

plt.tight_layout()
plt.show()
