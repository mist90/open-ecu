#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

def main():
    # Имя файла с результатами теста
    file_path = "motor_pll_test_results.csv"

    try:
        # Чтение данных из CSV-файла
        data = pd.read_csv(file_path)
    except FileNotFoundError:
        print(f"Ошибка: Файл '{file_path}' не найден.")
        print("Сначала запустите скомпилированный C++ тест, чтобы сгенерировать данные.")
        return

    # Настройка общего стиля графиков
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
    fig.suptitle("Анализ работы алгоритма MotorPLL (Имитационный тест)", fontsize=16, fontweight='bold')

    # 1. ГРАФИК УГЛОВ (Электрические градусы)
    ax1.plot(data['Time_s'], data['Real_Angle'], label='Реальный угол ротора', color='darkorange', alpha=0.6, linewidth=2)
    ax1.plot(data['Time_s'], data['PLL_Angle'], label='Виртуальный угол (PLL)', color='dodgerblue', linestyle='--', linewidth=1.5)
    ax1.set_ylabel("Номер шага", fontsize=12)
    ax1.set_title("Отслеживание угла ротора", fontsize=13, fontweight='bold')
    ax1.legend(loc="upper right", frameon=True)
    ax1.grid(True, alpha=0.5)

    # 2. ГРАФИК СКОРОСТЕЙ (Электрические градусы в секунду)
    ax2.plot(data['Time_s'], data['Real_Speed'], label='Реальная скорость', color='crimson', linewidth=2.5)
    ax2.plot(data['Time_s'], data['PLL_Speed'], label='Вычисленная скорость (PLL)', color='teal', linestyle='-.', linewidth=2)
    ax2.set_ylabel("Скорость (шаги/сек)", fontsize=12)
    ax2.set_title("Синхронизация скоростей и фильтрация шума", fontsize=13, fontweight='bold')
    ax2.legend(loc="upper right", frameon=True)
    ax2.grid(True, alpha=0.5)

    # 3. ГРАФИК КОММУТАЦИОННЫХ ШАГОВ (Сектора 0..5)
    ax3.step(data['Time_s'], data['Real_Hall'], label='Физический шаг Холла (Вход)', color='gray', where='post', alpha=0.7, linewidth=1.5)
    ax3.step(data['Time_s'], data['Output_Step'], label='Выходной шаг коммутации (90°)', color='forestgreen', where='post', linewidth=2)
    ax3.set_xlabel("Время (секунды)", fontsize=12)
    ax3.set_ylabel("Номер шага (0..5)", fontsize=12)
    ax3.set_title("Формирование коммутационных шагов статора", fontsize=13, fontweight='bold')
    ax3.set_yticks(range(6))
    ax3.legend(loc="upper right", frameon=True)
    ax3.grid(True, alpha=0.5)

    # Оптимизация расположения элементов
    plt.tight_layout()

    # Сохранение графика в файл изображения и показ на экране
    output_img = "motor_pll_plots.png"
    plt.savefig(output_img, dpi=300)
    print(f"Графики успешно построены и сохранены в файл: {output_img}")
    plt.show()

if __name__ == "__main__":
    main()
