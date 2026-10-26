# Документы для NUCLEO-F410RB + X-NUCLEO-IHM08M1

Скачаны 2026-10-23. Практическая распиновка и настройки:
[`../../NUCLEO_F410RB_IHM08M1.md`](../../NUCLEO_F410RB_IHM08M1.md).

| Локальный файл | Документ / официальный URL |
|---|---|
| [UM1724.pdf](UM1724.pdf) | [Nucleo-64 MB1136, Rev 17](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf) |
| [MB1136-C04.pdf](MB1136-C04.pdf) | [Nucleo schematic C-04](https://www.st.com/resource/en/schematic_pack/mb1136-default-c04_schematic.pdf) |
| [UM1996.pdf](UM1996.pdf) | [IHM08M1 user manual, Rev 4](https://www.st.com/resource/en/user_manual/um1996-getting-started-with-xnucleoihm08m1-lowvoltage-bldc-motor-driver-expansion-board-based-on-stl220n6f7-for-stm32-nucleo-stmicroelectronics.pdf) |
| [X-NUCLEO-IHM08M1-schematic.pdf](X-NUCLEO-IHM08M1-schematic.pdf) | [IHM08M1 schematic version 3](https://www.st.com/resource/en/schematic_pack/x-nucleo-ihm08m1_schematic.pdf) |
| [STM32F410RB.pdf](STM32F410RB.pdf) | [DS11144 Rev 8, pin/AF table](https://www.st.com/resource/en/datasheet/stm32f410rb.pdf) |
| [RM0401.pdf](RM0401.pdf) | [STM32F410 reference manual Rev 4](https://www.st.com/resource/en/reference_manual/rm0401-stm32f410-advanced-armbased-32bit-mcus-stmicroelectronics.pdf) |
| [L6398.pdf](L6398.pdf) | [Gate driver datasheet](https://www.st.com/resource/en/datasheet/l6398.pdf) |

UM1724, MB1136, DS11144, RM0401 получены с st.com. UM1996, схема IHM08M1 и
L6398 получены как копии PDF ST из
[`okhsunrog/oxifoc/docs/hw/x-nucleo-ihm08m1`](https://github.com/okhsunrog/oxifoc/tree/main/docs/hw/x-nucleo-ihm08m1)
после таймаута прямого скачивания; проверены название, ревизия и содержимое
схем. Оригинальные уведомления/лицензии сохранены внутри PDF.

Дополнительная проверка NTC из BOM:
[TDK NTCG103JF103FTDS, R25=10 kΩ, B25/85=3435 K](https://media.digikey.com/pdf/Data%20Sheets/TDK%20PDFs/NTCG103JF103FTDS_Spec.pdf).
Пропуск потенциометра R42 на схеме подтверждён
[ответом ST](https://community.st.com/power-management-53/schematic-x-nucleo-ihm08m1-does-not-seem-to-the-latest-27439).

Проверка локальных копий: `sha256sum -c SHA256SUMS` в этой папке.
