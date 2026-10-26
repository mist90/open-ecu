# Vendor sources

`Drivers.tar.gz` is unpacked by CMake, as in the STM32G431 target. Builds are
offline once this archive is present. `fetch_drivers.py` recreates it from:

* ST STM32F4 HAL v1.8.3: `c2e1406d7ea4b73aa42b98ddeb75a8670b1a5a16`.
* ST CMSIS Device F4 v2.6.10: `5f41fb29d22773896c780052bf61e47fc924d524`.
* ARM CMSIS Core 5.9.0: `2b7495b8535bdcb306dac29b9ded4cfb679d7e5c`.

Upstream licenses are included in the archive. Only required HAL modules are
compiled. The unmodified vendor `system_stm32f4xx.c` and
`startup_stm32f410rx.s` are built from the CMSIS `Source/Templates` directory;
the F410 vector table must not be replaced with an F401/F411 vector table.
