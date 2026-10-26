#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
build_dir=build-release
method=openocd
while [[ $# -gt 0 ]]; do
    case "$1" in
        -r|--release) build_dir=build-release; shift ;;
        -d|--debug) build_dir=build; shift ;;
        -m|--method) method="${2:?Specify openocd or stlink}"; shift 2 ;;
        -v|--verify) shift ;; # Verification is always enabled.
        -h|--help)
            echo "Usage: $0 [--debug] [--method openocd|stlink] [--verify]"
            echo "Programs via the Nucleo's ST-LINK USB connector, always verifies."
            exit 0 ;;
        *) echo "Unknown option: $1" >&2; exit 1 ;;
    esac
done
[[ -f "$build_dir/open-ecu.bin" ]] || { echo "Run ./build.sh first" >&2; exit 1; }
case "$method" in
    openocd)
        openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
            -c "program $build_dir/open-ecu.elf verify reset exit" ;;
    stlink)
        st-flash write "$build_dir/open-ecu.bin" 0x08000000
        verify=$(mktemp)
        trap 'rm -f "$verify"' EXIT
        st-flash read "$verify" 0x08000000 "$(stat -c%s "$build_dir/open-ecu.bin")"
        cmp "$build_dir/open-ecu.bin" "$verify"
        st-flash reset ;;
    *) echo "Unsupported method: $method (use openocd or stlink)" >&2; exit 1 ;;
esac
