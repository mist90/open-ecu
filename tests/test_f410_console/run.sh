#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
g++ -std=c++17 -O2 -Wall -Wextra -Werror -fsanitize=undefined -fno-sanitize-recover=all \
    -I../../STM32F410/ThirdParty/nanoprintf -I../../STM32F410/ThirdParty/fast_float/include \
    main.cpp ../../STM32F410/Core/Src/console_libc.cpp -o test
./test
