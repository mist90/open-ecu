#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
for directory in build build-release Drivers; do
    if [[ -d $directory ]]; then
        echo "Removing STM32F410/$directory/"
        rm -rf -- "$directory"
    fi
done
