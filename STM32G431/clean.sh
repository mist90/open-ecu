#!/bin/bash
# Clean script for STM32 BLDC ECU project
# Removes all build artifacts including extracted Drivers

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${YELLOW}STM32 BLDC ECU Clean${NC}"
echo "================================"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

cleaned=0

for dir in build build-release; do
    if [[ -d "$dir" ]]; then
        echo -e "  Removing ${RED}$dir/${NC}"
        rm -rf "$dir"
        cleaned=$((cleaned + 1))
    fi
done

if [[ -d "Drivers" ]]; then
    echo -e "  Removing ${RED}Drivers/${NC} (will be re-extracted from Drivers.tar.gz on next build)"
    rm -rf Drivers
    cleaned=$((cleaned + 1))
fi

if [[ $cleaned -eq 0 ]]; then
    echo -e "${GREEN}Nothing to clean.${NC}"
else
    echo -e "${GREEN}Cleaned $cleaned artifact(s).${NC}"
fi
