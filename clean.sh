#!/bin/bash
# Clean script wrapper for open-ecu project
# Supports multiple target platforms

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

PLATFORM="STM32G431"

while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--platform)
            PLATFORM="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  -p, --platform   Target platform (default: STM32G431)"
            echo "  -h, --help       Show this help message"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

PLATFORM_DIR="$PLATFORM"
if [[ ! -d "$PLATFORM_DIR" ]]; then
    echo -e "${RED}Error: Platform directory '$PLATFORM_DIR' not found!${NC}"
    exit 1
fi

PLATFORM_CLEAN_SCRIPT="$PLATFORM_DIR/clean.sh"
if [[ ! -f "$PLATFORM_CLEAN_SCRIPT" ]]; then
    echo -e "${RED}Error: Clean script not found at $PLATFORM_CLEAN_SCRIPT${NC}"
    exit 1
fi

echo -e "${GREEN}Cleaning platform: $PLATFORM${NC}"
echo ""

cd "$PLATFORM_DIR"
./clean.sh
