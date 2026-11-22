#!/bin/bash
# Flash script wrapper for open-ecu project
# Supports multiple target platforms

set -e  # Exit on any error
ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default platform
PLATFORM="STM32G431"
PLATFORM_SELECTED=false

# Parse platform selection
while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--platform)
            PLATFORM="${2:?Specify a platform}"
            PLATFORM_SELECTED=true
            shift 2
            ;;
        -h|--help)
            if "$PLATFORM_SELECTED"; then break; fi
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  -p, --platform   STM32G431 (default), STM32F410"
            echo ""
            echo "Flashes the Release build by default; pass -d/--debug for the Debug build."
            echo ""
            echo "All other options are passed to the platform-specific flash script"
            echo "Run '$0 --platform <platform> --help' for platform-specific options"
            exit 0
            ;;
        *)
            break
            ;;
    esac
done

# Validate platform directory exists
PLATFORM_DIR="$ROOT/$PLATFORM"
if [[ ! -d "$PLATFORM_DIR" ]]; then
    echo -e "${RED}Error: Platform directory '$PLATFORM_DIR' not found!${NC}"
    echo "Available platforms:"
    for dir in "$ROOT"/*/; do
        if [[ -f "$dir/flash.sh" ]]; then
            echo "  - $(basename "$dir")"
        fi
    done
    exit 1
fi

# Check if platform flash script exists
PLATFORM_FLASH_SCRIPT="$PLATFORM_DIR/flash.sh"
if [[ ! -f "$PLATFORM_FLASH_SCRIPT" ]]; then
    echo -e "${RED}Error: Flash script not found at $PLATFORM_FLASH_SCRIPT${NC}"
    exit 1
fi

echo -e "${GREEN}Flashing platform: $PLATFORM${NC}"
echo ""

# Run platform-specific flash script with remaining arguments
# Pass all remaining arguments to the platform flash script
cd "$PLATFORM_DIR"
./flash.sh "$@"
