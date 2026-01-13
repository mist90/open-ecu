#!/bin/bash
# Build script wrapper for open-ecu project
# Supports multiple target platforms

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default platform
PLATFORM="STM32G431"

# Parse platform selection
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
            echo ""
            echo "All other options are passed to the platform-specific build script"
            echo "Run '$0 --platform <platform> --help' for platform-specific options"
            exit 0
            ;;
        *)
            break
            ;;
    esac
done

# Validate platform directory exists
PLATFORM_DIR="$PLATFORM"
if [[ ! -d "$PLATFORM_DIR" ]]; then
    echo -e "${RED}Error: Platform directory '$PLATFORM_DIR' not found!${NC}"
    echo "Available platforms:"
    for dir in */; do
        if [[ -f "$dir/build.sh" ]]; then
            echo "  - ${dir%/}"
        fi
    done
    exit 1
fi

# Check if platform build script exists
PLATFORM_BUILD_SCRIPT="$PLATFORM_DIR/build.sh"
if [[ ! -f "$PLATFORM_BUILD_SCRIPT" ]]; then
    echo -e "${RED}Error: Build script not found at $PLATFORM_BUILD_SCRIPT${NC}"
    exit 1
fi

echo -e "${GREEN}Building for platform: $PLATFORM${NC}"
echo ""

# Run platform-specific build script with remaining arguments
# Pass all remaining arguments to the platform build script
cd "$PLATFORM_DIR"
./build.sh "$@"
