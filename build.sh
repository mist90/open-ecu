#!/bin/bash
# Build script for STM32 BLDC ECU project

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}STM32 BLDC ECU Build Script${NC}"
echo "================================"

# Check if arm-none-eabi-gcc is available
if ! command -v arm-none-eabi-gcc &> /dev/null; then
    echo -e "${RED}Error: arm-none-eabi-gcc not found!${NC}"
    echo "Please install ARM GCC toolchain:"
    echo "  sudo apt install gcc-arm-none-eabi"
    exit 1
fi

# Check if cmake is available
if ! command -v cmake &> /dev/null; then
    echo -e "${RED}Error: cmake not found!${NC}"
    echo "Please install CMake:"
    echo "  sudo apt install cmake"
    exit 1
fi

# Print toolchain information
echo -e "${GREEN}Toolchain Information:${NC}"
arm-none-eabi-gcc --version | head -1
cmake --version | head -1
echo

# Parse command line arguments
BUILD_TYPE="Debug"
CLEAN_BUILD=false
VERBOSE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -r|--release)
            BUILD_TYPE="Release"
            shift
            ;;
        -c|--clean)
            CLEAN_BUILD=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  -r, --release    Build in Release mode (default: Debug)"
            echo "  -c, --clean      Clean build directory before building"
            echo "  -v, --verbose    Verbose build output"
            echo "  -h, --help       Show this help message"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# Set build directory
BUILD_DIR="build"
if [[ $BUILD_TYPE == "Release" ]]; then
    BUILD_DIR="build-release"
fi

echo -e "${GREEN}Build Configuration:${NC}"
echo "  Build Type: $BUILD_TYPE"
echo "  Build Directory: $BUILD_DIR"
echo "  Clean Build: $CLEAN_BUILD"
echo

# Clean build directory if requested
if [[ $CLEAN_BUILD == true ]]; then
    echo -e "${YELLOW}Cleaning build directory...${NC}"
    rm -rf $BUILD_DIR
fi

# Create build directory
mkdir -p $BUILD_DIR
cd $BUILD_DIR

# Configure with CMake
echo -e "${YELLOW}Configuring with CMake...${NC}"
CMAKE_ARGS="-DCMAKE_BUILD_TYPE=$BUILD_TYPE"
if [[ $VERBOSE == true ]]; then
    CMAKE_ARGS="$CMAKE_ARGS -DCMAKE_VERBOSE_MAKEFILE=ON"
fi

cmake .. $CMAKE_ARGS

if [[ $? -ne 0 ]]; then
    echo -e "${RED}CMake configuration failed!${NC}"
    exit 1
fi

# Build the project
echo -e "${YELLOW}Building project...${NC}"
MAKE_ARGS=""
if [[ $VERBOSE == true ]]; then
    MAKE_ARGS="VERBOSE=1"
fi

make -j$(nproc) $MAKE_ARGS

if [[ $? -ne 0 ]]; then
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi

echo -e "${GREEN}Build completed successfully!${NC}"
echo
echo -e "${GREEN}Generated files:${NC}"
echo "  Firmware: open-ecu2.elf"
echo "  Intel HEX: open-ecu2.hex"
echo "  Binary: open-ecu2.bin"
echo
echo "Flash command: st-flash write open-ecu2.bin 0x8000000"