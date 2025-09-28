#!/bin/bash
# Flash script for STM32 BLDC ECU project

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}STM32 BLDC ECU Flash Script${NC}"
echo "================================"

# Parse command line arguments
BUILD_TYPE="Debug"
METHOD="stlink"
VERIFY=false

while [[ $# -gt 0 ]]; do
    case $1 in
        -r|--release)
            BUILD_TYPE="Release"
            shift
            ;;
        -m|--method)
            METHOD="$2"
            shift 2
            ;;
        -v|--verify)
            VERIFY=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  -r, --release    Use Release build (default: Debug)"
            echo "  -m, --method     Flash method: stlink, openocd, dfu (default: stlink)"
            echo "  -v, --verify     Verify flash after programming"
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

# Check if build exists
if [[ ! -d $BUILD_DIR ]]; then
    echo -e "${RED}Error: Build directory $BUILD_DIR not found!${NC}"
    echo "Please run ./build.sh first"
    exit 1
fi

# Check if firmware file exists
FIRMWARE_ELF="$BUILD_DIR/open-ecu2.elf"
FIRMWARE_BIN="$BUILD_DIR/open-ecu2.bin"
FIRMWARE_HEX="$BUILD_DIR/open-ecu2.hex"

if [[ ! -f $FIRMWARE_BIN ]]; then
    echo -e "${RED}Error: Firmware binary $FIRMWARE_BIN not found!${NC}"
    echo "Please run ./build.sh first"
    exit 1
fi

echo -e "${GREEN}Flash Configuration:${NC}"
echo "  Build Type: $BUILD_TYPE"
echo "  Flash Method: $METHOD"
echo "  Firmware: $FIRMWARE_BIN"
echo "  Verify: $VERIFY"
echo

# Flash based on selected method
case $METHOD in
    stlink)
        # Check if st-flash is available
        if ! command -v st-flash &> /dev/null; then
            echo -e "${RED}Error: st-flash not found!${NC}"
            echo "Please install stlink tools:"
            echo "  sudo apt install stlink-tools"
            exit 1
        fi
        
        echo -e "${YELLOW}Flashing with ST-Link...${NC}"
        st-flash write $FIRMWARE_BIN 0x8000000
        
        if [[ $VERIFY == true ]]; then
            echo -e "${YELLOW}Verifying flash...${NC}"
            st-flash read verification.bin 0x8000000 $(stat -c%s $FIRMWARE_BIN)
            if cmp -s $FIRMWARE_BIN verification.bin; then
                echo -e "${GREEN}Verification successful!${NC}"
                rm verification.bin
            else
                echo -e "${RED}Verification failed!${NC}"
                rm verification.bin
                exit 1
            fi
        fi
        ;;
        
    openocd)
        # Check if openocd is available
        if ! command -v openocd &> /dev/null; then
            echo -e "${RED}Error: openocd not found!${NC}"
            echo "Please install OpenOCD:"
            echo "  sudo apt install openocd"
            exit 1
        fi
        
        echo -e "${YELLOW}Flashing with OpenOCD...${NC}"
        openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program $FIRMWARE_ELF verify reset exit"
        ;;
        
    dfu)
        # Check if dfu-util is available
        if ! command -v dfu-util &> /dev/null; then
            echo -e "${RED}Error: dfu-util not found!${NC}"
            echo "Please install dfu-util:"
            echo "  sudo apt install dfu-util"
            exit 1
        fi
        
        echo -e "${YELLOW}Flashing with DFU...${NC}"
        echo "Put device in DFU mode (BOOT0=1, reset)"
        read -p "Press Enter when ready..."
        
        dfu-util -a 0 -s 0x08000000:leave -D $FIRMWARE_BIN
        ;;
        
    *)
        echo -e "${RED}Error: Unknown flash method: $METHOD${NC}"
        echo "Supported methods: stlink, openocd, dfu"
        exit 1
        ;;
esac

echo -e "${GREEN}Flash completed successfully!${NC}"
echo
echo -e "${GREEN}Device Information:${NC}"
echo "  Target: STM32G431CBU"
echo "  Flash Size: 128KB"
echo "  RAM Size: 32KB"
echo "  Flash Address: 0x08000000"