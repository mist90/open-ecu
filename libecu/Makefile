# Makefile for libecu library
# Platform-independent BLDC motor control library

# Compiler settings
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2 -fPIC
INCLUDES = -Iinclude

# Directories
SRC_DIR = src
INCLUDE_DIR = include
BUILD_DIR = build
LIB_DIR = lib

# Source files
SOURCES = $(wildcard $(SRC_DIR)/*.cpp)
OBJECTS = $(SOURCES:$(SRC_DIR)/%.cpp=$(BUILD_DIR)/%.o)

# Library name
LIBRARY = $(LIB_DIR)/liblibecu.a

# Default target
all: $(LIBRARY)

# Create build directories
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(LIB_DIR):
	mkdir -p $(LIB_DIR)

# Compile source files
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Create static library
$(LIBRARY): $(OBJECTS) | $(LIB_DIR)
	ar rcs $@ $^

# Clean build files
clean:
	rm -rf $(BUILD_DIR) $(LIB_DIR)

# Install headers (for system-wide installation)
install-headers:
	mkdir -p /usr/local/include/libecu
	cp -r $(INCLUDE_DIR)/* /usr/local/include/libecu/

# Uninstall
uninstall:
	rm -rf /usr/local/include/libecu

# Test compilation (without STM32 HAL dependencies)
test-compile: all
	@echo "Library compiled successfully!"
	@echo "Objects: $(OBJECTS)"
	@echo "Library: $(LIBRARY)"

# Show library info
info:
	@echo "libecu - Platform-independent BLDC motor control library"
	@echo "Version: 1.0.0"
	@echo "Features:"
	@echo "  - 6-step commutation algorithm"
	@echo "  - PID speed control with anti-windup"
	@echo "  - Comprehensive safety monitoring"
	@echo "  - Platform-independent interfaces"
	@echo ""
	@echo "Usage:"
	@echo "  make          - Build library"
	@echo "  make clean    - Clean build files"
	@echo "  make info     - Show this information"

.PHONY: all clean install-headers uninstall test-compile info