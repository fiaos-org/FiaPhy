#!/bin/bash

# FiaPhy Raspberry Pi Build Script

echo "======================================================="
echo "       FiaPhy Raspberry Pi Build System"
echo "======================================================="
echo ""

if ! command -v g++ &> /dev/null; then
    echo "ERROR: g++ not found. Install with:"
    echo "  sudo apt-get install build-essential"
    exit 1
fi

echo "Compiling..."

g++ -o solar_monitor \
    raspberry_pi_example.cpp \
    -I../../src \
    -std=c++11 \
    -O2 \
    -Wall \
    -Wextra

if [ $? -eq 0 ]; then
    echo "Build successful"
    echo ""
    echo "Run with: sudo ./solar_monitor"
    echo "Output: solar_data.csv"
    echo ""
else
    echo "Build failed"
    exit 1
fi
