#!/bin/bash
set -e

echo "=== Auto CoreBall - Setup ==="

echo "Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  libopencv-dev \
  libx11-dev \
  libxtst-dev \
  xdotool \
  git

echo "Building project..."
cd "$(dirname "$0")"
mkdir -p build
cd build
cmake ..
make -j$(nproc)

echo ""
echo "=== Setup complete! ==="
echo ""
echo "To run:"
echo "  1. Open https://www.arealme.com/coreball/cn/ in your browser"
echo "  2. Start the game"
echo "  3. Run: ./build/auto_coreball"
echo ""
echo "If the window isn't found, specify the window title substring:"
echo "  ./build/auto_coreball --window Chrome"
echo ""
echo "Recording: Use OBS Studio to record your screen."
echo "  Record both the game window and the visualization window."
