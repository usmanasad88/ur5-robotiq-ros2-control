#!/bin/bash
# GoPro Max 2 Streaming Environment Setup
# This script activates the ur5_python conda environment and installs required dependencies

set -e

echo "=== GoPro Max 2 Streaming Environment Setup ==="
echo ""

# Activate conda environment
echo "1. Activating ur5_python conda environment..."
source ~/miniconda3/etc/profile.d/conda.sh || source ~/anaconda3/etc/profile.d/conda.sh
conda activate ur5_python

echo "   ✓ Conda environment 'ur5_python' activated"
echo ""

# Install Python dependencies
echo "2. Installing Python dependencies..."
pip install open-gopro[wifi,ble] requests

echo "   ✓ open-gopro SDK installed"
echo ""

# Check ffmpeg installation
echo "3. Checking ffmpeg installation..."
if command -v ffmpeg &> /dev/null; then
    echo "   ✓ ffmpeg is already installed ($(ffmpeg -version | head -n1))"
else
    echo "   ⚠ ffmpeg not found. Installing via conda..."
    conda install -c conda-forge ffmpeg -y
    echo "   ✓ ffmpeg installed"
fi
echo ""

# Check NVIDIA GPU and CUDA support
echo "4. Checking NVIDIA GPU support..."
if command -v nvidia-smi &> /dev/null; then
    echo "   ✓ NVIDIA GPU detected:"
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader | head -n1
    echo ""
    
    # Check if ffmpeg has CUDA support
    if ffmpeg -hwaccels 2>&1 | grep -q cuda; then
        echo "   ✓ ffmpeg CUDA hardware acceleration available"
    else
        echo "   ⚠ ffmpeg may not have CUDA support compiled in"
        echo "     Stream decoding will still work but may use CPU"
    fi
else
    echo "   ⚠ nvidia-smi not found. GPU acceleration may not be available."
fi
echo ""

echo "=== Setup Complete ==="
echo "To activate this environment in the future, run:"
echo "  conda activate ur5_python"
echo ""
