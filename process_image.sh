#!/bin/bash
# Helper script to process a fisheye image through the dewarp engine
# Usage: ./process_image.sh <input_image>

set -e

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <input_image>"
    echo "Example: $0 fisheye_photo.jpg"
    exit 1
fi

INPUT_IMAGE="$1"

if [ ! -f "$INPUT_IMAGE" ]; then
    echo "Error: Input image '$INPUT_IMAGE' not found"
    exit 1
fi

echo "=== Fisheye Image Processing Pipeline ==="
echo ""

# Check if ImageMagick is available
if ! command -v convert &> /dev/null; then
    echo "Error: ImageMagick 'convert' command not found"
    echo "Please install ImageMagick: sudo apt-get install imagemagick"
    exit 1
fi

echo "Step 1: Converting input image to 1280x960 grayscale PGM..."
convert "$INPUT_IMAGE" -resize 1280x960! -colorspace Gray -compress none real_fisheye.pgm

if [ ! -f "real_fisheye.pgm" ]; then
    echo "Error: Failed to create real_fisheye.pgm"
    exit 1
fi

echo "Step 2: Building dewarp engine..."
make

echo "Step 3: Running dewarp process..."
./dewarp_test

echo ""
echo "=== Processing Complete ==="
echo "Output: rectlinear_output.pgm"
echo ""
echo "To view the output:"
echo "  - Using ImageMagick: display rectlinear_output.pgm"
echo "  - Using GIMP: gimp rectlinear_output.pgm"
echo "  - Convert to JPG: convert rectlinear_output.pgm output.jpg"
