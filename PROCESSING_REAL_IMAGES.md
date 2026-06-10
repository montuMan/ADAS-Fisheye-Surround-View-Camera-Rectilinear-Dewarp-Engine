# Processing Real Fisheye Images

This guide explains how to process your fisheye image through the ADAS dewarp engine.

## Quick Start

### Option 1: Using the Helper Script (Recommended)

```bash
# Install ImageMagick if not already installed
sudo apt-get install imagemagick

# Process your fisheye image
./process_image.sh your_fisheye_image.jpg
```

The script will:
1. Convert your image to 1280×960 grayscale PGM format
2. Build the dewarp engine
3. Run the dewarping process
4. Output `rectlinear_output.pgm`

### Option 2: Manual Process

1. **Convert your fisheye image to PGM format:**

```bash
# Using ImageMagick
convert your_fisheye_image.jpg -resize 1280x960! -colorspace Gray -compress none real_fisheye.pgm

# Or using Python + PIL
python3 << EOF
from PIL import Image
img = Image.open('your_fisheye_image.jpg')
img = img.resize((1280, 960))
img = img.convert('L')  # Convert to grayscale
img.save('real_fisheye.pgm')
EOF
```

2. **Build and run:**

```bash
make
./dewarp_test
```

3. **View the output:**

```bash
# Convert to a more common format
convert rectlinear_output.pgm output.jpg

# Or view directly
display rectlinear_output.pgm  # ImageMagick
gimp rectlinear_output.pgm     # GIMP
```

## For Your Specific Image

Your fisheye image (the canal scene with a bridge) shows typical fisheye distortion. To process it:

1. Save the image from GitHub to your local machine
2. Upload it to the repository directory or provide it to the system
3. Run the helper script:

```bash
./process_image.sh downloaded_fisheye_image.jpg
```

The dewarp engine will:
- Convert the circular fisheye projection to rectilinear (pinhole camera) projection
- Remove barrel distortion
- Generate a 1280×720 output image with natural perspective

## Expected Results

- **Input**: 1280×960 fisheye image with circular FOV and barrel distortion
- **Output**: 1280×720 rectilinear image with corrected perspective
- **Processing time**: ~4ms per frame on x86-64 (projected ~1ms on ARM Cortex-A72 with NEON)

## Image Requirements

- **Format**: Any common image format (JPG, PNG, etc.) - will be converted to PGM
- **Recommended size**: 1280×960 (will be resized if different)
- **Color**: Will be converted to grayscale for processing
- **Lens model**: Assumes Kannala-Brandt fisheye model (configurable in code)

## Troubleshooting

**"ImageMagick not found"**
- Install: `sudo apt-get install imagemagick` (Ubuntu/Debian)
- Or: `brew install imagemagick` (macOS)

**"PGM dimensions don't match"**
- The input PGM must be exactly 1280×960
- Use the conversion commands above to resize properly

**Output looks wrong**
- Check if the intrinsic parameters in `main.cpp` match your lens
- Adjust `max_theta_rad` for your lens FOV (currently set for 190° lens)
- Modify Kannala-Brandt coefficients `k[0]...k[3]` if you have calibration data
