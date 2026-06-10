# Quick Start Guide: Process Your Fisheye Image

## Your Fisheye Image
You have a beautiful canal scene with a bridge showing typical fisheye circular distortion. This guide will help you dewarp it to a normal rectilinear perspective.

## Option 1: Quick Processing (Recommended)

### Step 1: Clone and Setup
```bash
# Clone the repository
git clone https://github.com/montuMan/ADAS-Fisheye-Surround-View-Camera-Rectilinear-Dewarp-Engine.git
cd ADAS-Fisheye-Surround-View-Camera-Rectilinear-Dewarp-Engine

# Build the project
make
```

### Step 2: Process Your Image

**Method A: Using Python (Recommended)**
```bash
# Install Python dependencies
pip install Pillow

# Save your fisheye image as 'my_fisheye.jpg' in the project directory

# Convert to PGM format
python3 << 'EOF'
from PIL import Image

# Load your fisheye image
img = Image.open('my_fisheye.jpg')
print(f"Original: {img.size}")

# Resize to 1280x960 and convert to grayscale
img_resized = img.resize((1280, 960), Image.Resampling.LANCZOS)
img_gray = img_resized.convert('L')

# Save as PGM
img_gray.save('real_fisheye.pgm')
print("Converted to real_fisheye.pgm (1280x960 grayscale)")
EOF

# Run the dewarp engine
./dewarp_test

# Convert output back to JPG
python3 << 'EOF'
from PIL import Image
img = Image.open('rectlinear_output.pgm')
img.save('dewarped_output.jpg', quality=95)
print("Output saved as: dewarped_output.jpg")
EOF
```

**Method B: Using ImageMagick**
```bash
# Install ImageMagick (if not already installed)
# Ubuntu/Debian: sudo apt-get install imagemagick
# macOS: brew install imagemagick

# Save your fisheye image as 'my_fisheye.jpg'

# Convert to PGM format
convert my_fisheye.jpg -resize 1280x960! -colorspace Gray -compress none real_fisheye.pgm

# Run the dewarp engine
./dewarp_test

# Convert output to JPG
convert rectlinear_output.pgm dewarped_output.jpg
```

## Option 2: Using the Helper Script

```bash
# Make the script executable
chmod +x process_image.sh

# Process your image (any format: jpg, png, etc.)
./process_image.sh my_fisheye.jpg
```

## What the Engine Does

Your fisheye image will be transformed as follows:

1. **Input**: 1280×960 fisheye image with circular FOV and barrel distortion
2. **Processing**: Kannala-Brandt polynomial distortion model with Q15.16 fixed-point LUT
3. **Output**: 1280×720 rectilinear (normal perspective) image

### Performance
- **Processing speed**: ~4ms per frame on x86-64 (233 fps)
- **Projected speed**: ~1ms per frame on ARM Cortex-A72 with NEON (~820 fps)
- **Memory**: 9.08 MB static allocation (no heap, no dynamic memory)

## Expected Results

Your canal scene with the bridge will be transformed from:
- ❌ Circular fisheye distortion with curved lines
- ✅ Normal rectilinear perspective with straight lines

The bridge, buildings, and canal will appear with natural perspective, as if photographed with a standard pinhole camera.

## Customizing for Your Lens

If the output doesn't look right, you may need to adjust the lens parameters in `main.cpp`:

```cpp
// Line 179-193 in main.cpp
static FisheyeIntrinsics makeFisheyeIntrinsics()
{
    FisheyeIntrinsics fi;
    fi.fx   = 300.0f;         // Focal length in pixels
    fi.fy   = 300.0f;
    fi.cx   = IN_W  / 2.0f;   // Principal point (image center)
    fi.cy   = IN_H  / 2.0f;
    fi.skew = 0.0f;
    fi.k[0] =  0.0f;          // k1
    fi.k[1] = -6.0e-4f;       // k2 (primary fisheye barrel term)
    fi.k[2] =  5.0e-7f;       // k3
    fi.k[3] = -4.0e-10f;      // k4
    fi.max_theta_rad = 95.0f * (3.14159265f / 180.0f);  // 190° FOV
    return fi;
}
```

**Typical adjustments:**
- **180° FOV lens**: `max_theta_rad = 90.0f * (π / 180.0f)`
- **220° FOV lens**: `max_theta_rad = 110.0f * (π / 180.0f)`
- **k coefficients**: Use values from your lens calibration (if available)

## Viewing the Output

```bash
# Linux
display dewarped_output.jpg      # ImageMagick
eog dewarped_output.jpg          # Eye of GNOME
gimp dewarped_output.jpg         # GIMP

# macOS
open dewarped_output.jpg

# Windows
start dewarped_output.jpg
```

## Troubleshooting

**"PGM dimensions don't match"**
- Ensure the conversion resizes to exactly 1280×960

**Output is all black or distorted**
- Check `max_theta_rad` matches your lens FOV
- Try adjusting k[1] coefficient (typical range: -6e-4 to -8e-4)

**Image is cropped**
- The output is 1280×720 (16:9 aspect) vs input 1280×960 (4:3)
- This is normal - the central FOV is extracted

## Advanced: Processing Multiple Images

```bash
# Batch process all fisheye images in a directory
for img in fisheye_images/*.jpg; do
    echo "Processing: $img"
    convert "$img" -resize 1280x960! -colorspace Gray -compress none real_fisheye.pgm
    ./dewarp_test > /dev/null
    convert rectlinear_output.pgm "output/$(basename "$img" .jpg)_dewarped.jpg"
done
```

## Next Steps

1. Download your fisheye image from GitHub
2. Follow Option 1 or Option 2 above
3. View your dewarped output
4. Adjust lens parameters if needed
5. Enjoy your corrected image!

---

**Note**: This is a production-grade C++ engine with zero heap allocation, designed for real-time ADAS applications. It processes images using pre-computed fixed-point lookup tables for maximum performance on embedded ARM SoCs.
