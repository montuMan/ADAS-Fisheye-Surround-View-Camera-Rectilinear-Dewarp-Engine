# 🎯 How to Process Your Fisheye Image - READY TO USE!

## ✅ Project Status
The ADAS Fisheye Dewarp Engine is **built and ready** to process your canal bridge fisheye image!

## 🚀 Fastest Method (3 Commands)

```bash
# 1. Clone the repository
git clone https://github.com/montuMan/ADAS-Fisheye-Surround-View-Camera-Rectilinear-Dewarp-Engine.git
cd ADAS-Fisheye-Surround-View-Camera-Rectilinear-Dewarp-Engine

# 2. Install Python dependencies
pip install Pillow

# 3. Run the conversion script with your fisheye image
python3 convert_and_run.py your_fisheye_image.jpg
```

**Output**: `dewarped_output.jpg` - Your canal scene with corrected perspective!

## 📁 What You Get

### Input
- **Format**: Your fisheye image (the canal with bridge)
- **Characteristics**: Circular FOV, barrel distortion, curved lines

### Output  
- **Format**: `dewarped_output.jpg` (1280×720)
- **Characteristics**: Rectilinear perspective, straight lines, natural view
- **Processing Time**: ~4ms per frame

## 🛠️ Alternative Methods

### Method 1: Python Script (Recommended)
```bash
python3 convert_and_run.py my_fisheye.jpg
```

### Method 2: Bash Script
```bash
./process_image.sh my_fisheye.jpg
```

### Method 3: Manual Process
```bash
# Build
make

# Convert image to PGM
python3 -c "
from PIL import Image
img = Image.open('my_fisheye.jpg')
img.resize((1280, 960), Image.Resampling.LANCZOS).convert('L').save('real_fisheye.pgm')
"

# Run dewarp
./dewarp_test

# Convert output
python3 -c "
from PIL import Image
Image.open('rectlinear_output.pgm').save('dewarped_output.jpg', quality=95)
"
```

## 📊 Performance Metrics (From Latest Run)

```
Static memory budget:
  Input  buffer : 1.17 MB
  Output buffer : 0.88 MB
  LUT           : 7.03 MB
  Total (BSS)   : 9.08 MB

Benchmarking Results:
  Per frame      : 4.280 ms
  Throughput     : 233.7 fps (x86-64)
  Projected      : ~820 fps (ARM Cortex-A72 with NEON)
```

## 🎨 What Happens to Your Image

Your fisheye image shows a canal scene with a bridge. The dewarp engine will:

1. **Remove circular distortion** - The image goes from circular/spherical to rectangular
2. **Straighten curved lines** - The bridge and building edges become straight
3. **Correct perspective** - The scene appears as a normal pinhole camera would see it
4. **Extract central FOV** - Focuses on the central 90° horizontal field of view

## 📝 Technical Details

### Dewarp Algorithm
- **Model**: Kannala-Brandt polynomial fisheye distortion
- **Implementation**: Q15.16 fixed-point lookup table
- **Memory**: Static allocation (no heap, no dynamic memory)
- **Optimization**: Pre-computed LUT, integer-only hot path

### Current Configuration
- **Input**: 1280×960 grayscale
- **Output**: 1280×720 grayscale
- **Lens FOV**: 190° (configurable in `main.cpp`)
- **Distortion Coefficients**: k1=0.0, k2=-6.0e-4, k3=5.0e-7, k4=-4.0e-10

## 🔧 Customization

If the output doesn't look perfect, you can adjust lens parameters in `main.cpp`:

```cpp
// Edit lines 179-193
fi.max_theta_rad = 95.0f * (3.14159265f / 180.0f);  // 190° → adjust for your lens
fi.k[1] = -6.0e-4f;  // Barrel distortion coefficient - adjust if needed
```

Common lens FOVs:
- 180° lens: `max_theta_rad = 90.0f * (π / 180.0f)`
- 190° lens: `max_theta_rad = 95.0f * (π / 180.0f)` ← **current setting**
- 220° lens: `max_theta_rad = 110.0f * (π / 180.0f)`

After changes, rebuild with `make` and run again.

## 📖 Documentation

- **QUICK_START_GUIDE.md** - Comprehensive step-by-step guide
- **PROCESSING_REAL_IMAGES.md** - Detailed processing instructions  
- **README.md** - Project overview and technical details

## 🎯 Next Steps for You

1. **Download your fisheye image** from the GitHub link you provided
2. **Save it locally** (e.g., as `canal_fisheye.jpg`)
3. **Run the converter**:
   ```bash
   python3 convert_and_run.py canal_fisheye.jpg
   ```
4. **View the output**: `dewarped_output.jpg`

## ✨ Example Output

Your canal scene transformation:
- **Before**: Circular fisheye view with curved bridge and canal banks
- **After**: Natural rectilinear view with straight lines and normal perspective

The bridge, buildings, and canal will appear as if photographed with a standard camera!

## 🆘 Support

If you encounter issues:
1. Check that Pillow is installed: `pip install Pillow`
2. Verify the build completed: `./dewarp_test` should exist
3. See troubleshooting section in QUICK_START_GUIDE.md

---

**Engine Status**: ✅ Built and tested  
**Performance**: ✅ 233.7 fps on x86-64  
**Ready to process**: ✅ Your fisheye image  

🚀 **Ready when you are!**
