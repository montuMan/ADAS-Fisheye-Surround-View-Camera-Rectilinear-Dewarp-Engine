#!/usr/bin/env python3
"""
Fisheye Image Processing Script
Converts any image format to PGM, runs dewarp, and converts output to JPG
Usage: python3 convert_and_run.py <input_image>
"""

import sys
import os
import subprocess
from pathlib import Path

def main():
    if len(sys.argv) != 2:
        print("Usage: python3 convert_and_run.py <input_image>")
        print("Example: python3 convert_and_run.py my_fisheye.jpg")
        sys.exit(1)
    
    input_image = sys.argv[1]
    
    if not os.path.exists(input_image):
        print(f"Error: Input image '{input_image}' not found")
        sys.exit(1)
    
    try:
        from PIL import Image
    except ImportError:
        print("Error: Pillow not installed")
        print("Install with: pip install Pillow")
        sys.exit(1)
    
    print("=" * 60)
    print("Fisheye Image Dewarp Pipeline")
    print("=" * 60)
    print()
    
    # Step 1: Convert to PGM
    print(f"[1/4] Loading input image: {input_image}")
    try:
        img = Image.open(input_image)
        print(f"      Original size: {img.size[0]}×{img.size[1]}, mode: {img.mode}")
    except Exception as e:
        print(f"Error loading image: {e}")
        sys.exit(1)
    
    print("[2/4] Converting to 1280×960 grayscale PGM...")
    try:
        img_resized = img.resize((1280, 960), Image.Resampling.LANCZOS)
        img_gray = img_resized.convert('L')
        img_gray.save('real_fisheye.pgm')
        print("      Saved: real_fisheye.pgm")
    except Exception as e:
        print(f"Error converting image: {e}")
        sys.exit(1)
    
    # Step 2: Run dewarp
    print("[3/4] Running dewarp engine...")
    try:
        # Check if dewarp_test exists
        if not os.path.exists('./dewarp_test'):
            print("      Building dewarp_test...")
            result = subprocess.run(['make'], capture_output=True, text=True)
            if result.returncode != 0:
                print(f"Build error: {result.stderr}")
                sys.exit(1)
        
        result = subprocess.run(['./dewarp_test'], capture_output=True, text=True)
        if result.returncode != 0:
            print(f"Dewarp error: {result.stderr}")
            sys.exit(1)
        
        # Print relevant lines from output
        for line in result.stdout.split('\n'):
            if 'Throughput' in line or 'Per frame' in line or 'Saved:' in line:
                print(f"      {line.strip()}")
                
    except Exception as e:
        print(f"Error running dewarp: {e}")
        sys.exit(1)
    
    # Step 3: Convert output to JPG
    print("[4/4] Converting output to JPG...")
    try:
        output_img = Image.open('rectlinear_output.pgm')
        output_path = 'dewarped_output.jpg'
        output_img.save(output_path, quality=95)
        print(f"      Saved: {output_path} ({output_img.size[0]}×{output_img.size[1]})")
    except Exception as e:
        print(f"Error converting output: {e}")
        sys.exit(1)
    
    print()
    print("=" * 60)
    print("✓ Processing Complete!")
    print("=" * 60)
    print()
    print(f"Input:  {input_image}")
    print(f"Output: dewarped_output.jpg")
    print()
    print("Files generated:")
    print("  - real_fisheye.pgm       (1280×960 grayscale input)")
    print("  - rectlinear_output.pgm  (1280×720 dewarped output)")
    print("  - dewarped_output.jpg    (final output)")
    print()

if __name__ == '__main__':
    main()
