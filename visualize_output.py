#!/usr/bin/env python3
"""
Visualize the fisheye to rectlinear conversion
Reads PGM files and displays comparison
"""

import struct
import sys

def read_pgm(filename):
    """Read a binary PGM (P5) file"""
    with open(filename, 'rb') as f:
        # Read magic number
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError(f"Not a binary PGM file (P5): {magic}")
        
        # Skip comments
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        
        # Read dimensions
        width, height = map(int, line.split())
        
        # Read max value
        maxval = int(f.readline())
        if maxval != 255:
            raise ValueError(f"Unsupported maxval: {maxval}")
        
        # Read pixel data
        pixels = f.read(width * height)
        
        return width, height, pixels

def print_ascii_art(width, height, pixels, scale=8):
    """Print ASCII art representation of the image"""
    chars = " .:-=+*#%@"
    
    print(f"\nImage size: {width}×{height}")
    print("=" * (width // scale))
    
    for y in range(0, height, scale):
        line = ""
        for x in range(0, width, scale):
            idx = y * width + x
            if idx < len(pixels):
                intensity = pixels[idx]
                char_idx = min(intensity * len(chars) // 256, len(chars) - 1)
                line += chars[char_idx]
        print(line)
    
    print("=" * (width // scale))

def print_stats(filename, width, height, pixels):
    """Print statistics about the image"""
    print(f"\n{filename}:")
    print(f"  Dimensions: {width}×{height}")
    print(f"  Total pixels: {width * height:,}")
    
    # Calculate min, max, average
    min_val = min(pixels)
    max_val = max(pixels)
    avg_val = sum(pixels) / len(pixels)
    
    print(f"  Pixel values: min={min_val}, max={max_val}, avg={avg_val:.1f}")
    
    # Count non-zero pixels
    non_zero = sum(1 for p in pixels if p > 0)
    print(f"  Non-zero pixels: {non_zero:,} ({100*non_zero/len(pixels):.1f}%)")

def main():
    try:
        # Read fisheye input
        print("=" * 60)
        print("FISHEYE TO RECTLINEAR CONVERSION SUMMARY")
        print("=" * 60)
        
        w_in, h_in, pixels_in = read_pgm('fisheye_input.pgm')
        print_stats('fisheye_input.pgm', w_in, h_in, pixels_in)
        
        # Read rectlinear output
        w_out, h_out, pixels_out = read_pgm('rectlinear_output.pgm')
        print_stats('rectlinear_output.pgm', w_out, h_out, pixels_out)
        
        # Print ASCII preview
        print("\n" + "=" * 60)
        print("RECTLINEAR OUTPUT (ASCII Preview - downscaled)")
        print("=" * 60)
        print_ascii_art(w_out, h_out, pixels_out, scale=8)
        
        print("\n" + "=" * 60)
        print("✓ Image processing complete!")
        print("=" * 60)
        print("\nTo view full quality output:")
        print("  convert rectlinear_output.pgm output.jpg")
        print("  display output.jpg")
        
    except FileNotFoundError as e:
        print(f"Error: {e}")
        print("\nPlease run ./dewarp_test first to generate the images.")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
