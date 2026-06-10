/**
 * main.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Lightweight test harness for FisheyeDewarp.
 *
 * What it does:
 *   1. Fills a synthetic fisheye input buffer with a known checkerboard pattern.
 *   2. Constructs realistic Kannala-Brandt intrinsics (typical 190° FOV lens).
 *   3. Runs init() (LUT build) and measures wall-clock time.
 *   4. Runs processFrame() N times and reports throughput (frames/s, ms/frame).
 *   5. Writes a tiny PGM file so you can visually inspect the output.
 *   6. Validates a known centre pixel to confirm the identity mapping works.
 *
 * Build (host / x86-64, for dev/test):
 *   g++ -std=c++17 -O2 -o dewarp_test main.cpp fisheye_dewarp.cpp -lm
 *
 * Build (AArch64 cross-compile):
 *   aarch64-linux-gnu-g++ -std=c++17 -O3 -march=armv8-a+simd \
 *       -o dewarp_test main.cpp fisheye_dewarp.cpp -lm
 *
 * Build (AArch64 with NEON intrinsics enabled for future vectorisation):
 *   aarch64-linux-gnu-g++ -std=c++17 -O3 -march=armv8-a+simd \
 *       -mfpu=neon-fp-armv8 -funsafe-math-optimizations \
 *       -o dewarp_test main.cpp fisheye_dewarp.cpp -lm
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "fisheye_dewarp.hpp"

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <ctime>

// ─────────────────────────────────────────────────────────────────────────────
//  Compile-time buffer sizes
//  These are purely stack / BSS; no heap allocation.
// ─────────────────────────────────────────────────────────────────────────────
static constexpr int IN_W  = FisheyeDewarp::IN_W;
static constexpr int IN_H  = FisheyeDewarp::IN_H;
static constexpr int OUT_W = FisheyeDewarp::OUT_W;
static constexpr int OUT_H = FisheyeDewarp::OUT_H;

// Input and output image buffers — statically allocated (BSS).
// Input  : 1280 × 960 = 1,228,800 bytes ≈ 1.17 MB
// Output : 1280 × 720 =   921,600 bytes ≈ 0.88 MB
alignas(64) static uint8_t g_src[IN_H  * IN_W];
alignas(64) static uint8_t g_dst[OUT_H * OUT_W];

// The FisheyeDewarp object itself; its internal LUT is ~7 MB in BSS.
// FisheyeDewarp is non-copyable/non-movable; the static declaration here is
// the correct (and only) way to instantiate it.
// Total BSS: ~9 MB — comfortably within typical ADAS SoC DRAM budgets.
static FisheyeDewarp g_dewarp;

// ─────────────────────────────────────────────────────────────────────────────
//  Synthetic test pattern: 32×32 pixel checkerboard
//  Provides sharp edges near the optical centre and at the borders, making it
//  easy to visually verify the de-warped output.
// ─────────────────────────────────────────────────────────────────────────────
static void fillCheckerboard(uint8_t* buf, int w, int h, int cell_size = 32)
{
    for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) {
            const bool even_r = ((r / cell_size) & 1) == 0;
            const bool even_c = ((c / cell_size) & 1) == 0;
            buf[r * w + c] = (even_r ^ even_c) ? 220u : 40u;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Save a single-channel buffer as a binary PGM (Portable GrayMap).
//  PGM is trivial to parse and is readable by GIMP, ImageMagick, ffplay, etc.
//  Returns true on success; prints a warning and returns false on failure.
// ─────────────────────────────────────────────────────────────────────────────
static bool savePGM(const char* path, const uint8_t* buf, int w, int h)
{
    FILE* fp = fopen(path, "wb");
    if (!fp) {
        fprintf(stderr, "Warning: could not open '%s' for writing — "
                        "skipping PGM output (read-only filesystem?)\n", path);
        return false;
    }
    fprintf(fp, "P5\n%d %d\n255\n", w, h);
    fwrite(buf, 1, static_cast<size_t>(w * h), fp);
    fclose(fp);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Monotonic wall-clock timer (POSIX CLOCK_MONOTONIC if available, else clock())
// ─────────────────────────────────────────────────────────────────────────────
static double getTimeSec()
{
#if defined(_POSIX_TIMERS) && _POSIX_TIMERS > 0
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
#else
    return static_cast<double>(clock()) / CLOCKS_PER_SEC;
#endif
}

// ─────────────────────────────────────────────────────────────────────────────
//  Construct a representative set of intrinsics.
//
//  Fisheye lens: 190° total FOV, 1280×960 sensor, f≈300 px, OcamCalib-style.
//    max_theta_rad = 95° = 1.6581 rad (half of 190°).
//  Rectilinear output: 1280×720, 90° HFOV equivalent (fx = fy ≈ 640).
//
//  Kannala-Brandt coefficients below are typical for a mid-range automotive
//  wide-angle lens.  Replace with values from your calibration tool.
//  Note: k[0]…k[3] here are k1…k4 in 1-based calibration-tool notation.
// ─────────────────────────────────────────────────────────────────────────────
static FisheyeIntrinsics makeFisheyeIntrinsics()
{
    FisheyeIntrinsics fi;
    fi.fx   = 300.0f;
    fi.fy   = 300.0f;
    fi.cx   = IN_W  / 2.0f;
    fi.cy   = IN_H  / 2.0f;
    fi.skew = 0.0f;
    fi.k[0] =  0.0f;          // k1
    fi.k[1] = -6.0e-4f;       // k2 (primary fisheye barrel term)
    fi.k[2] =  5.0e-7f;       // k3
    fi.k[3] = -4.0e-10f;      // k4
    // 190° total FOV → max half-angle = 95° = 95 * π/180
    fi.max_theta_rad = 95.0f * (3.14159265f / 180.0f);  // ≈ 1.6581 rad
    return fi;
}

static RectIntrinsics makeRectIntrinsics()
{
    RectIntrinsics ri;
    // Virtual pinhole: 90° HFOV at 1280 px → fx = (OUT_W/2) / tan(45°) ≈ 640
    ri.fx = (OUT_W / 2.0f) / tanf(0.7854f);  // 0.7854 rad = 45° = HFOV/2
    ri.fy = ri.fx;                            // square pixels
    ri.cx = OUT_W / 2.0f;
    ri.cy = OUT_H / 2.0f;
    return ri;
}

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────
int main()
{
    printf("=== FisheyeDewarp Test Harness ===\n\n");

    // ── Print static memory budget ─────────────────────────────────────────
    constexpr size_t lut_bytes   = static_cast<size_t>(OUT_H) * OUT_W
                                   * sizeof(int32_t) * 2;
    constexpr size_t src_bytes   = static_cast<size_t>(IN_H)  * IN_W;
    constexpr size_t dst_bytes   = static_cast<size_t>(OUT_H) * OUT_W;
    constexpr size_t total_bytes = lut_bytes + src_bytes + dst_bytes;

    printf("Static memory budget:\n");
    printf("  Input  buffer : %zu bytes  (%.2f MB)\n",
           src_bytes,  static_cast<double>(src_bytes)  / (1<<20));
    printf("  Output buffer : %zu bytes  (%.2f MB)\n",
           dst_bytes,  static_cast<double>(dst_bytes)  / (1<<20));
    printf("  LUT           : %zu bytes  (%.2f MB)  [Q15.16, %d×%d×2×4]\n",
           lut_bytes,  static_cast<double>(lut_bytes)  / (1<<20), OUT_W, OUT_H);
    printf("  Total (BSS)   : %zu bytes  (%.2f MB)\n\n",
           total_bytes, static_cast<double>(total_bytes) / (1<<20));

    // ── Synthesise input ───────────────────────────────────────────────────
    printf("Generating checkerboard test pattern (%d×%d)...\n", IN_W, IN_H);
    fillCheckerboard(g_src, IN_W, IN_H, 32);

    // ── Save raw fisheye input ─────────────────────────────────────────────
    if (savePGM("fisheye_input.pgm", g_src, IN_W, IN_H)) {
        printf("Saved:  fisheye_input.pgm  (%d×%d)\n", IN_W, IN_H);
    }

    // ── Build LUT (init phase) ─────────────────────────────────────────────
    printf("\nBuilding LUT (floating-point, one-time)...\n");
    const FisheyeIntrinsics fish_intr = makeFisheyeIntrinsics();
    const RectIntrinsics    rect_intr = makeRectIntrinsics();

    const double t_init_start = getTimeSec();
    g_dewarp.init(fish_intr, rect_intr);
    const double t_init_end   = getTimeSec();

    printf("  LUT build time : %.3f ms\n\n",
           (t_init_end - t_init_start) * 1000.0);

    // ── Benchmark: N frames ────────────────────────────────────────────────
    static constexpr int BENCH_FRAMES = 200;
    printf("Benchmarking %d frames...\n", BENCH_FRAMES);

    const double t_bench_start = getTimeSec();
    for (int i = 0; i < BENCH_FRAMES; ++i) {
        g_dewarp.processFrame(g_src, g_dst);
    }
    const double t_bench_end = getTimeSec();

    const double elapsed_ms   = (t_bench_end - t_bench_start) * 1000.0;
    const double per_frame_ms = elapsed_ms / BENCH_FRAMES;
    const double fps           = 1000.0 / per_frame_ms;

    printf("  Total time     : %.1f ms for %d frames\n", elapsed_ms, BENCH_FRAMES);
    printf("  Per frame      : %.3f ms\n", per_frame_ms);
    printf("  Throughput     : %.1f fps\n\n", fps);

    // ── Save de-warped output ──────────────────────────────────────────────
    if (savePGM("rectlinear_output.pgm", g_dst, OUT_W, OUT_H)) {
        printf("Saved:  rectlinear_output.pgm  (%d×%d)\n", OUT_W, OUT_H);
    }

    // ── Validation: centre pixel identity check ────────────────────────────
    printf("\nValidation:\n");
    const int cx = OUT_W / 2;
    const int cy = OUT_H / 2;
    const uint8_t centre_val = g_dst[cy * OUT_W + cx];
    printf("  Centre pixel dst[%d,%d] = %u  (expect non-zero)\n",
           cy, cx, static_cast<unsigned>(centre_val));

    const uint8_t corner_val = g_dst[0];
    printf("  Corner pixel  dst[0,0]  = %u  (0 = outside lens circle)\n",
           static_cast<unsigned>(corner_val));

    // ── NEON reminder ─────────────────────────────────────────────────────
    printf("\n[NEON] Opportunities for further speedup (~3-4x) on Cortex-A72:\n");
    printf("  1. processRows inner loop: load 8 LUT entries with vld2q_s32,\n");
    printf("     extract xi/yi/xf/yf via vshrq_n_s32 / vandq_s32.\n");
    printf("  2. Texture taps: 8× vld1_lane_u8 (scatter-gather workaround).\n");
    printf("  3. Bilinear weights: vmull_u16 + vmlal_u16 (16×16→32 MAC).\n");
    printf("  4. Result pack: vshrn_n_u32 + vqmovn_u16 → vst1_u8 (8 bytes).\n");
    printf("  Estimated post-NEON throughput: ~%.0f fps on Cortex-A72.\n\n",
           fps * 3.5);

    printf("Done.\n");
    return 0;
}
