#pragma once

/**
 * fisheye_dewarp.hpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Fisheye → Rectilinear de-warping engine for ADAS Surround View Cameras.
 *
 * Algorithm : Kannala-Brandt / OcamCalib polynomial model
 *             r_d = θ + k1·θ³ + k2·θ⁵ + k3·θ⁷ + k4·θ⁹
 *             where θ = angle from optical axis,  r_d = distorted radius (px)
 *
 * LUT design : One-time init maps every output (rectilinear) pixel to a pair
 *              of sub-pixel source coordinates in the fisheye frame, stored as
 *              Q15.16 fixed-point integers (S16.16 fractional).
 *
 * Memory footprint (LUT) :
 *   OUT_W × OUT_H × 2 channels (x,y) × 4 bytes (int32) =
 *   e.g. 1280×720 → 1280×720×2×4 = 7,372,800 bytes ≈ 7.0 MB   (Q15.16)
 *
 * Fixed-point scheme : Q15.16
 *   integer part  : bits [31:16]  → value = raw >> FRAC_BITS
 *   fraction part : bits [15: 0]  → frac  = raw &  FRAC_MASK  (range 0…65535)
 *
 * Bilinear interpolation is executed entirely in integer arithmetic;
 * row blends are kept in int32, the final column blend uses int64 to avoid
 * an intermediate precision-loss shift.  No floating-point on the hot path.
 *
 * Thread-safety contract:
 *   init() must complete (happens-before) any call to processFrame/processRows.
 *   Multiple concurrent processRows() calls on non-overlapping row bands are
 *   safe: the function is read-only after init().  Concurrent init() with any
 *   process call is a data race — undefined behaviour.
 *
 * NEON opportunities are annotated with  // [NEON] comments.
 *
 * Author  : ADAS Prototype – embedded C++17, no heap allocations at runtime.
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <cstdint>
#include <cstddef>
#include <cmath>    // used ONLY during init(); not in the hot path
#include <cassert>

// ─── compile-time image geometry ─────────────────────────────────────────────
// Override these via compiler flags: -DFISHEYE_IN_W=1920 etc.
#ifndef FISHEYE_IN_W
#  define FISHEYE_IN_W   1280
#endif
#ifndef FISHEYE_IN_H
#  define FISHEYE_IN_H    960
#endif
#ifndef RECT_OUT_W
#  define RECT_OUT_W     1280
#endif
#ifndef RECT_OUT_H
#  define RECT_OUT_H      720
#endif

// ─── fixed-point configuration ───────────────────────────────────────────────
static constexpr int32_t FRAC_BITS  = 16;                    // Q15.16
static constexpr int32_t FRAC_SCALE = 1 << FRAC_BITS;       // 65536
static constexpr int32_t FRAC_MASK  = FRAC_SCALE - 1;       // 0x0000FFFF

// ─── camera intrinsic model ──────────────────────────────────────────────────
struct FisheyeIntrinsics {
    float fx;          ///< focal length x  (pixels)
    float fy;          ///< focal length y  (pixels)
    float cx;          ///< principal point x (pixels)
    float cy;          ///< principal point y (pixels)
    // Kannala-Brandt radial coefficients k[0]…k[3] correspond to k1…k4:
    //   r_d = θ + k[0]·θ³ + k[1]·θ⁵ + k[2]·θ⁷ + k[3]·θ⁹
    // Note: array is 0-based; calibration tools may label these k1–k4 (1-based).
    float k[4];
    float skew;        ///< axis skew (0 for most modern cameras)
    // Maximum angle from the optical axis that the lens physically covers
    // (radians).  Pixels whose incidence angle exceeds this value are marked
    // out-of-bounds and rendered black.  For a 190° total-FOV lens this is
    // 95° = 1.6581 rad.  Must be set per lens from calibration data.
    float max_theta_rad;
};

struct RectIntrinsics {
    float fx;          ///< virtual rectilinear focal length x
    float fy;          ///< virtual rectilinear focal length y
    float cx;          ///< principal point x in output frame
    float cy;          ///< principal point y in output frame
};

// ─── LUT entry ───────────────────────────────────────────────────────────────
// Stores the fisheye source coordinate for each output pixel in Q15.16.
// Two int32 per pixel → 8 bytes/pixel.
// A coordinate value of OOB_SENTINEL (= INT32_MIN, see .cpp) means the pixel
// lies outside the lens circle and must be rendered black.
struct LutEntry {
    int32_t src_x_fp;  ///< fisheye X in Q15.16  (OOB_SENTINEL means out-of-bounds)
    int32_t src_y_fp;  ///< fisheye Y in Q15.16
};

// ─── primary class ───────────────────────────────────────────────────────────

class FisheyeDewarp {
public:
    // Pixel format: 8-bit planar or interleaved.
    // All buffers are CALLER-owned; this class holds no owning pointers.
    static constexpr int IN_W  = FISHEYE_IN_W;
    static constexpr int IN_H  = FISHEYE_IN_H;
    static constexpr int OUT_W = RECT_OUT_W;
    static constexpr int OUT_H = RECT_OUT_H;

    // ── Non-copyable / non-movable ────────────────────────────────────────
    // The LUT alone is ~7 MB; accidental copy/move would be a silent perf
    // catastrophe.  Declare both operations deleted to turn them into a
    // compile-time error.
    FisheyeDewarp()                              = default;
    FisheyeDewarp(const FisheyeDewarp&)          = delete;
    FisheyeDewarp& operator=(const FisheyeDewarp&) = delete;
    FisheyeDewarp(FisheyeDewarp&&)               = delete;
    FisheyeDewarp& operator=(FisheyeDewarp&&)    = delete;
    ~FisheyeDewarp()                             = default;

    /**
     * @brief  One-time initialisation – builds the LUT.
     *         Must be called before the first processFrame().
     *         Floating-point heavy; NOT real-time safe; call at startup only.
     *
     * @param  fish   Fisheye camera intrinsics (including max_theta_rad)
     * @param  rect   Desired output rectilinear intrinsics
     */
    void init(const FisheyeIntrinsics& fish, const RectIntrinsics& rect) noexcept;

    /**
     * @brief  Convert one fisheye frame to rectilinear.
     *         No heap allocation.  No FP math (pure integer + bitwise).
     *         O(OUT_W × OUT_H) — each pixel: 1 LUT lookup + 4 texture taps
     *         + bilinear blend (all integer arithmetic).
     *
     * @param  src_y   Input  luma plane  [IN_H  × IN_W]  row-major
     * @param  dst_y   Output luma plane  [OUT_H × OUT_W] row-major
     *
     * Single-channel (Y) shown; to process U/V at half resolution construct
     * a separate FisheyeDewarp instance (or a scaled variant) sized to the
     * chroma dimensions and call init() with half-resolution intrinsics.
     */
    void processFrame(const uint8_t* __restrict__ src_y,
                            uint8_t* __restrict__ dst_y) const noexcept;

    /**
     * @brief  Same as processFrame() but processes a horizontal band of rows.
     *         Useful when splitting work across cores (no shared mutable state
     *         after init()).
     *
     *         Thread-safety: concurrent calls with non-overlapping [row_start,
     *         row_end) ranges and the same src/dst buffers are safe.
     *
     * @param  row_start  first row (inclusive)
     * @param  row_end    last  row (exclusive)
     */
    void processRows(const uint8_t* __restrict__ src_y,
                           uint8_t* __restrict__ dst_y,
                     int row_start, int row_end) const noexcept;

private:
    // ── static LUT storage ────────────────────────────────────────────────
    // Memory footprint: OUT_W × OUT_H × sizeof(LutEntry)
    //   = 1280 × 720 × 8  =  7,372,800 bytes  ≈  7.0 MB  (BSS segment)
    // Alignment to 64 bytes maximises cache-line utilisation and is required
    // for ARM NEON vld2q_s32 / vld4q_s32 instructions.
    alignas(64) LutEntry lut_[OUT_H][OUT_W];

    // Inverse input dimensions (Q15.16) – pre-computed once in init(),
    // available for debug/validation; not used in the hot path.
    int32_t inv_in_w_fp_{0};
    int32_t inv_in_h_fp_{0};

    // ── helpers ────────────────────────────────────────────────────────────

public:
    /**
     * @brief  Evaluate the Kannala-Brandt forward projection:
     *           r_d = θ + k[0]·θ³ + k[1]·θ⁵ + k[2]·θ⁷ + k[3]·θ⁹
     *         Returns the distorted pixel radius in floating-point.
     *         Called ONLY during init(); never on the hot path.
     *         Public so that unit tests and file-scope helpers can call it.
     */
    static float evalKBModel(float theta, const float k[4]) noexcept;

private:
    /**
     * @brief  Bilinear sample of src at position (x_fp, y_fp) in Q15.16.
     *
     * Precision contract:
     *   Row blends are computed in int32 (xf_inv·P ≤ 65536×255 = 16.7 M,
     *   well within int32).  The column blend uses int64 to avoid an early
     *   precision-loss shift: yf_inv·T ≤ 65536 × 16.7 M ≈ 1.1 × 10¹²
     *   which overflows int32 but fits comfortably in int64.  Final result
     *   is shifted right by FRAC_BITS*2 = 32 bits to recover [0, 255].
     *
     * Pre-condition (caller must guarantee):
     *   0 ≤ (x_fp >> FRAC_BITS) < IN_W-1
     *   0 ≤ (y_fp >> FRAC_BITS) < IN_H-1
     *   (processRows enforces this via the OOB guard before calling here)
     *
     * [NEON] This function is the primary SIMD target.
     *   Process 8 pixels simultaneously using:
     *     vld1q_u8  – load 8 top-left,  top-right,  bottom-left,  bottom-right
     *     vsubq_u16 – compute (1-frac_x), (1-frac_y) etc.
     *     vmull_u16 – 8-lane 16×16→32 multiply for bilinear weights
     *     vshrn_n_u32 – shift and narrow back to u8
     *   Expected throughput: ~4 clock cycles / pixel on Cortex-A72 vs ~12 scalar.
     */
    static inline uint8_t bilinearSample(const uint8_t* __restrict__ src,
                                         int32_t x_fp, int32_t y_fp) noexcept;
};
