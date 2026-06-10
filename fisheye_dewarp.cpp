/**
 * fisheye_dewarp.cpp
 * ─────────────────────────────────────────────────────────────────────────────
 * Implementation of the FisheyeDewarp engine.
 *
 * Hot-path summary (processRows):
 *   For each output pixel (r, c):
 *     1. Load LutEntry  [2 × int32  = 8 bytes]      → 1 cache line shared
 *        with ~7 neighbours (64-byte line / 8 bytes = 8 entries/line)
 *     2. OOB guard (unsigned compare)                → 2 comparisons, no branch
 *        on the common in-bounds case
 *     3. Extract integer and fractional parts         → 4 bitwise ops
 *     4. Bilinear sample (int32 row blends, int64     → 8 mul + 3 add + 1 shift
 *        column blend)
 *     5. Store uint8                                  → 1 write
 *   Total ≈ 20 integer ops/pixel  (no division, no FP).
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include "fisheye_dewarp.hpp"

#include <cmath>     // sqrtf, atan2f — init() only
#include <cstring>   // memset

// ─────────────────────────────────────────────────────────────────────────────
//  Internal constants
// ─────────────────────────────────────────────────────────────────────────────

// Newton-Raphson convergence parameters used only in init().
static constexpr int   NR_MAX_ITER = 8;       // raised from 6; improves accuracy
                                               // for wide-angle (>160°) lenses
static constexpr float NR_EPSILON  = 1e-7f;

// ─────────────────────────────────────────────────────────────────────────────
//  evalKBModel()
//  r_d = θ·(1 + θ²·(k[0] + θ²·(k[1] + θ²·(k[2] + θ²·k[3]))))
//  Horner's method — minimises multiplications.
//  k[0]…k[3] correspond to the calibration-tool labels k1…k4 (1-based).
//  Called ONLY from init() – floating-point is acceptable here.
// ─────────────────────────────────────────────────────────────────────────────
float FisheyeDewarp::evalKBModel(float theta,
                                 const float k[FisheyeIntrinsics::KB_COEFF_COUNT]) noexcept
{
    const float t2 = theta * theta;
    return theta * (1.0f + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3]))));
}

// ─────────────────────────────────────────────────────────────────────────────
//  evalKBDerivative()  (file-scope; used only during init())
//  dr_d/dθ = 1 + 3·k[0]·θ² + 5·k[1]·θ⁴ + 7·k[2]·θ⁶ + 9·k[3]·θ⁸
// ─────────────────────────────────────────────────────────────────────────────
static float evalKBDerivative(float theta,
                              const float k[FisheyeIntrinsics::KB_COEFF_COUNT]) noexcept
{
    const float t2 = theta * theta;
    return 1.0f + t2 * (3.0f*k[0] + t2 * (5.0f*k[1] + t2 * (7.0f*k[2] + t2 * 9.0f*k[3])));
}

// ─────────────────────────────────────────────────────────────────────────────
//  invertKBModel()
//  Given a distorted radius r_d, find θ such that evalKBModel(θ) = r_d via
//  Newton-Raphson.
//
//  Initial guess: θ₀ = atan(r_d / fx_approx)
//  Using atan rather than the identity (θ₀ = r_d) gives a much better
//  starting point for wide-angle lenses where r_d can be several focal
//  lengths, halving the required iteration count.
//
//  Not used in the current LUT build path (which goes output→ray→fisheye
//  using the forward model), but retained for future online calibration
//  refinement / inverse-mapping use cases.
// ─────────────────────────────────────────────────────────────────────────────
[[maybe_unused]] static float invertKBModel(float r_d,
                            const float k[FisheyeIntrinsics::KB_COEFF_COUNT],
                            float fx_approx = 1.0f) noexcept
{
    if (r_d < NR_EPSILON) return 0.0f;

    // Better initial guess than the raw identity for large r_d values.
    float theta = atanf(r_d / fx_approx);

    for (int i = 0; i < NR_MAX_ITER; ++i) {
        const float f  = FisheyeDewarp::evalKBModel(theta, k) - r_d;
        const float df = evalKBDerivative(theta, k);
        if (df < NR_EPSILON) break;
        const float delta = f / df;
        theta -= delta;
        if (theta < 0.0f) theta = 0.0f;
        if (fabsf(delta) < NR_EPSILON) break;
    }
    return theta;
}

// ─────────────────────────────────────────────────────────────────────────────
//  init()
//  Complexity : O(OUT_W × OUT_H)  floating-point ops — startup only.
//  For every output pixel (oc, or):
//    1. Unproject through virtual rectilinear camera → unit ray (X,Y,Z)
//    2. Compute incidence angle  θ = atan2(sqrt(X²+Y²), Z)
//    3. Evaluate KB model forward: r_d = KB(θ)
//    4. Project onto fisheye sensor plane → (u, v) in pixels
//    5. Store as Q15.16 fixed-point in LUT
// ─────────────────────────────────────────────────────────────────────────────
void FisheyeDewarp::init(const FisheyeIntrinsics& fish,
                         const RectIntrinsics&    rect) noexcept
{
    for (int or_ = 0; or_ < OUT_H; ++or_) {
        for (int oc = 0; oc < OUT_W; ++oc) {

            // ── Step 1: rectilinear pixel → normalised ray ──────────────────
            const float Xn = (static_cast<float>(oc) - rect.cx) / rect.fx;
            const float Yn = (static_cast<float>(or_) - rect.cy) / rect.fy;
            // Zn = 1.0 (optical axis)

            // ── Step 2: incidence angle θ ───────────────────────────────────
            const float r_norm = sqrtf(Xn*Xn + Yn*Yn);
            const float theta  = atan2f(r_norm, 1.0f);

            // Mark pixels beyond the lens's physical FOV as out-of-bounds.
            // max_theta_rad is supplied per-lens via FisheyeIntrinsics so that
            // lenses with >180° total FOV (θ > π/2 from axis) are handled
            // correctly — e.g. a 190° lens has max_theta_rad ≈ 95° = 1.658 rad.
            if (theta > fish.max_theta_rad) {
                lut_[or_][oc] = { LutEntry::OOB_SENTINEL, LutEntry::OOB_SENTINEL };
                continue;
            }

            // ── Step 3: KB forward model → distorted radius r_d ────────────
            const float r_d = FisheyeDewarp::evalKBModel(theta, fish.k);

            // ── Step 4: project onto fisheye sensor ─────────────────────────
            float src_x, src_y;
            if (r_norm > NR_EPSILON) {
                // u = fx·(r_d/r_norm)·Xn  +  skew·(r_d/r_norm)·Yn  +  cx
                // v = fy·(r_d/r_norm)·Yn                             +  cy
                const float scale = r_d / r_norm;
                src_x = fish.fx * scale * Xn + fish.skew * scale * Yn + fish.cx;
                src_y = fish.fy * scale * Yn                           + fish.cy;
            } else {
                // θ ≈ 0: output pixel maps to the fisheye principal point.
                src_x = fish.cx;
                src_y = fish.cy;
            }

            // ── Step 5: store in Q15.16 fixed-point ─────────────────────────
            // We need xi+1 and yi+1 to exist for bilinearSample, so the valid
            // range is [0, IN_W-2] and [0, IN_H-2] in integer coords, i.e.
            // [0, (IN_W-1)*FRAC_SCALE) and [0, (IN_H-1)*FRAC_SCALE) in Q15.16.
            if (src_x < 0.0f || src_x >= static_cast<float>(IN_W - 1) ||
                src_y < 0.0f || src_y >= static_cast<float>(IN_H - 1))
            {
                lut_[or_][oc] = { LutEntry::OOB_SENTINEL, LutEntry::OOB_SENTINEL };
            } else {
                lut_[or_][oc].src_x_fp = static_cast<int32_t>(src_x * FRAC_SCALE);
                lut_[or_][oc].src_y_fp = static_cast<int32_t>(src_y * FRAC_SCALE);
            }
        }
    }

    // Pre-compute inverse dimensions (debug/validation only; not in hot path).
    inv_in_w_fp_ = FRAC_SCALE / IN_W;
    inv_in_h_fp_ = FRAC_SCALE / IN_H;
    init_done_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  bilinearSample()
//  Performs bilinear interpolation at sub-pixel position (x_fp, y_fp) in Q15.16.
//
//  Let xi = x_fp >> 16,  xf = x_fp & 0xFFFF   (integer and fractional parts)
//      yi = y_fp >> 16,  yf = y_fp & 0xFFFF
//
//  Standard bilinear blend:
//    P = (1-xf)(1-yf)·P00  +  xf(1-yf)·P10  +  (1-xf)yf·P01  +  xf·yf·P11
//
//  Precision:
//    Row blends T, B stay in int32 (max 65536×255 = 16.7 M ≪ 2³¹).
//    Column blend uses int64 so we never truncate T/B before multiplying by
//    yf_inv/yf (which would lose up to 8 bits of sub-pixel precision).
//    Final right-shift of 32 (= FRAC_BITS * 2) recovers the 8-bit result.
//
//  Pre-condition: xi ∈ [0, IN_W-2], yi ∈ [0, IN_H-2]  (caller guarantees).
//
//  [NEON] This function is the primary SIMD target.
//   Process 8 pixels simultaneously using:
//     vld1q_u8  – load 8 top-left, top-right, bottom-left, bottom-right taps
//     vmull_u16 – 8-lane 16×16→32 multiply for bilinear row weights
//     vmlal_u16 – accumulate second term
//     vmull_u32 – 32-bit column blend (or widen to 64-bit lanes)
//     vshrn     – shift and narrow back to u8
//   Expected throughput: ~4 clock cycles / pixel on Cortex-A72 vs ~12 scalar.
// ─────────────────────────────────────────────────────────────────────────────
uint8_t FisheyeDewarp::bilinearSample(const uint8_t* __restrict__ src,
                                      int32_t x_fp,
                                      int32_t y_fp) noexcept
{
    // Integer pixel coords (floor).  Arithmetic right shift on AArch64: ASR.
    const int32_t xi = x_fp >> FRAC_BITS;
    const int32_t yi = y_fp >> FRAC_BITS;

    // Pre-condition: xi and yi are in-bounds.  Debug builds catch violations.
    assert(xi >= 0 && xi < IN_W - 1);
    assert(yi >= 0 && yi < IN_H - 1);

    // Fractional parts [0, 65535].
    const int32_t xf     = x_fp & FRAC_MASK;
    const int32_t yf     = y_fp & FRAC_MASK;
    const int32_t xf_inv = FRAC_SCALE - xf;   // (1 - xf) in Q0.16
    const int32_t yf_inv = FRAC_SCALE - yf;   // (1 - yf) in Q0.16

    // Four neighbouring pixel addresses.  Row stride = IN_W.
    const uint8_t* row0 = src + yi * IN_W + xi;
    const uint8_t* row1 = row0 + IN_W;

    // Load four 8-bit taps.
    // [NEON]: vld1_lane_u8 + vmovl_u8 to expand 8 taps at once.
    const int32_t P00 = static_cast<int32_t>(row0[0]);
    const int32_t P10 = static_cast<int32_t>(row0[1]);
    const int32_t P01 = static_cast<int32_t>(row1[0]);
    const int32_t P11 = static_cast<int32_t>(row1[1]);

    // ── Row blends (int32) ──────────────────────────────────────────────────
    // T = (1-xf)·P00 + xf·P10    max value = 65536 × 255 = 16,711,680  (< 2³¹)
    // B = (1-xf)·P01 + xf·P11
    // Units: Q0.16 × [0,255] → Q8.16 integer product, kept in int32.
    const int32_t T = xf_inv * P00 + xf * P10;   // Q8.16
    const int32_t B = xf_inv * P01 + xf * P11;   // Q8.16

    // ── Column blend (int64) ────────────────────────────────────────────────
    // yf_inv·T  max = 65536 × 16,711,680 ≈ 1.096 × 10¹²  → requires int64.
    // Total max = 2 × 1.096 × 10¹² ≈ 2.19 × 10¹²  < 2⁶³  → safe.
    // Final shift: FRAC_BITS*2 = 32  →  result ∈ [0, 255].
    const int64_t result = (static_cast<int64_t>(yf_inv) * T +
                            static_cast<int64_t>(yf)     * B) >> (FRAC_BITS * 2);

    return static_cast<uint8_t>(result);
}

// ─────────────────────────────────────────────────────────────────────────────
//  processFrame()  – full-frame wrapper; delegates to processRows().
// ─────────────────────────────────────────────────────────────────────────────
void FisheyeDewarp::processFrame(const uint8_t* __restrict__ src_y,
                                       uint8_t* __restrict__ dst_y) const noexcept
{
    assert(init_done_);
    processRows(src_y, dst_y, 0, OUT_H);
}

// ─────────────────────────────────────────────────────────────────────────────
//  processRows()  – the real-time hot path.
//
//  Complexity : O((row_end - row_start) × OUT_W)
//  Memory accesses per pixel:
//    Read  : 1 LutEntry (8 bytes) + 4 src pixels (4 bytes) = 12 bytes
//    Write : 1 dst pixel (1 byte)
//
//  Cache behaviour:
//    LUT  : accessed sequentially → excellent prefetch / HW prefetcher friendly.
//    src  : two adjacent rows accessed per output pixel; stride = IN_W bytes.
//           For IN_W=1280: two rows fit in ~2.5 KB → typically hot in L1.
//    dst  : written sequentially → store-buffer coalescing.
//
//  OOB guard:
//    Both x_fp and y_fp are compared as uint32_t against their respective
//    fixed-point upper bounds.  This single unsigned comparison simultaneously
//    rejects OOB_SENTINEL (= INT32_MIN, which casts to a huge uint32) and any
//    negative coordinate, with fully defined behaviour (no signed-overflow UB).
//
//  [NEON] Vectorisation strategy for the inner loop:
//    Process NEON_LANES=8 output pixels per iteration.
//    1. vld2q_s32  → load 8 x_fp (q0) and 8 y_fp (q1) from LUT
//    2. vshrq_n_s32(q0, 16) → xi[8],  vshrq_n_s32(q1, 16) → yi[8]
//    3. vandq_s32            → xf[8], yf[8]
//    4. Scatter-gather taps  (A72 has no native gather; use 8× vld1_lane_u8)
//    5. vmull / vmlal        → bilinear weights
//    6. vshrn_n_u32 → u8[8], vst1_u8 → store 8 bytes
//    Estimated IPC speedup: 3–4× over scalar on Cortex-A72.
// ─────────────────────────────────────────────────────────────────────────────
void FisheyeDewarp::processRows(const uint8_t* __restrict__ src_y,
                                      uint8_t* __restrict__ dst_y,
                                int row_start, int row_end) const noexcept
{
    assert(init_done_);
    static_assert(OUT_W > 0 && OUT_H > 0, "Output dimensions must be positive");
    static_assert(IN_W  > 0 && IN_H  > 0, "Input dimensions must be positive");

    // Upper bounds in Q15.16 (unsigned).  A pixel is valid iff
    //   (uint32_t)x_fp < x_max_fp  AND  (uint32_t)y_fp < y_max_fp.
    // We use (IN_W-1) / (IN_H-1) because bilinearSample reads xi+1 / yi+1.
    // Cast to uint32_t so the comparison rejects both negative values and
    // OOB_SENTINEL in a single branch — no signed-integer UB.
    const uint32_t x_max_fp = static_cast<uint32_t>((IN_W - 1) << FRAC_BITS);
    const uint32_t y_max_fp = static_cast<uint32_t>((IN_H - 1) << FRAC_BITS);

    for (int r = row_start; r < row_end; ++r) {
        uint8_t* __restrict__       dst_row = dst_y  + r * OUT_W;
        const LutEntry* __restrict__ lut_row = lut_[r];

        for (int c = 0; c < OUT_W; ++c) {
            // ── LUT lookup ──────────────────────────────────────────────────
            // [NEON]: vld2q_s32 to load 8 entries at once (interleaved x/y).
            const int32_t x_fp = lut_row[c].src_x_fp;
            const int32_t y_fp = lut_row[c].src_y_fp;

            // ── OOB guard (branchless-friendly, defined behaviour) ───────────
            // Cast to uint32_t: negative values and OOB_SENTINEL become large
            // unsigned numbers that fail the < x_max_fp test immediately.
            // Compiles to two unsigned CMP + conditional-zero on AArch64.
            if (static_cast<uint32_t>(x_fp) >= x_max_fp ||
                static_cast<uint32_t>(y_fp) >= y_max_fp)
            {
                dst_row[c] = 0u;
                continue;
            }

            // ── Bilinear sample ─────────────────────────────────────────────
            // [NEON]: replace this call with the 8-lane vectorised version.
            dst_row[c] = bilinearSample(src_y, x_fp, y_fp);
        }
    }
}
