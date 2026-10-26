// fumio_core.h - platform-independent core of the fumio MS-20 style filter chain.
//
// Based on fumio~.c (Johannes Regnier), itself based on Miller Puckette's bob~
// (Runge-Kutta 4th order, copyright 2015 Miller Puckette, BSD license).
//
// Signal path: input -> drive (soft saturator) -> resonant high-pass -> resonant low-pass -> dry/wet mix.
//
// Both sections are 2-pole filters with tanh feedback and the same small-signal response
//   LP(s) = 1 / (s^2 + (2-r) s + 1),   HP(s) = s^2 / (s^2 + (2-r) s + 1)     (s in units of the cutoff)
// so both ring more as r rises and both start to self-oscillate at exactly r = 2.
//
// The low-pass is the original external's equations. The original external's high-pass mode never
// used its resonance term at all (it was a plain 1-pole high-pass), so the high-pass here is a
// new resonant section built as the exact dual of the low-pass (see HpChannel).
//
// Differences from the Pd external, driven by the NTS-3 target (Cortex-M7, single-precision FPU
// only, no malloc):
//   * float instead of double (double is software-emulated on this FPU)
//   * tanh() replaced by a rational approximation (see fast_tanh)
//   * state clamp as a guard against runaway values
//   * parameters are smoothed per sample (pad moves are not zippery)
//
// No Pd or SDK dependencies, so it can be unit tested on a desktop.
#pragma once

#include <stdint.h>
#include <math.h>

namespace fumio {

static const float kTwoPi = 6.2831853f;
static const float kStateLimit = 8.0f;  // guard only; normal signals stay well below
static constexpr float kSelfOscRes = 2.0f;  // raw resonance value where both sections start to self-oscillate

// tanh approximation: (7,6) Pade form, max error ~1e-4, clamped to +-1 where the rational
// reaches 1 (|x| ~ 4.97) so it stays monotonic and continuous. The cheaper (3,2) form is only
// ~2% accurate, which audibly changes the filter near self-oscillation.
static inline float fast_tanh(float x) {
#ifdef FUMIO_EXACT_TANH
  return tanhf(x);  // desktop reference builds only
#endif
  if (x >= 4.97f) return 1.f;
  if (x <= -4.97f) return -1.f;
  const float x2 = x * x;
  const float num = x * (135135.f + x2 * (17325.f + x2 * (378.f + x2)));
  const float den = 135135.f + x2 * (62370.f + x2 * (3150.f + 28.f * x2));
  return num / den;
}

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// ---- Low-pass section (the original external's equations) ----------------------------------
static inline void lp_derivs(float in, float k, float r, float a, float b, float &da, float &db) {
  const float t = fast_tanh(r * b);
  da = k * (in - a - t);
  db = k * (a - b + t);
}

// Two states, RK4 integrator with `os` substeps per sample. `in` is held constant across substeps.
struct LpChannel {
  float s0 = 0.f, s1 = 0.f;

  void reset() { s0 = s1 = 0.f; }

  inline float tick(float in, float k, float r, float h, int os) {
    float a = s0, b = s1;
    for (int j = 0; j < os; ++j) {
      float a1, b1, a2, b2, a3, b3, a4, b4;
      lp_derivs(in, k, r, a, b, a1, b1);
      lp_derivs(in, k, r, a + 0.5f * h * a1, b + 0.5f * h * b1, a2, b2);
      lp_derivs(in, k, r, a + 0.5f * h * a2, b + 0.5f * h * b2, a3, b3);
      lp_derivs(in, k, r, a + h * a3, b + h * b3, a4, b4);
      a += (h / 6.f) * (a1 + 2.f * a2 + 2.f * a3 + a4);
      b += (h / 6.f) * (b1 + 2.f * b2 + 2.f * b3 + b4);
      a = clampf(a, -kStateLimit, kStateLimit);
      b = clampf(b, -kStateLimit, kStateLimit);
    }
    s0 = a;
    s1 = b;
    return b;
  }
};

// ---- Resonant high-pass section ------------------------------------------------------------
// Dual of the low-pass: each of its two one-pole low-passes is replaced by a one-pole high-pass
// (v - lowpass(v)), with the same tanh feedback taken from the output. The feedback term cancels
// out of the output, which leaves an explicit system in two states p, q:
//   y  = in - p - q                       (output)
//   p' = k (in - p - tanh(r y))
//   q' = k y
// Its small-signal response is s^2 / (s^2 + (2-r) s + 1): same poles as the low-pass, so the same
// resonance value r gives the same peak height and the same self-oscillation threshold (r = 2).
static inline void hp_derivs(float in, float k, float r, float p, float q, float &dp, float &dq) {
  const float y = in - p - q;
  dp = k * (in - p - fast_tanh(r * y));
  dq = k * y;
}

struct HpChannel {
  float p = 0.f, q = 0.f;

  void reset() { p = q = 0.f; }

  inline float tick(float in, float k, float r, float h, int os) {
    float a = p, b = q;
    for (int j = 0; j < os; ++j) {
      float a1, b1, a2, b2, a3, b3, a4, b4;
      hp_derivs(in, k, r, a, b, a1, b1);
      hp_derivs(in, k, r, a + 0.5f * h * a1, b + 0.5f * h * b1, a2, b2);
      hp_derivs(in, k, r, a + 0.5f * h * a2, b + 0.5f * h * b2, a3, b3);
      hp_derivs(in, k, r, a + h * a3, b + h * b3, a4, b4);
      a += (h / 6.f) * (a1 + 2.f * a2 + 2.f * a3 + a4);
      b += (h / 6.f) * (b1 + 2.f * b2 + 2.f * b3 + b4);
      a = clampf(a, -kStateLimit, kStateLimit);
      b = clampf(b, -kStateLimit, kStateLimit);
    }
    p = a;
    q = b;
    return in - a - b;
  }
};

// ---- Stereo effect: high-pass into low-pass -------------------------------------------------
// The low-pass resonance is a live (smoothed) control; the high-pass resonance is a fixed value
// (hp_res) set by the host.
struct Fx {
  static const int kMaxCh = 2;

  // user-facing targets (set from the parameter callbacks)
  float hp_hz_target = 20.f;
  float lp_hz_target = 12000.f;
  float lp_res_target = 0.f;
  float mix_target = 1.f;
  float drive_target = 0.f;  // saturation amount a >= 0: input goes through tanh(a x) / a. 0 = clean
  float hp_res = 0.f;  // fixed high-pass resonance (raw r, not smoothed)
  int oversample = 2;

  // smoothed state
  float hp_hz = 20.f, lp_hz = 12000.f, lp_res = 0.f, mix = 1.f, drive = 0.f;
  float sr = 48000.f;
  float smooth = 0.004f;
  int channels = 2;
  HpChannel hp[kMaxCh];
  LpChannel lp[kMaxCh];

  void init(float samplerate, int channels_) {
    // Set every field explicitly: on the device the unit may be a static object whose
    // constructor/initializers are not guaranteed to have run.
    sr = samplerate;
    channels = channels_ > kMaxCh ? kMaxCh : channels_;
    smooth = 1.f - expf(-1.f / (0.005f * sr));  // ~5 ms
    hp_hz_target = 20.f;
    lp_hz_target = 12000.f;
    lp_res_target = 0.f;
    mix_target = 1.f;
    drive_target = 0.f;
    hp_res = 0.f;
    oversample = 2;
    snap();
    reset();
  }

  void snap() {
    hp_hz = hp_hz_target;
    lp_hz = lp_hz_target;
    lp_res = lp_res_target;
    mix = mix_target;
    drive = drive_target;
  }

  void reset() {
    for (int c = 0; c < kMaxCh; ++c) {
      hp[c].reset();
      lp[c].reset();
    }
  }

  void set_oversample(int o) { oversample = o < 1 ? 1 : (o > 4 ? 4 : o); }

  // `in` / `out` are interleaved with `channels` samples per frame.
  void process(const float *in, float *out, uint32_t frames) {
    const int os = oversample;
    const float h = 1.f / ((float)os * sr);
    // Keep k*h (cutoff x sub-step) in RK4's stable range: the ceiling scales with oversampling,
    // 12 kHz at 1x, up to 20 kHz from 2x.
    float fmax = 0.25f * sr * (float)os;
    if (fmax > 20000.f) fmax = 20000.f;
    for (uint32_t n = 0; n < frames; ++n) {
      hp_hz += smooth * (hp_hz_target - hp_hz);
      lp_hz += smooth * (lp_hz_target - lp_hz);
      lp_res += smooth * (lp_res_target - lp_res);
      mix += smooth * (mix_target - mix);
      drive += smooth * (drive_target - drive);
      const float k_hp = kTwoPi * (hp_hz < fmax ? hp_hz : fmax);
      const float k_lp = kTwoPi * (lp_hz < fmax ? lp_hz : fmax);
      // DRIVE: soft saturation in front of the filters, y = tanh(a x) / a. It has unity gain for
      // small signals (so quiet material is untouched and a = 0 is exactly clean) and squashes and
      // distorts louder material more and more as `a` rises. The dry path stays clean.
      const bool sat = drive > 1e-3f;
      const float inv_a = sat ? 1.f / drive : 0.f;
      for (int c = 0; c < channels; ++c) {
        const float x = in[n * channels + c];
        const float xs = sat ? fast_tanh(drive * x) * inv_a : x;
        const float v = hp[c].tick(xs, k_hp, hp_res, h, os);
        const float w = lp[c].tick(v, k_lp, lp_res, h, os);
        out[n * channels + c] = x + mix * (w - x);
      }
    }
  }
};

}  // namespace fumio
