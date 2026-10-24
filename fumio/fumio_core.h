// fumio_core.h - platform-independent core of the fumio~ MS-20 style 2-pole filter.
//
// Port of fumio~.c (Johannes Regnier, based on Miller Puckette's bob~, BSD license).
// Differences from the Pd external, all driven by the NTS-3 target (Cortex-M7, single
// precision FPU only, no malloc):
//   * float instead of double (double is software-emulated on this FPU)
//   * tanh() replaced by a rational approximation (see fast_tanh)
//   * state clamp as a guard against runaway values
//   * cutoff / resonance / mix / gain are smoothed per sample (pad moves are not zippery)
//
// No Pd or SDK dependencies, so it can be unit tested on a desktop.
#pragma once

#include <stdint.h>
#include <math.h>

namespace fumio {

enum Mode : int { kLP = 1, kBP = 2, kHP = 3 };

static const float kTwoPi = 6.2831853f;
static const float kStateLimit = 8.0f;  // guard only; normal signals stay well below

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

static inline void derivs(int mode, float in, float k, float r, float a, float b,
                          float &da, float &db) {
  const float t = fast_tanh(r * b);
  if (mode == kLP) {
    da = k * (in - a - t);
    db = k * (a - b + t);
  } else if (mode == kBP) {
    da = k * (-in - a - t);
    db = k * (in + a - b + t);
  } else {  // kHP, same equations as the Pd external
    da = k * (a - t);
    db = k * (-in - b);
  }
}

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// One filter channel: two states, RK4 integrator with `os` substeps per sample.
struct Channel {
  float s0 = 0.f, s1 = 0.f;

  void reset() { s0 = s1 = 0.f; }

  // `in` is held constant across substeps (as in the Pd external).
  inline float tick(int mode, float in, float k, float r, float h, int os) {
    float a = s0, b = s1;
    for (int j = 0; j < os; ++j) {
      float a1, b1, a2, b2, a3, b3, a4, b4;
      derivs(mode, in, k, r, a, b, a1, b1);
      derivs(mode, in, k, r, a + 0.5f * h * a1, b + 0.5f * h * b1, a2, b2);
      derivs(mode, in, k, r, a + 0.5f * h * a2, b + 0.5f * h * b2, a3, b3);
      derivs(mode, in, k, r, a + h * a3, b + h * b3, a4, b4);
      a += (h / 6.f) * (a1 + 2.f * a2 + 2.f * a3 + a4);
      b += (h / 6.f) * (b1 + 2.f * b2 + 2.f * b3 + b4);
      a = clampf(a, -kStateLimit, kStateLimit);
      b = clampf(b, -kStateLimit, kStateLimit);
    }
    s0 = a;
    s1 = b;
    return (mode == kHP) ? b + in : b;
  }
};

// Stereo effect: MS-20 style chain, high-pass into low-pass, one shared resonance
// (like the single PEAK control on the MS-20), with parameter smoothing.
struct Fx {
  static const int kMaxCh = 2;

  // user-facing targets (set from the parameter callbacks)
  float hp_hz_target = 20.f;
  float lp_hz_target = 12000.f;
  float res_target = 0.f;
  float mix_target = 1.f;
  float gain_target = 1.f;
  int oversample = 2;

  // smoothed state
  float hp_hz = 20.f, lp_hz = 12000.f, res = 0.f, mix = 1.f, gain = 1.f;
  float sr = 48000.f;
  float smooth = 0.004f;
  int channels = 2;
  Channel hp[kMaxCh], lp[kMaxCh];

  void init(float samplerate, int channels_) {
    // Set every field explicitly: on the device the unit may be a static object whose
    // constructor/initializers are not guaranteed to have run.
    sr = samplerate;
    channels = channels_ > kMaxCh ? kMaxCh : channels_;
    smooth = 1.f - expf(-1.f / (0.005f * sr));  // ~5 ms
    hp_hz_target = 20.f;
    lp_hz_target = 12000.f;
    res_target = 0.f;
    mix_target = 1.f;
    gain_target = 1.f;
    oversample = 2;
    snap();
    reset();
  }

  void snap() {
    hp_hz = hp_hz_target;
    lp_hz = lp_hz_target;
    res = res_target;
    mix = mix_target;
    gain = gain_target;
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
      res += smooth * (res_target - res);
      mix += smooth * (mix_target - mix);
      gain += smooth * (gain_target - gain);
      const float k_hp = kTwoPi * (hp_hz < fmax ? hp_hz : fmax);
      const float k_lp = kTwoPi * (lp_hz < fmax ? lp_hz : fmax);
      // gain >= 1 is "drive": it hits the filters harder (more resonance saturation) and
      // is divided back out of the wet signal so the passband level stays about unity.
      const float g = gain < 1.f ? 1.f : gain;
      const float inv_g = 1.f / g;
      for (int c = 0; c < channels; ++c) {
        const float x = in[n * channels + c];
        const float v = hp[c].tick(kHP, x * g, k_hp, res, h, os);
        const float w = lp[c].tick(kLP, v, k_lp, res, h, os) * inv_g;
        out[n * channels + c] = x + mix * (w - x);
      }
    }
  }
};

}  // namespace fumio
