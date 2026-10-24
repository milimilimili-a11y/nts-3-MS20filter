// Desktop test for the fumio chain (HPF -> LPF, shared resonance).
// Compares the float / fast-tanh port against a double-precision transcription of the original
// fumio~ maths, checks stability at extreme settings, and sanity-checks the frequency response.
// Build: g++ -O2 -std=c++14 test_core.cc -o test_core && ./test_core
#include <cmath>
#include <cstdio>
#include <vector>
#include "fumio_core.h"

// Reference: straight transcription of the Pd external (double, libm tanh).
struct Ref {
  double s[2] = {0, 0};
  int mode = 1;
  double in = 0, cutoff = 1000, res = 0;
  void d(double *ds, const double *st) {
    double k = 2 * 3.14159 * cutoff;
    double t = tanh(res * st[1]);
    if (mode == 1) { ds[0] = k * (in - st[0] - t); ds[1] = k * (st[0] - st[1] + t); }
    else if (mode == 3) { ds[0] = k * (st[0] - t); ds[1] = k * (-in - st[1]); }
    else { ds[0] = k * (-in - st[0] - t); ds[1] = k * (in + st[0] - st[1] + t); }
  }
  double tick(double x, double sr, int os) {
    in = x;
    double h = 1.0 / (os * sr);
    for (int j = 0; j < os; ++j) {
      double d1[2], d2[2], d3[2], d4[2], t[2];
      d(d1, s);
      for (int i = 0; i < 2; ++i) t[i] = s[i] + 0.5 * h * d1[i];
      d(d2, t);
      for (int i = 0; i < 2; ++i) t[i] = s[i] + 0.5 * h * d2[i];
      d(d3, t);
      for (int i = 0; i < 2; ++i) t[i] = s[i] + h * d3[i];
      d(d4, t);
      for (int i = 0; i < 2; ++i) s[i] += (1. / 6.) * h * (d1[i] + 2 * d2[i] + 2 * d3[i] + d4[i]);
    }
    return mode == 3 ? s[1] + in : s[1];
  }
};

static double rms(const std::vector<float> &v, size_t from) {
  double a = 0;
  for (size_t i = from; i < v.size(); ++i) a += (double)v[i] * v[i];
  return sqrt(a / (v.size() - from));
}

static void setup(fumio::Fx &fx, int ch, float hp, float lp, float res, float mix, float gain, int os) {
  fx.init(48000.f, ch);  // init first (resets to defaults), then set params, as on the device
  fx.hp_hz_target = hp; fx.lp_hz_target = lp; fx.res_target = res;
  fx.mix_target = mix; fx.gain_target = gain;
  fx.set_oversample(os);
  fx.snap();
}

int main() {
  const float sr = 48000.f;
  const int N = 48000;
  int fails = 0;

  // 1) Chain vs double reference (mono, mix=1, gain=1)
  for (float res : {0.f, 1.f, 3.f}) {
    fumio::Fx fx;
    setup(fx, 1, 300.f, 3000.f, res, 1.f, 1.f, 2);
    Ref rh, rl; rh.mode = 3; rh.cutoff = 300; rh.res = res; rl.mode = 1; rl.cutoff = 3000; rl.res = res;
    std::vector<float> in(N), out(N);
    for (int n = 0; n < N; ++n)
      in[n] = 0.4f * sinf(6.2831853f * 220.f * n / sr) + 0.3f * sinf(6.2831853f * 1500.f * n / sr) + 0.2f * sinf(6.2831853f * 9000.f * n / sr);
    fx.process(in.data(), out.data(), N);
    double err = 0, sig = 0;
    for (int n = 0; n < N; ++n) {
      double y = rl.tick(rh.tick(in[n], sr, 2), sr, 2);
      if (n >= 2000) { err += (y - out[n]) * (y - out[n]); sig += y * y; }
    }
    double rel = sqrt(err / (sig + 1e-12));
    printf("chain res %.0f: relative error vs double reference = %.4f%s\n", res, rel, rel > 0.05 ? "  <-- FAIL" : "");
    if (rel > 0.05) ++fails;
  }

  // 2) Stability at extremes: noise in, every cutoff pairing, max resonance, max drive, stereo
  for (float hp : {20.f, 1000.f, 12000.f}) {
    for (float lp : {20.f, 1000.f, 20000.f}) {
      for (float res : {0.f, 6.f, 20.f}) {
        for (int os : {1, 4}) {
          fumio::Fx fx;
          setup(fx, 2, hp, lp, res, 1.f, 4.f, os);
          std::vector<float> in(2 * N), out(2 * N);
          unsigned s = 12345;
          for (auto &x : in) { s = s * 1664525u + 1013904223u; x = ((s >> 8) & 0xffff) / 32768.f - 1.f; }
          fx.process(in.data(), out.data(), N);
          bool bad = false; float peak = 0;
          for (float y : out) { if (!std::isfinite(y)) bad = true; peak = fmaxf(peak, fabsf(y)); }
          if (bad || peak > 20.f) { printf("UNSTABLE hp %.0f lp %.0f res %.0f os %d peak %.2f\n", hp, lp, res, os, peak); ++fails; }
        }
      }
    }
  }
  printf("stability sweep done\n");

  // 3) Frequency response sanity (res 0, mix 1, drive 1)
  auto gain_at = [&](float hp, float lp, float f) {
    fumio::Fx fx; setup(fx, 1, hp, lp, 0.f, 1.f, 1.f, 2);
    std::vector<float> in(N), out(N);
    for (int n = 0; n < N; ++n) in[n] = 0.5f * sinf(6.2831853f * f * n / sr);
    fx.process(in.data(), out.data(), N);
    return rms(out, 4000) / rms(in, 4000);
  };
  double hp_lo = gain_at(1000, 20000, 100), hp_hi = gain_at(1000, 20000, 8000);
  double lp_lo = gain_at(20, 1000, 100),   lp_hi = gain_at(20, 1000, 8000);
  double bp_lo = gain_at(500, 2000, 50),   bp_mid = gain_at(500, 2000, 1000), bp_hi = gain_at(500, 2000, 10000);
  double open_mid = gain_at(20, 20000, 1000);
  printf("HPF 1 kHz:       100 Hz %.3f   8 kHz %.3f\n", hp_lo, hp_hi);
  printf("LPF 1 kHz:       100 Hz %.3f   8 kHz %.3f\n", lp_lo, lp_hi);
  printf("HPF 500/LPF 2k:  50 Hz %.3f   1 kHz %.3f   10 kHz %.3f\n", bp_lo, bp_mid, bp_hi);
  printf("fully open (HPF 20 Hz, LPF 20 kHz): 1 kHz gain %.3f\n", open_mid);
  // Even fully open the 2-pole LPF rolls off the very top a little, so judge the HPF relative to fully open.
  const double open_hi = gain_at(20, 20000, 8000);
  printf("fully open:      8 kHz %.3f  (HPF 1 kHz passes %.0f%% of that)\n", open_hi, 100.0 * hp_hi / open_hi);
  if (!(hp_lo < 0.2 && hp_hi / open_hi > 0.85)) { puts("HPF response FAIL"); ++fails; }
  if (!(lp_lo > 0.9 && lp_hi < 0.1))  { puts("LPF response FAIL"); ++fails; }
  if (!(bp_mid > 0.5 && bp_lo < 0.2 && bp_hi < 0.2)) { puts("band response FAIL"); ++fails; }
  if (!(open_mid > 0.9)) { puts("open response FAIL"); ++fails; }

  printf(fails ? "\n%d FAILURE(S)\n" : "\nall checks passed\n", fails);
  return fails != 0;
}
