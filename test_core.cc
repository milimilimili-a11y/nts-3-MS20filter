// Desktop test of the fumio core (HPF -> LPF).
//   1. float / fast-tanh port vs a double-precision transcription (libm tanh) of the same equations
//   2. both sections vs the analytic small-signal response  LP = 1/(s^2+(2-r)s+1),  HP = s^2/(...)
//   3. stability at extreme settings
// Build: g++ -O2 -std=c++14 test_core.cc -o test_core && ./test_core
#include <cmath>
#include <cstdio>
#include <complex>
#include <vector>
#include "fumio_core.h"

static const double kPi = 3.14159265358979323846;

// Double-precision references. The low-pass is the original fumio~ code; the high-pass is the
// dual described in fumio_core.h.
struct RefLp {
  double s[2] = {0, 0}, in = 0, cutoff = 1000, res = 0;
  void d(double *ds, const double *st) {
    double k = 2 * 3.14159 * cutoff, t = tanh(res * st[1]);
    ds[0] = k * (in - st[0] - t);
    ds[1] = k * (st[0] - st[1] + t);
  }
  double tick(double x, double sr, int os) {
    in = x; double h = 1.0 / (os * sr);
    for (int j = 0; j < os; ++j) {
      double d1[2], d2[2], d3[2], d4[2], t[2];
      d(d1, s);
      for (int i = 0; i < 2; ++i) t[i] = s[i] + 0.5 * h * d1[i];
      d(d2, t);
      for (int i = 0; i < 2; ++i) t[i] = s[i] + 0.5 * h * d2[i];
      d(d3, t);
      for (int i = 0; i < 2; ++i) t[i] = s[i] + h * d3[i];
      d(d4, t);
      for (int i = 0; i < 2; ++i) s[i] += (h / 6.) * (d1[i] + 2 * d2[i] + 2 * d3[i] + d4[i]);
    }
    return s[1];
  }
};
struct RefHp {
  double s[2] = {0, 0}, in = 0, cutoff = 1000, res = 0;
  void d(double *ds, const double *st) {
    double k = 2 * 3.14159 * cutoff, y = in - st[0] - st[1];
    ds[0] = k * (in - st[0] - tanh(res * y));
    ds[1] = k * y;
  }
  double tick(double x, double sr, int os) {
    in = x; double h = 1.0 / (os * sr);
    for (int j = 0; j < os; ++j) {
      double d1[2], d2[2], d3[2], d4[2], t[2];
      d(d1, s);
      for (int i = 0; i < 2; ++i) t[i] = s[i] + 0.5 * h * d1[i];
      d(d2, t);
      for (int i = 0; i < 2; ++i) t[i] = s[i] + 0.5 * h * d2[i];
      d(d3, t);
      for (int i = 0; i < 2; ++i) t[i] = s[i] + h * d3[i];
      d(d4, t);
      for (int i = 0; i < 2; ++i) s[i] += (h / 6.) * (d1[i] + 2 * d2[i] + 2 * d3[i] + d4[i]);
    }
    return in - s[0] - s[1];
  }
};

static double rms(const std::vector<float> &v, size_t from) {
  double a = 0;
  for (size_t i = from; i < v.size(); ++i) a += (double)v[i] * v[i];
  return sqrt(a / (v.size() - from));
}

static void setup(fumio::Fx &fx, int ch, float hp, float lp, float hpres, float lpres, float mix, float drive, int os) {
  fx.init(48000.f, ch);  // init first (resets to defaults), then set params, as on the device
  fx.hp_hz_target = hp; fx.lp_hz_target = lp; fx.hp_res = hpres; fx.lp_res_target = lpres;
  fx.mix_target = mix; fx.drive_target = drive;
  fx.set_oversample(os);
  fx.snap();
}

int main() {
  const float sr = 48000.f;
  const int N = 48000;
  int fails = 0;

  // 1) Whole chain vs double reference, mono, mix = 1. Includes drive (reference: libm tanh(a x) in front of
  //    the filters, then the (1+a)^(2/3)/a makeup) and checks the state clamp never engages in normal use (reference has none).
  for (float drive : {0.f, 6.f, 100.f}) {
    for (float lpres : {0.f, 1.f, 1.9f}) {
      fumio::Fx fx;
      setup(fx, 1, 300.f, 3000.f, 0.8f, lpres, 1.f, drive, 2);
      RefHp rh; rh.cutoff = 300; rh.res = 0.8;
      RefLp rl; rl.cutoff = 3000; rl.res = lpres;
      std::vector<float> in(N), out(N);
      for (int n = 0; n < N; ++n)
        in[n] = 0.4f * sinf(6.2831853f * 220.f * n / sr) + 0.3f * sinf(6.2831853f * 1500.f * n / sr) + 0.2f * sinf(6.2831853f * 9000.f * n / sr);
      fx.process(in.data(), out.data(), N);
      double err = 0, sig = 0;
      for (int n = 0; n < N; ++n) {
        double xs = drive > 1e-3 ? tanh(drive * (double)in[n]) : (double)in[n];
        double y = rl.tick(rh.tick(xs, sr, 2), sr, 2);
        if (drive > 1e-3) y *= pow(1.0 + drive, (double)fumio::kDriveMakeupExp) / drive;
        if (n >= 2000) { err += (y - out[n]) * (y - out[n]); sig += y * y; }
      }
      double rel = sqrt(err / (sig + 1e-12));
      printf("chain drive a=%.0f, lp res %.1f: relative error vs double reference = %.4f%s\n", drive, lpres, rel, rel > 0.03 ? "  <-- FAIL" : "");
      if (rel > 0.03) ++fails;
    }
  }

  // 2) Analytic frequency response (small signal so tanh stays linear).
  //    Uses a 50 Hz cutoff and test tones <= 400 Hz: the filter holds each input sample for one
  //    sample period (as the original external does), which droops/shifts the response slightly at
  //    frequencies that are a sizeable fraction of the sample rate. Keeping f << fs isolates the
  //    equations themselves from that sampling effect.
  {
    const double fc = 50.0;
    const int M = 96000;  // 2 s: lets even the r = 1.9 ringing settle completely
    double worst = 0;
    for (double r : {0.0, 0.8, 1.9}) {
      for (double f : {5.0, 25.0, 50.0, 100.0, 400.0}) {
        for (int which = 0; which < 2; ++which) {  // 0 = low-pass, 1 = high-pass
          std::vector<float> in(M), out(M);
          const float amp = 0.001f;
          for (int n = 0; n < M; ++n) in[n] = amp * sinf((float)(2 * kPi * f * n / sr));
          const int os = 4;
          const float k = (float)(2 * kPi * fc), h = 1.f / (os * sr);
          fumio::LpChannel lp; fumio::HpChannel hp;
          for (int n = 0; n < M; ++n)
            out[n] = which == 0 ? lp.tick(in[n], k, (float)r, h, os) : hp.tick(in[n], k, (float)r, h, os);
          std::complex<double> s(0, f / fc);
          std::complex<double> den = s * s + (2 - r) * s + 1.0;
          double want = std::abs((which == 0 ? std::complex<double>(1) : s * s) / den);
          double got = rms(out, M / 2) / rms(in, M / 2);
          double rel = fabs(got - want) / want;
          if (rel > worst) worst = rel;
          if (rel > 0.02) { printf("  %s r=%.1f f=%.0f: got %.4f want %.4f  <-- FAIL\n", which ? "HP" : "LP", r, f, got, want); ++fails; }
        }
      }
    }
    printf("analytic response (LP and HP, r = 0, 0.8, 1.9): worst error %.2f%%\n", worst * 100);
  }

  // 2b) DRIVE behaves as designed: clean at 0, clearly distorted on any normal level, bounded output.
  {
    auto thd_and_level = [&](float amp, float a, double &thd, double &level_db) {
      fumio::Fx fx; setup(fx, 1, 20.f, 20000.f, 0.f, 0.f, 1.f, a, 2);
      std::vector<float> in(N), out(N);
      for (int n = 0; n < N; ++n) in[n] = amp * sinf((float)(2 * kPi * 220.0 * n / sr));
      fx.process(in.data(), out.data(), N);
      auto mag = [&](double f) {
        double re = 0, im = 0;
        for (int n = 9600; n < N; ++n) { double w = 2 * kPi * f * n / sr; re += out[n] * cos(w); im += out[n] * sin(w); }
        return 2 * sqrt(re * re + im * im) / (N - 9600);
      };
      double f1 = mag(220.0), h = 0;
      for (int k = 2; k <= 15; ++k) { double m = mag(220.0 * k); h += m * m; }
      thd = 100 * sqrt(h) / f1;
      level_db = 10 * log10(rms(out, 9600) * rms(out, 9600) / (rms(in, 9600) * rms(in, 9600)));
    };
    double thd0, thd512, thd1023, thdq, lv;
    thd_and_level(0.1f, 0.f, thd0, lv);
    thd_and_level(0.1f, 25.f, thd512, lv);       // DRIVE 512  -> a = 25
    thd_and_level(0.1f, 100.f, thd1023, lv);     // DRIVE 1023 -> a = 100
    thd_and_level(0.03f, 100.f, thdq, lv);       // a quiet (-30 dBFS) signal at full drive
    printf("DRIVE distortion on a 0.1-peak sine: off %.1f %%, a=25 %.1f %%, a=100 %.1f %%; quiet 0.03-peak at a=100: %.1f %%\n", thd0, thd512, thd1023, thdq);
    if (!(thd0 < 0.5)) { puts("  FAIL: DRIVE off must be clean"); ++fails; }
    if (!(thd512 > 15.0)) { puts("  FAIL: mid DRIVE is not clearly distorted"); ++fails; }
    if (!(thd1023 > 30.0)) { puts("  FAIL: full DRIVE is not a heavy overdrive"); ++fails; }
    if (!(thdq > 15.0)) { puts("  FAIL: DRIVE does not distort quiet signals"); ++fails; }
    // output stays bounded at full drive even for full-scale noise (filters open, no resonance)
    fumio::Fx fx; setup(fx, 1, 20.f, 20000.f, 0.f, 0.f, 1.f, 100.f, 2);
    std::vector<float> in(N), out(N);
    unsigned sd = 777; for (auto &x : in) { sd = sd * 1664525u + 1013904223u; x = ((sd >> 8) & 0xffff) / 32768.f - 1.f; }
    fx.process(in.data(), out.data(), N);
    float peak = 0; for (float y : out) peak = fmaxf(peak, fabsf(y));
    printf("full-scale noise at full DRIVE: output peak %.2f\n", peak);
    if (!(peak < 0.5f)) { puts("  FAIL: output not bounded"); ++fails; }
    // drive 0 must be bit-exact with the bare filters
    fumio::Fx f0; setup(f0, 1, 20.f, 3000.f, 0.8f, 0.4f, 1.f, 0.f, 2);
    std::vector<float> in2(2000), o0(2000);
    for (int n = 0; n < 2000; ++n) in2[n] = 0.5f * sinf(6.2831853f * 440.f * n / sr);
    f0.process(in2.data(), o0.data(), 2000);
    fumio::LpChannel lp; fumio::HpChannel hp; bool exact = true;
    const float h = 1.f / (2 * sr);
    for (int n = 0; n < 2000; ++n) {
      float w = lp.tick(hp.tick(in2[n], fumio::kTwoPi * 20.f, 0.8f, h, 2), fumio::kTwoPi * 3000.f, 0.4f, h, 2);
      const float expect = in2[n] + 1.f * (w - in2[n]);  // the dry/wet mix step at mix = 1, as in Fx::process
      if (expect != o0[n]) exact = false;
    }
    printf("drive 0 is bit-identical to the bare filters (plus the mix step): %s\n", exact ? "yes" : "NO");
    if (!exact) { puts("  FAIL"); ++fails; }
  }

  // 3) Stability at extremes: noise in, all cutoff pairings, resonances well past self-oscillation, max drive, stereo
  {
    float worst_peak = 0;
    for (float hp : {20.f, 1000.f, 12000.f})
      for (float lp : {20.f, 1000.f, 20000.f})
        for (float hpres : {0.f, 0.8f, 2.4f, 10.f})
          for (float lpres : {0.f, 2.f, 2.4f, 10.f})
            for (int os : {1, 4}) {
              fumio::Fx fx;
              setup(fx, 2, hp, lp, hpres, lpres, 1.f, 100.f, os);
              std::vector<float> in(2 * N), out(2 * N);
              unsigned s = 12345;
              for (auto &x : in) { s = s * 1664525u + 1013904223u; x = ((s >> 8) & 0xffff) / 32768.f - 1.f; }
              fx.process(in.data(), out.data(), N);
              bool bad = false; float peak = 0;
              for (float y : out) { if (!std::isfinite(y)) bad = true; peak = fmaxf(peak, fabsf(y)); }
              if (peak > worst_peak) worst_peak = peak;
              if (bad || peak > 30.f) { printf("UNSTABLE hp %.0f lp %.0f hpres %.1f lpres %.1f os %d peak %.2f\n", hp, lp, hpres, lpres, os, peak); ++fails; }
            }
    printf("stability sweep done (worst peak %.2f with full-scale noise at maximum drive)\n", worst_peak);
  }

  printf(fails ? "\n%d FAILURE(S)\n" : "\nall checks passed\n", fails);
  return fails != 0;
}
