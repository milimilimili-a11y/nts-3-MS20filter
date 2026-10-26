// End-to-end test of the unit as the device uses it: the real Effect class (effect.h) driven with
// the integer parameter values the hardware sends, and the real parameter table from header.c.
//
// Host build (needs the SDK headers + CMSIS; the -fpermissive/-w flags only quiet CMSIS's own
// 32-bit pointer casts on a 64-bit PC):
//   gcc -c -std=c11 -fpermissive -w -I. -I../common -I../../ext/CMSIS/CMSIS/Include -DSTM32H725xE -DCORTEX_USE_FPU=TRUE -DARM_MATH_CM7 -D__FPU_PRESENT header.c -o header.o
//   g++ -O2 -std=c++11 -fpermissive -w -I. -I../common -I../../ext/CMSIS/CMSIS/Include -DSTM32H725xE -DCORTEX_USE_FPU=TRUE -DARM_MATH_CM7 -D__FPU_PRESENT test_effect.cc header.o -o test_effect && ./test_effect
#include <cmath>
#include <cstdio>
#include <vector>
#include "effect.h"

static int fails = 0;
static void check(bool ok, const char *what) { if (!ok) { printf("  FAIL: %s\n", what); ++fails; } }

// Value that puts a cutoff at (about) the given frequency, 0..1023 exponential 20 Hz..20 kHz
static int cutoff_value(double hz) { return (int)lround(1023.0 * log(hz / 20.0) / log(1000.0)); }

struct Rig {
  Effect fx;
  Rig(int hpf, int lpf, int reso, int quality, int drive = 0, int mix = 1000) {
    fx.init(nullptr);
    fx.setParameter(Effect::HPF, hpf);
    fx.setParameter(Effect::LPF, lpf);
    fx.setParameter(Effect::RESO, reso);
    fx.setParameter(Effect::MIX, mix);
    fx.setParameter(Effect::DRIVE, drive);
    fx.setParameter(Effect::QUALITY, quality);
    run_silence(4800);  // let the 5 ms parameter smoothing settle
  }
  // returns peak |output| over the last `tail` frames
  float run(const std::vector<float> &in, uint32_t tail) {
    const uint32_t B = 256;
    std::vector<float> out(2 * B);
    float peak = 0;
    uint32_t frames = in.size() / 2;
    for (uint32_t pos = 0; pos < frames; pos += B) {
      uint32_t n = frames - pos < B ? frames - pos : B;
      fx.process(&in[2 * pos], out.data(), n);
      if (pos + n + tail > frames)
        for (uint32_t i = 0; i < 2 * n; ++i) peak = fmaxf(peak, fabsf(out[i]));
    }
    return peak;
  }
  void run_silence(uint32_t frames) { run(std::vector<float>(2 * frames, 0.f), 0); }
};

// Kick the filter once, then 8 s of silence. True if it is still ringing at the end.
static bool self_oscillates(int lpf, int reso, int quality, float *level = nullptr) {
  Rig r(0, lpf, reso, quality);
  const uint32_t frames = 8 * 48000;
  std::vector<float> in(2 * frames, 0.f);
  in[0] = in[1] = 0.5f;
  float tail = r.run(in, 12000);
  if (level) *level = tail;
  return tail > 1e-3f;
}

int main() {
  // --- parameter table ---
  const unit_param_t &reso = unit_header.common.params[Effect::RESO];
  printf("RESO descriptor: min %d max %d init %d, %d decimals (type none)\n", reso.min, reso.max, reso.init, reso.frac);
  check(reso.min == 0 && reso.max == 6000, "RESO range is 0..6000 (0.000..6.000)");
  check(reso.frac == 3 && reso.frac_mode == 1, "RESO shows 3 decimals");
  check(unit_header.default_mappings[Effect::RESO].assign == k_genericfx_param_assign_depth, "RESO is on the FX depth strip");
  check(unit_header.default_mappings[Effect::HPF].assign == k_genericfx_param_assign_x, "HPF is on pad X");
  check(unit_header.default_mappings[Effect::LPF].assign == k_genericfx_param_assign_y, "LPF is on pad Y");
  check(unit_header.default_mappings[Effect::RESO].max == 6000, "depth mapping spans the whole 0..6000");
  check(unit_header.common.num_params == Effect::NUM_PARAMS, "param count matches enum");

  // --- self-oscillation starts at 5.000 on the RESO scale ---
  puts("\nSelf-oscillation onset (4.990 must decay, 5.010 must keep ringing):");
  const int lpf_values[] = {cutoff_value(150), cutoff_value(630), cutoff_value(2300), cutoff_value(8000), cutoff_value(12000)};
  for (int quality = 0; quality < 3; ++quality) {
    for (int lpf : lpf_values) {
      const double hz = 20.0 * pow(1000.0, lpf / 1023.0);
      float lo_level, hi_level;
      bool below = self_oscillates(lpf, 4990, quality, &lo_level);
      bool above = self_oscillates(lpf, 5010, quality, &hi_level);
      // At 1X quality the integrator is least accurate above ~8 kHz; onset can sit a little higher there.
      const bool known_shift = (quality == 0 && hz > 7000);
      printf("  quality %dX  LPF %6.0f Hz:  4.990 -> %s,  5.010 -> %s%s\n", 1 << quality, hz,
             below ? "RINGS" : "decays", above ? "rings" : "DECAYS", known_shift ? "   (1X, high cutoff: shift allowed)" : "");
      if (!known_shift) { check(!below, "decays just below 5.000"); check(above, "self-oscillates just above 5.000"); }
      else { check(!below, "decays just below 5.000"); }
    }
  }
  { // how far past 5.000 does it take to ring at 1X / high cutoff?
    int lpf = cutoff_value(12000);
    int onset = -1;
    for (int v = 5000; v <= 6000; v += 10) if (self_oscillates(lpf, v, 0)) { onset = v; break; }
    printf("  (quality 1X, LPF 12 kHz: onset at %.3f)\n", onset / 1000.0);
    check(onset > 0 && onset <= 5250, "1X/12 kHz onset within 0.25 of 5.000");
  }

  // --- how loud is the oscillation across the top of the scale? ---
  puts("\nOscillation level (peak) after a kick, 4X quality:");
  for (int reso_v : {5010, 5500, 6000}) {
    printf("  RESO %.3f:", reso_v / 1000.0);
    for (double hz : {150.0, 1000.0, 8000.0}) {
      float lvl = 0; self_oscillates(cutoff_value(hz), reso_v, 2, &lvl);
      printf("  %5.0f Hz -> %.2f", hz, lvl);
    }
    puts("");
  }

  // --- HPF resonance is fixed (2 of 6) and independent of RESO ---
  puts("\nHPF resonance (HPF at 1 kHz, LPF fully open, 0.01 sine at 1 kHz):");
  {
    const int hpf = cutoff_value(1000), lpf = 1023;
    std::vector<float> in(2 * 48000);
    for (int n = 0; n < 48000; ++n) in[2 * n] = in[2 * n + 1] = 0.01f * sinf(6.2831853f * 1000.f * n / 48000.f);
    Rig a(hpf, lpf, 0, 2), b(hpf, lpf, 4900, 2);
    float ga = a.run(in, 24000) / 0.01f, gb = b.run(in, 24000) / 0.01f;
    // Fixed HPF r = 2/5 * 2 = 0.8 -> ideal gain at cutoff 1/(2-r) = 0.833. (The filter holds each input
    // sample for one sample period, which lowers it by under 1 dB at 1 kHz; plain r=0 would give 0.5.)
    printf("  gain at cutoff: RESO 0.000 -> %.3f   RESO 4.900 -> %.3f   (ideal continuous value 0.833; no resonance would be 0.5)\n", ga, gb);
    check(fabsf(ga - gb) < 0.01f, "HPF peak does not change with RESO");
    check(ga > 0.72f && ga < 0.86f, "HPF peak matches the fixed resonance (about 0.8)");
  }

  printf(fails ? "\n%d FAILURE(S)\n" : "\nall checks passed\n", fails);
  return fails != 0;
}
