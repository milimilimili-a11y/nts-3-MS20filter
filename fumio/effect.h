#pragma once
/*
 *  File: effect.h
 *
 *  fumio - MS-20 style filter for the NTS-3 kaoss pad kit: high-pass into low-pass.
 *  The RESO control drives the low-pass only; the high-pass has a fixed resonance.
 *
 *  Port of the fumio~ Pure Data external by Johannes Regnier, itself based on
 *  Miller Puckette's bob~ (Runge-Kutta 4th order, BSD license, copyright 2015 Miller Puckette).
 *  The DSP lives in fumio_core.h (no SDK dependencies, unit tested on a desktop);
 *  this file only adapts it to the logue SDK Processor interface.
 */
#include "processor.h"
#include "unit_genericfx.h"
#include "fumio_core.h"

class Effect : public Processor
{
public:
  // No delay lines needed, so ask for no SDRAM (the template default is a large buffer).
  uint32_t getBufferSize() const override final { return 0U; }

  // Parameter indices. Must match the order of the descriptors in header.c.
  enum
  {
    HPF = 0U,
    LPF,
    RESO,
    MIX,
    DRIVE,
    QUALITY,
    NUM_PARAMS
  };

  enum
  {
    QUALITY_1X = 0,
    QUALITY_2X,
    QUALITY_4X,
    NUM_QUALITIES
  };

  static constexpr float kMinHz = 20.f;
  static constexpr float kMaxHz = 20000.f;  // the core limits this to 12 kHz at QUALITY 1X
  // DRIVE: saturation amount a of the input stage tanh(a x) / a, a = kMaxDrive * (DRIVE / 1023)^2.
  // 0 = clean. The squared taper gives fine control at low settings. For a 0.3-peak signal,
  // a = 3 gives about 6 % harmonic distortion and a = 10 about 25 %.
  static constexpr float kMaxDrive = 10.f;

  // RESO scale. The parameter is shown as 0.000 .. 6.000 (value 0 .. 6000, three decimals).
  // The raw filter resonance r is mapped so that the low-pass starts to self-oscillate at
  // kResoSelfOscAt on that scale (r = fumio::kSelfOscRes there, measured, see test_effect.cc).
  static constexpr float kResoDisplayScale = 1000.f;  // parameter units per displayed unit
  static constexpr float kResoSelfOscAt = 5.f;        // displayed value where self-oscillation starts
  static constexpr float kResoToRaw = fumio::kSelfOscRes / kResoSelfOscAt;  // raw r per displayed unit

  // Fixed high-pass resonance, on the same displayed 0..6 scale as RESO (2 = 40% of the way
  // to self-oscillation). Change this number to change the high-pass resonance.
  static constexpr float kHpfResoDisplay = 2.f;

  inline void setParameter(uint8_t index, int32_t value) override final
  {
    switch (index)
    {
    case HPF:
      fx_.hp_hz_target = cutoffHz(value);
      break;

    case LPF:
      fx_.lp_hz_target = cutoffHz(value);
      break;

    case RESO:
      // 0.000 .. 6.000 displayed -> raw resonance of the low-pass only
      fx_.lp_res_target = (value / kResoDisplayScale) * kResoToRaw;
      break;

    case MIX:
      // 0.0 .. 100.0 % in tenths of a percent
      fx_.mix_target = value / 1000.f;
      break;

    case DRIVE:
    {
      const float t = param_10bit_to_f32(value);
      fx_.drive_target = kMaxDrive * t * t;
      break;
    }

    case QUALITY:
      fx_.set_oversample(value == QUALITY_1X ? 1 : (value == QUALITY_2X ? 2 : 4));
      break;

    default:
      break;
    }
  }

  inline const char *getParameterStrValue(uint8_t index, int32_t value) const override final
  {
    static const char *quality_strings[NUM_QUALITIES] = {"1X", "2X", "4X"};

    switch (index)
    {
    case QUALITY:
      if (value >= 0 && value < NUM_QUALITIES)
        return quality_strings[value];
      break;
    default:
      break;
    }
    return nullptr;
  }

  // life-cycle
  void init(float *allocated_buffer) override final
  {
    (void)allocated_buffer;  // not used: no SDRAM requested
    fx_.init(getSampleRate(), 2);
    fx_.hp_res = kHpfResoDisplay * kResoToRaw;  // fixed, after init() has reset everything
  }

  void teardown() override final {}

  // The runtime calls this when the unit is (re)selected. Parameters are kept, filter state is cleared.
  void reset() override final { fx_.reset(); }

  // audio: interleaved stereo in / out
  void process(const float *__restrict in, float *__restrict out, uint32_t frames) override final
  {
    fx_.process(in, out, frames);
  }

private:
  // 0..1023 -> 20 Hz .. 20 kHz, exponential (equal pad travel per octave)
  static inline float cutoffHz(int32_t value)
  {
    return kMinHz * powf(kMaxHz / kMinHz, param_10bit_to_f32(value));
  }

  fumio::Fx fx_;
};
