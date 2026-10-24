#pragma once
/*
 *  File: effect.h
 *
 *  fumio - MS-20 style filter for the NTS-3 kaoss pad kit: high-pass into low-pass,
 *  one shared resonance.
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
  static constexpr float kMaxReso = 6.f;    // the original external is unbounded; ~3+ self-oscillates
  static constexpr float kMaxDrive = 4.f;   // input gain at DRIVE = 1023

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
      // 0.0 .. 100.0 % in tenths of a percent -> 0 .. kMaxReso
      fx_.res_target = (value / 1000.f) * kMaxReso;
      break;

    case MIX:
      // 0.0 .. 100.0 % in tenths of a percent
      fx_.mix_target = value / 1000.f;
      break;

    case DRIVE:
      fx_.gain_target = 1.f + (kMaxDrive - 1.f) * param_10bit_to_f32(value);
      break;

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
