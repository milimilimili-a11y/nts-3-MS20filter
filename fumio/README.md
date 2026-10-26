# fumio (ms20) - MS-20 style filter for the Korg NTS-3 kaoss pad kit

Port of the `fumio~` Pure Data external (Johannes Regnier, based on Miller Puckette's `bob~`,
BSD license). Same maths: 2-pole filters with `tanh` feedback, integrated with 4th-order
Runge-Kutta. Arranged like the MS-20: a high-pass feeding a low-pass.

## Controls (defaults)
| Param   | Default control | Notes |
|---------|-----------------|-------|
| HPF     | Pad X           | High-pass cutoff, 20 Hz - 20 kHz, exponential. Fully left = open |
| LPF     | Pad Y           | Low-pass cutoff, 20 Hz - 20 kHz, exponential. Fully up = open |
| RESO    | FX DEPTH strip  | **Low-pass** resonance, shown as 0.000 - 6.000. **Self-oscillation starts at 5.000** |
| MIX     | fixed           | 0 - 100 % dry/wet (default 100 %) |
| DRIVE   | fixed           | Input saturation, 0 (clean) - 1023 (heavy overdrive). Quiet signals are left alone; loud ones are squashed and distorted |
| QUALITY | fixed           | 1X / 2X / 4X RK4 sub-steps per sample. Higher = more accurate, more CPU |

Any parameter can be re-assigned on the device.

## Resonance
- RESO controls the low-pass only. The high-pass has a **fixed** resonance of 2 on the same 0-6 scale
  (`kHpfResoDisplay` in `effect.h`).
- 5.000 is exactly where the low-pass starts to self-oscillate; 5 - 6 is the self-oscillating range
  (tone level grows from about -28 dBFS at 5.01 to about -8 dBFS at 6.0).
- RESO has 6001 steps (0.001 each) and 5/6 of the strip's travel is the non-oscillating range.
- The high-pass is a resonant 2-pole (12 dB/oct). The original Pd external's high-pass mode ignored its
  resonance setting and was only 1-pole, so this section is new: it is the exact mirror of the low-pass,
  with the same resonance behaviour.
- At QUALITY 1X, with the low-pass cutoff above about 8 kHz, self-oscillation starts a little above 5.000
  (about 5.24 at 12 kHz). 2X and 4X are accurate across the whole range.
- Like the original external, the filter holds each input sample for one sample period. The high-pass
  response near and above a high cutoff therefore sits roughly 1 dB below the ideal curve.

## Changing DRIVE
DRIVE is 0 - 1023 and sits in front of the filters as a soft saturator, `tanh(a x) / a`, with
`a = 10 x (DRIVE / 1023)^2`. At 0 it is exactly clean; it has no effect on quiet signals; on louder
material it adds harmonic distortion and reduces the peak level. Measured on a 0.3-peak signal:
DRIVE 512 (a = 2.5) is subtle warmth (about 4 % distortion), DRIVE 1023 (a = 10) is heavy overdrive
(about 25 % distortion, peaks about 8 dB lower). The dry part of MIX stays clean.

To change it:
- on the unit / in Korg's editor: assign DRIVE to X, Y or the depth strip, or set its fixed value;
- as the default: in `header.c`, edit the DRIVE row of `default_mappings` (the last number is the
  fixed value) and the DRIVE descriptor's default in `params` (keep both the same). Raise `kMaxDrive`
  in `effect.h` for a stronger top end.

## Build
1. Get the SDK: `git clone https://github.com/korginc/logue-sdk && cd logue-sdk && git submodule update --init`
2. Copy this folder to `platform/nts-3_kaoss/fumio/`
3. Open the SDK's Docker build environment (see the SDK README), then: `build nts-3_kaoss/fumio`
4. The unit file is written to `platform/nts-3_kaoss/fumio/fumio.nts3unit`
5. Load it on the NTS-3 with Korg's NTS-3 editor / Kontrol Editor app.

Before building, set your own `dev_id` / `unit_id` in `header.c` (the shipped value is a placeholder).

## Files
- `fumio_core.h`  DSP, no SDK dependencies
- `effect.h`      adapts the core to the SDK `Processor` interface (params, strings, process)
- `header.c`      unit metadata, 6 parameters, default pad mappings
- `unit.cc`, `Makefile`, `wasm.cc`  unchanged from Korg's `dummy-genericfx` template
- `config.mk`     project name only
- `test_core.cc`  desktop test of the DSP: `g++ -O2 -std=c++14 test_core.cc -o test_core && ./test_core`
- `test_effect.cc`  desktop test of the whole unit (real `effect.h` + `header.c`): build command in its header comment (takes about 3 minutes)
