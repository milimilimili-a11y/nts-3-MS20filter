# fumio - MS-20 style filter for the Korg NTS-3 kaoss pad kit

Port of the `fumio~` Pure Data external (Johannes Regnier, based on Miller Puckette's `bob~`,
BSD license). Same maths: 2-pole filters with `tanh` feedback, integrated with 4th-order
Runge-Kutta. Arranged like the MS-20: a high-pass feeding a low-pass, one shared resonance.

## Controls (defaults)
| Param   | Default control | Notes |
|---------|-----------------|-------|
| HPF     | Pad X           | High-pass cutoff, 20 Hz - 20 kHz, exponential. Fully left = open |
| LPF     | Pad Y           | Low-pass cutoff, 20 Hz - 20 kHz, exponential. Fully up = open |
| RESO    | FX DEPTH strip  | 0 - 100 %, shared by both filters. Self-oscillates from about 50 % |
| MIX     | fixed           | 0 - 100 % dry/wet (default 100 %) |
| DRIVE   | fixed           | 1x - 4x input gain into the resonance saturation, level-compensated |
| QUALITY | fixed           | 1X / 2X / 4X RK4 sub-steps per sample. Higher = more accurate, more CPU |

Any parameter can be re-assigned on the device. At QUALITY 1X the top cutoff is limited to 12 kHz.
Both filters run, so CPU use is about double a single filter.

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
- `test_core.cc`  desktop test: `g++ -O2 -std=c++14 test_core.cc -o test_core && ./test_core`
