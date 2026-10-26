/*
    BSD 3-Clause License

    Copyright (c) 2023, KORG INC.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
      list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//*/

/*
 *  File: header.c
 *
 *  fumio - NTS-3 kaoss pad kit generic effect unit header definition
 *
 */

#include "unit_genericfx.h"   // Note: Include base definitions for genericfx units

// ---- Unit header definition  --------------------------------------------------------------------

const __unit_header genericfx_unit_header_t unit_header = {
  .common = {
    .header_size = sizeof(genericfx_unit_header_t),           // Size of this header. Leave as is.
    .target = UNIT_TARGET_PLATFORM | k_unit_module_genericfx, // Target platform and module pair for this unit
    .api = UNIT_API_VERSION,                                  // API version for which unit was built. See runtime.h
    .dev_id = 0x6D696C69,                                     // 'mili' as a 32-bit number (must be a numeric constant, not a bare word)
    .unit_id = 0x1U,                                          // ID for this unit, scoped within your dev_id
    .version = 0x00010200U,                                   // major<<16 | minor<<8 | patch
    .name = "ms20",                                           // Name displayed on device (max 19 chars)
    .num_params = 6,                                          // Number of valid parameter descriptors. (max. 8)

    .params = {
      // Format: min, max, center (unused), default, type, frac. bits, frac. mode, <reserved>, name

      // HPF: high-pass cutoff, 20 Hz .. 20 kHz, exponential (curve applied in effect.h). Default: fully open (20 Hz)
      {0, 1023, 0, 0, k_unit_param_type_none, 0, 0, 0, {"HPF"}},

      // LPF: low-pass cutoff, 20 Hz .. 20 kHz, exponential (curve applied in effect.h). Default: fully open (20 kHz)
      {0, 1023, 0, 1023, k_unit_param_type_none, 0, 0, 0, {"LPF"}},

      // RESO: low-pass resonance, shown as 0.000 .. 6.000 (value 0 .. 6000, 3 decimals). The low-pass
      // starts to self-oscillate at 5.000. The high-pass has its own fixed resonance (see effect.h).
      {0, 6000, 0, 1000, k_unit_param_type_none, 3, 1, 0, {"RESO"}},

      // MIX: dry/wet in tenths of a percent, displayed as 0.0 .. 100.0 %
      {0, 1000, 0, 1000, k_unit_param_type_percent, 1, 1, 0, {"MIX"}},

      // DRIVE: soft saturation on the input, 0 = clean .. 1023 = heavy overdrive (quiet signals are left alone)
      {0, 1023, 0, 0, k_unit_param_type_none, 0, 0, 0, {"DRIVE"}},

      // QUALITY: RK4 sub-steps per sample, 0 = 1x, 1 = 2x, 2 = 4x (higher = more CPU)
      {0, 2, 0, 1, k_unit_param_type_strings, 0, 0, 0, {"QUALITY"}},

      {0, 0, 0, 0, k_unit_param_type_none, 0, 0, 0, {""}},
      {0, 0, 0, 0, k_unit_param_type_none, 0, 0, 0, {""}}},
  },
  .default_mappings = {
    // Format: assign, curve, curve polarity, min, max, default value

    // HPF cutoff on the pad's X axis, full range (already exponential in effect.h, so a linear curve here)
    {k_genericfx_param_assign_x, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 1023, 0},

    // LPF cutoff on the pad's Y axis, full range
    {k_genericfx_param_assign_y, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 1023, 1023},

    // RESO on the FX DEPTH strip, 0.000 .. 6.000
    {k_genericfx_param_assign_depth, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 6000, 1000},

    // MIX, DRIVE and QUALITY are not mapped to a control by default (fixed values)
    {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 1000, 1000},
    {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 1023, 0},
    {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 2, 1},

    {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 0, 0},
    {k_genericfx_param_assign_none, k_genericfx_curve_linear, k_genericfx_curve_unipolar, 0, 0, 0}
  }
};
