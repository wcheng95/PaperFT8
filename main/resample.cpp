#include "resample.h"
#include <string.h>

void resample_init(resample_state_t* state) {
    // No-op - no filter state to initialize
    (void)state;
}

int convert_24bit_stereo_to_mono_float(
    const uint8_t* in,
    float* out,
    int num_stereo_samples
) {
    const float scale = 1.0f / 8388608.0f;  // 2^23 for 24-bit normalization

    for (int i = 0; i < num_stereo_samples; i++) {
        // Read left channel (24-bit LE)
        int offset = i * 6;  // 6 bytes per stereo sample (3+3)
        int32_t left = in[offset] | (in[offset + 1] << 8) | (in[offset + 2] << 16);
        // Sign extend from 24-bit to 32-bit
        if (left & 0x800000) {
            left |= 0xFF000000;
        }

        // Read right channel (24-bit LE)
        int32_t right = in[offset + 3] | (in[offset + 4] << 8) | (in[offset + 5] << 16);
        // Sign extend from 24-bit to 32-bit
        if (right & 0x800000) {
            right |= 0xFF000000;
        }

        // Downmix to mono: (L + R) / 2
        float mono = ((float)left + (float)right) * 0.5f * scale;
        out[i] = mono;
    }

    return num_stereo_samples;
}

int resample_48k_to_12k(
    resample_state_t* state,
    const float* in,
    float* out,
    int in_samples
) {
    (void)state;  // Unused - no filter state needed

    int out_samples = in_samples / RESAMPLE_FACTOR;

    // Simple decimation: take every RESAMPLE_FACTOR-th sample
    // No anti-aliasing filter needed - input is already bandwidth-limited
    for (int i = 0; i < out_samples; i++) {
        out[i] = in[i * RESAMPLE_FACTOR];
    }

    return out_samples;
}

int uac_to_ft8_samples(
    resample_state_t* state,
    const uint8_t* in,
    float* out,
    int num_stereo_samples
) {
    // Temporary buffer for 48kHz mono samples
    // Max input: typically 288 bytes = 48 stereo samples per ms
    // For larger buffers, process in chunks
    static float temp_mono[4096];

    int total_out = 0;
    int remaining = num_stereo_samples;
    const uint8_t* in_ptr = in;
    float* out_ptr = out;

    while (remaining > 0) {
        int chunk = (remaining > 4096) ? 4096 : remaining;

        // Step 1: Convert 24-bit stereo to mono float
        convert_24bit_stereo_to_mono_float(in_ptr, temp_mono, chunk);

        // Step 2: Decimate 48kHz to 12kHz (no filtering needed)
        int out_count = resample_48k_to_12k(state, temp_mono, out_ptr, chunk);

        in_ptr += chunk * 6;  // 6 bytes per stereo sample
        out_ptr += out_count;
        total_out += out_count;
        remaining -= chunk;
    }

    return total_out;
}
