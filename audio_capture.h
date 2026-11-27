#pragma once

#include "config.h"
#include <Arduino.h>
#include <Wire.h>

// ============================================================================
// Phase 1: ES7210 + I2S Audio Capture Interface
// ============================================================================

typedef struct {
  uint32_t sample_count; // Mono samples processed in 1s window
                         // (AUDIO_SAMPLE_RATE_HZ)
  float dc_mean_pcm;     // 1s mean DC offset in 16-bit PCM counts
  float dc_mean_norm;    // 1s mean DC offset normalized [-1.0, 1.0]
  float rms_pcm;         // DC-removed 1s broadband RMS in PCM counts
  float rms_norm;        // DC-removed 1s broadband RMS normalized [0.0, 1.0]
  float
      rms_dbfs; // DC-removed 1s broadband RMS in dBFS (+ CALIBRATION_OFFSET_DB)
  int32_t peak_pcm;    // Max absolute raw PCM sample [0, 32768]
  float peak_norm;     // Peak normalized [0.0, 1.0]
  float peak_dbfs;     // Peak in dBFS (+ CALIBRATION_OFFSET_DB)
  uint32_t clip_count; // Samples with |s| >= AUDIO_CLIP_THRESHOLD_PCM (32600)
  bool clipped;        // True if clip_count > 0
} AudioMetrics1s_t;

/**
 * Initialize I2S Master RX (ESP-IDF v5 driver/i2s_std.h) and configure ES7210
 * over the shared I2C bus with a fixed manual gain and AGC/automute disabled.
 */
bool AudioCapture_Init(TwoWire &wire, float fixed_gain_db = MIC_GAIN_DB);

/**
 * Capture 1 second of audio from DMA in chunks, compute DC mean, DC-removed
 * broadband RMS (dBFS), Peak (dBFS), and clipping count in RAM, and immediately
 * wipe every raw PCM buffer before returning.
 */
bool AudioCapture_CaptureOneSecond(AudioMetrics1s_t *out_metrics);

/**
 * Read exact `sample_count` mono samples from the primary ES7210 mic channel
 * into `mono_out`, updating raw peak PCM and clip count (`|s| >= AUDIO_CLIP_THRESHOLD_PCM`),
 * and immediately wiping the temporary stereo DMA chunk in RAM.
 */
bool AudioCapture_ReadMonoSamples(int16_t *mono_out,
                                  size_t sample_count,
                                  int32_t *inout_peak_pcm,
                                  uint32_t *inout_clip_count);

/**
 * Read back an ES7210 register over I2C for diagnostic verification.
 */
bool AudioCapture_ReadEs7210Reg(uint8_t reg_addr, uint8_t *out_val);
