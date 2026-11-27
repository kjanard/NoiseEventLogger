#pragma once

#include <Arduino.h>
#include "config.h"

// ============================================================================
// Phase 2: Real-Time esp-dsp 24-Band (1/3-Octave) Spectrum & 1m Leq/L90
// ============================================================================

extern const char * const OCTAVE_BAND_LABELS[OCTAVE_BAND_COUNT];

typedef struct {
  bool     passed;
  float    tone1k_td_dbfs;
  float    tone1k_spec_dbfs;
  float    tone1k_band12_dbfs;  // Band 12 = 1 kHz (1/3-octave)
  float    tone4k_td_dbfs;
  float    tone4k_spec_dbfs;
  float    tone4k_band18_dbfs;  // Band 18 = 4 kHz (1/3-octave)
  uint32_t single_fft_us;
} OctaveSelfTestResult_t;

typedef struct {
  uint32_t uptime_s;
  uint32_t fft_frames;
  float    bb_leq_1s;
  float    bb_peak_1s;
  uint32_t clips_1s;
  float    oct_1s[OCTAVE_BAND_COUNT];  // 63 Hz .. 12.5 kHz (24 x 1/3-octave)
  uint32_t dsp_avg_us;
  uint32_t dsp_max_us;
  uint32_t free_internal_sram;
} SecondReport_t;

typedef struct {
  int64_t  epoch_s;   // Unix timestamp stamped when the 60s window closes
  uint32_t uptime_s;
  float    bb_leq;
  float    bb_l90;
  float    bb_peak;
  uint32_t clips;
  float    oct_leq[OCTAVE_BAND_COUNT];
  float    oct_l90[OCTAVE_BAND_COUNT];
} MinuteReport_t;

typedef struct {
  const char *ts_iso;   // nullptr or "" -> emits "ts":null
  uint32_t    vbat_mv;
  int         rssi;
  bool        sd_ok;
} MinuteSysInfo_t;

/**
 * Initialize esp-dsp 4096-pt FFT tables, Hann window, and 24-band bin map.
 */
bool DspOctave_Init();

/**
 * Run synthetic 1 kHz and 4 kHz sine wave verification against Parseval's theorem
 * and 1/3-octave band bin mapping (must fall within +/- 0.5 dB of -3.01 dBFS).
 */
bool DspOctave_RunSelfTest(OctaveSelfTestResult_t *out_result);

/**
 * Reset sliding window and 1-second / 1-minute accumulators.
 */
void DspOctave_ResetState();

/**
 * Push a hop of mono 16-bit PCM samples into the sliding 4096-sample FFT window,
 * accumulate 1s linear power, and emit 1s / 1m reports at exact sample boundaries.
 */
bool DspOctave_PushHopSamples(const int16_t *hop_pcm,
                              size_t hop_len,
                              int32_t hop_peak_pcm,
                              uint32_t hop_clips,
                              SecondReport_t *out_sec,
                              bool *out_sec_ready,
                              MinuteReport_t *out_min,
                              bool *out_min_ready);

/**
 * Format a 1-minute summary record into compact single-line JSON (SPEC.md 6.1).
 * When `sys_info != nullptr`, includes ISO-8601 `ts` and `"sys":{vbat_mv,rssi,sd_ok}`.
 * Returns characters written (excluding null terminator), or < 0 on truncation/error.
 */
int DspOctave_FormatMinuteJson(const MinuteReport_t &rec,
                               const MinuteSysInfo_t *sys_info,
                               char *out_buf,
                               size_t out_buf_size);
