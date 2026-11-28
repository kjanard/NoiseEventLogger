#pragma once

#include <Arduino.h>
#include "config.h"

// ============================================================================
// Phase 4: Real-Time esp-dsp 26-Band (1/3-Octave) + IEC 61672 A-Weighting
// ============================================================================

extern const char * const OCTAVE_BAND_LABELS[OCTAVE_BAND_COUNT];

typedef struct {
  bool     passed;
  float    tone1k_td_dbfs;
  float    tone1k_spec_dbfs;
  float    tone1k_band12_dbfs;   // Band 12 = 1 kHz
  float    tone1k_laeq_dbfs;     // IEC 61672-1 A(1k) = 0.00 dB
  float    tone4k_td_dbfs;
  float    tone4k_spec_dbfs;
  float    tone4k_band18_dbfs;   // Band 18 = 4 kHz
  float    tone4k_laeq_dbfs;     // IEC 61672-1 A(4k) = +0.96 dB
  float    tone16k_td_dbfs;
  float    tone16k_spec_dbfs;
  float    tone16k_band24_dbfs;  // Band 24 = 16 kHz
  float    tone16k_laeq_dbfs;    // IEC 61672-1 A(16k) = -6.71 dB
  uint32_t single_fft_us;
} OctaveSelfTestResult_t;

typedef struct {
  uint32_t uptime_s;
  uint32_t fft_frames;
  float    bb_leq_1s;
  float    bb_peak_1s;
  float    laeq_1s;
  float    laf_max_1s;
  uint32_t clips_1s;
  bool     overload_1s;
  bool     underrange_1s;
  float    oct_1s[OCTAVE_BAND_COUNT];  // 63 Hz .. 20 kHz (26 x 1/3-octave)
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
  float    laeq;
  float    la90;
  float    laf_max;
  float    las_max;
  uint32_t clips;
  bool     overload;
  bool     underrange;
  bool     hf_alert;
  uint8_t  hf_peak_band;
  float    hf_prom_db;
  float    hf_emerg_db;
  float    oct_leq[OCTAVE_BAND_COUNT];
  float    oct_l90[OCTAVE_BAND_COUNT];
} MinuteReport_t;

typedef struct {
  const char *ts_iso;   // nullptr or "" -> emits "ts":null
  float       gain_db;  // Configured ES7210 PGA gain (dB)
  bool        marker;   // Manual KEY button ground-truth event flag
  uint32_t    vbat_mv;
  int         rssi;
  bool        sd_ok;
  uint32_t    hf_events_today;
} MinuteSysInfo_t;

/**
 * Initialize esp-dsp 4096-pt FFT tables, Hann window, and 26-band bin map.
 */
bool DspOctave_Init();

/**
 * Run synthetic 1 kHz, 4 kHz, and 16 kHz sine wave verification against Parseval's
 * theorem, 1/3-octave band bin mapping, and IEC 61672-1 A-weighting accuracy.
 */
bool DspOctave_RunSelfTest(OctaveSelfTestResult_t *out_result);

/**
 * Reset sliding window, A-weighting IIR states, F/S detectors, and 1s/1m accumulators.
 */
void DspOctave_ResetState();

/**
 * Push a hop of mono 16-bit PCM samples into the sliding 4096-sample FFT window
 * and time-domain IEC 61672 A-weighting + Fast/Slow integrators, emitting 1s / 1m
 * reports at exact sample boundaries.
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
 * Format a 1-minute summary record into compact single-line JSON (SPEC.md 6.4).
 * When `sys_info != nullptr`, includes ISO-8601 `ts`, `"hf"`, and `"sys"`.
 * Returns characters written (excluding null terminator), or < 0 on truncation/error.
 */
int DspOctave_FormatMinuteJson(const MinuteReport_t &rec,
                               const MinuteSysInfo_t *sys_info,
                               char *out_buf,
                               size_t out_buf_size);
