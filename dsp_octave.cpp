#include "dsp_octave.h"
#include "calibration.h"

#include <esp_dsp.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

const char * const OCTAVE_BAND_LABELS[OCTAVE_BAND_COUNT] = {
  "63", "80", "100", "125", "160", "200", "250", "315",
  "400", "500", "630", "800", "1k", "1.25k", "1.6k", "2k",
  "2.5k", "3.15k", "4k", "5k", "6.3k", "8k", "10k", "12.5k",
  "16k", "20k"
};

namespace {

constexpr float   PCM_FULL_SCALE      = 32768.0f;
constexpr float   INV_PCM_FULL_SCALE  = 1.0f / PCM_FULL_SCALE;
constexpr float   THEORETICAL_SINE_DB = -3.0103f;
constexpr float   SELF_TEST_TOL_DB    = 0.5f;
constexpr uint8_t BAND_IDX_1K         = 12;
constexpr uint8_t BAND_IDX_4K         = 18;
constexpr uint8_t BAND_IDX_16K        = 24;

// IEC 61672-1 A-weighting response at 1k, 4k, 16k Hz (dB relative to 1 kHz)
constexpr float   IEC_AW_1K_DB        = 0.000f;
constexpr float   IEC_AW_4K_DB        = +0.963f;
constexpr float   IEC_AW_16K_DB       = -6.706f;

// Exact 4-biquad (8th-order) DF2T IEC 61672-1 A-weighting filter at fs = 48000 Hz
// Verified in float32 (ESP32-S3 hardware FPU): max |error| <= 0.0054 dB (20 Hz .. 20 kHz)
struct BiquadCoeff {
  float b0, b1, b2, a1, a2;
};

constexpr size_t AW_NUM_SECTIONS = 4;
constexpr BiquadCoeff AW_SOS_48K[AW_NUM_SECTIONS] = {
  // Sec 0: max|p| = 0.997307
  { +1.0000000f, -2.0000000f, +1.0000000f, -1.9946145f, +0.9946217f },
  // Sec 1: max|p| = 0.986007
  { +1.0000000f, -2.0000000f, +1.0000000f, -1.8938019f, +0.8950921f },
  // Sec 2: max|p| = 0.912993
  { +0.7687251f, +0.4244548f, -0.0108968f, -1.1176897f, +0.1868866f },
  // Sec 3: max|p| = 0.490387
  { +0.7687251f, -0.5228836f, -0.1632751f, +0.2856912f, -0.1003804f }
};

// Exponential time-weighting constants @ fs = 48000 Hz (float32 FPU):
// alpha = exp(-1 / (fs * tau)), y[n] = alpha * y[n-1] + (1 - alpha) * x_a[n]^2
constexpr float   FAST_TAU_S          = 0.125f;
constexpr float   SLOW_TAU_S          = 1.000f;
const float       kAlphaFast          = expf(-1.0f / (static_cast<float>(AUDIO_SAMPLE_RATE_HZ) * FAST_TAU_S));
const float       kAlphaSlow          = expf(-1.0f / (static_cast<float>(AUDIO_SAMPLE_RATE_HZ) * SLOW_TAU_S));
const float       kOneMinusFast       = 1.0f - kAlphaFast;
const float       kOneMinusSlow       = 1.0f - kAlphaSlow;

// 16-byte aligned internal SRAM buffers for ESP32-S3 PIE SIMD acceleration
alignas(16) float s_hann_win[AUDIO_FFT_SIZE];
alignas(16) float s_fft_buf[2 * AUDIO_FFT_SIZE];
alignas(16) float s_ring_x[AUDIO_FFT_SIZE];

int8_t   s_bin_to_band[AUDIO_FFT_SIZE / 2];
bool     s_inited          = false;
float    s_spec_norm_scale = 0.0f;

// Time-domain A-weighting & exponential detector state (Core 1 only, single-precision FPU)
float    s_aw_z1[AW_NUM_SECTIONS] = {};
float    s_aw_z2[AW_NUM_SECTIONS] = {};
float    s_fast_pow        = 0.0f;
float    s_slow_pow        = 0.0f;

// Sliding window & 1s/1m aggregation state (raw uncalibrated dBFS internally)
size_t   s_ring_fill       = 0;
uint32_t s_sec_samples     = 0;
uint32_t s_sec_fft_frames  = 0;
double   s_sec_bb_power    = 0.0;
double   s_sec_a_pow_sum   = 0.0;
float    s_sec_laf_max_pow = 0.0f;
double   s_sec_band_pow[OCTAVE_BAND_COUNT];
int32_t  s_sec_peak_pcm    = 0;
uint32_t s_sec_clips       = 0;
uint32_t s_sec_dsp_sum_us  = 0;
uint32_t s_sec_dsp_max_us  = 0;

uint8_t  s_min_sec_idx     = 0;
float    s_bb_1s_raw[SECONDS_PER_MINUTE];
float    s_la_1s_raw[SECONDS_PER_MINUTE];
float    s_oct_1s_raw[OCTAVE_BAND_COUNT][SECONDS_PER_MINUTE];
int32_t  s_min_peak_pcm    = 0;
float    s_min_laf_max_pow = 0.0f;
float    s_min_las_max_pow = 0.0f;
uint32_t s_min_clips       = 0;

inline float powerToDb(float power, float offset_db) {
  const float clamped = (power > DBFS_POWER_EPSILON) ? power : DBFS_POWER_EPSILON;
  return 10.0f * log10f(clamped) + offset_db;
}

inline float peakPcmToDb(int32_t peak_pcm, float offset_db) {
  const float norm = static_cast<float>(peak_pcm) * INV_PCM_FULL_SCALE;
  return powerToDb(norm * norm, offset_db);
}

inline float processSampleAWeight(float x) {
  float y = x;
  #pragma GCC unroll 4
  for (size_t s = 0; s < AW_NUM_SECTIONS; ++s) {
    const BiquadCoeff &c = AW_SOS_48K[s];
    const float out = (c.b0 * y) + s_aw_z1[s];
    s_aw_z1[s] = (c.b1 * y) - (c.a1 * out) + s_aw_z2[s];
    s_aw_z2[s] = (c.b2 * y) - (c.a2 * out);
    y = out;
  }
  return y;
}

void sortAscending(float *arr, size_t n) {
  for (size_t i = 1; i < n; ++i) {
    const float key = arr[i];
    size_t j = i;
    while (j > 0 && arr[j - 1] > key) {
      arr[j] = arr[j - 1];
      --j;
    }
    arr[j] = key;
  }
}

void computeMinuteStats(const float raw_values[SECONDS_PER_MINUTE],
                        float offset_db,
                        float *out_leq,
                        float *out_l90) {
  double lin_sum = 0.0;
  float sorted[SECONDS_PER_MINUTE];

  for (size_t i = 0; i < SECONDS_PER_MINUTE; ++i) {
    const float db_raw = raw_values[i];
    sorted[i] = db_raw;
    lin_sum += pow(10.0, static_cast<double>(db_raw) / 10.0);
  }

  const double mean_lin = lin_sum / static_cast<double>(SECONDS_PER_MINUTE);
  *out_leq = powerToDb(static_cast<float>(mean_lin), offset_db);

  // L90 = 10th percentile of 1s levels (rank 0.10 * (60 - 1) = 5.9)
  sortAscending(sorted, SECONDS_PER_MINUTE);
  *out_l90 = (sorted[5] + 0.9f * (sorted[6] - sorted[5])) + offset_db;
}

void evaluateHfNuisance(const float raw_leq[OCTAVE_BAND_COUNT],
                        const float raw_l90[OCTAVE_BAND_COUNT],
                        bool *out_alert,
                        uint8_t *out_peak_band,
                        float *out_prom_db,
                        float *out_emerg_db) {
  // Apply per-band trim relative to hardware noise floor (global SPL offset invariant)
  float l90_eff[OCTAVE_BAND_COUNT];
  float leq_eff[OCTAVE_BAND_COUNT];
  for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
    const float b_trim = Calibration_GetBandOffsetDb(b);
    l90_eff[b] = raw_l90[b] + b_trim;
    leq_eff[b] = raw_leq[b] + b_trim;
  }

  // Mid-HF reference anchor: mean L90 over 3.15k, 4k, 5k, 6.3k Hz (indices 17..20)
  const float mid_anchor = 0.25f * (l90_eff[17] + l90_eff[18] + l90_eff[19] + l90_eff[20]);

  bool    alert_hit   = false;
  uint8_t best_band   = HF_DET_START_BAND;
  float   best_prom   = -99.9f;
  float   best_emerg  = -99.9f;

  uint8_t disp_band   = HF_DET_START_BAND;
  float   disp_prom   = -99.9f;
  float   disp_emerg  = -99.9f;

  for (uint8_t b = HF_DET_START_BAND; b < OCTAVE_BAND_COUNT; ++b) {
    const float emerg = l90_eff[b] - AnechoicOctL90Dbfs(b);
    const float crest = leq_eff[b] - l90_eff[b];

    const float p1_ref = (b + 1 < OCTAVE_BAND_COUNT)
                             ? (0.5f * (l90_eff[b - 1] + l90_eff[b + 1]))
                             : l90_eff[b - 1];
    const float p1 = l90_eff[b] - p1_ref;

    // Two-band jet peak prominence handles hiss straddling two adjacent bands when bounded above
    float adj_prom = p1;
    if (b + 2 < OCTAVE_BAND_COUNT) {
      const float p2 = fminf(l90_eff[b], l90_eff[b + 1]) - (0.5f * (l90_eff[b - 1] + l90_eff[b + 2]));
      if (p2 > adj_prom) {
        adj_prom = p2;
      }
    }

    const float mid_prom = l90_eff[b] - mid_anchor;

    if (adj_prom > disp_prom) {
      disp_prom  = adj_prom;
      disp_emerg = emerg;
      disp_band  = b;
    }

    if (emerg >= HF_DET_MIN_EMERG_DB &&
        crest <= HF_DET_MAX_CREST_DB &&
        adj_prom >= HF_DET_MIN_ADJ_PROM_DB &&
        mid_prom >= HF_DET_MIN_MID_PROM_DB) {
      if (!alert_hit || adj_prom > best_prom) {
        alert_hit  = true;
        best_prom  = adj_prom;
        best_emerg = emerg;
        best_band  = b;
      }
    }
  }

  *out_alert     = alert_hit;
  *out_peak_band = alert_hit ? best_band : disp_band;
  *out_prom_db   = alert_hit ? best_prom : disp_prom;
  *out_emerg_db  = alert_hit ? best_emerg : disp_emerg;
}

void processWindowFrame(const float *frame_x,
                        float *out_bb_power,
                        float out_band_power[OCTAVE_BAND_COUNT],
                        float *out_total_spec_power,
                        uint32_t *out_exec_us) {
  const int64_t t_start = esp_timer_get_time();

  double mean_acc = 0.0;
  for (int n = 0; n < AUDIO_FFT_SIZE; ++n) {
    mean_acc += static_cast<double>(frame_x[n]);
  }
  const float dc_mean = static_cast<float>(mean_acc / static_cast<double>(AUDIO_FFT_SIZE));

  double td_sum_sq = 0.0;
  for (int n = 0; n < AUDIO_FFT_SIZE; ++n) {
    const float u = frame_x[n] - dc_mean;
    td_sum_sq += static_cast<double>(u) * static_cast<double>(u);
    s_fft_buf[2 * n]     = u * s_hann_win[n];
    s_fft_buf[2 * n + 1] = 0.0f;
  }

  dsps_fft2r_fc32(s_fft_buf, AUDIO_FFT_SIZE);
  dsps_bit_rev2r_fc32(s_fft_buf, AUDIO_FFT_SIZE);

  for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
    out_band_power[b] = 0.0f;
  }

  double total_spec_pow = 0.0;
  const int max_bin = (AUDIO_FFT_SIZE / 2) - 1;

  for (int k = 1; k <= max_bin; ++k) {
    const float re = s_fft_buf[2 * k];
    const float im = s_fft_buf[2 * k + 1];
    const float p_k = (re * re + im * im) * s_spec_norm_scale;
    total_spec_pow += static_cast<double>(p_k);

    const int8_t band = s_bin_to_band[k];
    if (band >= 0) {
      out_band_power[band] += p_k;
    }
  }

  *out_bb_power = static_cast<float>(td_sum_sq / static_cast<double>(AUDIO_FFT_SIZE));
  if (out_total_spec_power != nullptr) {
    *out_total_spec_power = static_cast<float>(total_spec_pow);
  }
  if (out_exec_us != nullptr) {
    *out_exec_us = static_cast<uint32_t>(esp_timer_get_time() - t_start);
  }
}

bool buildBandBinMap() {
  const float df = static_cast<float>(AUDIO_SAMPLE_RATE_HZ) / static_cast<float>(AUDIO_FFT_SIZE);
  const int max_bin = (AUDIO_FFT_SIZE / 2) - 1;
  const float edge_ratio_low  = powf(2.0f, -1.0f / 6.0f);
  const float edge_ratio_high = powf(2.0f, +1.0f / 6.0f);

  memset(s_bin_to_band, -1, sizeof(s_bin_to_band));

  bool all_bands_nonempty = true;
  for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
    const float fc = 1000.0f * powf(2.0f, static_cast<float>(b - static_cast<int>(BAND_IDX_1K)) / 3.0f);
    // Clamp lowest band (63 Hz) lower edge to 54.0 Hz so bin 5 (58.59 Hz) @ 48kHz maps cleanly
    const float f_low  = (b == 0) ? 54.0f : (fc * edge_ratio_low);
    const float f_high = fc * edge_ratio_high;

    uint16_t bin_count = 0;
    for (int k = 1; k <= max_bin; ++k) {
      const float fk = static_cast<float>(k) * df;
      if (fk >= f_low && fk < f_high) {
        s_bin_to_band[k] = static_cast<int8_t>(b);
        bin_count++;
      }
    }

    if (bin_count == 0) {
      Serial.printf("[DSP] ERROR: 1/3-octave band %u (%s Hz) has 0 FFT bins!\n", b, OCTAVE_BAND_LABELS[b]);
      all_bands_nonempty = false;
    }
  }
  return all_bands_nonempty;
}

float measureAWeightSineDb(float freq_hz) {
  memset(s_aw_z1, 0, sizeof(s_aw_z1));
  memset(s_aw_z2, 0, sizeof(s_aw_z2));

  constexpr uint32_t TOTAL_SAMPLES  = AUDIO_SAMPLE_RATE_HZ / 2U;  // 0.50 s = 24000
  constexpr uint32_t SETTLE_SAMPLES = AUDIO_SAMPLE_RATE_HZ / 4U;  // discard first 0.25 s
  const double phase_inc = (2.0 * M_PI * static_cast<double>(freq_hz)) /
                           static_cast<double>(AUDIO_SAMPLE_RATE_HZ);

  double sum_sq = 0.0;
  for (uint32_t n = 0; n < TOTAL_SAMPLES; ++n) {
    const float x = static_cast<float>(sin(phase_inc * static_cast<double>(n)));
    const double y = static_cast<double>(processSampleAWeight(x));
    if (n >= SETTLE_SAMPLES) {
      sum_sq += y * y;
    }
  }
  const float mean_pow = static_cast<float>(sum_sq / static_cast<double>(TOTAL_SAMPLES - SETTLE_SAMPLES));
  return powerToDb(mean_pow, 0.0f);
}

}  // namespace

bool DspOctave_Init() {
  if (s_inited) {
    return true;
  }

  esp_err_t ret = dsps_fft2r_init_fc32(nullptr, CONFIG_DSP_MAX_FFT_SIZE);
  if (ret != ESP_OK && ret != ESP_ERR_DSP_REINITIALIZED) {
    Serial.printf("[DSP] dsps_fft2r_init_fc32 failed: %d\n", static_cast<int>(ret));
    return false;
  }

  double wss = 0.0;
  for (int n = 0; n < AUDIO_FFT_SIZE; ++n) {
    const float w = 0.5f * (1.0f - cosf((2.0f * static_cast<float>(M_PI) * static_cast<float>(n)) /
                                        static_cast<float>(AUDIO_FFT_SIZE - 1)));
    s_hann_win[n] = w;
    wss += static_cast<double>(w) * static_cast<double>(w);
  }
  s_spec_norm_scale = static_cast<float>(2.0 / (static_cast<double>(AUDIO_FFT_SIZE) * wss));

  if (!buildBandBinMap()) {
    return false;
  }

  DspOctave_ResetState();
  s_inited = true;
  return true;
}

void DspOctave_ResetState() {
  memset(s_ring_x, 0, sizeof(s_ring_x));
  memset(s_fft_buf, 0, sizeof(s_fft_buf));
  memset(s_aw_z1, 0, sizeof(s_aw_z1));
  memset(s_aw_z2, 0, sizeof(s_aw_z2));
  memset(s_sec_band_pow, 0, sizeof(s_sec_band_pow));
  memset(s_bb_1s_raw, 0, sizeof(s_bb_1s_raw));
  memset(s_la_1s_raw, 0, sizeof(s_la_1s_raw));
  memset(s_oct_1s_raw, 0, sizeof(s_oct_1s_raw));

  s_fast_pow        = 0.0f;
  s_slow_pow        = 0.0f;

  s_ring_fill       = 0;
  s_sec_samples     = 0;
  s_sec_fft_frames  = 0;
  s_sec_bb_power    = 0.0;
  s_sec_a_pow_sum   = 0.0;
  s_sec_laf_max_pow = 0.0f;
  s_sec_peak_pcm    = 0;
  s_sec_clips       = 0;
  s_sec_dsp_sum_us  = 0;
  s_sec_dsp_max_us  = 0;

  s_min_sec_idx     = 0;
  s_min_peak_pcm    = 0;
  s_min_laf_max_pow = 0.0f;
  s_min_las_max_pow = 0.0f;
  s_min_clips       = 0;
}

bool DspOctave_RunSelfTest(OctaveSelfTestResult_t *out_result) {
  if (!s_inited || out_result == nullptr) {
    return false;
  }

  float band_pow[OCTAVE_BAND_COUNT] = {};
  float bb_pow = 0.0f;
  float spec_pow = 0.0f;
  uint32_t exec_us = 0;

  // Test 1: 1000 Hz sine -> Band 12 (1 kHz) & A(1k) = 0.00 dB
  for (int n = 0; n < AUDIO_FFT_SIZE; ++n) {
    s_ring_x[n] = sinf((2.0f * static_cast<float>(M_PI) * 1000.0f * static_cast<float>(n)) /
                       static_cast<float>(AUDIO_SAMPLE_RATE_HZ));
  }
  processWindowFrame(s_ring_x, &bb_pow, band_pow, &spec_pow, &exec_us);
  out_result->tone1k_td_dbfs     = powerToDb(bb_pow, 0.0f);
  out_result->tone1k_spec_dbfs   = powerToDb(spec_pow, 0.0f);
  out_result->tone1k_band12_dbfs = powerToDb(band_pow[BAND_IDX_1K], 0.0f);
  out_result->tone1k_laeq_dbfs   = measureAWeightSineDb(1000.0f);
  out_result->single_fft_us      = exec_us;

  // Test 2: 4000 Hz sine -> Band 18 (4 kHz) & A(4k) = +0.96 dB
  for (int n = 0; n < AUDIO_FFT_SIZE; ++n) {
    s_ring_x[n] = sinf((2.0f * static_cast<float>(M_PI) * 4000.0f * static_cast<float>(n)) /
                       static_cast<float>(AUDIO_SAMPLE_RATE_HZ));
  }
  processWindowFrame(s_ring_x, &bb_pow, band_pow, &spec_pow, &exec_us);
  out_result->tone4k_td_dbfs     = powerToDb(bb_pow, 0.0f);
  out_result->tone4k_spec_dbfs   = powerToDb(spec_pow, 0.0f);
  out_result->tone4k_band18_dbfs = powerToDb(band_pow[BAND_IDX_4K], 0.0f);
  out_result->tone4k_laeq_dbfs   = measureAWeightSineDb(4000.0f);
  if (exec_us < out_result->single_fft_us) {
    out_result->single_fft_us = exec_us;
  }

  // Test 3: 16000 Hz sine -> Band 24 (16 kHz) & A(16k) = -6.71 dB
  for (int n = 0; n < AUDIO_FFT_SIZE; ++n) {
    s_ring_x[n] = sinf((2.0f * static_cast<float>(M_PI) * 16000.0f * static_cast<float>(n)) /
                       static_cast<float>(AUDIO_SAMPLE_RATE_HZ));
  }
  processWindowFrame(s_ring_x, &bb_pow, band_pow, &spec_pow, &exec_us);
  out_result->tone16k_td_dbfs     = powerToDb(bb_pow, 0.0f);
  out_result->tone16k_spec_dbfs   = powerToDb(spec_pow, 0.0f);
  out_result->tone16k_band24_dbfs = powerToDb(band_pow[BAND_IDX_16K], 0.0f);
  out_result->tone16k_laeq_dbfs   = measureAWeightSineDb(16000.0f);

  DspOctave_ResetState();

  const bool pass_1k = (fabsf(out_result->tone1k_td_dbfs     - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB) &&
                       (fabsf(out_result->tone1k_spec_dbfs   - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB) &&
                       (fabsf(out_result->tone1k_band12_dbfs - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB) &&
                       (fabsf(out_result->tone1k_laeq_dbfs   - (THEORETICAL_SINE_DB + IEC_AW_1K_DB)) <= SELF_TEST_TOL_DB);
  const bool pass_4k = (fabsf(out_result->tone4k_td_dbfs     - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB) &&
                       (fabsf(out_result->tone4k_spec_dbfs   - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB) &&
                       (fabsf(out_result->tone4k_band18_dbfs - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB) &&
                       (fabsf(out_result->tone4k_laeq_dbfs   - (THEORETICAL_SINE_DB + IEC_AW_4K_DB)) <= SELF_TEST_TOL_DB);
  const bool pass_16k = (fabsf(out_result->tone16k_td_dbfs     - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB) &&
                        (fabsf(out_result->tone16k_spec_dbfs   - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB) &&
                        (fabsf(out_result->tone16k_band24_dbfs - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB) &&
                        (fabsf(out_result->tone16k_laeq_dbfs   - (THEORETICAL_SINE_DB + IEC_AW_16K_DB)) <= SELF_TEST_TOL_DB);

  out_result->passed = (pass_1k && pass_4k && pass_16k);
  return out_result->passed;
}

bool DspOctave_PushHopSamples(const int16_t *hop_pcm,
                              size_t hop_len,
                              int32_t hop_peak_pcm,
                              uint32_t hop_clips,
                              SecondReport_t *out_sec,
                              bool *out_sec_ready,
                              MinuteReport_t *out_min,
                              bool *out_min_ready) {
  if (!s_inited || hop_pcm == nullptr || hop_len == 0 || hop_len > AUDIO_FFT_SIZE) {
    return false;
  }

  if (out_sec_ready != nullptr) {
    *out_sec_ready = false;
  }
  if (out_min_ready != nullptr) {
    *out_min_ready = false;
  }

  // Shift sliding window left by hop_len and append normalized float samples
  const int64_t hop_t0 = esp_timer_get_time();
  const size_t keep_len = AUDIO_FFT_SIZE - hop_len;
  memmove(s_ring_x, &s_ring_x[hop_len], keep_len * sizeof(float));

  // Time-domain IEC 61672-1 A-weighting + Fast/Slow integrators (hardware float32 FPU)
  float fast_pow = s_fast_pow;
  float slow_pow = s_slow_pow;
  float a_pow_sum = 0.0f;
  float sec_laf_max = s_sec_laf_max_pow;
  float min_laf_max = s_min_laf_max_pow;
  float min_las_max = s_min_las_max_pow;

  const BiquadCoeff c0 = AW_SOS_48K[0];
  const BiquadCoeff c1 = AW_SOS_48K[1];
  const BiquadCoeff c2 = AW_SOS_48K[2];
  const BiquadCoeff c3 = AW_SOS_48K[3];
  float z1_0 = s_aw_z1[0], z2_0 = s_aw_z2[0];
  float z1_1 = s_aw_z1[1], z2_1 = s_aw_z2[1];
  float z1_2 = s_aw_z1[2], z2_2 = s_aw_z2[2];
  float z1_3 = s_aw_z1[3], z2_3 = s_aw_z2[3];

  for (size_t i = 0; i < hop_len; ++i) {
    const float x_norm = static_cast<float>(hop_pcm[i]) * INV_PCM_FULL_SCALE;
    s_ring_x[keep_len + i] = x_norm;

    float y = (c0.b0 * x_norm) + z1_0;
    z1_0 = (c0.b1 * x_norm) - (c0.a1 * y) + z2_0;
    z2_0 = (c0.b2 * x_norm) - (c0.a2 * y);

    float y1 = (c1.b0 * y) + z1_1;
    z1_1 = (c1.b1 * y) - (c1.a1 * y1) + z2_1;
    z2_1 = (c1.b2 * y) - (c1.a2 * y1);

    float y2 = (c2.b0 * y1) + z1_2;
    z1_2 = (c2.b1 * y1) - (c2.a1 * y2) + z2_2;
    z2_2 = (c2.b2 * y1) - (c2.a2 * y2);

    float ya = (c3.b0 * y2) + z1_3;
    z1_3 = (c3.b1 * y2) - (c3.a1 * ya) + z2_3;
    z2_3 = (c3.b2 * y2) - (c3.a2 * ya);

    const float p_a = ya * ya;
    a_pow_sum += p_a;

    fast_pow = (kAlphaFast * fast_pow) + (kOneMinusFast * p_a);
    slow_pow = (kAlphaSlow * slow_pow) + (kOneMinusSlow * p_a);

    if (fast_pow > sec_laf_max) sec_laf_max = fast_pow;
    if (fast_pow > min_laf_max) min_laf_max = fast_pow;
    if (slow_pow > min_las_max) min_las_max = slow_pow;
  }

  s_aw_z1[0] = z1_0; s_aw_z2[0] = z2_0;
  s_aw_z1[1] = z1_1; s_aw_z2[1] = z2_1;
  s_aw_z1[2] = z1_2; s_aw_z2[2] = z2_2;
  s_aw_z1[3] = z1_3; s_aw_z2[3] = z2_3;
  s_fast_pow = fast_pow;
  s_slow_pow = slow_pow;
  s_sec_a_pow_sum += static_cast<double>(a_pow_sum);
  s_sec_laf_max_pow = sec_laf_max;
  s_min_laf_max_pow = min_laf_max;
  s_min_las_max_pow = min_las_max;

  s_ring_fill += hop_len;
  if (s_ring_fill > AUDIO_FFT_SIZE) {
    s_ring_fill = AUDIO_FFT_SIZE;
  }

  if (hop_peak_pcm > s_sec_peak_pcm) {
    s_sec_peak_pcm = hop_peak_pcm;
  }
  s_sec_clips += hop_clips;

  if (hop_peak_pcm > s_min_peak_pcm) {
    s_min_peak_pcm = hop_peak_pcm;
  }
  s_min_clips += hop_clips;

  if (s_ring_fill >= AUDIO_FFT_SIZE) {
    float bb_pow = 0.0f;
    float band_pow[OCTAVE_BAND_COUNT] = {};

    processWindowFrame(s_ring_x, &bb_pow, band_pow, nullptr, nullptr);
    const uint32_t exec_us = static_cast<uint32_t>(esp_timer_get_time() - hop_t0);

    s_sec_bb_power += static_cast<double>(bb_pow);
    for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
      s_sec_band_pow[b] += static_cast<double>(band_pow[b]);
    }
    s_sec_fft_frames++;
    s_sec_dsp_sum_us += exec_us;
    if (exec_us > s_sec_dsp_max_us) {
      s_sec_dsp_max_us = exec_us;
    }
  }

  s_sec_samples += static_cast<uint32_t>(hop_len);
  if (s_sec_samples >= AUDIO_SAMPLE_RATE_HZ) {
    const uint32_t now_s = static_cast<uint32_t>(millis() / 1000UL);
    const float inv_frames = (s_sec_fft_frames > 0) ? (1.0f / static_cast<float>(s_sec_fft_frames)) : 1.0f;
    const float cal_ofs = Calibration_GetOffsetDb();

    const float avg_bb_pow  = static_cast<float>(s_sec_bb_power) * inv_frames;
    const float bb_raw_1s   = powerToDb(avg_bb_pow, 0.0f);
    const float la_raw_1s   = powerToDb(static_cast<float>(s_sec_a_pow_sum / static_cast<double>(s_sec_samples)), 0.0f);

    s_bb_1s_raw[s_min_sec_idx] = bb_raw_1s;
    s_la_1s_raw[s_min_sec_idx] = la_raw_1s;
    for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
      const float avg_band_pow = static_cast<float>(s_sec_band_pow[b]) * inv_frames;
      s_oct_1s_raw[b][s_min_sec_idx] = powerToDb(avg_band_pow, 0.0f);
    }

    if (out_sec != nullptr && out_sec_ready != nullptr) {
      out_sec->uptime_s           = now_s;
      out_sec->fft_frames         = s_sec_fft_frames;
      out_sec->bb_leq_1s          = bb_raw_1s + cal_ofs;
      out_sec->bb_peak_1s         = peakPcmToDb(s_sec_peak_pcm, cal_ofs);
      out_sec->laeq_1s            = la_raw_1s + cal_ofs;
      out_sec->laf_max_1s         = powerToDb(static_cast<float>(s_sec_laf_max_pow), cal_ofs);
      out_sec->clips_1s           = s_sec_clips;
      out_sec->overload_1s        = (s_sec_clips > 0);
      out_sec->underrange_1s      = (bb_raw_1s <= (AnechoicBbLeqDbfs() + UNDER_RANGE_MARGIN_DB));
      for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
        out_sec->oct_1s[b] = s_oct_1s_raw[b][s_min_sec_idx] + cal_ofs + Calibration_GetBandOffsetDb(b);
      }
      out_sec->dsp_avg_us         = (s_sec_fft_frames > 0) ? (s_sec_dsp_sum_us / s_sec_fft_frames) : 0;
      out_sec->dsp_max_us         = s_sec_dsp_max_us;
      out_sec->free_internal_sram = static_cast<uint32_t>(
          heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
      *out_sec_ready = true;
    }

    // Reset 1-second accumulators
    s_sec_samples     = 0;
    s_sec_fft_frames  = 0;
    s_sec_bb_power    = 0.0;
    s_sec_a_pow_sum   = 0.0;
    s_sec_laf_max_pow = 0.0;
    memset(s_sec_band_pow, 0, sizeof(s_sec_band_pow));
    s_sec_peak_pcm    = 0;
    s_sec_clips       = 0;
    s_sec_dsp_sum_us  = 0;
    s_sec_dsp_max_us  = 0;

    s_min_sec_idx++;
    if (s_min_sec_idx >= SECONDS_PER_MINUTE) {
      if (out_min != nullptr && out_min_ready != nullptr) {
        const float min_cal_ofs = Calibration_GetOffsetDb();
        float raw_bb_leq = 0.0f;
        float raw_bb_l90 = 0.0f;
        computeMinuteStats(s_bb_1s_raw, 0.0f, &raw_bb_leq, &raw_bb_l90);

        float raw_oct_leq[OCTAVE_BAND_COUNT] = {};
        float raw_oct_l90[OCTAVE_BAND_COUNT] = {};
        for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
          computeMinuteStats(s_oct_1s_raw[b], 0.0f, &raw_oct_leq[b], &raw_oct_l90[b]);
        }

        out_min->epoch_s    = static_cast<int64_t>(time(nullptr));
        out_min->uptime_s   = now_s;
        out_min->bb_leq     = raw_bb_leq + min_cal_ofs;
        out_min->bb_l90     = raw_bb_l90 + min_cal_ofs;
        out_min->bb_peak    = peakPcmToDb(s_min_peak_pcm, min_cal_ofs);
        computeMinuteStats(s_la_1s_raw, min_cal_ofs, &out_min->laeq, &out_min->la90);
        out_min->laf_max    = powerToDb(static_cast<float>(s_min_laf_max_pow), min_cal_ofs);
        out_min->las_max    = powerToDb(static_cast<float>(s_min_las_max_pow), min_cal_ofs);
        out_min->clips      = s_min_clips;
        out_min->overload   = (s_min_clips > 0);
        out_min->underrange = (raw_bb_leq <= (AnechoicBbLeqDbfs() + UNDER_RANGE_MARGIN_DB));

        evaluateHfNuisance(raw_oct_leq,
                           raw_oct_l90,
                           &out_min->hf_alert,
                           &out_min->hf_peak_band,
                           &out_min->hf_prom_db,
                           &out_min->hf_emerg_db);

        for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
          const float b_trim = Calibration_GetBandOffsetDb(b);
          out_min->oct_leq[b] = raw_oct_leq[b] + min_cal_ofs + b_trim;
          out_min->oct_l90[b] = raw_oct_l90[b] + min_cal_ofs + b_trim;
        }
        *out_min_ready = true;
      }

      // Reset 1-minute accumulators
      s_min_sec_idx     = 0;
      s_min_peak_pcm    = 0;
      s_min_laf_max_pow = 0.0;
      s_min_las_max_pow = 0.0;
      s_min_clips       = 0;
    }
  }

  return true;
}

static bool appendFmt(char *buf, size_t cap, size_t *pos, const char *fmt, ...) {
  if (*pos >= cap) {
    return false;
  }
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(buf + *pos, cap - *pos, fmt, ap);
  va_end(ap);
  if (n < 0 || static_cast<size_t>(n) >= (cap - *pos)) {
    return false;
  }
  *pos += static_cast<size_t>(n);
  return true;
}

int DspOctave_FormatMinuteJson(const MinuteReport_t &rec,
                               const MinuteSysInfo_t *sys_info,
                               char *out_buf,
                               size_t out_buf_size) {
  if (out_buf == nullptr || out_buf_size == 0) {
    return -1;
  }

  const bool has_ts = (sys_info != nullptr && sys_info->ts_iso != nullptr && sys_info->ts_iso[0] != '\0');
  const char *unit_lbl = Calibration_GetUnitLabel();
  const float cal_ofs  = Calibration_GetOffsetDb();
  const float gain_db  = (sys_info != nullptr) ? sys_info->gain_db : ES7210_MIC_GAIN_DB;
  const unsigned mark  = (sys_info != nullptr && sys_info->marker) ? 1U : 0U;
  const char *hf_band  = (rec.hf_peak_band < OCTAVE_BAND_COUNT) ? OCTAVE_BAND_LABELS[rec.hf_peak_band] : "8k";
  const uint32_t hf_day = (sys_info != nullptr) ? sys_info->hf_events_today : 0UL;
  size_t pos = 0;

  bool ok = false;
  if (has_ts) {
    ok = appendFmt(out_buf, out_buf_size, &pos,
                   "{\"device_id\":\"%s\",\"ts\":\"%s\",\"uptime_s\":%lu,"
                   "\"unit\":\"%s\",\"cal_db\":%.2f,\"fs_hz\":%lu,\"gain_db\":%.1f,\"marker\":%u,",
                   DEVICE_ID,
                   sys_info->ts_iso,
                   static_cast<unsigned long>(rec.uptime_s),
                   unit_lbl,
                   cal_ofs,
                   static_cast<unsigned long>(AUDIO_SAMPLE_RATE_HZ),
                   gain_db,
                   mark);
  } else {
    ok = appendFmt(out_buf, out_buf_size, &pos,
                   "{\"device_id\":\"%s\",\"ts\":null,\"uptime_s\":%lu,"
                   "\"unit\":\"%s\",\"cal_db\":%.2f,\"fs_hz\":%lu,\"gain_db\":%.1f,\"marker\":%u,",
                   DEVICE_ID,
                   static_cast<unsigned long>(rec.uptime_s),
                   unit_lbl,
                   cal_ofs,
                   static_cast<unsigned long>(AUDIO_SAMPLE_RATE_HZ),
                   gain_db,
                   mark);
  }
  if (!ok) {
    return -1;
  }

  if (!appendFmt(out_buf, out_buf_size, &pos,
                 "\"bb\":{\"leq\":%.1f,\"l90\":%.1f,\"peak\":%.1f,"
                 "\"laeq\":%.1f,\"la90\":%.1f,\"laf_max\":%.1f,\"las_max\":%.1f,"
                 "\"clips\":%lu,\"ovl\":%s,\"ur\":%s},"
                 "\"hf\":{\"alert\":%s,\"band\":\"%s\",\"prom_db\":%.1f,\"emerg_db\":%.1f,\"min_today\":%lu},"
                 "\"oct_leq\":[",
                 rec.bb_leq,
                 rec.bb_l90,
                 rec.bb_peak,
                 rec.laeq,
                 rec.la90,
                 rec.laf_max,
                 rec.las_max,
                 static_cast<unsigned long>(rec.clips),
                 rec.overload ? "true" : "false",
                 rec.underrange ? "true" : "false",
                 rec.hf_alert ? "true" : "false",
                 hf_band,
                 rec.hf_prom_db,
                 rec.hf_emerg_db,
                 static_cast<unsigned long>(hf_day))) {
    return -1;
  }

  for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
    if (!appendFmt(out_buf, out_buf_size, &pos, "%s%.1f", (b == 0) ? "" : ",", rec.oct_leq[b])) {
      return -1;
    }
  }

  if (!appendFmt(out_buf, out_buf_size, &pos, "],\"oct_l90\":[")) {
    return -1;
  }

  for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
    if (!appendFmt(out_buf, out_buf_size, &pos, "%s%.1f", (b == 0) ? "" : ",", rec.oct_l90[b])) {
      return -1;
    }
  }

  if (sys_info != nullptr) {
    if (!appendFmt(out_buf, out_buf_size, &pos,
                   "],\"sys\":{\"vbat_mv\":%lu,\"rssi\":%d,\"sd_ok\":%s}}",
                   static_cast<unsigned long>(sys_info->vbat_mv),
                   sys_info->rssi,
                   sys_info->sd_ok ? "true" : "false")) {
      return -1;
    }
  } else {
    if (!appendFmt(out_buf, out_buf_size, &pos, "]}")) {
      return -1;
    }
  }

  return static_cast<int>(pos);
}
