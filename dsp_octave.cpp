#include "dsp_octave.h"

#include <esp_dsp.h>
#include <esp_heap_caps.h>
#include <esp_timer.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

const char * const OCTAVE_BAND_LABELS[OCTAVE_BAND_COUNT] = {
  "63", "80", "100", "125", "160", "200", "250", "315",
  "400", "500", "630", "800", "1k", "1.25k", "1.6k", "2k",
  "2.5k", "3.15k", "4k", "5k", "6.3k", "8k", "10k", "12.5k"
};

namespace {

constexpr float   PCM_FULL_SCALE      = 32768.0f;
constexpr float   INV_PCM_FULL_SCALE  = 1.0f / PCM_FULL_SCALE;
constexpr float   THEORETICAL_SINE_DB = -3.0103f;
constexpr float   SELF_TEST_TOL_DB    = 0.5f;
constexpr uint8_t BAND_IDX_1K         = 12;
constexpr uint8_t BAND_IDX_4K         = 18;

// 16-byte aligned internal SRAM buffers for ESP32-S3 PIE SIMD acceleration
alignas(16) float s_hann_win[AUDIO_FFT_SIZE];
alignas(16) float s_fft_buf[2 * AUDIO_FFT_SIZE];
alignas(16) float s_ring_x[AUDIO_FFT_SIZE];

int8_t   s_bin_to_band[AUDIO_FFT_SIZE / 2];
bool     s_inited          = false;
float    s_spec_norm_scale = 0.0f;

// Sliding window & 1s/1m aggregation state (Core 1 only)
size_t   s_ring_fill       = 0;
uint32_t s_sec_samples     = 0;
uint32_t s_sec_fft_frames  = 0;
double   s_sec_bb_power    = 0.0;
double   s_sec_band_pow[OCTAVE_BAND_COUNT];
int32_t  s_sec_peak_pcm    = 0;
uint32_t s_sec_clips       = 0;
uint32_t s_sec_dsp_sum_us  = 0;
uint32_t s_sec_dsp_max_us  = 0;

uint8_t  s_min_sec_idx     = 0;
float    s_bb_1s[SECONDS_PER_MINUTE];
float    s_oct_1s[OCTAVE_BAND_COUNT][SECONDS_PER_MINUTE];
int32_t  s_min_peak_pcm    = 0;
uint32_t s_min_clips       = 0;

inline float powerToDb(float power) {
  const float clamped = (power > DBFS_POWER_EPSILON) ? power : DBFS_POWER_EPSILON;
  return 10.0f * log10f(clamped) + CALIBRATION_OFFSET_DB;
}

inline float peakPcmToDb(int32_t peak_pcm) {
  const float norm = static_cast<float>(peak_pcm) * INV_PCM_FULL_SCALE;
  return powerToDb(norm * norm);
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

void computeMinuteStats(const float values[SECONDS_PER_MINUTE], float *out_leq, float *out_l90) {
  double lin_sum = 0.0;
  float sorted[SECONDS_PER_MINUTE];

  for (size_t i = 0; i < SECONDS_PER_MINUTE; ++i) {
    const float db = values[i];
    sorted[i] = db;
    lin_sum += pow(10.0, static_cast<double>(db - CALIBRATION_OFFSET_DB) / 10.0);
  }

  const double mean_lin = lin_sum / static_cast<double>(SECONDS_PER_MINUTE);
  *out_leq = powerToDb(static_cast<float>(mean_lin));

  // L90 = 10th percentile of 1s levels (rank 0.10 * (60 - 1) = 5.9)
  sortAscending(sorted, SECONDS_PER_MINUTE);
  *out_l90 = sorted[5] + 0.9f * (sorted[6] - sorted[5]);
}

void processWindowFrame(const float *frame_x,
                        float *out_bb_power,
                        float out_band_power[OCTAVE_BAND_COUNT],
                        float *out_total_spec_power,
                        uint32_t *out_exec_us) {
  const int64_t t_start = esp_timer_get_time();

  // 1. Frame DC mean
  double mean_acc = 0.0;
  for (int n = 0; n < AUDIO_FFT_SIZE; ++n) {
    mean_acc += static_cast<double>(frame_x[n]);
  }
  const float dc_mean = static_cast<float>(mean_acc / static_cast<double>(AUDIO_FFT_SIZE));

  // 2. DC removal, time-domain RMS power, and Hann windowing into complex FFT buffer
  double td_sum_sq = 0.0;
  for (int n = 0; n < AUDIO_FFT_SIZE; ++n) {
    const float u = frame_x[n] - dc_mean;
    td_sum_sq += static_cast<double>(u) * static_cast<double>(u);
    s_fft_buf[2 * n]     = u * s_hann_win[n];
    s_fft_buf[2 * n + 1] = 0.0f;
  }

  // 3. Hardware-accelerated radix-2 complex FFT (ESP32-S3 PIE SIMD via esp-dsp)
  dsps_fft2r_fc32(s_fft_buf, AUDIO_FFT_SIZE);
  dsps_bit_rev2r_fc32(s_fft_buf, AUDIO_FFT_SIZE);

  // 4. Single-sided power spectrum & O(1) 24-band 1/3-octave accumulation
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
    // Clamp lowest band (63 Hz) lower edge to 54.0 Hz so bin 7 (54.69 Hz) and bin 8 (62.5 Hz) both map
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

  // Precompute Hann window and Parseval energy normalization factor Wss
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
  memset(s_sec_band_pow, 0, sizeof(s_sec_band_pow));
  memset(s_bb_1s, 0, sizeof(s_bb_1s));
  memset(s_oct_1s, 0, sizeof(s_oct_1s));

  s_ring_fill      = 0;
  s_sec_samples    = 0;
  s_sec_fft_frames = 0;
  s_sec_bb_power   = 0.0;
  s_sec_peak_pcm   = 0;
  s_sec_clips      = 0;
  s_sec_dsp_sum_us = 0;
  s_sec_dsp_max_us = 0;

  s_min_sec_idx    = 0;
  s_min_peak_pcm   = 0;
  s_min_clips      = 0;
}

bool DspOctave_RunSelfTest(OctaveSelfTestResult_t *out_result) {
  if (!s_inited || out_result == nullptr) {
    return false;
  }

  // Reuse static s_ring_x in internal SRAM so loopTask stack does not overflow
  float band_pow[OCTAVE_BAND_COUNT] = {};
  float bb_pow = 0.0f;
  float spec_pow = 0.0f;
  uint32_t exec_us = 0;

  // Test 1: 1000 Hz full-scale sine -> Band 12 (1 kHz)
  for (int n = 0; n < AUDIO_FFT_SIZE; ++n) {
    s_ring_x[n] = sinf((2.0f * static_cast<float>(M_PI) * 1000.0f * static_cast<float>(n)) /
                       static_cast<float>(AUDIO_SAMPLE_RATE_HZ));
  }
  processWindowFrame(s_ring_x, &bb_pow, band_pow, &spec_pow, &exec_us);
  out_result->tone1k_td_dbfs     = powerToDb(bb_pow) - CALIBRATION_OFFSET_DB;
  out_result->tone1k_spec_dbfs   = powerToDb(spec_pow) - CALIBRATION_OFFSET_DB;
  out_result->tone1k_band12_dbfs = powerToDb(band_pow[BAND_IDX_1K]) - CALIBRATION_OFFSET_DB;
  out_result->single_fft_us      = exec_us;

  // Test 2: 4000 Hz full-scale sine -> Band 18 (4 kHz)
  for (int n = 0; n < AUDIO_FFT_SIZE; ++n) {
    s_ring_x[n] = sinf((2.0f * static_cast<float>(M_PI) * 4000.0f * static_cast<float>(n)) /
                       static_cast<float>(AUDIO_SAMPLE_RATE_HZ));
  }
  processWindowFrame(s_ring_x, &bb_pow, band_pow, &spec_pow, &exec_us);
  out_result->tone4k_td_dbfs     = powerToDb(bb_pow) - CALIBRATION_OFFSET_DB;
  out_result->tone4k_spec_dbfs   = powerToDb(spec_pow) - CALIBRATION_OFFSET_DB;
  out_result->tone4k_band18_dbfs = powerToDb(band_pow[BAND_IDX_4K]) - CALIBRATION_OFFSET_DB;
  if (exec_us < out_result->single_fft_us) {
    out_result->single_fft_us = exec_us;
  }

  DspOctave_ResetState();

  const bool pass_1k = (fabsf(out_result->tone1k_td_dbfs     - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB) &&
                       (fabsf(out_result->tone1k_spec_dbfs   - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB) &&
                       (fabsf(out_result->tone1k_band12_dbfs - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB);
  const bool pass_4k = (fabsf(out_result->tone4k_td_dbfs     - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB) &&
                       (fabsf(out_result->tone4k_spec_dbfs   - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB) &&
                       (fabsf(out_result->tone4k_band18_dbfs - THEORETICAL_SINE_DB) <= SELF_TEST_TOL_DB);

  out_result->passed = (pass_1k && pass_4k);
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
  const size_t keep_len = AUDIO_FFT_SIZE - hop_len;
  memmove(s_ring_x, &s_ring_x[hop_len], keep_len * sizeof(float));
  for (size_t i = 0; i < hop_len; ++i) {
    s_ring_x[keep_len + i] = static_cast<float>(hop_pcm[i]) * INV_PCM_FULL_SCALE;
  }

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
    uint32_t exec_us = 0;

    processWindowFrame(s_ring_x, &bb_pow, band_pow, nullptr, &exec_us);

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

    const float avg_bb_pow = static_cast<float>(s_sec_bb_power) * inv_frames;
    const float bb_leq_1s  = powerToDb(avg_bb_pow);
    const float bb_peak_1s = peakPcmToDb(s_sec_peak_pcm);

    s_bb_1s[s_min_sec_idx] = bb_leq_1s;
    for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
      const float avg_band_pow = static_cast<float>(s_sec_band_pow[b]) * inv_frames;
      s_oct_1s[b][s_min_sec_idx] = powerToDb(avg_band_pow);
    }

    if (out_sec != nullptr && out_sec_ready != nullptr) {
      out_sec->uptime_s           = now_s;
      out_sec->fft_frames         = s_sec_fft_frames;
      out_sec->bb_leq_1s          = bb_leq_1s;
      out_sec->bb_peak_1s         = bb_peak_1s;
      out_sec->clips_1s           = s_sec_clips;
      for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
        out_sec->oct_1s[b] = s_oct_1s[b][s_min_sec_idx];
      }
      out_sec->dsp_avg_us         = (s_sec_fft_frames > 0) ? (s_sec_dsp_sum_us / s_sec_fft_frames) : 0;
      out_sec->dsp_max_us         = s_sec_dsp_max_us;
      out_sec->free_internal_sram = static_cast<uint32_t>(
          heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
      *out_sec_ready = true;
    }

    // Reset 1-second accumulators
    s_sec_samples    = 0;
    s_sec_fft_frames = 0;
    s_sec_bb_power   = 0.0;
    memset(s_sec_band_pow, 0, sizeof(s_sec_band_pow));
    s_sec_peak_pcm   = 0;
    s_sec_clips      = 0;
    s_sec_dsp_sum_us = 0;
    s_sec_dsp_max_us = 0;

    s_min_sec_idx++;
    if (s_min_sec_idx >= SECONDS_PER_MINUTE) {
      if (out_min != nullptr && out_min_ready != nullptr) {
        out_min->epoch_s  = static_cast<int64_t>(time(nullptr));
        out_min->uptime_s = now_s;
        computeMinuteStats(s_bb_1s, &out_min->bb_leq, &out_min->bb_l90);
        out_min->bb_peak  = peakPcmToDb(s_min_peak_pcm);
        out_min->clips    = s_min_clips;

        for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
          computeMinuteStats(s_oct_1s[b], &out_min->oct_leq[b], &out_min->oct_l90[b]);
        }
        *out_min_ready = true;
      }

      // Reset 1-minute accumulators
      s_min_sec_idx  = 0;
      s_min_peak_pcm = 0;
      s_min_clips    = 0;
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
  size_t pos = 0;

  bool ok = false;
  if (has_ts) {
    ok = appendFmt(out_buf, out_buf_size, &pos,
                   "{\"device_id\":\"%s\",\"ts\":\"%s\",\"uptime_s\":%lu,\"unit\":\"dBFS\",",
                   DEVICE_ID,
                   sys_info->ts_iso,
                   static_cast<unsigned long>(rec.uptime_s));
  } else {
    ok = appendFmt(out_buf, out_buf_size, &pos,
                   "{\"device_id\":\"%s\",\"ts\":null,\"uptime_s\":%lu,\"unit\":\"dBFS\",",
                   DEVICE_ID,
                   static_cast<unsigned long>(rec.uptime_s));
  }
  if (!ok) {
    return -1;
  }

  if (!appendFmt(out_buf, out_buf_size, &pos,
                 "\"bb\":{\"leq\":%.1f,\"l90\":%.1f,\"peak\":%.1f,\"clips\":%lu},\"oct_leq\":[",
                 rec.bb_leq,
                 rec.bb_l90,
                 rec.bb_peak,
                 static_cast<unsigned long>(rec.clips))) {
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
