#include "audio_capture.h"
#include "board_pins.h"
#include "calibration.h"
#include "config.h"

#include <math.h>
#include <string.h>
#include <driver/i2s_tdm.h>

// ============================================================================
// ES7210 Register Definitions
// Reference: reference/examples/arduino/examples/07_Audio_Test/src/ExternLib/
//            esp_codec_dev/device/es7210/es7210.c
// For fs = 48000 Hz, MCLK = 256 * 48000 = 12,288,000 Hz (0 ppm from PLL_F160M):
//   es7210.c line 98: {12288000, 48000, 0x00, 0x01, 0x01, 0x01, 0x20, ...}
//   Slave mode REG02 = 0xC1, REG07 = 0x20 (es7210.c lines 443, 449-451)
// ============================================================================
namespace {

constexpr uint8_t ES7210_RESET_REG00           = 0x00;
constexpr uint8_t ES7210_CLOCK_OFF_REG01       = 0x01;
constexpr uint8_t ES7210_MAINCLK_REG02         = 0x02;
constexpr uint8_t ES7210_POWER_DOWN_REG06      = 0x06;
constexpr uint8_t ES7210_OSR_REG07             = 0x07;
constexpr uint8_t ES7210_MODE_CONFIG_REG08     = 0x08;
constexpr uint8_t ES7210_TIME_CONTROL0_REG09   = 0x09;
constexpr uint8_t ES7210_TIME_CONTROL1_REG0A   = 0x0A;
constexpr uint8_t ES7210_SDP_INTERFACE1_REG11  = 0x11;
constexpr uint8_t ES7210_SDP_INTERFACE2_REG12  = 0x12;
constexpr uint8_t ES7210_ADC_AUTOMUTE_REG13    = 0x13;
constexpr uint8_t ES7210_ADC34_MUTERANGE_REG14 = 0x14;
constexpr uint8_t ES7210_ADC12_MUTERANGE_REG15 = 0x15;
constexpr uint8_t ES7210_ADC34_HPF2_REG20      = 0x20;
constexpr uint8_t ES7210_ADC34_HPF1_REG21      = 0x21;
constexpr uint8_t ES7210_ADC12_HPF1_REG22      = 0x22;
constexpr uint8_t ES7210_ADC12_HPF2_REG23      = 0x23;
constexpr uint8_t ES7210_ANALOG_REG40          = 0x40;
constexpr uint8_t ES7210_MIC12_BIAS_REG41      = 0x41;
constexpr uint8_t ES7210_MIC34_BIAS_REG42      = 0x42;
constexpr uint8_t ES7210_MIC1_GAIN_REG43       = 0x43;
constexpr uint8_t ES7210_MIC2_GAIN_REG44       = 0x44;
constexpr uint8_t ES7210_MIC3_GAIN_REG45       = 0x45;
constexpr uint8_t ES7210_MIC4_GAIN_REG46       = 0x46;
constexpr uint8_t ES7210_MIC1_POWER_REG47      = 0x47;
constexpr uint8_t ES7210_MIC2_POWER_REG48      = 0x48;
constexpr uint8_t ES7210_MIC3_POWER_REG49      = 0x49;
constexpr uint8_t ES7210_MIC4_POWER_REG4A      = 0x4A;
constexpr uint8_t ES7210_MIC12_POWER_REG4B     = 0x4B;
constexpr uint8_t ES7210_MIC34_POWER_REG4C     = 0x4C;

constexpr float   PCM_FULL_SCALE               = 32768.0f;

TwoWire          *s_wire               = nullptr;
i2s_chan_handle_t s_rx_chan            = nullptr;
bool              s_inited             = false;
float             s_configured_gain_db = ES7210_MIC_GAIN_DB;

// Map dB value to ES7210 4-bit PGA gain code (matches es7210_reg.h gain_value enum):
// 0..10 => 0..30 dB (3 dB steps), 11 => 33.0 dB, 12 => 34.5 dB, 13 => 36.0 dB, 14 => 37.5 dB
uint8_t es7210GainCodeFromDb(float db) {
  if (db < 1.5f) {
    return 0;  // 0 dB
  }
  if (db < 31.5f) {
    return static_cast<uint8_t>((db + 1.5f) / 3.0f);  // 1..10 => 3..30 dB
  }
  if (db < 33.75f) {
    return 11;  // 33.0 dB (GAIN_33DB)
  }
  if (db < 35.25f) {
    return 12;  // 34.5 dB (GAIN_34_5DB)
  }
  if (db < 36.75f) {
    return 13;  // 36.0 dB (GAIN_36DB)
  }
  return 14;    // 37.5 dB (GAIN_37_5DB)
}

float es7210DbFromGainCode(uint8_t code) {
  if (code <= 11) {
    return static_cast<float>(code) * 3.0f;
  }
  if (code == 12) return 34.5f;
  if (code == 13) return 36.0f;
  return 37.5f;
}

bool es7210WriteReg(uint8_t reg, uint8_t val) {
  if (s_wire == nullptr) {
    return false;
  }
  s_wire->beginTransmission(I2C_ADDR_ES7210);
  s_wire->write(reg);
  s_wire->write(val);
  return (s_wire->endTransmission() == 0);
}

bool es7210ReadReg(uint8_t reg, uint8_t *out_val) {
  if (s_wire == nullptr || out_val == nullptr) {
    return false;
  }
  s_wire->beginTransmission(I2C_ADDR_ES7210);
  s_wire->write(reg);
  if (s_wire->endTransmission(false) != 0) {
    return false;
  }
  if (s_wire->requestFrom(I2C_ADDR_ES7210, static_cast<uint8_t>(1)) != 1) {
    return false;
  }
  *out_val = static_cast<uint8_t>(s_wire->read());
  return true;
}

bool es7210UpdateRegBit(uint8_t reg, uint8_t update_bits, uint8_t data) {
  uint8_t regv = 0;
  if (!es7210ReadReg(reg, &regv)) {
    return false;
  }
  regv = static_cast<uint8_t>((regv & (~update_bits)) | (update_bits & data));
  return es7210WriteReg(reg, regv);
}

bool initI2sTdmRx() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;
  chan_cfg.dma_desc_num = AUDIO_DMA_DESC_NUM;
  chan_cfg.dma_frame_num = AUDIO_DMA_FRAME_NUM;

  esp_err_t err = i2s_new_channel(&chan_cfg, nullptr, &s_rx_chan);
  if (err != ESP_OK || s_rx_chan == nullptr) {
    Serial.printf("[Audio] i2s_new_channel failed: %d\n", static_cast<int>(err));
    return false;
  }

  const i2s_tdm_slot_mask_t slot_mask = static_cast<i2s_tdm_slot_mask_t>(
      I2S_TDM_SLOT0 | I2S_TDM_SLOT1 | I2S_TDM_SLOT2 | I2S_TDM_SLOT3);

  i2s_tdm_config_t tdm_cfg = {};
  tdm_cfg.clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE_HZ);
  tdm_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

  tdm_cfg.slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO, slot_mask);
  tdm_cfg.slot_cfg.total_slot = AUDIO_TDM_SLOTS;

  tdm_cfg.gpio_cfg.mclk = static_cast<gpio_num_t>(PIN_I2S_MCLK);
  tdm_cfg.gpio_cfg.bclk = static_cast<gpio_num_t>(PIN_I2S_BCLK);
  tdm_cfg.gpio_cfg.ws   = static_cast<gpio_num_t>(PIN_I2S_WS);
  tdm_cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
  tdm_cfg.gpio_cfg.din  = static_cast<gpio_num_t>(PIN_I2S_DIN);
  tdm_cfg.gpio_cfg.invert_flags.mclk_inv = false;
  tdm_cfg.gpio_cfg.invert_flags.bclk_inv = false;
  tdm_cfg.gpio_cfg.invert_flags.ws_inv   = false;

  err = i2s_channel_init_tdm_mode(s_rx_chan, &tdm_cfg);
  if (err != ESP_OK) {
    Serial.printf("[Audio] i2s_channel_init_tdm_mode failed: %d\n", static_cast<int>(err));
    i2s_del_channel(s_rx_chan);
    s_rx_chan = nullptr;
    return false;
  }

  err = i2s_channel_enable(s_rx_chan);
  if (err != ESP_OK) {
    Serial.printf("[Audio] i2s_channel_enable failed: %d\n", static_cast<int>(err));
    i2s_del_channel(s_rx_chan);
    s_rx_chan = nullptr;
    return false;
  }

  // Allow 12.288 MHz MCLK / BCLK / WS to stabilize before ES7210 I2C writes
  delay(10);
  return true;
}

bool initEs7210FixedGain(float fixed_gain_db) {
  s_wire->beginTransmission(I2C_ADDR_ES7210);
  if (s_wire->endTransmission() != 0) {
    Serial.printf("[Audio] ES7210 not responding at I2C 0x%02X\n", I2C_ADDR_ES7210);
    return false;
  }

  const uint8_t gain_code = es7210GainCodeFromDb(fixed_gain_db);
  const uint8_t gain_reg_val = static_cast<uint8_t>(0x10U | (gain_code & 0x0FU));
  s_configured_gain_db = es7210DbFromGainCode(gain_code);

  bool ok = true;
  ok &= es7210WriteReg(ES7210_RESET_REG00, 0xFF);
  delay(10);
  ok &= es7210WriteReg(ES7210_RESET_REG00, 0x41);
  ok &= es7210WriteReg(ES7210_CLOCK_OFF_REG01, 0x3F);
  ok &= es7210WriteReg(ES7210_TIME_CONTROL0_REG09, 0x30);
  ok &= es7210WriteReg(ES7210_TIME_CONTROL1_REG0A, 0x30);

  // Enable built-in DC-blocking HPF (reference es7210.c lines 420-423)
  ok &= es7210WriteReg(ES7210_ADC12_HPF2_REG23, 0x2A);
  ok &= es7210WriteReg(ES7210_ADC12_HPF1_REG22, 0x0A);
  ok &= es7210WriteReg(ES7210_ADC34_HPF2_REG20, 0x0A);
  ok &= es7210WriteReg(ES7210_ADC34_HPF1_REG21, 0x2A);

  // Slave mode preserving bit 4 (LRCK_RATE_MODE = 0x10), matching es7210.c line 443
  ok &= es7210UpdateRegBit(ES7210_MODE_CONFIG_REG08, 0x01, 0x00);
  ok &= es7210WriteReg(ES7210_ANALOG_REG40, 0x43);
  ok &= es7210WriteReg(ES7210_MIC12_BIAS_REG41, 0x70);
  ok &= es7210WriteReg(ES7210_MIC34_BIAS_REG42, 0x70);
  // REG07 = 0x20, REG02 = 0xC1 match es7210.c line 98 for 48 kHz @ 12.288 MHz MCLK
  ok &= es7210WriteReg(ES7210_OSR_REG07, 0x20);
  ok &= es7210WriteReg(ES7210_MAINCLK_REG02, 0xC1);

  // Explicitly disable automute / dynamic gain control
  ok &= es7210WriteReg(ES7210_ADC_AUTOMUTE_REG13, 0x00);
  ok &= es7210UpdateRegBit(ES7210_ADC34_MUTERANGE_REG14, 0x03, 0x00);
  ok &= es7210UpdateRegBit(ES7210_ADC12_MUTERANGE_REG15, 0x03, 0x00);

  // 16-bit Philips I2S + 4-channel TDM mode on SDOUT1 (es7210.c lines 230, 243, 276)
  ok &= es7210UpdateRegBit(ES7210_SDP_INTERFACE1_REG11, 0xE3, 0x60);
  ok &= es7210WriteReg(ES7210_SDP_INTERFACE2_REG12, 0x02);

  // Power up ADC & MIC PGAs and lock fixed manual gain
  ok &= es7210WriteReg(ES7210_POWER_DOWN_REG06, 0x00);
  ok &= es7210WriteReg(ES7210_ANALOG_REG40, 0x43);
  ok &= es7210WriteReg(ES7210_MIC1_POWER_REG47, 0x08);
  ok &= es7210WriteReg(ES7210_MIC2_POWER_REG48, 0x08);
  ok &= es7210WriteReg(ES7210_MIC3_POWER_REG49, 0x08);
  ok &= es7210WriteReg(ES7210_MIC4_POWER_REG4A, 0x08);
  ok &= es7210WriteReg(ES7210_MIC12_POWER_REG4B, 0x00);
  ok &= es7210WriteReg(ES7210_MIC34_POWER_REG4C, 0x00);

  ok &= es7210UpdateRegBit(ES7210_MIC1_GAIN_REG43, 0x1F, gain_reg_val);
  ok &= es7210UpdateRegBit(ES7210_MIC2_GAIN_REG44, 0x1F, gain_reg_val);
  ok &= es7210UpdateRegBit(ES7210_MIC3_GAIN_REG45, 0x1F, gain_reg_val);
  ok &= es7210UpdateRegBit(ES7210_MIC4_GAIN_REG46, 0x1F, gain_reg_val);

  // Enable ADC1-4 clocks matching es7210.c
  ok &= es7210UpdateRegBit(ES7210_CLOCK_OFF_REG01, 0x1F, 0x00);
  ok &= es7210WriteReg(ES7210_ANALOG_REG40, 0x43);
  ok &= es7210WriteReg(ES7210_RESET_REG00, 0x71);
  ok &= es7210WriteReg(ES7210_RESET_REG00, 0x41);

  if (!ok) {
    Serial.println("[Audio] ES7210 I2C register configuration failed");
    return false;
  }

  return true;
}

void discardStartupTransient() {
  int16_t scratch_buf[AUDIO_DMA_FRAME_NUM * AUDIO_TDM_SLOTS];
  const uint32_t frames_to_discard = AUDIO_SAMPLE_RATE_HZ / 10;  // ~100 ms
  uint32_t discarded = 0;

  while (discarded < frames_to_discard) {
    size_t bytes_read = 0;
    esp_err_t err = i2s_channel_read(s_rx_chan, scratch_buf, sizeof(scratch_buf), &bytes_read, pdMS_TO_TICKS(500));
    if (err != ESP_OK || bytes_read == 0) {
      break;
    }
    discarded += static_cast<uint32_t>(bytes_read / (AUDIO_TDM_SLOTS * sizeof(int16_t)));
    // Strict privacy: scrub transient buffer immediately
    memset(scratch_buf, 0, sizeof(scratch_buf));
  }
}

void measureStartupTdmSlots() {
  int16_t tdm_buf[AUDIO_DMA_FRAME_NUM * AUDIO_TDM_SLOTS];
  const uint32_t target_frames = AUDIO_SAMPLE_RATE_HZ / 10;  // 100 ms diagnostic window
  uint32_t total_frames = 0;
  double sum_x[AUDIO_TDM_SLOTS] = {};
  double sum_x2[AUDIO_TDM_SLOTS] = {};

  while (total_frames < target_frames) {
    size_t bytes_read = 0;
    esp_err_t err = i2s_channel_read(s_rx_chan, tdm_buf, sizeof(tdm_buf), &bytes_read, pdMS_TO_TICKS(500));
    if (err != ESP_OK || bytes_read == 0) {
      memset(tdm_buf, 0, sizeof(tdm_buf));
      return;
    }

    const uint32_t frames_read = static_cast<uint32_t>(bytes_read / (AUDIO_TDM_SLOTS * sizeof(int16_t)));
    for (uint32_t i = 0; i < frames_read; ++i) {
      for (uint8_t s = 0; s < AUDIO_TDM_SLOTS; ++s) {
        const double x = static_cast<double>(tdm_buf[(i * AUDIO_TDM_SLOTS) + s]);
        sum_x[s] += x;
        sum_x2[s] += x * x;
      }
    }
    memset(tdm_buf, 0, sizeof(tdm_buf));
    total_frames += frames_read;
  }

  float slot_lvl[AUDIO_TDM_SLOTS] = {};
  const float cal_ofs = Calibration_GetOffsetDb();
  const double inv_n = 1.0 / static_cast<double>(total_frames);
  for (uint8_t s = 0; s < AUDIO_TDM_SLOTS; ++s) {
    const double mean = sum_x[s] * inv_n;
    double var = (sum_x2[s] * inv_n) - (mean * mean);
    if (var < 0.0) {
      var = 0.0;
    }
    const float rms_norm = static_cast<float>(sqrt(var)) / PCM_FULL_SCALE;
    const float power = rms_norm * rms_norm;
    slot_lvl[s] = 10.0f * log10f(fmaxf(power, DBFS_POWER_EPSILON)) + cal_ofs;
  }

  Serial.printf("[Audio] TDM 4-Slot Floor (100ms): S0(MIC1)=%.1f S1(MIC3)=%.1f S2(MIC2)=%.1f S3(MIC4)=%.1f %s\n",
                slot_lvl[0], slot_lvl[1], slot_lvl[2], slot_lvl[3], Calibration_GetUnitLabel());
}

}  // namespace

bool AudioCapture_Init(TwoWire &wire, float fixed_gain_db) {
  if (s_inited) {
    return true;
  }

  s_wire = &wire;

  if (!initI2sTdmRx()) {
    return false;
  }

  if (!initEs7210FixedGain(fixed_gain_db)) {
    i2s_channel_disable(s_rx_chan);
    i2s_del_channel(s_rx_chan);
    s_rx_chan = nullptr;
    return false;
  }

  discardStartupTransient();
  measureStartupTdmSlots();
  s_inited = true;
  return true;
}

bool AudioCapture_CaptureOneSecond(AudioMetrics1s_t *out_metrics) {
  if (!s_inited || s_rx_chan == nullptr || out_metrics == nullptr) {
    return false;
  }

  int16_t tdm_buf[AUDIO_DMA_FRAME_NUM * AUDIO_TDM_SLOTS];
  const uint32_t target_samples = AUDIO_SAMPLE_RATE_HZ;
  uint32_t total_samples = 0;
  double sum_x = 0.0;
  double sum_x2 = 0.0;
  int32_t max_abs_pcm = 0;
  uint32_t clip_count = 0;

  while (total_samples < target_samples) {
    const uint32_t remaining = target_samples - total_samples;
    const uint32_t frames_to_read = (remaining < AUDIO_DMA_FRAME_NUM) ? remaining : AUDIO_DMA_FRAME_NUM;
    const size_t bytes_requested = frames_to_read * AUDIO_TDM_SLOTS * sizeof(int16_t);
    size_t bytes_read = 0;

    esp_err_t err = i2s_channel_read(s_rx_chan, tdm_buf, bytes_requested, &bytes_read, pdMS_TO_TICKS(1000));
    if (err != ESP_OK || bytes_read == 0) {
      memset(tdm_buf, 0, sizeof(tdm_buf));
      return false;
    }

    const uint32_t frames_read = static_cast<uint32_t>(bytes_read / (AUDIO_TDM_SLOTS * sizeof(int16_t)));
    for (uint32_t i = 0; i < frames_read; ++i) {
      const int32_t raw_sample = static_cast<int32_t>(tdm_buf[(i * AUDIO_TDM_SLOTS) + AUDIO_PRIMARY_MIC_CHANNEL]);
      const int32_t abs_raw = (raw_sample < 0) ? -raw_sample : raw_sample;

      if (abs_raw > max_abs_pcm) {
        max_abs_pcm = abs_raw;
      }
      if (abs_raw >= AUDIO_CLIP_THRESHOLD_PCM) {
        clip_count++;
      }

      const double x = static_cast<double>(raw_sample);
      sum_x += x;
      sum_x2 += x * x;
    }

    memset(tdm_buf, 0, sizeof(tdm_buf));
    total_samples += frames_read;
  }

  const float cal_ofs = Calibration_GetOffsetDb();
  const double inv_n = 1.0 / static_cast<double>(total_samples);
  const double mean_pcm = sum_x * inv_n;
  double var_pcm = (sum_x2 * inv_n) - (mean_pcm * mean_pcm);
  if (var_pcm < 0.0) {
    var_pcm = 0.0;
  }

  const float rms_pcm = static_cast<float>(sqrt(var_pcm));
  const float rms_norm = rms_pcm / PCM_FULL_SCALE;
  const float rms_power = rms_norm * rms_norm;
  const float rms_dbfs = 10.0f * log10f(fmaxf(rms_power, DBFS_POWER_EPSILON)) + cal_ofs;

  const float peak_norm = static_cast<float>(max_abs_pcm) / PCM_FULL_SCALE;
  const float peak_power = peak_norm * peak_norm;
  const float peak_dbfs = 10.0f * log10f(fmaxf(peak_power, DBFS_POWER_EPSILON)) + cal_ofs;

  out_metrics->sample_count = total_samples;
  out_metrics->dc_mean_pcm  = static_cast<float>(mean_pcm);
  out_metrics->dc_mean_norm = static_cast<float>(mean_pcm / static_cast<double>(PCM_FULL_SCALE));
  out_metrics->rms_pcm      = rms_pcm;
  out_metrics->rms_norm     = rms_norm;
  out_metrics->rms_dbfs     = rms_dbfs;
  out_metrics->peak_pcm     = max_abs_pcm;
  out_metrics->peak_norm    = peak_norm;
  out_metrics->peak_dbfs    = peak_dbfs;
  out_metrics->clip_count   = clip_count;
  out_metrics->clipped      = (clip_count > 0);

  return true;
}

bool AudioCapture_ReadMonoSamples(int16_t *mono_out,
                                  size_t sample_count,
                                  int32_t *inout_peak_pcm,
                                  uint32_t *inout_clip_count) {
  if (!s_inited || s_rx_chan == nullptr || mono_out == nullptr) {
    return false;
  }

  int16_t tdm_buf[AUDIO_DMA_FRAME_NUM * AUDIO_TDM_SLOTS];
  size_t collected = 0;

  while (collected < sample_count) {
    const size_t remaining = sample_count - collected;
    const size_t frames_to_read = (remaining < AUDIO_DMA_FRAME_NUM) ? remaining : AUDIO_DMA_FRAME_NUM;
    const size_t bytes_requested = frames_to_read * AUDIO_TDM_SLOTS * sizeof(int16_t);
    size_t bytes_read = 0;

    esp_err_t err = i2s_channel_read(s_rx_chan, tdm_buf, bytes_requested, &bytes_read, pdMS_TO_TICKS(1000));
    if (err != ESP_OK || bytes_read == 0) {
      memset(tdm_buf, 0, sizeof(tdm_buf));
      return false;
    }

    const size_t frames_read = bytes_read / (AUDIO_TDM_SLOTS * sizeof(int16_t));
    for (size_t i = 0; i < frames_read; ++i) {
      const int16_t sample = tdm_buf[(i * AUDIO_TDM_SLOTS) + AUDIO_PRIMARY_MIC_CHANNEL];
      mono_out[collected + i] = sample;

      const int32_t raw_i32 = static_cast<int32_t>(sample);
      const int32_t abs_raw = (raw_i32 < 0) ? -raw_i32 : raw_i32;
      if (inout_peak_pcm != nullptr && abs_raw > *inout_peak_pcm) {
        *inout_peak_pcm = abs_raw;
      }
      if (inout_clip_count != nullptr && abs_raw >= AUDIO_CLIP_THRESHOLD_PCM) {
        (*inout_clip_count)++;
      }
    }

    // Strict privacy: wipe raw TDM DMA chunk immediately after extraction
    memset(tdm_buf, 0, sizeof(tdm_buf));
    collected += frames_read;
  }

  return true;
}

bool AudioCapture_ReadEs7210Reg(uint8_t reg_addr, uint8_t *out_val) {
  return es7210ReadReg(reg_addr, out_val);
}

float AudioCapture_GetConfiguredGainDb() {
  return s_configured_gain_db;
}

void AudioCapture_PowerDown(SemaphoreHandle_t i2c_mutex) {
  // Match reference es7210_stop() (es7210.c lines 313-325)
  bool locked = true;
  if (i2c_mutex != nullptr) {
    locked = (xSemaphoreTake(i2c_mutex, pdMS_TO_TICKS(200)) == pdTRUE);
  }
  if (locked && s_wire != nullptr) {
    es7210WriteReg(ES7210_MIC1_POWER_REG47, 0xFF);
    es7210WriteReg(ES7210_MIC2_POWER_REG48, 0xFF);
    es7210WriteReg(ES7210_MIC3_POWER_REG49, 0xFF);
    es7210WriteReg(ES7210_MIC4_POWER_REG4A, 0xFF);
    es7210WriteReg(ES7210_MIC12_POWER_REG4B, 0xFF);
    es7210WriteReg(ES7210_MIC34_POWER_REG4C, 0xFF);
    es7210WriteReg(ES7210_ANALOG_REG40, 0xC0);
    es7210WriteReg(ES7210_CLOCK_OFF_REG01, 0x7F);
    es7210WriteReg(ES7210_POWER_DOWN_REG06, 0x07);
  }
  if (i2c_mutex != nullptr && locked) {
    xSemaphoreGive(i2c_mutex);
  }

  if (s_rx_chan != nullptr) {
    i2s_channel_disable(s_rx_chan);
    i2s_del_channel(s_rx_chan);
    s_rx_chan = nullptr;
  }
  s_inited = false;
}
