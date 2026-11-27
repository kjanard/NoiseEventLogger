#include <Arduino.h>
#include <Wire.h>
#include <atomic>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <string.h>

#include "config.h"
#include "board_pins.h"
#include "audio_capture.h"
#include "dsp_octave.h"
#include "rtc_time.h"
#include "storage_log.h"
#include "network_sync.h"
#include "display_ui.h"

namespace {

SemaphoreHandle_t s_i2cMutex = nullptr;
QueueHandle_t     s_secQueue = nullptr;
QueueHandle_t     s_minQueue = nullptr;

// Static SRAM hop buffer (4000 B) so AudioDspTask stack stays light
int16_t s_hop_buf[AUDIO_HOP_SAMPLES];

// Static Core 0 UI snapshot to avoid putting large structs on SystemIoTask stack
DisplayUiState_t s_uiState = {};
char             s_uiTimeHm[24] = {};
char             s_uiSdPath[48] = {};

std::atomic<uint32_t> s_i2sReadErrors{0};
std::atomic<uint32_t> s_secQueueDropped{0};
std::atomic<uint32_t> s_minQueueDropped{0};

uint32_t readBatteryMilliVolts() {
  uint32_t sum_mv = 0;
  for (uint8_t i = 0; i < BAT_ADC_SAMPLES; ++i) {
    sum_mv += analogReadMilliVolts(PIN_BAT_ADC);
  }
  const float avg_pin_mv = static_cast<float>(sum_mv) / static_cast<float>(BAT_ADC_SAMPLES);
  return static_cast<uint32_t>(lroundf(avg_pin_mv * BAT_ADC_DIVIDER));
}

uint8_t batteryPercentFromMv(uint32_t vbat_mv) {
  if (vbat_mv <= 3000U) {
    return 0U;
  }
  if (vbat_mv >= 4120U) {
    return 100U;
  }
  return static_cast<uint8_t>(((vbat_mv - 3000U) * 100U + 560U) / 1120U);
}

const char *lookupKnownI2cDevice(uint8_t addr) {
  switch (addr) {
    case I2C_ADDR_ES8311:   return "ES8311 (Audio DAC)";
    case I2C_ADDR_ES7210:   return "ES7210 (Audio ADC)";
    case I2C_ADDR_PCF85063: return "PCF85063 (RTC)";
    case I2C_ADDR_SHTC3:    return "SHTC3 (Temp/RH)";
    default:                return "Unknown";
  }
}

void printBootSystemInfo() {
  Serial.println();
  Serial.println("==================================================================");
  Serial.printf("NoiseEventLogger (%s) — Phase 3: Full System Integration\n", DEVICE_ID);
  Serial.println("Board: Waveshare ESP32-S3-RLCD-4.2");
  Serial.println("==================================================================");
  Serial.printf("[Sys] Chip: %s Rev %u | Cores: %u @ %lu MHz\n",
                ESP.getChipModel(),
                ESP.getChipRevision(),
                ESP.getChipCores(),
                static_cast<unsigned long>(ESP.getCpuFreqMHz()));
  Serial.printf("[Sys] Flash: %lu MB | PSRAM: %lu bytes (Free: %lu bytes)\n",
                static_cast<unsigned long>(ESP.getFlashChipSize() / (1024UL * 1024UL)),
                static_cast<unsigned long>(ESP.getPsramSize()),
                static_cast<unsigned long>(ESP.getFreePsram()));
  const uint32_t vbat_mv = readBatteryMilliVolts();
  Serial.printf("[Sys] Internal SRAM Free: %lu bytes | VBAT: %lu mV (%u%%)\n",
                static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
                static_cast<unsigned long>(vbat_mv),
                static_cast<unsigned>(batteryPercentFromMv(vbat_mv)));
  Serial.printf("[Pins] I2C(SDA=%d,SCL=%d) | I2S(MCLK=%d,BCLK=%d,WS=%d,DIN=%d,PA=%d)\n",
                PIN_I2C_SDA, PIN_I2C_SCL,
                PIN_I2S_MCLK, PIN_I2S_BCLK, PIN_I2S_WS, PIN_I2S_DIN, PIN_PA_CTRL);
  Serial.printf("[Pins] RLCD(SCK=%d,MOSI=%d,DC=%d,CS=%d,RST=%d,TE=%d) | SD_MMC(CLK=%d,CMD=%d,D0=%d)\n",
                PIN_RLCD_SCK, PIN_RLCD_MOSI, PIN_RLCD_DC, PIN_RLCD_CS, PIN_RLCD_RST, PIN_RLCD_TE,
                PIN_SDMMC_CLK, PIN_SDMMC_CMD, PIN_SDMMC_D0);
}

bool scanI2cDevices(bool *out_es7210, bool *out_rtc) {
  bool found_es7210 = false;
  bool found_rtc = false;
  uint8_t count = 0;

  if (s_i2cMutex != nullptr) {
    xSemaphoreTake(s_i2cMutex, portMAX_DELAY);
  }
  Serial.println("[I2C] Scanning bus...");
  for (uint8_t addr = 0x08; addr <= 0x77; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      count++;
      Serial.printf("  ACK @ 0x%02X -> %s\n", addr, lookupKnownI2cDevice(addr));
      if (addr == I2C_ADDR_ES7210) {
        found_es7210 = true;
      } else if (addr == I2C_ADDR_PCF85063) {
        found_rtc = true;
      }
    }
  }
  if (s_i2cMutex != nullptr) {
    xSemaphoreGive(s_i2cMutex);
  }

  Serial.printf("[I2C] Scan complete: %u device(s) | ES7210(0x%02X): %s | PCF85063(0x%02X): %s\n",
                count,
                I2C_ADDR_ES7210, found_es7210 ? "OK" : "MISSING",
                I2C_ADDR_PCF85063, found_rtc ? "OK" : "MISSING");

  if (out_es7210 != nullptr) *out_es7210 = found_es7210;
  if (out_rtc != nullptr) *out_rtc = found_rtc;
  return found_es7210;
}

void serialPrintlnNonBlocking(const char *str) {
  if (str == nullptr || !Serial) {
    return;
  }
  const size_t len = strlen(str);
  if (Serial.availableForWrite() >= static_cast<int>(len + 2U)) {
    Serial.println(str);
  }
}

void printEs7210RegisterSummary() {
  const uint8_t regs[] = { 0x01, 0x02, 0x07, 0x08, 0x11, 0x12, 0x13, 0x43 };
  char line[160];
  int pos = snprintf(line, sizeof(line), "[Audio] ES7210 Regs:");
  for (size_t i = 0; i < sizeof(regs) && pos > 0 && static_cast<size_t>(pos) < sizeof(line); ++i) {
    uint8_t val = 0;
    if (AudioCapture_ReadEs7210Reg(regs[i], &val)) {
      pos += snprintf(line + pos, sizeof(line) - static_cast<size_t>(pos), " R%02X=0x%02X", regs[i], val);
    } else {
      pos += snprintf(line + pos, sizeof(line) - static_cast<size_t>(pos), " R%02X=ERR", regs[i]);
    }
  }
  Serial.println(line);
}

void printBandLabels() {
  char line[256];
  int pos = snprintf(line, sizeof(line), "[Bands] idx 0..23:");
  for (uint8_t b = 0; b < OCTAVE_BAND_COUNT && pos > 0 && static_cast<size_t>(pos) < sizeof(line); ++b) {
    pos += snprintf(line + pos, sizeof(line) - static_cast<size_t>(pos), " %s", OCTAVE_BAND_LABELS[b]);
  }
  if (pos > 0 && static_cast<size_t>(pos) < sizeof(line)) {
    snprintf(line + pos, sizeof(line) - static_cast<size_t>(pos), " Hz");
  }
  Serial.println(line);
}

void refreshUiSnapshotAndDraw(uint32_t vbat_mv) {
  const time_t now_epoch = RtcTime_NowEpoch();
  if (!RtcTime_FormatDisplayHms(now_epoch, s_uiTimeHm, sizeof(s_uiTimeHm))) {
    snprintf(s_uiTimeHm, sizeof(s_uiTimeHm), "1970-01-01 00:00:00");
  }
  snprintf(s_uiSdPath, sizeof(s_uiSdPath), "%s", StorageLog_GetLastFilePath());

  s_uiState.time_hm         = s_uiTimeHm;
  s_uiState.ntp_synced      = RtcTime_IsNtpSynced();
  s_uiState.rtc_ok          = RtcTime_IsRtcOk();
  s_uiState.wifi_configured = NetworkSync_IsWifiConfigured();
  s_uiState.wifi_connected  = NetworkSync_IsWifiConnected();
  s_uiState.rssi            = NetworkSync_GetRssi();
  s_uiState.sd_ok           = StorageLog_IsOk();
  s_uiState.sd_path         = s_uiSdPath;
  s_uiState.vbat_mv         = vbat_mv;
  s_uiState.http_state      = NetworkSync_GetHttpState();
  s_uiState.http_queued     = NetworkSync_GetQueuedCount();
  s_uiState.uptime_s        = static_cast<uint32_t>(millis() / 1000UL);

  DisplayUi_Render(s_uiState);
}

// High-priority real-time audio & FFT task pinned strictly to Core 1
void AudioDspTask(void *arg) {
  (void)arg;
  SecondReport_t sec_report = {};
  MinuteReport_t min_report = {};

  for (;;) {
    int32_t hop_peak_pcm = 0;
    uint32_t hop_clips = 0;

    if (!AudioCapture_ReadMonoSamples(s_hop_buf, AUDIO_HOP_SAMPLES, &hop_peak_pcm, &hop_clips)) {
      s_i2sReadErrors++;
      memset(s_hop_buf, 0, sizeof(s_hop_buf));
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    bool sec_ready = false;
    bool min_ready = false;
    DspOctave_PushHopSamples(s_hop_buf,
                             AUDIO_HOP_SAMPLES,
                             hop_peak_pcm,
                             hop_clips,
                             &sec_report,
                             &sec_ready,
                             &min_report,
                             &min_ready);

    // Strict privacy: wipe raw hop PCM buffer immediately after FFT processing
    memset(s_hop_buf, 0, sizeof(s_hop_buf));

    if (sec_ready && s_secQueue != nullptr) {
      if (xQueueSend(s_secQueue, &sec_report, 0) != pdTRUE) {
        s_secQueueDropped++;
      }
    }
    if (min_ready && s_minQueue != nullptr) {
      if (xQueueSend(s_minQueue, &min_report, 0) != pdTRUE) {
        s_minQueueDropped++;
      }
    }
  }
}

// Non-real-time Core 0 task: RTC/NTP, SD_MMC CSV, Wi-Fi/HTTP, ST7305 RLCD, Serial
void SystemIoTask(void *arg) {
  (void)arg;
  SecondReport_t sec_report = {};
  MinuteReport_t min_report = {};
  char sec_line[384];
  char json_buf[1024];
  char ts_iso[32];
  char date_ymd[16];

  uint32_t last_lcd_ms = millis();

  for (;;) {
    NetworkSync_Poll();
    RtcTime_Poll();

    bool need_lcd_refresh = false;

    for (uint8_t burst = 0;
         burst < 4 && s_secQueue != nullptr && xQueueReceive(s_secQueue, &sec_report, 0) == pdTRUE;
         ++burst) {
      s_uiState.has_sec = true;
      s_uiState.sec     = sec_report;
      if (s_uiState.warmup_sec < SECONDS_PER_MINUTE) {
        s_uiState.warmup_sec++;
      }
      need_lcd_refresh  = true;

#if VERBOSE_1S_SERIAL
      int pos = snprintf(
          sec_line,
          sizeof(sec_line),
          "[1s] t=%lus | bb=%.1f pk=%.1f clips=%lu | oct=[",
          static_cast<unsigned long>(sec_report.uptime_s),
          sec_report.bb_leq_1s,
          sec_report.bb_peak_1s,
          static_cast<unsigned long>(sec_report.clips_1s));
      for (uint8_t b = 0; b < OCTAVE_BAND_COUNT && pos > 0 && static_cast<size_t>(pos) < sizeof(sec_line); ++b) {
        pos += snprintf(sec_line + pos,
                        sizeof(sec_line) - static_cast<size_t>(pos),
                        "%s%.1f",
                        (b == 0) ? "" : " ",
                        sec_report.oct_1s[b]);
      }
      if (pos > 0 && static_cast<size_t>(pos) < sizeof(sec_line)) {
        snprintf(sec_line + pos,
                 sizeof(sec_line) - static_cast<size_t>(pos),
                 "] | fft=%u (%lu/%luus) | sram=%lu",
                 static_cast<unsigned>(sec_report.fft_frames),
                 static_cast<unsigned long>(sec_report.dsp_avg_us),
                 static_cast<unsigned long>(sec_report.dsp_max_us),
                 static_cast<unsigned long>(sec_report.free_internal_sram));
      }
      serialPrintlnNonBlocking(sec_line);
#endif
    }

    while (s_minQueue != nullptr && xQueueReceive(s_minQueue, &min_report, 0) == pdTRUE) {
      const time_t rec_epoch = (min_report.epoch_s > 0) ? static_cast<time_t>(min_report.epoch_s) : RtcTime_NowEpoch();
      if (!RtcTime_FormatIso8601(rec_epoch, ts_iso, sizeof(ts_iso))) {
        ts_iso[0] = '\0';
      }
      if (!RtcTime_FormatDateYmd(rec_epoch, date_ymd, sizeof(date_ymd))) {
        snprintf(date_ymd, sizeof(date_ymd), "unsynced");
      }

      const uint32_t vbat_mv = readBatteryMilliVolts();
      const int rssi = NetworkSync_GetRssi();

      const bool sd_ok = StorageLog_AppendMinute(min_report, ts_iso, date_ymd, vbat_mv, rssi);

      MinuteSysInfo_t sys_info = {};
      sys_info.ts_iso  = ts_iso;
      sys_info.vbat_mv = vbat_mv;
      sys_info.rssi    = rssi;
      sys_info.sd_ok   = sd_ok;

      if (DspOctave_FormatMinuteJson(min_report, &sys_info, json_buf, sizeof(json_buf)) > 0) {
        NetworkSync_EnqueueJson(json_buf);
        serialPrintlnNonBlocking(json_buf);
      } else {
        serialPrintlnNonBlocking("[ERROR] Minute JSON buffer overflow");
      }

      s_uiState.has_min    = true;
      s_uiState.min        = min_report;
      s_uiState.minute_count++;
      s_uiState.warmup_sec = 0;
      need_lcd_refresh     = true;
    }

    const uint32_t now_ms = millis();
    if (need_lcd_refresh || (now_ms - last_lcd_ms) >= RLCD_REFRESH_INTERVAL_MS) {
      last_lcd_ms = now_ms;
      refreshUiSnapshotAndDraw(readBatteryMilliVolts());
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

}  // namespace

void setup() {
  Serial.setTxBufferSize(2048);
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.setTxTimeoutMs(2);  // Prevent 2000ms HWCDC backpressure stalls when no Serial Monitor is open
  delay(200);                // Cold power-on 3.3V rail stabilization for ST7305 / ES7210 / PCF85063

  // Configure battery divider ADC (GPIO 4, 12 dB attenuation, 3.0x divider)
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BAT_ADC, ADC_11db);

  printBootSystemInfo();

  // Keep speaker amplifier powered down during acoustic logging
  pinMode(PIN_PA_CTRL, OUTPUT);
  digitalWrite(PIN_PA_CTRL, LOW);

  s_i2cMutex = xSemaphoreCreateMutex();

  // Explicit I2C pins required to prevent pin collisions with I2S
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_BUS_FREQ_HZ);
  bool found_es7210 = false;
  bool found_rtc = false;
  scanI2cDevices(&found_es7210, &found_rtc);
  if (!found_es7210) {
    Serial.println("[ERROR] ES7210 did not ACK on I2C! Halting.");
    while (true) {
      delay(1000);
    }
  }

  // Phase 3 Core 0 peripherals: RTC, SD_MMC 1-bit, ST7305 RLCD
  RtcTime_Init(Wire, s_i2cMutex, __DATE__, __TIME__);
  StorageLog_Init();
  DisplayUi_Init();
  refreshUiSnapshotAndDraw(readBatteryMilliVolts());  // Render immediately on cold boot
  NetworkSync_Init();

  // Phase 2 DSP initialization & self-test
  if (!DspOctave_Init()) {
    Serial.println("[ERROR] DspOctave_Init failed! Halting.");
    while (true) {
      delay(1000);
    }
  }

#if RUN_DSP_SELF_TEST
  OctaveSelfTestResult_t self_test = {};
  const bool ok = DspOctave_RunSelfTest(&self_test);
  Serial.printf("[SelfTest] 1kHz: td=%.2f spec=%.2f band12=%.2f dBFS | 4kHz: td=%.2f spec=%.2f band18=%.2f dBFS | fft=%lu us -> %s\n",
                self_test.tone1k_td_dbfs,
                self_test.tone1k_spec_dbfs,
                self_test.tone1k_band12_dbfs,
                self_test.tone4k_td_dbfs,
                self_test.tone4k_spec_dbfs,
                self_test.tone4k_band18_dbfs,
                static_cast<unsigned long>(self_test.single_fft_us),
                ok ? "PASS" : "FAIL");
  if (!ok) {
    Serial.println("[ERROR] DSP Self-Test failed! Halting.");
    while (true) {
      delay(1000);
    }
  }
#endif

  // Phase 1 Audio capture initialization (I2S TDM + ES7210)
  if (!AudioCapture_Init(Wire, MIC_GAIN_DB)) {
    Serial.println("[ERROR] AudioCapture_Init failed! Halting.");
    while (true) {
      delay(1000);
    }
  }

  Serial.printf("[Audio] I2S RX @ %lu Hz, %u-bit | FFT=%u hop=%u (16/s) | Fixed Gain = %.1f dB (AGC=OFF)\n",
                static_cast<unsigned long>(AUDIO_SAMPLE_RATE_HZ),
                AUDIO_BITS_PER_SAMPLE,
                AUDIO_FFT_SIZE,
                AUDIO_HOP_SAMPLES,
                MIC_GAIN_DB);
  printEs7210RegisterSummary();
  printBandLabels();
  Serial.println("[Privacy] Raw PCM DMA & hop buffers wiped immediately after DSP processing.");

  // Draw initial boot status screen before RTOS tasks start
  refreshUiSnapshotAndDraw(readBatteryMilliVolts());

  s_secQueue = xQueueCreate(4, sizeof(SecondReport_t));
  s_minQueue = xQueueCreate(MIN_QUEUE_DEPTH, sizeof(MinuteReport_t));
  if (s_secQueue == nullptr || s_minQueue == nullptr) {
    Serial.println("[ERROR] Failed to create FreeRTOS queues! Halting.");
    while (true) {
      delay(1000);
    }
  }

  xTaskCreatePinnedToCore(AudioDspTask,
                          "AudioDspTask",
                          AUDIO_DSP_TASK_STACK,
                          nullptr,
                          AUDIO_DSP_TASK_PRIO,
                          nullptr,
                          AUDIO_DSP_TASK_CORE);

  xTaskCreatePinnedToCore(SystemIoTask,
                          "SystemIoTask",
                          SYSTEM_IO_TASK_STACK,
                          nullptr,
                          SYSTEM_IO_TASK_PRIO,
                          nullptr,
                          SYSTEM_IO_TASK_CORE);

  Serial.printf("[RTOS] AudioDspTask->Core%d (prio %u) | SystemIoTask->Core%d (prio %u) | MinQ=%u\n\n",
                AUDIO_DSP_TASK_CORE, static_cast<unsigned>(AUDIO_DSP_TASK_PRIO),
                SYSTEM_IO_TASK_CORE, static_cast<unsigned>(SYSTEM_IO_TASK_PRIO),
                static_cast<unsigned>(MIN_QUEUE_DEPTH));
}

void loop() {
  static uint32_t last_health_check_ms = 0;
  const uint32_t now_ms = millis();
  if (now_ms - last_health_check_ms >= 10000UL) {
    last_health_check_ms = now_ms;
    const uint32_t i2s_err  = s_i2sReadErrors.load();
    const uint32_t sec_drop = s_secQueueDropped.load();
    const uint32_t min_drop = s_minQueueDropped.load();
    if (i2s_err > 0 || sec_drop > 0 || min_drop > 0) {
      Serial.printf("[Warn] i2s_err=%lu sec_drop=%lu min_drop=%lu\n",
                    static_cast<unsigned long>(i2s_err),
                    static_cast<unsigned long>(sec_drop),
                    static_cast<unsigned long>(min_drop));
    }
  }
  vTaskDelay(pdMS_TO_TICKS(1000));
}
