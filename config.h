#pragma once

#include <Arduino.h>

// ============================================================================
// Noise Event Logger — Phase 4 Firmware Configuration (SPEC.md)
// ============================================================================

static constexpr const char *DEVICE_ID = "rlcd-noise-01";
static constexpr uint32_t SERIAL_BAUD_RATE = 115200;

// --- Audio & DSP Parameters (Phase 4: 48 kHz / 26-Band 1/3-Octave) ---
// fs = 48000 Hz => Nyquist = 24000 Hz (> 22627.4 Hz upper edge of 20 kHz
// 1/3-octave band) and MCLK = 256 * 48000 = 12.288 MHz exact from PLL_F160M.
static constexpr uint32_t AUDIO_SAMPLE_RATE_HZ = 48000;
static constexpr uint8_t AUDIO_BITS_PER_SAMPLE = 16;
static constexpr uint16_t AUDIO_FFT_SIZE = 4096; // df = 11.71875 Hz @ 48 kHz
static constexpr uint16_t AUDIO_HOP_SAMPLES =
    3000; // 16 hops/s => exact 1.000 s / 48000 samples (62.5 ms hop)

// I2S TDM DMA buffer geometry: 12 x 384 4-slot frames = 96.0 ms DMA ring @ 48 kHz
static constexpr uint32_t AUDIO_DMA_DESC_NUM = 12;
static constexpr uint32_t AUDIO_DMA_FRAME_NUM = 384;
static constexpr uint8_t AUDIO_TDM_SLOTS =
    4; // Slot0=MIC1, Slot1=MIC3, Slot2=MIC2, Slot3=MIC4

// ES7210 fixed manual gain (+33 dB => code 11 / GAIN_33DB; REG 0x43..0x46 = 0x1B).
// +9 dB above initial 24 dB baseline to pull 4-20 kHz L90 out of ADC self-noise
// while preserving headroom for > -9 dBFS outdoor peaks. AGC/AEC/automute = OFF.
static constexpr float ES7210_MIC_GAIN_DB = 33.0f;
static constexpr float MIC_GAIN_DB = ES7210_MIC_GAIN_DB;
static constexpr float ANECHOIC_REF_MIC_GAIN_DB = 24.0f;
static constexpr uint8_t AUDIO_PRIMARY_MIC_CHANNEL = 0; // Left I2S slot = MIC1

// Clipping / Overload threshold: >= 99.5% of 16-bit signed full-scale (32768)
static constexpr int32_t AUDIO_CLIP_THRESHOLD_PCM = 32600;

// Default acoustic calibration offset (overridden at runtime by NVS calibration)
static constexpr float DEFAULT_CALIBRATION_OFFSET_DB = 0.0f; // 0.0 = raw dBFS
static constexpr float DBFS_POWER_EPSILON = 1e-12f;          // -120.0 dBFS floor
static constexpr float UNDER_RANGE_MARGIN_DB = 1.5f;         // Within 1.5 dB of floor

// --- Phase 4: 26-Band IEC 61260 1/3-Octave Spectrum (63 Hz .. 20 kHz) ---
// 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800,
// 1k, 1.25k, 1.6k, 2k, 2.5k, 3.15k, 4k, 5k, 6.3k, 8k, 10k, 12.5k, 16k, 20k Hz
static constexpr uint8_t OCTAVE_BAND_COUNT = 26;
static constexpr uint8_t SECONDS_PER_MINUTE = 60;

// --- High-Frequency Stationary Jet Nuisance Detector (e.g. gas burner / hiss) ---
// Evaluates L90 emergence, stationarity (Leq-L90), and narrowband prominence in
// 8 kHz .. 20 kHz (indices 21..25) to reject broadband water splash & low-freq AC.
static constexpr uint8_t HF_DET_START_BAND = 21;       // 8 kHz .. 20 kHz
static constexpr float HF_DET_MIN_EMERG_DB = 6.0f;     // >= +6 dB vs noise floor
static constexpr float HF_DET_MAX_CREST_DB = 2.5f;     // Leq - L90 <= 2.5 dB
static constexpr float HF_DET_MIN_ADJ_PROM_DB = 4.0f;  // >= +4 dB above neighbors
static constexpr float HF_DET_MIN_MID_PROM_DB = 3.8f;  // >= +3.8 dB above 3.15k-6.3k

// Compile-time Phase 2/4 diagnostics
#define RUN_DSP_SELF_TEST 1
#define VERBOSE_1S_SERIAL 1

// --- FreeRTOS Task Isolation (Core 1 = Real-Time Audio/DSP, Core 0 = I/O & Logs) ---
static constexpr BaseType_t AUDIO_DSP_TASK_CORE = 1;
static constexpr UBaseType_t AUDIO_DSP_TASK_PRIO = 5;
static constexpr uint32_t AUDIO_DSP_TASK_STACK = 12288;

static constexpr BaseType_t SERIAL_LOG_TASK_CORE = 0;
static constexpr UBaseType_t SERIAL_LOG_TASK_PRIO = 2;
static constexpr uint32_t SERIAL_LOG_TASK_STACK = 6144;

// --- Phase 3/4: RTC + NTP, SD_MMC, Wi-Fi/HTTP, RLCD, Battery ADC ---
// POSIX TZ for UTC+7 (Indochina Time: ICT is UTC+7, POSIX sign is inverted:
// ICT-7)
static constexpr const char *NTP_POSIX_TZ = "ICT-7";
static constexpr const char *NTP_SERVER_1 = "pool.ntp.org";
static constexpr const char *NTP_SERVER_2 = "time.google.com";
static constexpr const char *NTP_SERVER_3 = "time.nist.gov";
static constexpr uint32_t NTP_RTC_SYNC_INTERVAL_MS = 24UL * 3600UL * 1000UL;
static constexpr uint32_t NTP_WIFI_SYNC_TIMEOUT_MS = 30000UL;

static constexpr UBaseType_t SEC_QUEUE_DEPTH = 10;
static constexpr UBaseType_t MIN_QUEUE_DEPTH = 15;
static constexpr size_t HTTP_QUEUE_DEPTH = 16;
static constexpr size_t HTTP_JSON_SLOT_SIZE = 1536;
static constexpr uint16_t HTTP_TIMEOUT_MS = 3000;
static constexpr uint32_t HTTP_RETRY_INTERVAL_MS = 10000UL;

static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000UL;
static constexpr uint32_t WIFI_BACKOFF_MIN_MS = 5000UL;
static constexpr uint32_t WIFI_BACKOFF_MAX_MS = 300000UL;

static constexpr uint32_t RLCD_REFRESH_INTERVAL_MS = 1000UL;
static constexpr uint32_t SD_REMOUNT_RETRY_MS = 60000UL;
static constexpr uint8_t BAT_ADC_SAMPLES = 16;

// --- Physical KEY Button & Low-Battery SD Protection ---
static constexpr uint32_t KEY_DEBOUNCE_MS = 50UL;
static constexpr uint32_t LOW_BAT_MIN_PRESENT_MV = 2500U; // Ignore <2.5V when USB-powered with battery switch OFF
static constexpr uint32_t LOW_BAT_CUTOFF_MV = 3350U;
static constexpr uint8_t LOW_BAT_CONSECUTIVE_MIN = 3U;

static constexpr BaseType_t SYSTEM_IO_TASK_CORE = 0;
static constexpr UBaseType_t SYSTEM_IO_TASK_PRIO = 2;
static constexpr uint32_t SYSTEM_IO_TASK_STACK = 12288;

// --- Verified Hardware Self-Noise Floor (Anechoic Chamber, MIC_GAIN_DB = 24.0 dB) ---
// Always stored in raw uncalibrated dBFS.
// Bands 0..23 (63 Hz .. 12.5 kHz) measured on rlcd-noise-01 in anechoic chamber.
// Bands 24..25 (16 kHz, 20 kHz) calibrated from 48 kHz quiet-room measurements scaled to 24 dB ref.
static constexpr float ANECHOIC_BB_LEQ_DBFS = -79.4f;
static constexpr float ANECHOIC_BB_L90_DBFS = -79.6f;
static constexpr float ANECHOIC_OCT_L90_DBFS[OCTAVE_BAND_COUNT] = {
    -94.2f, -92.3f, -92.8f, -93.7f, -92.3f, -92.8f, -93.2f, -92.8f,
    -93.6f, -94.2f, -94.2f, -94.5f, -95.0f, -95.2f, -95.5f, -95.9f,
    -95.8f, -96.0f, -96.0f, -95.8f, -95.6f, -95.2f, -94.7f, -93.9f,
    -90.5f, -90.7f};

// Scale the 24 dB anechoic self-noise reference linearly to ES7210_MIC_GAIN_DB
inline constexpr float AnechoicGainShiftDb() {
  return ES7210_MIC_GAIN_DB - ANECHOIC_REF_MIC_GAIN_DB;
}
inline constexpr float AnechoicBbLeqDbfs() {
  return ANECHOIC_BB_LEQ_DBFS + AnechoicGainShiftDb();
}
inline constexpr float AnechoicOctL90Dbfs(uint8_t b) {
  return ANECHOIC_OCT_L90_DBFS[b] + AnechoicGainShiftDb();
}
