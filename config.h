#pragma once

#include <Arduino.h>

// ============================================================================
// Noise Event Logger — Firmware Configuration (SPEC.md)
// ============================================================================

static constexpr const char *DEVICE_ID = "rlcd-noise-01";
static constexpr uint32_t SERIAL_BAUD_RATE = 115200;

// --- Audio & DSP Parameters ---
// fs = 32000 Hz => Nyquist = 16000 Hz (> 14254.4 Hz upper edge of 12.5 kHz
// 1/3-octave band)
static constexpr uint32_t AUDIO_SAMPLE_RATE_HZ = 32000;
static constexpr uint8_t AUDIO_BITS_PER_SAMPLE = 16;
static constexpr uint16_t AUDIO_FFT_SIZE = 4096; // df = 7.8125 Hz @ 32 kHz
static constexpr uint16_t AUDIO_HOP_SAMPLES =
    2000; // 16 hops/s => exact 1.000 s/32000 samples

// I2S TDM DMA buffer geometry: 256 4-slot frames = 8 ms per chunk @ 32 kHz
// (2048 B)
static constexpr uint32_t AUDIO_DMA_DESC_NUM = 6;
static constexpr uint32_t AUDIO_DMA_FRAME_NUM = 256;
static constexpr uint8_t AUDIO_TDM_SLOTS =
    4; // Slot0=MIC1, Slot1=MIC3, Slot2=MIC2, Slot3=MIC4

// ES7210 fixed manual gain (+24 dB => code 0x08; REG 0x43..0x46 = 0x18).
// AGC, AEC, and hardware noise suppression are explicitly disabled.
static constexpr float MIC_GAIN_DB = 24.0f;
static constexpr uint8_t AUDIO_PRIMARY_MIC_CHANNEL = 0; // Left I2S slot = MIC1

// Clipping threshold: >= 99.5% of 16-bit signed full-scale (32768)
static constexpr int32_t AUDIO_CLIP_THRESHOLD_PCM = 32600;

// Acoustic calibration & dB floor
static constexpr float CALIBRATION_OFFSET_DB = 0.0f; // 0.0 = raw dBFS
static constexpr float DBFS_POWER_EPSILON = 1e-12f;  // -120.0 dBFS floor

// --- Phase 2: 24-Band IEC 61260 1/3-Octave Spectrum & Temporal Aggregation ---
// 63, 80, 100, 125, 160, 200, 250, 315, 400, 500, 630, 800,
// 1k, 1.25k, 1.6k, 2k, 2.5k, 3.15k, 4k, 5k, 6.3k, 8k, 10k, 12.5k Hz
static constexpr uint8_t OCTAVE_BAND_COUNT = 24;
static constexpr uint8_t SECONDS_PER_MINUTE = 60;

// Compile-time Phase 2 diagnostics
#define RUN_DSP_SELF_TEST 1
#define VERBOSE_1S_SERIAL 1

// --- FreeRTOS Task Isolation (Core 1 = Real-Time Audio/DSP, Core 0 = I/O &
// Logs) ---
static constexpr BaseType_t AUDIO_DSP_TASK_CORE = 1;
static constexpr UBaseType_t AUDIO_DSP_TASK_PRIO = 5;
static constexpr uint32_t AUDIO_DSP_TASK_STACK = 12288;

static constexpr BaseType_t SERIAL_LOG_TASK_CORE = 0;
static constexpr UBaseType_t SERIAL_LOG_TASK_PRIO = 2;
static constexpr uint32_t SERIAL_LOG_TASK_STACK = 6144;

// --- Phase 3: RTC + NTP, SD_MMC, Wi-Fi/HTTP, RLCD, Battery ADC ---
// POSIX TZ for UTC+7 (Indochina Time: ICT is UTC+7, POSIX sign is inverted:
// ICT-7)
static constexpr const char *NTP_POSIX_TZ = "ICT-7";
static constexpr const char *NTP_SERVER_1 = "pool.ntp.org";
static constexpr const char *NTP_SERVER_2 = "time.google.com";
static constexpr const char *NTP_SERVER_3 = "time.nist.gov";
static constexpr uint32_t NTP_RTC_SYNC_INTERVAL_MS = 24UL * 3600UL * 1000UL;

static constexpr UBaseType_t MIN_QUEUE_DEPTH = 15;
static constexpr size_t HTTP_QUEUE_DEPTH = 16;
static constexpr size_t HTTP_JSON_SLOT_SIZE = 768;
static constexpr uint16_t HTTP_TIMEOUT_MS = 3000;
static constexpr uint32_t HTTP_RETRY_INTERVAL_MS = 10000UL;

static constexpr uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000UL;
static constexpr uint32_t WIFI_BACKOFF_MIN_MS = 5000UL;
static constexpr uint32_t WIFI_BACKOFF_MAX_MS = 300000UL;

static constexpr uint32_t RLCD_REFRESH_INTERVAL_MS = 1000UL;
static constexpr uint32_t SD_REMOUNT_RETRY_MS = 60000UL;
static constexpr uint8_t BAT_ADC_SAMPLES = 16;

static constexpr BaseType_t SYSTEM_IO_TASK_CORE = 0;
static constexpr UBaseType_t SYSTEM_IO_TASK_PRIO = 2;
static constexpr uint32_t SYSTEM_IO_TASK_STACK = 10240;

// --- Verified Hardware Self-Noise Floor (Anechoic Chamber, MIC_GAIN_DB = 24.0 dB) ---
// Measured across 120s (2 consecutive 1m records) on rlcd-noise-01:
// Broadband mean=-79.34 dBFS (stdev=0.167 dB), 1m L90=-79.6 dBFS, Peak=-65.2 dBFS
static constexpr float ANECHOIC_BB_LEQ_DBFS = -79.4f;
static constexpr float ANECHOIC_BB_L90_DBFS = -79.6f;
static constexpr float ANECHOIC_OCT_L90_DBFS[OCTAVE_BAND_COUNT] = {
    -94.2f, -92.3f, -92.8f, -93.7f, -92.3f, -92.8f, -93.2f, -92.8f,
    -93.6f, -94.2f, -94.2f, -94.5f, -95.0f, -95.2f, -95.5f, -95.9f,
    -95.8f, -96.0f, -96.0f, -95.8f, -95.6f, -95.2f, -94.7f, -93.9f};
