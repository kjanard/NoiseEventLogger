#pragma once

#include <Arduino.h>
#include <driver/gpio.h>

// ============================================================================
// Waveshare ESP32-S3-RLCD-4.2 Verified Hardware Pinout
// Target MCU: ESP32-S3-WROOM-1-N16R8 (16MB Flash, 8MB OPI PSRAM)
// Arduino-ESP32 Core: 3.3.12 (ESP-IDF v5.5.5) -> driver/i2s_std.h
//   Verified in:
//   - reference/README.md (line 91)
//   - reference/.github/product.json (lines 7-11)
// ============================================================================

// --- I2C Bus (Shared: ES7210, ES8311, PCF85063, SHTC3) ---
// NEVER call Wire.begin() without these explicit pins.
// Sources:
// - reference/examples/factory/main/user_config.h (lines 17-18)
// - reference/examples/arduino/examples/04_I2C_PCF85063/04_I2C_PCF85063.ino (line 4)
// - reference/examples/arduino/examples/07_Audio_Test/src/ExternLib/codec_board/board_cfg.h (line 11)
static constexpr int      PIN_I2C_SDA        = 13;
static constexpr int      PIN_I2C_SCL        = 14;
static constexpr uint32_t I2C_BUS_FREQ_HZ    = 400000;

// 7-bit I2C device addresses:
// - ES7210:   reference/examples/arduino/examples/07_Audio_Test/codec_bsp.h (line 15)
// - ES8311:   reference/examples/arduino/examples/07_Audio_Test/codec_bsp.h (line 14)
// - PCF85063: reference/examples/arduino/examples/04_I2C_PCF85063/04_I2C_PCF85063.ino (line 10)
// - SHTC3:    reference/examples/arduino/examples/04_I2C_PCF85063/i2c_equipment.h (line 31)
static constexpr uint8_t  I2C_ADDR_ES7210    = 0x40;  // Dual-mic audio ADC
static constexpr uint8_t  I2C_ADDR_ES8311    = 0x18;  // Audio DAC codec (unused in logger)
static constexpr uint8_t  I2C_ADDR_PCF85063  = 0x51;  // Hardware RTC
static constexpr uint8_t  I2C_ADDR_SHTC3     = 0x70;  // Temp & humidity sensor

// --- I2S Audio Bus & Speaker Amplifier Control ---
// Sources:
// - reference/examples/arduino/examples/07_Audio_Test/src/ExternLib/codec_board/board_cfg.h (lines 12-13)
// - reference/examples/esphome/examples/esp32-s3-rlcd-42-wake-word.yaml (lines 58-76)
// - reference/examples/esphome/examples/esp32-s3-rlcd-42-speaker.yaml (lines 177-182)
static constexpr int      PIN_I2S_MCLK       = 16;
static constexpr int      PIN_I2S_BCLK       = 9;
static constexpr int      PIN_I2S_WS         = 45;
static constexpr int      PIN_I2S_DIN        = 10;    // ESP32-S3 RX <- ES7210 SDOUT
static constexpr int      PIN_I2S_DOUT       = 8;     // ESP32-S3 TX -> ES8311 SDIN (unused)
static constexpr int      PIN_PA_CTRL        = 46;    // Speaker amp enable (hold LOW)

// --- ST7305 Reflective LCD 4.2" (400x300) SPI ---
// Sources:
// - reference/examples/factory/main/user_config.h (lines 9-14)
// - reference/examples/arduino/examples/10_U8G2_Test/10_U8G2_Test.ino (lines 3-10)
static constexpr int      PIN_RLCD_SCK       = 11;
static constexpr int      PIN_RLCD_MOSI      = 12;
static constexpr int      PIN_RLCD_DC        = 5;
static constexpr int      PIN_RLCD_CS        = 40;
static constexpr int      PIN_RLCD_RST       = 41;
static constexpr int      PIN_RLCD_TE        = 6;
static constexpr int      RLCD_WIDTH         = 400;
static constexpr int      RLCD_HEIGHT        = 300;

// --- SD_MMC Storage (1-bit mode) ---
// Source:
// - reference/examples/arduino/examples/06_SD_Card/sdcard_bsp.h (line 14)
static constexpr int      PIN_SDMMC_CLK      = 38;
static constexpr int      PIN_SDMMC_CMD      = 21;
static constexpr int      PIN_SDMMC_D0       = 39;
static constexpr int      SDMMC_BUS_WIDTH    = 1;

// --- Battery ADC & User Buttons ---
// Sources:
// - reference/examples/arduino/examples/03_ADC_Test/adc_bsp.cpp (lines 24-35)
// - reference/examples/esphome/examples/esp32-s3-rlcd-42-sensor.yaml (lines 120-127)
// - reference/examples/factory/components/port_bsp/button_bsp.c (lines 12-19)
static constexpr int      PIN_BAT_ADC        = 4;     // ADC1_CH3, 12 dB atten, 3.0x divider
static constexpr float    BAT_ADC_DIVIDER    = 3.0f;
static constexpr int      PIN_BTN_BOOT       = 0;     // Active LOW
static constexpr int      PIN_BTN_KEY        = 18;    // Active LOW
