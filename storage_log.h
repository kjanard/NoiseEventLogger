#pragma once

#include <Arduino.h>
#include "dsp_octave.h"

// ============================================================================
// Phase 3: MicroSD Daily Rotating CSV Logger (SD_MMC 1-bit mode)
// Reference: reference/examples/arduino/examples/06_SD_Card/sdcard_bsp.cpp
// ============================================================================

bool StorageLog_Init();

bool StorageLog_AppendMinute(const MinuteReport_t &rec,
                             const char *ts_iso,
                             const char *date_ymd,
                             float gain_db,
                             bool marker,
                             uint32_t vbat_mv,
                             int rssi);

void StorageLog_Shutdown();
bool StorageLog_IsOk();
const char *StorageLog_GetLastFilePath();
uint32_t StorageLog_GetLastFileBytes();
