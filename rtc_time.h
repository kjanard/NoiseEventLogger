#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <time.h>

// ============================================================================
// Phase 3: PCF85063 Hardware RTC + NTP Synchronization (UTC+7 / ICT-7)
// Reference: reference/examples/arduino/libraries/SensorLib/src/SensorPCF85063.hpp
// ============================================================================

bool RtcTime_Init(TwoWire &wire,
                  SemaphoreHandle_t i2c_mutex,
                  const char *build_date,
                  const char *build_time);
void RtcTime_OnWifiConnected();
void RtcTime_OnWifiDisconnected();
void RtcTime_Poll();

time_t RtcTime_NowEpoch();
bool RtcTime_IsRtcOk();
bool RtcTime_IsNtpSynced();
uint32_t RtcTime_GetNtpSyncCount();

// Format helpers using fixed caller-provided char buffers (no heap String)
bool RtcTime_FormatIso8601(time_t epoch, char *buf, size_t len);    // YYYY-MM-DDTHH:MM:SS+07:00
bool RtcTime_FormatDateYmd(time_t epoch, char *buf, size_t len);    // YYYY-MM-DD
bool RtcTime_FormatDisplayHm(time_t epoch, char *buf, size_t len);   // YYYY-MM-DD HH:MM
bool RtcTime_FormatDisplayHms(time_t epoch, char *buf, size_t len);  // YYYY-MM-DD HH:MM:SS
