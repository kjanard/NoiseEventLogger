#pragma once

#include <Arduino.h>
#include "dsp_octave.h"
#include "network_sync.h"

// ============================================================================
// Phase 4: Waveshare 4.2" ST7305 Monochrome RLCD (400x300) Status Display
// Reference: reference/examples/arduino/examples/09_LVGL_V9_Test/display_bsp.cpp
// ============================================================================

typedef struct {
  const char     *time_hm;       // "YYYY-MM-DD HH:MM:SS"
  bool            ntp_synced;
  bool            rtc_ok;
  bool            wifi_configured;
  bool            wifi_connected;
  bool            wifi_sleeping;
  int             rssi;
  bool            sd_ok;
  const char     *sd_path;
  uint32_t        vbat_mv;
  HttpSyncState_t http_state;
  size_t          http_queued;

  bool            calibrated;
  float           cal_offset_db;
  float           gain_db;
  bool            marker_active;
  uint32_t        hf_events_today;

  bool            has_sec;
  SecondReport_t  sec;
  bool            has_min;
  MinuteReport_t  min;
  uint32_t        minute_count;
  uint8_t         warmup_sec;    // 0..59 progress toward next minute
  uint32_t        uptime_s;
} DisplayUiState_t;

bool DisplayUi_Init();
void DisplayUi_Render(const DisplayUiState_t &state);
void DisplayUi_RenderLowBatterySleep(uint32_t vbat_mv, const char *time_hm);
bool DisplayUi_IsOk();
