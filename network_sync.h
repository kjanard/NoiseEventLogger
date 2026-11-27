#pragma once

#include <Arduino.h>

// ============================================================================
// Phase 3: Non-Blocking Wi-Fi Manager & PSRAM HTTP POST Retry Queue
// ============================================================================

typedef enum {
  HTTP_STATE_DISABLED = 0,  // INGEST_URL or WIFI_SSID is empty in secrets.h
  HTTP_STATE_IDLE     = 1,  // Configured, waiting for next record or Wi-Fi
  HTTP_STATE_OK       = 2,  // Last POST returned 2xx
  HTTP_STATE_ERR      = 3   // Last POST failed (retrying from PSRAM queue)
} HttpSyncState_t;

bool NetworkSync_Init();
void NetworkSync_Poll();
bool NetworkSync_EnqueueJson(const char *json_payload);

bool NetworkSync_IsWifiConfigured();
bool NetworkSync_IsWifiConnected();
int  NetworkSync_GetRssi();
bool NetworkSync_IsHttpConfigured();
HttpSyncState_t NetworkSync_GetHttpState();
int  NetworkSync_GetLastHttpCode();
size_t NetworkSync_GetQueuedCount();
