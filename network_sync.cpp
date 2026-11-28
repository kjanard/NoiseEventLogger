#include "network_sync.h"
#include "config.h"
#include "rtc_time.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#include "secrets.example.h"
#endif

#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_heap_caps.h>
#include <stdio.h>
#include <string.h>

namespace {

struct JsonSlot {
  char payload[HTTP_JSON_SLOT_SIZE];
};

enum WifiSmState {
  WIFI_SM_DISABLED = 0,
  WIFI_SM_CONNECTING,
  WIFI_SM_CONNECTED,
  WIFI_SM_BACKOFF,
  WIFI_SM_NTP_SLEEP
};

JsonSlot       *s_http_ring        = nullptr;
size_t          s_ring_head        = 0;
size_t          s_ring_tail        = 0;
size_t          s_ring_count       = 0;

WifiSmState     s_wifi_state       = WIFI_SM_DISABLED;
uint32_t        s_wifi_state_ts_ms = 0;
uint32_t        s_backoff_ms       = WIFI_BACKOFF_MIN_MS;
uint32_t        s_ntp_sync_count_seen = 0;

HttpSyncState_t s_http_state       = HTTP_STATE_DISABLED;
int             s_last_http_code   = 0;
uint32_t        s_last_http_try_ms = 0;
bool            s_inited           = false;

bool hasStr(const char *s) {
  return (s != nullptr && s[0] != '\0');
}

void powerDownWifiRadio() {
  WiFi.disconnect(true, false);
  WiFi.mode(WIFI_OFF);
}

void startWifiAttempt() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  s_wifi_state = WIFI_SM_CONNECTING;
  s_wifi_state_ts_ms = millis();
  s_ntp_sync_count_seen = RtcTime_GetNtpSyncCount();
  Serial.println("[WiFi] Connecting (non-blocking)...");
}

void processHttpQueue() {
  if (!hasStr(INGEST_URL) || s_wifi_state != WIFI_SM_CONNECTED || s_ring_count == 0 || s_http_ring == nullptr) {
    return;
  }

  const uint32_t now_ms = millis();
  if (s_http_state == HTTP_STATE_ERR && (now_ms - s_last_http_try_ms) < HTTP_RETRY_INTERVAL_MS) {
    return;
  }
  s_last_http_try_ms = now_ms;

  HTTPClient http;
  http.setConnectTimeout(HTTP_TIMEOUT_MS);
  http.setTimeout(HTTP_TIMEOUT_MS);

  if (!http.begin(INGEST_URL)) {
    s_last_http_code = -1;
    s_http_state = HTTP_STATE_ERR;
    Serial.println("[HTTP] http.begin() failed");
    return;
  }

  http.addHeader("Content-Type", "application/json");
  if (hasStr(INGEST_AUTH_TOKEN)) {
    char auth_hdr[192];
    snprintf(auth_hdr, sizeof(auth_hdr), "Bearer %s", INGEST_AUTH_TOKEN);
    http.addHeader("Authorization", auth_hdr);
  }

  char *body = s_http_ring[s_ring_head].payload;
  const int code = http.POST(reinterpret_cast<uint8_t *>(body), strlen(body));
  http.end();

  s_last_http_code = code;
  if (code >= 200 && code < 300) {
    s_ring_head = (s_ring_head + 1U) % HTTP_QUEUE_DEPTH;
    s_ring_count--;
    s_http_state = HTTP_STATE_OK;
    Serial.printf("[HTTP] POST OK (code=%d, remaining=%u)\n", code, static_cast<unsigned>(s_ring_count));
  } else {
    s_http_state = HTTP_STATE_ERR;
    Serial.printf("[HTTP] POST failed (code=%d, queued=%u)\n", code, static_cast<unsigned>(s_ring_count));
  }
}

}  // namespace

bool NetworkSync_Init() {
  if (s_inited) {
    return true;
  }

  // Allocate HTTP retry ring buffer in OPI PSRAM (falls back to heap if needed)
  const size_t ring_bytes = sizeof(JsonSlot) * HTTP_QUEUE_DEPTH;
  s_http_ring = static_cast<JsonSlot *>(heap_caps_malloc(ring_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (s_http_ring == nullptr) {
    s_http_ring = static_cast<JsonSlot *>(malloc(ring_bytes));
  }
  if (s_http_ring != nullptr) {
    memset(s_http_ring, 0, ring_bytes);
  }

  const bool wifi_cfg = hasStr(WIFI_SSID);
  const bool http_cfg = wifi_cfg && hasStr(INGEST_URL);

  s_http_state = http_cfg ? HTTP_STATE_IDLE : HTTP_STATE_DISABLED;

  if (!wifi_cfg) {
    WiFi.mode(WIFI_OFF);
    s_wifi_state = WIFI_SM_DISABLED;
    Serial.println("[Net] WIFI_SSID empty -> Offline mode (RTC + SD + RLCD only)");
    s_inited = true;
    return true;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(false);
  WiFi.setSleep(false);
  s_backoff_ms = WIFI_BACKOFF_MIN_MS;
  startWifiAttempt();

  Serial.printf("[Net] PSRAM HTTP queue=%u slots (%lu B) | HTTP POST: %s\n",
                static_cast<unsigned>(HTTP_QUEUE_DEPTH),
                static_cast<unsigned long>(ring_bytes),
                http_cfg ? "ENABLED" : "DISABLED (NTP-only)");

  s_inited = true;
  return true;
}

bool NetworkSync_IsWifiSleeping() {
  return (s_wifi_state == WIFI_SM_NTP_SLEEP);
}

void NetworkSync_Poll() {
  if (!s_inited || s_wifi_state == WIFI_SM_DISABLED) {
    return;
  }

  const uint32_t now_ms = millis();
  const wl_status_t st = WiFi.status();

  switch (s_wifi_state) {
    case WIFI_SM_CONNECTING:
      if (st == WL_CONNECTED) {
        s_wifi_state = WIFI_SM_CONNECTED;
        s_wifi_state_ts_ms = now_ms;
        s_backoff_ms = WIFI_BACKOFF_MIN_MS;
        Serial.printf("[WiFi] Connected | IP=%s | RSSI=%d dBm\n",
                      WiFi.localIP().toString().c_str(),
                      WiFi.RSSI());
        RtcTime_OnWifiConnected();
      } else if ((now_ms - s_wifi_state_ts_ms) >= WIFI_CONNECT_TIMEOUT_MS) {
        RtcTime_OnWifiDisconnected();
        powerDownWifiRadio();
        s_wifi_state = WIFI_SM_BACKOFF;
        s_wifi_state_ts_ms = now_ms;
        Serial.printf("[WiFi] Connect timeout -> Wi-Fi OFF, retry in %lu ms\n", static_cast<unsigned long>(s_backoff_ms));
      }
      break;

    case WIFI_SM_CONNECTED:
      if (st != WL_CONNECTED) {
        RtcTime_OnWifiDisconnected();
        powerDownWifiRadio();
        s_wifi_state = WIFI_SM_BACKOFF;
        s_wifi_state_ts_ms = now_ms;
        Serial.printf("[WiFi] Disconnected -> Wi-Fi OFF, retry in %lu ms\n", static_cast<unsigned long>(s_backoff_ms));
      } else if (!hasStr(INGEST_URL)) {
        const bool ntp_updated = (RtcTime_GetNtpSyncCount() != s_ntp_sync_count_seen);
        if (ntp_updated) {
          RtcTime_OnWifiDisconnected();
          powerDownWifiRadio();
          s_wifi_state = WIFI_SM_NTP_SLEEP;
          s_wifi_state_ts_ms = now_ms;
          Serial.println("[Net] NTP-only sync complete -> Wi-Fi powered OFF (next sync in 24h)");
        } else if ((now_ms - s_wifi_state_ts_ms) >= NTP_WIFI_SYNC_TIMEOUT_MS) {
          RtcTime_OnWifiDisconnected();
          powerDownWifiRadio();
          s_wifi_state = WIFI_SM_BACKOFF;
          s_wifi_state_ts_ms = now_ms;
          Serial.printf("[Net] NTP sync timeout -> Wi-Fi powered OFF, retry in %lu ms\n",
                        static_cast<unsigned long>(s_backoff_ms));
        }
      } else {
        processHttpQueue();
      }
      break;

    case WIFI_SM_BACKOFF:
      if ((now_ms - s_wifi_state_ts_ms) >= s_backoff_ms) {
        s_backoff_ms = (s_backoff_ms < WIFI_BACKOFF_MAX_MS) ? (s_backoff_ms * 2UL) : WIFI_BACKOFF_MAX_MS;
        if (s_backoff_ms > WIFI_BACKOFF_MAX_MS) {
          s_backoff_ms = WIFI_BACKOFF_MAX_MS;
        }
        startWifiAttempt();
      }
      break;

    case WIFI_SM_NTP_SLEEP:
      if ((now_ms - s_wifi_state_ts_ms) >= NTP_RTC_SYNC_INTERVAL_MS) {
        Serial.println("[Net] 24h NTP refresh due -> waking Wi-Fi");
        startWifiAttempt();
      }
      break;

    default:
      break;
  }
}

bool NetworkSync_EnqueueJson(const char *json_payload) {
  if (!NetworkSync_IsHttpConfigured() || s_http_ring == nullptr || json_payload == nullptr) {
    return false;
  }
  const size_t len = strlen(json_payload);
  if (len == 0 || len >= HTTP_JSON_SLOT_SIZE) {
    return false;
  }

  if (s_ring_count >= HTTP_QUEUE_DEPTH) {
    // Drop oldest queued sample to retain freshest telemetry on extended outage
    s_ring_head = (s_ring_head + 1U) % HTTP_QUEUE_DEPTH;
    s_ring_count--;
  }

  memcpy(s_http_ring[s_ring_tail].payload, json_payload, len + 1U);
  s_ring_tail = (s_ring_tail + 1U) % HTTP_QUEUE_DEPTH;
  s_ring_count++;
  return true;
}

bool NetworkSync_IsWifiConfigured() {
  return hasStr(WIFI_SSID);
}

bool NetworkSync_IsWifiConnected() {
  return (s_wifi_state == WIFI_SM_CONNECTED && WiFi.status() == WL_CONNECTED);
}

int NetworkSync_GetRssi() {
  return NetworkSync_IsWifiConnected() ? WiFi.RSSI() : -127;
}

bool NetworkSync_IsHttpConfigured() {
  return (hasStr(WIFI_SSID) && hasStr(INGEST_URL));
}

HttpSyncState_t NetworkSync_GetHttpState() {
  return s_http_state;
}

int NetworkSync_GetLastHttpCode() {
  return s_last_http_code;
}

size_t NetworkSync_GetQueuedCount() {
  return s_ring_count;
}
