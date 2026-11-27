#pragma once

// ============================================================================
// Credentials & Endpoint Configuration Template
// Copy this file to "secrets.h" and fill in your credentials.
// "secrets.h" is listed in .gitignore so secrets are never committed.
//
// Notes:
// - Leave WIFI_SSID as "" for pure offline mode (RTC + SD_MMC + RLCD only).
// - Set WIFI_SSID / WIFI_PASSWORD and leave INGEST_URL as "" to use Wi-Fi
//   strictly for NTP time sync into PCF85063 while logging locally to SD card.
// ============================================================================

static constexpr const char *WIFI_SSID         = "";
static constexpr const char *WIFI_PASSWORD     = "";
static constexpr const char *INGEST_URL        = "";
static constexpr const char *INGEST_AUTH_TOKEN = "";
