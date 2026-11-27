#include "storage_log.h"
#include "board_pins.h"
#include "config.h"

#include <SD_MMC.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

namespace {

constexpr const char *LOG_DIR_PATH = "/logs";
constexpr const char *CSV_HEADER =
    "ts,uptime_s,bb_leq,bb_l90,bb_peak,clips,"
    "leq_63,leq_80,leq_100,leq_125,leq_160,leq_200,leq_250,leq_315,"
    "leq_400,leq_500,leq_630,leq_800,leq_1k,leq_1k25,leq_1k6,leq_2k,"
    "leq_2k5,leq_3k15,leq_4k,leq_5k,leq_6k3,leq_8k,leq_10k,leq_12k5,"
    "l90_63,l90_80,l90_100,l90_125,l90_160,l90_200,l90_250,l90_315,"
    "l90_400,l90_500,l90_630,l90_800,l90_1k,l90_1k25,l90_1k6,l90_2k,"
    "l90_2k5,l90_3k15,l90_4k,l90_5k,l90_6k3,l90_8k,l90_10k,l90_12k5,"
    "vbat_mv,rssi";

bool     s_sd_ok           = false;
uint32_t s_last_mount_try  = 0;
char     s_last_path[48]   = "NONE";
uint32_t s_last_file_bytes = 0;

bool appendFmt(char *buf, size_t cap, size_t *pos, const char *fmt, ...) {
  if (*pos >= cap) {
    return false;
  }
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(buf + *pos, cap - *pos, fmt, ap);
  va_end(ap);
  if (n < 0 || static_cast<size_t>(n) >= (cap - *pos)) {
    return false;
  }
  *pos += static_cast<size_t>(n);
  return true;
}

const char *cardTypeName(uint8_t t) {
  switch (t) {
    case CARD_MMC:  return "MMC";
    case CARD_SD:   return "SDSC";
    case CARD_SDHC: return "SDHC/SDXC";
    default:        return "UNKNOWN";
  }
}

bool mountSdMmc() {
  s_last_mount_try = millis();
  SD_MMC.end();
  if (!SD_MMC.setPins(PIN_SDMMC_CLK, PIN_SDMMC_CMD, PIN_SDMMC_D0)) {
    Serial.println("[SD] SD_MMC.setPins failed");
    s_sd_ok = false;
    return false;
  }

  // 1-bit SD_MMC mode matching Waveshare reference sdcard_bsp.cpp
  if (!SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT, 5)) {
    Serial.println("[SD] No SD card detected or mount failed (degraded: sd_ok=false)");
    s_sd_ok = false;
    return false;
  }

  const uint8_t card_type = SD_MMC.cardType();
  if (card_type == CARD_NONE) {
    Serial.println("[SD] CARD_NONE (degraded: sd_ok=false)");
    SD_MMC.end();
    s_sd_ok = false;
    return false;
  }

  if (!SD_MMC.exists(LOG_DIR_PATH)) {
    if (!SD_MMC.mkdir(LOG_DIR_PATH)) {
      Serial.println("[SD] Failed to create /logs directory");
      SD_MMC.end();
      s_sd_ok = false;
      return false;
    }
  }

  const uint64_t size_mb = SD_MMC.cardSize() / (1024ULL * 1024ULL);
  const uint64_t total_mb = SD_MMC.totalBytes() / (1024ULL * 1024ULL);
  const uint64_t used_mb = SD_MMC.usedBytes() / (1024ULL * 1024ULL);
  Serial.printf("[SD] Mounted 1-bit %s | Size=%llu MB (Total=%llu MB, Used=%llu MB) | /logs OK\n",
                cardTypeName(card_type),
                static_cast<unsigned long long>(size_mb),
                static_cast<unsigned long long>(total_mb),
                static_cast<unsigned long long>(used_mb));
  s_sd_ok = true;
  return true;
}

}  // namespace

bool StorageLog_Init() {
  return mountSdMmc();
}

bool StorageLog_AppendMinute(const MinuteReport_t &rec,
                             const char *ts_iso,
                             const char *date_ymd,
                             uint32_t vbat_mv,
                             int rssi) {
  if (!s_sd_ok) {
    const uint32_t now_ms = millis();
    if ((now_ms - s_last_mount_try) < SD_REMOUNT_RETRY_MS || !mountSdMmc()) {
      return false;
    }
  }

  char path[48] = {};
  const char *safe_date = (date_ymd != nullptr && date_ymd[0] != '\0') ? date_ymd : "unsynced";
  snprintf(path, sizeof(path), "%s/%s.csv", LOG_DIR_PATH, safe_date);

  if (!SD_MMC.exists(LOG_DIR_PATH)) {
    SD_MMC.mkdir(LOG_DIR_PATH);
  }

  bool need_header = !SD_MMC.exists(path);
  File f = SD_MMC.open(path, FILE_APPEND);
  if (!f) {
    Serial.printf("[SD] Open failed for %s -> marking sd_ok=false\n", path);
    s_sd_ok = false;
    return false;
  }

  if (!need_header && f.size() == 0) {
    need_header = true;
  }

  if (need_header) {
    if (f.println(CSV_HEADER) == 0) {
      Serial.printf("[SD] Header write failed for %s\n", path);
      f.close();
      s_sd_ok = false;
      return false;
    }
  }

  char row[768];
  size_t pos = 0;
  const char *safe_ts = (ts_iso != nullptr && ts_iso[0] != '\0') ? ts_iso : "null";

  bool ok = appendFmt(row, sizeof(row), &pos, "%s,%lu,%.1f,%.1f,%.1f,%lu",
                      safe_ts,
                      static_cast<unsigned long>(rec.uptime_s),
                      rec.bb_leq,
                      rec.bb_l90,
                      rec.bb_peak,
                      static_cast<unsigned long>(rec.clips));

  for (uint8_t b = 0; ok && b < OCTAVE_BAND_COUNT; ++b) {
    ok = appendFmt(row, sizeof(row), &pos, ",%.1f", rec.oct_leq[b]);
  }
  for (uint8_t b = 0; ok && b < OCTAVE_BAND_COUNT; ++b) {
    ok = appendFmt(row, sizeof(row), &pos, ",%.1f", rec.oct_l90[b]);
  }
  if (ok) {
    ok = appendFmt(row, sizeof(row), &pos, ",%lu,%d",
                   static_cast<unsigned long>(vbat_mv),
                   rssi);
  }

  if (!ok || f.println(row) == 0) {
    Serial.printf("[SD] Row format/write failed for %s\n", path);
    f.close();
    s_sd_ok = false;
    return false;
  }

  f.flush();
  const uint32_t new_size = static_cast<uint32_t>(f.size());
  f.close();

  snprintf(s_last_path, sizeof(s_last_path), "%s", path);
  s_last_file_bytes = new_size;
  Serial.printf("[SD] Appended %s (bytes=%lu%s)\n",
                s_last_path,
                static_cast<unsigned long>(s_last_file_bytes),
                need_header ? ", new file+header" : "");
  return true;
}

bool StorageLog_IsOk() {
  return s_sd_ok;
}

const char *StorageLog_GetLastFilePath() {
  return s_last_path;
}

uint32_t StorageLog_GetLastFileBytes() {
  return s_last_file_bytes;
}
