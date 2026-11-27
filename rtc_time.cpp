#include "rtc_time.h"
#include "board_pins.h"
#include "config.h"

#include <Preferences.h>
#include <esp_sntp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

namespace {

// PCF85063 register addresses (verified in SensorLib PCF85063Constants.h)
constexpr uint8_t PCF85063_REG_CTRL1 = 0x00;
constexpr uint8_t PCF85063_REG_SEC   = 0x04;

constexpr uint8_t PCF85063_CTRL1_STOP_BIT = (1U << 5);
constexpr uint8_t PCF85063_CTRL1_12H_BIT  = (1U << 1);
constexpr uint8_t PCF85063_SEC_OS_BIT     = (1U << 7);

TwoWire          *s_wire              = nullptr;
SemaphoreHandle_t s_i2c_mutex         = nullptr;
bool              s_rtc_ok            = false;
bool              s_ntp_cb_registered = false;
volatile bool     s_ntp_pending_write = false;
bool              s_ntp_synced        = false;
uint32_t          s_last_rtc_sync_ms  = 0;

uint8_t decToBcd(uint8_t val) {
  return static_cast<uint8_t>(((val / 10U) << 4) | (val % 10U));
}

uint8_t bcdToDec(uint8_t val) {
  return static_cast<uint8_t>(((val >> 4) * 10U) + (val & 0x0FU));
}

bool lockI2c() {
  if (s_i2c_mutex == nullptr) {
    return true;
  }
  return (xSemaphoreTake(s_i2c_mutex, pdMS_TO_TICKS(100)) == pdTRUE);
}

void unlockI2c() {
  if (s_i2c_mutex != nullptr) {
    xSemaphoreGive(s_i2c_mutex);
  }
}

bool pcfWriteBytesUnlocked(uint8_t reg, const uint8_t *data, size_t len) {
  if (s_wire == nullptr) {
    return false;
  }
  s_wire->beginTransmission(I2C_ADDR_PCF85063);
  s_wire->write(reg);
  for (size_t i = 0; i < len; ++i) {
    s_wire->write(data[i]);
  }
  return (s_wire->endTransmission() == 0);
}

bool pcfReadBytesUnlocked(uint8_t reg, uint8_t *out_buf, size_t len) {
  if (s_wire == nullptr || out_buf == nullptr) {
    return false;
  }
  s_wire->beginTransmission(I2C_ADDR_PCF85063);
  s_wire->write(reg);
  if (s_wire->endTransmission(false) != 0) {
    return false;
  }
  const uint8_t got = s_wire->requestFrom(I2C_ADDR_PCF85063, static_cast<uint8_t>(len));
  if (got != len) {
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    out_buf[i] = static_cast<uint8_t>(s_wire->read());
  }
  return true;
}

bool pcfWriteBytes(uint8_t reg, const uint8_t *data, size_t len) {
  if (!lockI2c()) {
    return false;
  }
  const bool ok = pcfWriteBytesUnlocked(reg, data, len);
  unlockI2c();
  return ok;
}

bool pcfReadBytes(uint8_t reg, uint8_t *out_buf, size_t len) {
  if (!lockI2c()) {
    return false;
  }
  const bool ok = pcfReadBytesUnlocked(reg, out_buf, len);
  unlockI2c();
  return ok;
}

bool pcfEnsureRunning24h() {
  uint8_t ctrl1 = 0;
  if (!pcfReadBytes(PCF85063_REG_CTRL1, &ctrl1, 1)) {
    return false;
  }
  const uint8_t desired = static_cast<uint8_t>(ctrl1 & ~(PCF85063_CTRL1_STOP_BIT | PCF85063_CTRL1_12H_BIT));
  if (desired != ctrl1) {
    return pcfWriteBytes(PCF85063_REG_CTRL1, &desired, 1);
  }
  return true;
}

// Per NXP PCF85063A datasheet 8.2.1: set STOP=1 while writing 0x04..0x0A, then clear STOP=0
bool pcfWriteLocalTm(const struct tm &local_tm) {
  const int year = local_tm.tm_year + 1900;
  if (year < 2000 || year > 2099 || !lockI2c()) {
    return false;
  }

  uint8_t ctrl1 = 0;
  bool ok = pcfReadBytesUnlocked(PCF85063_REG_CTRL1, &ctrl1, 1);
  if (ok) {
    const uint8_t ctrl1_stop = static_cast<uint8_t>((ctrl1 & ~PCF85063_CTRL1_12H_BIT) | PCF85063_CTRL1_STOP_BIT);
    ok = pcfWriteBytesUnlocked(PCF85063_REG_CTRL1, &ctrl1_stop, 1);
  }

  if (ok) {
    uint8_t buf[7];
    buf[0] = static_cast<uint8_t>(decToBcd(static_cast<uint8_t>(local_tm.tm_sec)) & 0x7FU);  // Clears OS bit
    buf[1] = decToBcd(static_cast<uint8_t>(local_tm.tm_min));
    buf[2] = decToBcd(static_cast<uint8_t>(local_tm.tm_hour));
    buf[3] = decToBcd(static_cast<uint8_t>(local_tm.tm_mday));
    buf[4] = decToBcd(static_cast<uint8_t>(local_tm.tm_wday & 0x07));
    buf[5] = decToBcd(static_cast<uint8_t>(local_tm.tm_mon + 1));
    buf[6] = decToBcd(static_cast<uint8_t>(year % 100));
    ok = pcfWriteBytesUnlocked(PCF85063_REG_SEC, buf, sizeof(buf));
  }

  if (ok) {
    const uint8_t ctrl1_run = static_cast<uint8_t>(ctrl1 & ~(PCF85063_CTRL1_STOP_BIT | PCF85063_CTRL1_12H_BIT));
    ok = pcfWriteBytesUnlocked(PCF85063_REG_CTRL1, &ctrl1_run, 1);
  }

  unlockI2c();
  return ok;
}

bool pcfReadLocalTm(struct tm *out_tm, bool *out_os_stopped) {
  if (out_tm == nullptr) {
    return false;
  }
  uint8_t raw[7] = {};
  if (!pcfReadBytes(PCF85063_REG_SEC, raw, sizeof(raw))) {
    return false;
  }

  if (out_os_stopped != nullptr) {
    *out_os_stopped = ((raw[0] & PCF85063_SEC_OS_BIT) != 0);
  }

  const uint8_t sec   = bcdToDec(raw[0] & 0x7FU);
  const uint8_t min   = bcdToDec(raw[1] & 0x7FU);
  const uint8_t hour  = bcdToDec(raw[2] & 0x3FU);
  const uint8_t day   = bcdToDec(raw[3] & 0x3FU);
  const uint8_t wday  = bcdToDec(raw[4] & 0x07U);
  const uint8_t month = bcdToDec(raw[5] & 0x1FU);
  const uint16_t year = static_cast<uint16_t>(bcdToDec(raw[6]) + 2000U);

  if (sec > 59 || min > 59 || hour > 23 || day < 1 || day > 31 || month < 1 || month > 12) {
    return false;
  }

  memset(out_tm, 0, sizeof(struct tm));
  out_tm->tm_sec   = sec;
  out_tm->tm_min   = min;
  out_tm->tm_hour  = hour;
  out_tm->tm_mday  = day;
  out_tm->tm_wday  = wday;
  out_tm->tm_mon   = month - 1;
  out_tm->tm_year  = static_cast<int>(year) - 1900;
  out_tm->tm_isdst = 0;
  return true;
}

time_t parseBuildEpochLocal(const char *build_date, const char *build_time) {
  if (build_date == nullptr || build_time == nullptr) {
    return 0;
  }
  static const char * const kMonths = "JanFebMarAprMayJunJulAugSepOctNovDec";
  char mon_str[4] = {};
  int day = 1;
  int year = 2026;
  int hour = 0;
  int min = 0;
  int sec = 0;

  if (sscanf(build_date, "%3s %d %d", mon_str, &day, &year) != 3) {
    return 0;
  }
  if (sscanf(build_time, "%d:%d:%d", &hour, &min, &sec) != 3) {
    return 0;
  }

  const char *pos = strstr(kMonths, mon_str);
  const int mon_idx = (pos != nullptr) ? static_cast<int>((pos - kMonths) / 3) : 0;

  struct tm out_tm = {};
  out_tm.tm_year  = year - 1900;
  out_tm.tm_mon   = mon_idx;
  out_tm.tm_mday  = day;
  out_tm.tm_hour  = hour;
  out_tm.tm_min   = min;
  out_tm.tm_sec   = sec;
  out_tm.tm_isdst = 0;
  return mktime(&out_tm);
}

void setSystemClockFromEpoch(time_t epoch) {
  if (epoch > 0) {
    struct timeval tv = {};
    tv.tv_sec = epoch;
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
  }
}

void ntpSyncNotificationCallback(struct timeval *tv) {
  (void)tv;
  s_ntp_pending_write = true;
}

}  // namespace

bool RtcTime_Init(TwoWire &wire,
                  SemaphoreHandle_t i2c_mutex,
                  const char *build_date,
                  const char *build_time) {
  s_wire = &wire;
  s_i2c_mutex = i2c_mutex;

  setenv("TZ", NTP_POSIX_TZ, 1);
  tzset();

  const time_t build_epoch = parseBuildEpochLocal(build_date, build_time);
  const time_t upload_sync_epoch = (build_epoch > 0) ? (build_epoch + 8) : 0;  // +8s compile/flash lag

  Preferences prefs;
  const bool nvs_ok = prefs.begin("rtc", false);
  const int64_t stored_bld = nvs_ok ? prefs.getLong64("bld_ep", 0) : 0;
  const bool is_new_upload = (build_epoch > 0) && (stored_bld != static_cast<int64_t>(build_epoch));

  if (!pcfEnsureRunning24h()) {
    if (nvs_ok) prefs.end();
    Serial.printf("[RTC] PCF85063 (0x%02X) init/comm failed\n", I2C_ADDR_PCF85063);
    s_rtc_ok = false;
    if (build_epoch > 0) {
      setSystemClockFromEpoch(is_new_upload ? upload_sync_epoch : build_epoch);
    }
    return false;
  }

  struct tm rtc_tm = {};
  bool os_stopped = false;
  const bool valid_read = pcfReadLocalTm(&rtc_tm, &os_stopped);
  const time_t rtc_epoch = valid_read ? mktime(&rtc_tm) : 0;
  const int rtc_year = rtc_tm.tm_year + 1900;

  if (!is_new_upload && valid_read && !os_stopped && rtc_epoch >= build_epoch) {
    if (nvs_ok) prefs.end();
    setSystemClockFromEpoch(rtc_epoch);
    s_rtc_ok = true;
    char iso[32] = {};
    RtcTime_FormatIso8601(time(nullptr), iso, sizeof(iso));
    Serial.printf("[RTC] PCF85063 OK (OS=0) -> System clock set: %s\n", iso);
    return true;
  }

  const time_t target_epoch = is_new_upload ? upload_sync_epoch : build_epoch;
  struct tm target_tm = {};
  localtime_r(&target_epoch, &target_tm);
  setSystemClockFromEpoch(target_epoch);

  const bool wrote = pcfWriteLocalTm(target_tm);
  if (wrote && is_new_upload && nvs_ok) {
    prefs.putLong64("bld_ep", static_cast<int64_t>(build_epoch));
  }
  if (nvs_ok) prefs.end();

  s_rtc_ok = wrote;
  char iso[32] = {};
  RtcTime_FormatIso8601(time(nullptr), iso, sizeof(iso));
  if (is_new_upload) {
    Serial.printf("[RTC] New firmware upload -> PCF85063 synced: %s (%s)\n", iso, wrote ? "OK" : "ERR");
  } else {
    Serial.printf("[RTC] PCF85063 uninitialized (OS=%u, yr=%d) -> seeded from build time: %s (%s)\n",
                  os_stopped ? 1U : 0U,
                  valid_read ? rtc_year : 0,
                  iso,
                  wrote ? "OK" : "ERR");
  }
  return s_rtc_ok;
}

void RtcTime_OnWifiConnected() {
  if (!s_ntp_cb_registered) {
    sntp_set_time_sync_notification_cb(ntpSyncNotificationCallback);
    s_ntp_cb_registered = true;
  }
  configTzTime(NTP_POSIX_TZ, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);
  Serial.printf("[RTC] SNTP started (TZ=%s, servers=%s, %s)\n", NTP_POSIX_TZ, NTP_SERVER_1, NTP_SERVER_2);
}

void RtcTime_Poll() {
  const uint32_t now_ms = millis();
  const bool periodic_due = s_ntp_synced && ((now_ms - s_last_rtc_sync_ms) >= NTP_RTC_SYNC_INTERVAL_MS);

  if (!s_ntp_pending_write && !periodic_due) {
    return;
  }

  const time_t now_epoch = time(nullptr);
  struct tm local_tm = {};
  localtime_r(&now_epoch, &local_tm);
  if ((local_tm.tm_year + 1900) < 2026) {
    return;
  }

  s_ntp_pending_write = false;
  const bool wrote = pcfWriteLocalTm(local_tm);
  if (wrote) {
    s_rtc_ok = true;
    s_ntp_synced = true;
    s_last_rtc_sync_ms = now_ms;
    char iso[32] = {};
    RtcTime_FormatIso8601(now_epoch, iso, sizeof(iso));
    Serial.printf("[RTC] NTP synced -> PCF85063 updated: %s\n", iso);
  }
}

time_t RtcTime_NowEpoch() {
  return time(nullptr);
}

bool RtcTime_IsRtcOk() {
  return s_rtc_ok;
}

bool RtcTime_IsNtpSynced() {
  return s_ntp_synced;
}

bool RtcTime_FormatIso8601(time_t epoch, char *buf, size_t len) {
  if (buf == nullptr || len < 26) {
    return false;
  }
  struct tm local_tm = {};
  if (localtime_r(&epoch, &local_tm) == nullptr || (local_tm.tm_year + 1900) < 2000) {
    return false;
  }
  const int n = snprintf(buf, len, "%04d-%02d-%02dT%02d:%02d:%02d+07:00",
                         local_tm.tm_year + 1900,
                         local_tm.tm_mon + 1,
                         local_tm.tm_mday,
                         local_tm.tm_hour,
                         local_tm.tm_min,
                         local_tm.tm_sec);
  return (n > 0 && static_cast<size_t>(n) < len);
}

bool RtcTime_FormatDateYmd(time_t epoch, char *buf, size_t len) {
  if (buf == nullptr || len < 11) {
    return false;
  }
  struct tm local_tm = {};
  if (localtime_r(&epoch, &local_tm) == nullptr || (local_tm.tm_year + 1900) < 2000) {
    return false;
  }
  const int n = snprintf(buf, len, "%04d-%02d-%02d",
                         local_tm.tm_year + 1900,
                         local_tm.tm_mon + 1,
                         local_tm.tm_mday);
  return (n > 0 && static_cast<size_t>(n) < len);
}

bool RtcTime_FormatDisplayHm(time_t epoch, char *buf, size_t len) {
  if (buf == nullptr || len < 17) {
    return false;
  }
  struct tm local_tm = {};
  if (localtime_r(&epoch, &local_tm) == nullptr || (local_tm.tm_year + 1900) < 2000) {
    return false;
  }
  const int n = snprintf(buf, len, "%04d-%02d-%02d %02d:%02d",
                         local_tm.tm_year + 1900,
                         local_tm.tm_mon + 1,
                         local_tm.tm_mday,
                         local_tm.tm_hour,
                         local_tm.tm_min);
  return (n > 0 && static_cast<size_t>(n) < len);
}

bool RtcTime_FormatDisplayHms(time_t epoch, char *buf, size_t len) {
  if (buf == nullptr || len < 20) {
    return false;
  }
  struct tm local_tm = {};
  if (localtime_r(&epoch, &local_tm) == nullptr || (local_tm.tm_year + 1900) < 2000) {
    return false;
  }
  const int n = snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d",
                         local_tm.tm_year + 1900,
                         local_tm.tm_mon + 1,
                         local_tm.tm_mday,
                         local_tm.tm_hour,
                         local_tm.tm_min,
                         local_tm.tm_sec);
  return (n > 0 && static_cast<size_t>(n) < len);
}
