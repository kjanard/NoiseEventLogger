#include "calibration.h"

#include <Preferences.h>
#include <atomic>
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

namespace {

constexpr const char *CAL_NVS_NS       = "cal";
constexpr const char *KEY_ACTIVE       = "active";
constexpr const char *KEY_OFFSET_DB    = "ofs_db";
constexpr const char *KEY_BAND_OFS     = "b_ofs";
constexpr uint8_t     BAND_IDX_1K      = 12;

std::atomic<bool>  s_calibrated{false};
std::atomic<float> s_offset_db{DEFAULT_CALIBRATION_OFFSET_DB};
std::atomic<float> s_band_offset_db[OCTAVE_BAND_COUNT];

char   s_cmd_buf[96] = {};
size_t s_cmd_len     = 0;

bool saveToNvs() {
  Preferences prefs;
  if (!prefs.begin(CAL_NVS_NS, false)) {
    Serial.println("[CAL] ERROR: Failed to open NVS namespace 'cal' for writing");
    return false;
  }

  float band_copy[OCTAVE_BAND_COUNT] = {};
  for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
    band_copy[b] = s_band_offset_db[b].load(std::memory_order_relaxed);
  }

  const bool  active = s_calibrated.load(std::memory_order_relaxed);
  const float ofs    = s_offset_db.load(std::memory_order_relaxed);

  prefs.putBool(KEY_ACTIVE, active);
  prefs.putFloat(KEY_OFFSET_DB, ofs);
  const size_t written = prefs.putBytes(KEY_BAND_OFS, band_copy, sizeof(band_copy));
  prefs.end();

  if (written != sizeof(band_copy)) {
    Serial.println("[CAL] WARN: Incomplete band offset write to NVS");
    return false;
  }
  return true;
}

void printHelp() {
  Serial.println("[CAL] Commands:");
  Serial.println("  CAL STATUS                  - Show current calibration state & 1s levels");
  Serial.println("  CAL <ref_dBSPL>             - Calibrate using 1 kHz tone (e.g. CAL 94.0)");
  Serial.println("  CAL OFFSET <dB>             - Set global offset directly (-20..+160)");
  Serial.println("  CAL BAND <idx|label> <dB>   - Set per-band trim (e.g. CAL BAND 8k -0.8)");
  Serial.println("  CAL RESET                   - Revert to raw uncalibrated dBFS (0.0 dB)");
}

void printStatus(const SecondReport_t *latest_sec, bool has_sec) {
  const bool  cal = s_calibrated.load(std::memory_order_relaxed);
  const float ofs = s_offset_db.load(std::memory_order_relaxed);
  Serial.printf("[CAL] calibrated=%s | unit=%s | global_offset=%+.2f dB\n",
                cal ? "YES" : "NO",
                cal ? "dBSPL" : "dBFS",
                ofs);

  char line[256];
  int pos = snprintf(line, sizeof(line), "[CAL] band_ofs:");
  for (uint8_t b = 0; b < OCTAVE_BAND_COUNT && pos > 0 && static_cast<size_t>(pos) < sizeof(line); ++b) {
    const float bo = s_band_offset_db[b].load(std::memory_order_relaxed);
    pos += snprintf(line + pos, sizeof(line) - static_cast<size_t>(pos), " %s:%+.1f", OCTAVE_BAND_LABELS[b], bo);
  }
  Serial.println(line);

  if (has_sec && latest_sec != nullptr) {
    const float b12_ofs = s_band_offset_db[BAND_IDX_1K].load(std::memory_order_relaxed);
    const float raw_1k  = latest_sec->oct_1s[BAND_IDX_1K] - ofs - b12_ofs;
    Serial.printf("[CAL] live_1s: bb=%.2f LAeq=%.2f 1kHz=%.2f %s (1kHz raw=%.2f dBFS, clips=%lu)\n",
                  latest_sec->bb_leq_1s,
                  latest_sec->laeq_1s,
                  latest_sec->oct_1s[BAND_IDX_1K],
                  cal ? "dBSPL" : "dBFS",
                  raw_1k,
                  static_cast<unsigned long>(latest_sec->clips_1s));
  }
}

int parseBandToken(const char *tok) {
  if (tok == nullptr || tok[0] == '\0') {
    return -1;
  }

  for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
    if (strcasecmp(tok, OCTAVE_BAND_LABELS[b]) == 0) {
      return static_cast<int>(b);
    }
  }

  char *endptr = nullptr;
  long idx = strtol(tok, &endptr, 10);
  if (endptr != tok && *endptr == '\0' && idx >= 0 && idx < OCTAVE_BAND_COUNT) {
    return static_cast<int>(idx);
  }
  return -1;
}

bool parseFloatStrict(const char *tok, float *out_val) {
  if (tok == nullptr || tok[0] == '\0' || out_val == nullptr) {
    return false;
  }
  char *endptr = nullptr;
  const float val = strtof(tok, &endptr);
  if (endptr == tok || *endptr != '\0' || !isfinite(val)) {
    return false;
  }
  *out_val = val;
  return true;
}

void handleCommand(char *line, const SecondReport_t *latest_sec, bool has_sec) {
  // Trim leading/trailing ASCII whitespace
  while (*line != '\0' && isspace(static_cast<unsigned char>(*line))) {
    ++line;
  }
  size_t len = strlen(line);
  while (len > 0 && isspace(static_cast<unsigned char>(line[len - 1]))) {
    line[--len] = '\0';
  }
  if (len == 0) {
    return;
  }

  char *saveptr = nullptr;
  char *tok0 = strtok_r(line, " \t", &saveptr);
  if (tok0 == nullptr || strcasecmp(tok0, "CAL") != 0) {
    Serial.printf("[CAL] Unknown command '%s' (type 'CAL HELP')\n", tok0 ? tok0 : "");
    return;
  }

  char *tok1 = strtok_r(nullptr, " \t", &saveptr);
  if (tok1 == nullptr || strcasecmp(tok1, "HELP") == 0) {
    printHelp();
    return;
  }

  if (strcasecmp(tok1, "STATUS") == 0) {
    printStatus(latest_sec, has_sec);
    return;
  }

  if (strcasecmp(tok1, "RESET") == 0) {
    s_calibrated.store(false, std::memory_order_relaxed);
    s_offset_db.store(DEFAULT_CALIBRATION_OFFSET_DB, std::memory_order_relaxed);
    for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
      s_band_offset_db[b].store(0.0f, std::memory_order_relaxed);
    }
    saveToNvs();
    Serial.println("[CAL] Reset complete -> unit=dBFS, global_offset=0.00 dB, band_ofs=0.0");
    return;
  }

  if (strcasecmp(tok1, "OFFSET") == 0) {
    char *tok2 = strtok_r(nullptr, " \t", &saveptr);
    float val = 0.0f;
    if (!parseFloatStrict(tok2, &val) || val < -20.0f || val > 160.0f) {
      Serial.println("[CAL] ERROR: Usage 'CAL OFFSET <dB>' (valid range -20.0..+160.0)");
      return;
    }
    s_offset_db.store(val, std::memory_order_relaxed);
    s_calibrated.store(fabsf(val) >= 0.05f, std::memory_order_relaxed);
    saveToNvs();
    Serial.printf("[CAL] Global offset set to %+.2f dB (unit=%s)\n",
                  val, s_calibrated.load(std::memory_order_relaxed) ? "dBSPL" : "dBFS");
    return;
  }

  if (strcasecmp(tok1, "BAND") == 0) {
    char *tok2 = strtok_r(nullptr, " \t", &saveptr);
    char *tok3 = strtok_r(nullptr, " \t", &saveptr);
    const int b = parseBandToken(tok2);
    float val = 0.0f;
    if (b < 0 || !parseFloatStrict(tok3, &val) || val < -20.0f || val > 20.0f) {
      Serial.println("[CAL] ERROR: Usage 'CAL BAND <0..25|label> <dB>' (valid -20.0..+20.0)");
      return;
    }
    s_band_offset_db[b].store(val, std::memory_order_relaxed);
    saveToNvs();
    Serial.printf("[CAL] Band %d (%s Hz) trim set to %+.2f dB\n", b, OCTAVE_BAND_LABELS[b], val);
    return;
  }

  // Otherwise expect numeric target SPL: CAL <ref_spl_db> (e.g., CAL 94.0)
  float ref_spl = 0.0f;
  if (!parseFloatStrict(tok1, &ref_spl) || ref_spl < 70.0f || ref_spl > 130.0f) {
    Serial.println("[CAL] ERROR: Invalid CAL argument. Use 'CAL 94.0' (70..130) or 'CAL HELP'");
    return;
  }

  if (!has_sec || latest_sec == nullptr) {
    Serial.println("[CAL] ERROR: Waiting for first 1s audio frame before calibrating");
    return;
  }
  if (latest_sec->clips_1s > 0 || latest_sec->overload_1s) {
    Serial.println("[CAL] ERROR: Input clipped/overloaded during last 1s; calibration aborted");
    return;
  }

  const float cur_ofs = s_offset_db.load(std::memory_order_relaxed);
  const float b12_ofs = s_band_offset_db[BAND_IDX_1K].load(std::memory_order_relaxed);
  const float raw_1k  = latest_sec->oct_1s[BAND_IDX_1K] - cur_ofs - b12_ofs;

  if (raw_1k < -65.0f) {
    Serial.printf("[CAL] ERROR: 1 kHz level too low (raw=%.2f dBFS < -65.0 dBFS). "
                  "Apply calibrator tone first.\n", raw_1k);
    return;
  }

  const float new_ofs = ref_spl - raw_1k - b12_ofs;
  if (new_ofs < 50.0f || new_ofs > 160.0f) {
    Serial.printf("[CAL] ERROR: Computed offset %+.2f dB out of sane bounds [50..160]\n", new_ofs);
    return;
  }

  s_offset_db.store(new_ofs, std::memory_order_relaxed);
  s_calibrated.store(true, std::memory_order_relaxed);
  saveToNvs();
  Serial.printf("[CAL] OK: 1kHz raw=%.2f dBFS -> %.1f dBSPL (offset=%+.2f dB)\n",
                raw_1k, ref_spl, new_ofs);
}

}  // namespace

bool Calibration_Init() {
  for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
    s_band_offset_db[b].store(0.0f, std::memory_order_relaxed);
  }
  s_offset_db.store(DEFAULT_CALIBRATION_OFFSET_DB, std::memory_order_relaxed);
  s_calibrated.store(false, std::memory_order_relaxed);

  Preferences prefs;
  if (!prefs.begin(CAL_NVS_NS, true)) {
    Serial.println("[CAL] No stored NVS calibration (using raw dBFS, offset=0.00 dB)");
    return false;
  }

  const bool  active = prefs.getBool(KEY_ACTIVE, false);
  float       ofs    = prefs.getFloat(KEY_OFFSET_DB, DEFAULT_CALIBRATION_OFFSET_DB);
  if (!isfinite(ofs) || ofs < -20.0f || ofs > 160.0f) {
    ofs = DEFAULT_CALIBRATION_OFFSET_DB;
  }

  float band_copy[OCTAVE_BAND_COUNT] = {};
  const size_t got = prefs.getBytes(KEY_BAND_OFS, band_copy, sizeof(band_copy));
  prefs.end();

  s_calibrated.store(active, std::memory_order_relaxed);
  s_offset_db.store(ofs, std::memory_order_relaxed);
  if (got == sizeof(band_copy)) {
    for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
      if (isfinite(band_copy[b]) && band_copy[b] >= -20.0f && band_copy[b] <= 20.0f) {
        s_band_offset_db[b].store(band_copy[b], std::memory_order_relaxed);
      }
    }
  }

  Serial.printf("[CAL] NVS loaded: calibrated=%s | unit=%s | offset=%+.2f dB\n",
                active ? "YES" : "NO",
                active ? "dBSPL" : "dBFS",
                ofs);
  return true;
}

float Calibration_GetOffsetDb() {
  return s_offset_db.load(std::memory_order_relaxed);
}

float Calibration_GetBandOffsetDb(uint8_t band_idx) {
  if (band_idx >= OCTAVE_BAND_COUNT) {
    return 0.0f;
  }
  return s_band_offset_db[band_idx].load(std::memory_order_relaxed);
}

bool Calibration_IsCalibrated() {
  return s_calibrated.load(std::memory_order_relaxed);
}

const char *Calibration_GetUnitLabel() {
  return s_calibrated.load(std::memory_order_relaxed) ? "dBSPL" : "dBFS";
}

void Calibration_PollSerial(const SecondReport_t *latest_sec, bool has_sec) {
  while (Serial.available() > 0) {
    const int ch = Serial.read();
    if (ch < 0) {
      break;
    }
    if (ch == '\r' || ch == '\n') {
      if (s_cmd_len > 0) {
        s_cmd_buf[s_cmd_len] = '\0';
        handleCommand(s_cmd_buf, latest_sec, has_sec);
        s_cmd_len = 0;
      }
    } else if (s_cmd_len + 1U < sizeof(s_cmd_buf)) {
      s_cmd_buf[s_cmd_len++] = static_cast<char>(ch);
    }
  }
}
