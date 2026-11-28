# Changelog

All notable changes to **NoiseEventLogger** are documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

Because this project collects long-term acoustic logs and MicroSD CSV evidence across multiple days, every firmware release that alters sample rate, microphone PGA gain, FFT/band layout, weighting filters, or CSV schema is recorded here for historical log auditability.

---

## [Unreleased]
- Planned (`Phase 5`, see [TODO.md](TODO.md)): `SHTC3` (`0x70`) ambient temperature (`temp_c`) & relative humidity (`rh_pct`) logging and IEC 61672 / PTEC lab calibration workflow.

---

## [0.4.0] - 2026-11-28 (Phase 4)
**Git commit:** `c91cd2b`

### Added
- **48 kHz TDM Audio & 26-Band `1/3`-Octave Spectrum (`63 Hz – 20 kHz`):**
  - Switched I2S TDM capture to `48,000 Hz` (`I2S_MCLK_MULTIPLE_256`, exact `12.288 MHz` MCLK from `PLL_F160M`) with `4096`-pt Hann FFT (`hop = 3000`, `16 hops/s`, $\Delta f = 11.71875\text{ Hz}$).
  - Extended IEC 61260 bands from 24 to **26 bands**, adding `16 kHz` (bins `1217..1532`) and `20 kHz` (bins `1533..1930`), plus `1k`/`4k`/`16k` boot Parseval self-test (`[SelfTest] ... -> PASS`).
- **IEC 61672-1 Time-Domain A-Weighting & F/S Detectors:**
  - 4-biquad DF2T A-weighting filter (`double` accumulator state) and Fast (`125 ms`) / Slow (`1.0 s`) exponential integrators producing `laeq`, `la90`, `laf_max`, `las_max`, `overload`, and `underrange`.
- **Stationary High-Frequency Jet Nuisance Detector (`hf_alert`):**
  - Evaluates `8 kHz – 20 kHz` `1/3`-octave $L_{90}$ in real time against hardware self-noise (`>= 6.0 dB` emergence, `Leq - L90 <= 2.5 dB` stationarity, `>= 4.0 dB` adjacent-band prominence, `>= 3.8 dB` emergence over `3.15 – 6.3 kHz`), suppressing broadband water spray and low-frequency compressor false positives while tracking `hf_events_today`.
- **Persistent NVS Calibration & Serial `CAL` CLI (`calibration.h/.cpp`):**
  - Added non-blocking `115200` baud CLI (`CAL HELP`, `CAL STATUS`, `CAL <ref_dBSPL>`, `CAL OFFSET <dB>`, `CAL BAND <idx|label> <dB>`, `CAL RESET`) stored in NVS namespace `"cal"`.
- **Manual Ground-Truth Tagging (`KEY` Button) & Schema-Safe 69-Column CSV:**
  - Mapped onboard `KEY` (`GPIO 18`, `50 ms` debounce + within-minute latch) to toggle ground-truth `marker` (`0`/`1`), displayed as `*MARK*` / `M:OFF` alongside `G:+33dB` on the `ST7305` RLCD.
  - Expanded MicroSD CSV to **69 columns** (adds `gain_db`, `marker`, `laeq`, `la90`, `laf_max`, `las_max`, `overload`, `underrange`, `hf_alert`, `hf_band_hz`, `hf_prom_db`, `leq_16k`, `leq_20k`, `l90_16k`, `l90_20k`). Automatically switches to `_p4.csv` / `_p4v2.csv` when an older schema exists for the same date.
- **Low-Battery Deep Sleep SD Protection:**
  - Enters `esp_deep_sleep_start()` showing `LOW BATTERY - SLEEP` (held via `gpio_deep_sleep_hold_en()`) after safely flushing/unmounting `SD_MMC`, powering down Wi-Fi & `ES7210`, whenever `2500 mV <= vbat_mv <= 3350 mV` persists for `3` consecutive minutes (wakeable via `KEY` or reset).
- **NTP-Only Wi-Fi Auto Power-Off (`WIFI_SM_NTP_SLEEP` / `W:SLP`):**
  - When `WIFI_SSID` is configured and `INGEST_URL` is empty, automatically runs `WiFi.mode(WIFI_OFF)` once `PCF85063` is synced from SNTP (`NTP_WIFI_SYNC_TIMEOUT_MS = 30000 ms` fallback to `WIFI_OFF` backoff) and wakes once every `24 h` (`NTP_RTC_SYNC_INTERVAL_MS`) to re-sync RTC time.
- **Offline Evidence Analyzer (`tools/analyze_noise_log.py`):**
  - Parses both 24-band (Phase 3) and 26-band (Phase 4) logs to produce 3-panel high-resolution spectrograms, rejected non-target event clusters, episode CSV summaries, and bilingual Thai/English Markdown reports.

### Changed
- **Fixed `ES7210` Microphone PGA Gain Increased from `+24.0 dB` to `+33.0 dB`:**
  - Updated `ES7210_MIC_GAIN_DB = 33.0f` (`REG 0x43..0x46 = 0x1B`) and fixed `es7210GainCodeFromDb()` rounding so outdoor `4 kHz – 20 kHz` ambient tails lift clear of the ADC noise floor while keeping AGC/ALC off.
- **HTTP JSON Ring & Serial TX:** Increased PSRAM slot size (`HTTP_JSON_SLOT_SIZE = 1536`) and `Serial.setTxBufferSize(4096)` for non-blocking 1-minute JSON output.

### Fixed
- Guarded `periodic_due` in `RtcTime_Poll()` with `esp_sntp_enabled()` so 24-hour periodic refresh triggers actual Wi-Fi wake & SNTP sync instead of re-stamping stale RTC time while Wi-Fi is sleeping.

---

## [0.3.0] - 2026-11-27 (Phases 1–3)
**Git commits:** `286c184`, `1670bad`, `0447fe3`

### Added
- **Initial Hardware & Dual-Core Firmware Architecture (`Phase 1–3`):**
  - Waveshare `ESP32-S3-RLCD-4.2` pin verification (`board_pins.h`) and Waveshare demo reference archive (`reference/`).
  - Core 1 `ES7210` 4-slot TDM capture (`32 kHz`, `+24 dB` PGA) + `esp-dsp` `2048`-pt Hann FFT into `24` IEC 61260 `1/3`-octave bands (`63 Hz – 12.5 kHz`) with strict RAM zero-wipe privacy.
  - Core 0 `PCF85063` RTC + SNTP (`ICT-7`), 1-bit `SD_MMC` `56`-column daily CSV (`/logs/YYYY-MM-DD.csv`), `ST7305` `400x300` RLCD spectrum & readout UI, and 16-slot PSRAM HTTP POST retry queue.
  - `TODO.md` roadmap covering IEC 61672 L1–L3 compliance and PTEC calibration cost estimates.
