# NoiseEventLogger

Privacy-preserving **26-band IEC 61260 1/3-octave acoustic noise event & spectrum logger** (`63 Hz – 20 kHz`) + **IEC 61672-1 A-weighting (`LAeq`, `LA90`, `LAFmax`, `LASmax`)** for the **Waveshare `ESP32-S3-RLCD-4.2`** (`ESP32-S3-WROOM-1-N16R8`, 16 MB Flash, 8 MB OPI PSRAM).

Designed for continuous 24/7 deployment to separate stationary high-frequency jet emissions (`8 kHz – 20 kHz`, e.g., high-pressure gas burners / เตาฟู่) from broadband water spray and low-frequency compressor/traffic ambient noise.

> Release history & acoustic parameter changes are tracked in [CHANGELOG.md](CHANGELOG.md). Technical architecture & math are documented in [SPEC.md](SPEC.md).

---

## Key Features (Phase 4)
- **Privacy-by-Design (Zero Raw Audio Retention):** Raw PCM DMA and hop buffers live strictly in RAM and are zero-wiped (`memset(..., 0, ...)`) immediately after DSP/FFT processing. Audio is never saved or streamed.
- **48 kHz Precision Acoustic Front-End:** `ES7210` dual-mic ADC in 4-slot TDM Master RX at `48,000 Hz` (`MCLK = 12.288 MHz`, `0 ppm` divider error on `PLL_F160M`), fixed manual PGA gain (`+33.0 dB` via `ES7210_MIC_GAIN_DB` / `REG 0x43..0x46 = 0x1B`), AGC and automute disabled.
- **Real-Time Core 1 DSP (`esp-dsp` + Time-Domain IEC 61672-1):**
  - `4096`-pt Hann-windowed FFT (`hop = 3000`, `16 frames/s`, $\Delta f = 11.71875\text{ Hz}$) into **26 IEC 61260 1/3-octave bands (`63 Hz – 20 kHz`)** with boot Parseval self-test (`1k`, `4k`, `16k`).
  - Sample-by-sample 4-biquad DF2T **A-weighting** filter ($\le \pm 0.0012\text{ dB}$ from `20 Hz – 20 kHz`), **Fast (`125 ms`)** & **Slow (`1000 ms`)** time-weighted detectors (`laeq`, `la90`, `laf_max`, `las_max`), plus `overload` & `underrange` flags.
- **Automatic HF Jet Nuisance Detector (`hf_alert`):** Flags stationary narrowband `8 kHz – 20 kHz` $L_{90}$ emissions (`>= 6.0 dB` above anechoic floor, $L_{eq} - L_{90} \le 2.5\text{ dB}$, adjacent prominence `>= 4.0 dB`, `>= 3.8 dB` over `3.15–6.3 kHz`), rejecting broadband water splashes and compressors while counting `hf_events_today`.
- **NTP-Only Wi-Fi Auto-Sleep (`W:SLP`):** When `WIFI_SSID` is set and `INGEST_URL` is empty, Wi-Fi automatically powers off (`WiFi.mode(WIFI_OFF)`) as soon as `PCF85063` is synced from SNTP, waking once every `24 h` to re-sync.
- **Manual Marker (`KEY`), Schema-Safe CSV & Low-Battery Deep Sleep:** `KEY` (`GPIO 18`) toggles live ground-truth `marker`; MicroSD writes 69-column daily CSVs (protecting older headers automatically); battery $\le 3350\text{ mV}$ for `3` consecutive minutes safely unmounts SD and enters Deep Sleep.

---

## RLCD Display & Onboard Button Guide (`ST7305` $400 \times 300$)

### Top Black Header Bar
| Indicator | Meaning |
| :--- | :--- |
| `YYYY-MM-DD HH:MM:SS` | Local system time (`ICT-7`, UTC+7). |
| `NTP` / `RTC` / `SYS` | Active clock confidence: synced from SNTP (`NTP`), `PCF85063` RTC (`RTC`), or build-time fallback (`SYS`). |
| `W:-XXdB` / `W:SLP` / `W:OFF` / `W:...` | Wi-Fi state: connected with RSSI (`W:-73dB`), sleeping between 24h NTP syncs (`W:SLP`), disabled (`W:OFF`), or connecting/backing off (`W:...`). |
| `SD:OK` / `SD:ERR` | MicroSD mount state. |
| `BAT:XX%` | 18650 battery estimate mapped from `3000 mV` (`0%`) to `4120 mV` (`100%`). |
| `HTTP:OFF` / `IDLE` / `OK` / `ERR` | HTTP upload queue status (`OFF` when `INGEST_URL` is empty). |

### Broadband & Alert Cards
- **Left Card (`LIVE 1s`):** Shows `1s LAeq (dBA)` when calibrated (`LIVE 1s LAeq (dBA)`) or broadband `dBFS` when uncalibrated (`LIVE 1s LEQ (dBFS)`), plus `LAFmax`/`LZ` (calibrated) or `A-wt`/`Pk` (uncalibrated), `1m` running averages (`1m Avg: Warmup XX/60s` during the first minute), and an inverted **`OVL`** badge if ADC clipping occurs.
- **Right Card (`1m L90 FLOOR` / `! HF JET ALERT (<band>Hz) !`):** Displays 1-minute stationary background floor (`la90` or `bb_l90`), HF peak band prominence & daily active alert minutes (`HF(16k):+X.XdB Day:Xm`), plus second countdown `Sec:XX/60`, clip/queue counters (`Clp:X Q:X`), or **`OVL!`** / **`UR`** badges. When the HF jet detector triggers, the card header inverts to `! HF JET ALERT (...) !`.

### 26-Band Spectrum Graph (`63 Hz – 20 kHz`)
- `G:+33dB`: Configured fixed `ES7210` mic PGA gain (`ES7210_MIC_GAIN_DB`).
- `*MARK*` (inverted black pill) / `M:OFF`: Live ground-truth `marker` state toggled by the onboard `KEY` button (`GPIO 18`). A press during any 1-minute window latches `marker=1` for that minute's CSV/JSON record even if toggled back off before the minute rolls over.
- **Bars & Overlay (`Bar=L90 Box=1s --=Floor`):**
  - **Solid black bars:** Completed 1-minute `1/3`-octave $L_{90}$ (or live `1s` bars during the first 60-second warmup).
  - **Hollow outline box above bar:** Live `1s` level exceeding `1m L90`.
  - **Contrast-inverting dashed line (`--=Floor`):** Measured anechoic self-noise floor reference across all 26 bands.

---

## Power, `secrets.h` Modes & Low-Battery Protection

Copy [secrets.example.h](secrets.example.h) to `secrets.h` (gitignored) and choose one of three operating profiles:

| Mode | `WIFI_SSID` | `INGEST_URL` | Wi-Fi Behavior |
| :--- | :---: | :---: | :--- |
| **1. Pure Offline** | `""` | `""` | Wi-Fi disabled (`W:OFF`). Uses `PCF85063` RTC + MicroSD + RLCD only. |
| **2. NTP-Only Auto-Sleep** *(Recommended for field battery use)* | `"YourSSID"` | `""` | Connects at boot, updates `PCF85063` via SNTP, then calls `WiFi.mode(WIFI_OFF)` (`W:SLP`). Wakes once every `24 h` to re-sync NTP. Radio also powers off during retry backoff. |
| **3. Live HTTP Ingest** | `"YourSSID"` | `"https://..."` | Keeps Wi-Fi active (`W:-XXdB`) and flushes the 16-slot PSRAM ring buffer (`POST` with Bearer auth if `INGEST_AUTH_TOKEN` is set). |

### Low-Battery SD Card Integrity Guard
- Reads `PIN_BAT_ADC` (`GPIO 4`) each minute.
- If `2500 mV <= vbat_mv <= 3350 mV` (`LOW_BAT_MIN_PRESENT_MV` .. `LOW_BAT_CUTOFF_MV`) for **`3` consecutive minutes**:
  1. Safely flushes and unmounts `SD_MMC` (`StorageLog_Shutdown()`).
  2. Powers down Wi-Fi and `ES7210` (`AudioCapture_PowerDown()`).
  3. Renders **`LOW BATTERY - SLEEP`** on the `ST7305` RLCD and locks `PIN_RLCD_RST` via `gpio_deep_sleep_hold_en()` so the alert remains visible.
  4. Enters `esp_deep_sleep_start()` (wakeable via `KEY` on `GPIO 18` or `RESET` / charging).

---

## Serial Calibration CLI (`115200 baud`)

No reflashing is needed when calibrating with a 94 dB / 1 kHz IEC 60942 calibrator. Offsets persist in ESP32 NVS namespace `"cal"` while internal DSP buffers remain in raw `dBFS`.

| Command | Description |
| :--- | :--- |
| `CAL HELP` | Print supported calibration commands. |
| `CAL STATUS` | Show current calibration status (`dBFS`/`dBSPL`), global offset, per-band trims, and live 1s readings. |
| `CAL <ref_dBSPL>` | Calibrate global offset from the live 1 kHz (`1k` band) level (e.g. `CAL 94.0`). |
| `CAL OFFSET <dB>` | Set global offset directly (`-20` to `+160` dB). |
| `CAL BAND <idx\|label> <dB>` | Apply per-band trim by index `0..25` or label (e.g. `CAL BAND 8k -0.8`). |
| `CAL RESET` | Clear calibration and revert to raw `dBFS` (`offset = 0.0 dB`). |

---

## MicroSD CSV & JSON Schemas

- **MicroSD Path:** `/logs/YYYY-MM-DD.csv` (`unsynced.csv` if RTC/NTP has not set year $\ge 2026$).
  - If an existing same-date CSV uses an older 56/67-column header, Phase 4 automatically protects it by writing to `_p4.csv` (or `_p4v2.csv`).
- **69 CSV Columns (`1`-minute cadence):**
  `ts,uptime_s,unit,gain_db,marker,bb_leq,bb_l90,bb_peak,laeq,la90,laf_max,las_max,clips,overload,underrange,hf_alert,hf_band_hz,hf_prom_db,leq_63..leq_20k (26),l90_63..l90_20k (26),vbat_mv,rssi`
- **Serial & HTTP JSON (`HTTP_JSON_SLOT_SIZE = 1536`):**
  Includes `"device_id"`, `"ts"`, `"uptime_s"`, `"unit"`, `"cal_db"`, `"fs_hz":48000`, `"gain_db"`, `"marker"`, `"bb":{leq,l90,peak,laeq,la90,laf_max,las_max,clips,ovl,ur}`, `"hf":{alert,band,prom_db,emerg_db,min_today}`, 26-element `"oct_leq"` / `"oct_l90"`, and `"sys":{vbat_mv,rssi,sd_ok}`. When Wi-Fi is sleeping/off, `"rssi"` reports `-127`.

---

## Repository Layout
```text
NoiseEventLogger/
├── reference/              # Official Waveshare ESP32-S3-RLCD-4.2 demos (read-only, Waveshare license)
├── LICENSE                 # MIT License
├── CHANGELOG.md            # Versioned acoustic/firmware/schema changelog
├── SPEC.md                 # Full engineering & mathematical specification (Phases 1–4)
├── TODO.md                 # IEC 61672 L1–L3 PTEC roadmap & Phase 5 SHTC3 backlog
├── requirements.txt        # Python packages for offline analyzer
├── tools/
│   └── analyze_noise_log.py# Daily spectrogram & nuisance evidence report generator
├── NoiseEventLogger.ino    # Boot sequence, dual-core FreeRTOS orchestration
├── config.h                # 48 kHz DSP constants, 26 bands, HF detector & battery thresholds
├── board_pins.h            # Verified Waveshare ESP32-S3-RLCD-4.2 pinout
├── secrets.example.h       # Copy to secrets.h with Wi-Fi / HTTP credentials (gitignored)
├── calibration.h / .cpp    # NVS SPL calibration & non-blocking Serial CAL CLI
├── audio_capture.h / .cpp  # I2S TDM (48 kHz) + fixed-gain ES7210 driver
├── dsp_octave.h / .cpp     # esp-dsp 4096-pt FFT, 26 bands, IEC A-wt F/S, HF detector
├── rtc_time.h / .cpp       # PCF85063 driver + NTP sync (UTC+7 / ICT-7)
├── storage_log.h / .cpp    # SD_MMC 1-bit 69-column daily CSV logger (schema-safe)
├── network_sync.h / .cpp   # Non-blocking Wi-Fi (NTP-only auto WIFI_OFF) & 1536B HTTP queue
└── display_ui.h / .cpp     # ST7305 400x300 26-band RLCD driver & live spectrum UI
```

---

## Arduino IDE Quick Start
1. **Board Package:** `arduino-esp32` `3.3.12` (ESP-IDF `v5.5.5`) — **no external libraries required**.
2. **Board Settings:**
   - **Board:** `ESP32S3 Dev Module`
   - **USB CDC On Boot:** `Enabled`
   - **Flash Size:** `16MB (128Mb)`
   - **Partition Scheme:** `16M Flash (3MB APP/9.9MB FATFS)`
   - **PSRAM:** `OPI PSRAM`
3. **Credentials:** Copy `secrets.example.h` to `secrets.h` (gitignored). Leave `INGEST_URL` empty for battery-friendly NTP-only auto-sleep, or leave `WIFI_SSID` empty for pure offline logging.

---

## Python Evidence Analyzer ([tools/analyze_noise_log.py](tools/analyze_noise_log.py))

Works seamlessly on both legacy 24-band Phase 3 CSVs and 26-band Phase 4 CSVs (scaling anechoic floor by `gain_db` and plotting `marker` intervals when present):

```powershell
python -m venv .venv
.\.venv\Scripts\pip.exe install -r requirements.txt
.\.venv\Scripts\python.exe tools\analyze_noise_log.py logs\2026-11-28.csv --start 08:10 --end 20:06 --baseline 18:10-18:44 --gap-bridge 3
```

| Option | Default | Description |
| :--- | :--- | :--- |
| `csv_path` | *(required)* | Path to daily CSV log file (`YYYY-MM-DD.csv` or `_p4.csv`). |
| `--out-dir` | `reports/<date>` | Output folder for generated evidence artifacts. |
| `--start` | Full file start | Optional window start (`HH:MM`). |
| `--end` | Full file end | Optional window end (`HH:MM`). |
| `--baseline` | Auto quietest window | Quiet baseline range (`HH:MM-HH:MM`) for $\Delta L_{90}$ contrast plots. |
| `--gap-bridge` | `3` | Maximum gap (minutes) bridged inside one continuous HF episode. |

Generated outputs under `reports/<date>/`:
- `<date>_evidence_spectrogram.png`: 3-panel spectrogram (`L90` waterfall, $\Delta L_{90}$ vs quiet baseline, and `HF` vs `LA90` time series highlighting broadband masking).
- `<date>_evidence_report.md`: Bilingual Thai/English evidence summary, detected episodes, and rejected non-target clusters (water spray / compressor).
- `<date>_hf_episodes.csv`: Per-episode timestamps, dominant band, max emergence, prominence, and `delta_la90_dba`.

---

**Hardware Reference:** [waveshareteam/ESP32-S3-RLCD-4.2](https://github.com/waveshareteam/ESP32-S3-RLCD-4.2) · [Waveshare Wiki](https://docs.waveshare.com/ESP32-S3-RLCD-4.2) · [SPEC.md](SPEC.md) · [CHANGELOG.md](CHANGELOG.md) · [LICENSE](LICENSE)
