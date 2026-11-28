# NoiseEventLogger

Privacy-preserving **26-band IEC 61260 1/3-octave acoustic noise event & spectrum logger** (`63 Hz – 20 kHz`) + **IEC 61672-1 A-weighting (`LAeq`, `LA90`, `LAFmax`, `LASmax`)** for the **Waveshare `ESP32-S3-RLCD-4.2`** (`ESP32-S3-WROOM-1-N16R8`, 16 MB Flash, 8 MB OPI PSRAM).

Designed to run continuously 24/7 to separate continuous stationary high-frequency jet noise (`8 kHz – 20 kHz`, such as high-pressure gas burners) from broadband water splashes and transient/low-frequency ambient noise.

---

## Key Features (Phase 4)
- **Privacy-by-Design (Zero Raw Audio Retention):** Raw PCM DMA and hop buffers exist strictly in RAM and are wiped (`memset(..., 0, ...)`) immediately after DSP/FFT computation. Audio is never saved or streamed.
- **48 kHz Precision Acoustic Front-End:** `ES7210` dual-mic ADC in 4-slot TDM Master RX at `48 kHz` (`MCLK = 12.288 MHz`, `0 ppm` divider error on ESP32-S3 `PLL_F160M`), fixed manual gain (`+33.0 dB` via `ES7210_MIC_GAIN_DB` / `REG 0x43..0x46 = 0x1B`), AGC and automute explicitly disabled.
- **Real-Time Core 1 DSP (`esp-dsp` + Time-Domain IEC 61672-1):**
  - `4096`-pt Hann-windowed FFT (`hop = 3000`, `16 frames/s`, $\Delta f = 11.71875\text{ Hz}$) aggregated into **26 IEC 61260 1/3-octave bands (`63 Hz – 20 kHz`)** with boot Parseval self-test (`1k`, `4k`, `16k`).
  - Sample-by-sample 4-biquad DF2T **A-weighting** filter ($\le \pm 0.0012\text{ dB}$ from `20 Hz – 20 kHz`), **Fast (`125 ms`)** & **Slow (`1000 ms`)** time-weighted detectors (`LAeq`, `LA90`, `LAFmax`, `LASmax`), plus overload & under-range flags.
- **NVS SPL Calibration CLI (`CAL`):** Calibrate on-device using a 94 dB / 1 kHz IEC 60942 calibrator over Serial (`CAL 94.0`, `CAL OFFSET`, `CAL BAND`, `CAL STATUS`, `CAL RESET`) with persistent NVS storage — switches seamlessly between `dBFS` and `dBSPL` / `dBA`.
- **Automatic HF Jet Nuisance Detector (`hf_alert`):** Flags stationary narrowband high-frequency emissions (`8 kHz – 20 kHz` L90 emergence $\ge 6\text{ dB}$, $L_{eq}-L_{90} \le 2.5\text{ dB}$, prominence $\ge 4\text{ dB}$), rejecting water spray and AC compressors, and tracks cumulative daily event minutes.
- **Reflective LCD UI (`ST7305` $400 \times 300$):** 1 Hz 26-band spectrum (`63 Hz – 20 kHz`), solid `1m L90` bars + `1s` live overlay, contrast-inverting anechoic `--=Floor` reference, `LAeq`/`LA90` readouts, `! HF JET ALERT !` banner, `OVL`/`UR` badges, `YYYY-MM-DD HH:MM:SS`, `BAT%`, Wi-Fi RSSI, SD status, and HTTP queue status.
- **Manual Event Marker (`KEY` Button), Wi-Fi Auto-Sleep & Low-Battery Protection:** Pressing onboard `KEY` (`GPIO 18`) toggles live ground-truth `marker` (`*MARK*` on RLCD + `marker=1` in CSV/JSON). When `INGEST_URL` is empty (NTP-only mode), Wi-Fi powers down (`WiFi.mode(WIFI_OFF)`, RLCD `W:SLP`) immediately after updating `PCF85063` and wakes once every 24h for time re-sync. If battery stays `2500–3350 mV` for 3 consecutive minutes, `SD_MMC` safely unmounts and enters `esp_deep_sleep_start()` showing `LOW BATTERY - SLEEP`.
- **Dual Persistence (Core 0):** Daily 69-column CSV on MicroSD (`/logs/YYYY-MM-DD.csv`, with automatic `_p4.csv` / `_p4v2.csv` schema protection) and 16-slot `1536 B` OPI PSRAM HTTP POST retry queue.
- **Offline Daily Evidence Analyzer (`tools/analyze_noise_log.py`):** Generates high-res 3-panel spectrograms, rejected non-target event summaries, and bilingual Thai/English Markdown/CSV evidence reports from both Phase 3 (24-band) and Phase 4 (26-band) logs.

---

## Repository Layout
```text
NoiseEventLogger/
├── reference/              # Official Waveshare ESP32-S3-RLCD-4.2 demos (read-only)
├── SPEC.md                 # Full engineering & mathematical specification (Phases 1–4)
├── TODO.md                 # IEC 61672 L1–L3 & PTEC calibration roadmap
├── requirements.txt        # Python packages for offline analyzer
├── tools/
│   └── analyze_noise_log.py# Daily spectrogram & nuisance evidence report generator
├── NoiseEventLogger.ino    # Boot sequence, dual-core FreeRTOS orchestration
├── config.h                # 48 kHz DSP constants, 26 bands, HF detector thresholds
├── board_pins.h            # Verified Waveshare ESP32-S3-RLCD-4.2 pinout
├── secrets.example.h       # Copy to secrets.h with Wi-Fi / HTTP credentials
├── calibration.h / .cpp    # NVS SPL calibration & non-blocking Serial CAL CLI
├── audio_capture.h / .cpp  # I2S TDM (48 kHz) + fixed-gain ES7210 driver
├── dsp_octave.h / .cpp     # esp-dsp 4096-pt FFT, 26 bands, IEC A-wt F/S, HF detector
├── rtc_time.h / .cpp       # PCF85063 driver + NTP sync (UTC+7 / ICT-7)
├── storage_log.h / .cpp    # SD_MMC 1-bit 69-column daily CSV logger
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
3. **Credentials:** Copy `secrets.example.h` to `secrets.h` (gitignored), or leave `WIFI_SSID` empty for pure offline RTC + SD + RLCD logging.
4. **Serial SPL Calibration (`115200 baud`):** Attach an IEC 60942 94.0 dB / 1 kHz calibrator, wait ~2 seconds, and send `CAL 94.0` in Serial Monitor (`CAL HELP` lists all commands).

---

## Python Evidence Analyzer Quick Start
```powershell
python -m venv .venv
.\.venv\Scripts\pip.exe install -r requirements.txt
.\.venv\Scripts\python.exe tools\analyze_noise_log.py logs\YYYY-MM-DD.csv --start 08:10
```
Outputs `<date>_evidence_spectrogram.png`, `<date>_evidence_report.md`, and `<date>_hf_episodes.csv` under `reports/<date>/`.

---

**Hardware Reference:** [waveshareteam/ESP32-S3-RLCD-4.2](https://github.com/waveshareteam/ESP32-S3-RLCD-4.2) · [Waveshare Wiki](https://docs.waveshare.com/ESP32-S3-RLCD-4.2) · [SPEC.md](SPEC.md)
