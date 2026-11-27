# NoiseEventLogger

Privacy-preserving **24-band IEC 61260 1/3-octave acoustic noise event & spectrum logger** (`63 Hz – 12.5 kHz`) for the **Waveshare `ESP32-S3-RLCD-4.2`** (`ESP32-S3-WROOM-1-N16R8`, 16 MB Flash, 8 MB OPI PSRAM).

Designed to run continuously 24/7 to separate stationary mid-to-high-frequency noise (`1 kHz – 12.5 kHz`) from transient environmental events by logging both broadband and per-band $L_{eq}$ and $L_{90}$ every minute.

---

## Key Features
- **Privacy-by-Design (Zero Raw Audio Retention):** Raw PCM DMA and hop buffers exist strictly in RAM and are wiped (`memset(..., 0, ...)`) immediately after FFT computation. Audio is never saved or streamed.
- **Stable Acoustic Front-End:** `ES7210` dual-mic ADC configured in 4-slot TDM Master RX at `32 kHz` (`MCLK = 8.192 MHz`), fixed manual gain (`+24 dB`), AGC and automute explicitly disabled.
- **Real-Time Core 1 DSP (`esp-dsp`):** `4096`-pt Hann-windowed FFT (`hop = 2000`, `16 frames/s`, $\Delta f = 7.8125\text{ Hz}$) aggregated into 24 IEC 61260 1/3-octave bands (`63 Hz` to `12.5 kHz`) with Parseval self-test on boot.
- **Anechoic-Calibrated Hardware Baseline:** Empirical 24-band self-noise floor measured in an anechoic chamber (`bb_l90 = -79.6 dBFS`, `1k–12.5k = -93.9 to -96.0 dBFS`) embedded in `config.h` (`ANECHOIC_OCT_L90_DBFS`).
- **Low-Power Reflective LCD UI (`ST7305` $400 \times 300$):** 1 Hz dual-layer spectrum display showing solid `1m L90` bars, live `1s` level overlay boxes, contrast-inverting dashed `--=Floor` anechoic reference line, `YYYY-MM-DD HH:MM:SS`, `BAT%`, Wi-Fi RSSI, SD status, and HTTP queue status.
- **Timekeeping (`PCF85063` + NTP):** Auto-syncs on new firmware upload via NVS build tracking, preserves RTC across reboots, and syncs via SNTP (`UTC+7` / `ICT-7`) when Wi-Fi connects.
- **Dual Persistence (Core 0):**
  - Daily rotating 56-column CSV files on MicroSD (`/logs/YYYY-MM-DD.csv`) over 1-bit `SD_MMC`.
  - Non-blocking Wi-Fi + `16`-slot OPI PSRAM `HTTP POST` JSON ring buffer (`3000 ms` timeout).

---

## Repository Layout
```text
NoiseEventLogger/
├── SPEC.md                 # Full engineering & mathematical specification
├── NoiseEventLogger.ino    # Boot sequence, dual-core FreeRTOS orchestration
├── config.h                # DSP constants, intervals, Anechoic baseline array
├── board_pins.h            # Verified Waveshare ESP32-S3-RLCD-4.2 pinout
├── secrets.example.h       # Copy to secrets.h with Wi-Fi / HTTP credentials
├── audio_capture.h / .cpp  # I2S TDM + fixed-gain ES7210 driver
├── dsp_octave.h / .cpp     # esp-dsp 4096-pt FFT, 24 1/3-octave bands, Leq/L90
├── rtc_time.h / .cpp       # PCF85063 driver + NTP sync (UTC+7 / ICT-7)
├── storage_log.h / .cpp    # SD_MMC 1-bit daily CSV logger
├── network_sync.h / .cpp   # Non-blocking Wi-Fi & PSRAM HTTP POST queue
└── display_ui.h / .cpp     # ST7305 400x300 RLCD driver & live spectrum UI
```

---

## Arduino IDE Quick Start
1. **Board Package:** `arduino-esp32` `3.3.12` (ESP-IDF `v5.5.5`) — **no external libraries required** (uses only built-in `Wire`, `SD_MMC`, `WiFi`, `HTTPClient`, `Preferences`, `esp_dsp`, and `esp_lcd_panel_io`).
2. **Board Settings:**
   - **Board:** `ESP32S3 Dev Module`
   - **USB CDC On Boot:** `Enabled`
   - **Flash Size:** `16MB (128Mb)`
   - **Partition Scheme:** `16M Flash (3MB APP/9.9MB FATFS)`
   - **PSRAM:** `OPI PSRAM`
3. **Credentials:**
   - Copy `secrets.example.h` to `secrets.h` (already in `.gitignore`).
   - Leave `WIFI_SSID` empty (`""`) for pure offline RTC + SD + RLCD logging, or fill `WIFI_SSID` / `WIFI_PASSWORD` for automatic NTP sync (and optionally `INGEST_URL` for HTTP POST).
4. **Hardware Reference:** Official Waveshare schematics and demos are available at [waveshareteam/ESP32-S3-RLCD-4.2](https://github.com/waveshareteam/ESP32-S3-RLCD-4.2) and the [Waveshare Wiki](https://docs.waveshare.com/ESP32-S3-RLCD-4.2). See [SPEC.md](SPEC.md) for the full design specification.
