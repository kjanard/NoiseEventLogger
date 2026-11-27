# Noise Event Logger — Firmware Specification (`SPEC.md`)

> **Target Platform:** Waveshare `ESP32-S3-RLCD-4.2` (`ESP32-S3-WROOM-1-N16R8`)  
> **Framework:** Arduino IDE / `arduino-esp32` (`.ino` entry point with modular `.h`/`.cpp` files)  
> **Development Strategy:** Strict Phased Delivery (Phase 1 $\rightarrow$ Phase 2 $\rightarrow$ Phase 3) on USB 5V power first. Power optimization and solar/battery sizing are explicitly deferred until Phase 3 is hardware-verified.

---

## 1. Project Purpose & Scope

Build a dedicated, privacy-preserving **Acoustic Noise Event & 1/3-Octave Spectrum Logger** capable of running continuously 24/7 to log broadband sound levels and **24-band 1/3-octave spectra** ($63\text{ Hz}$ to $12.5\text{ kHz}$, IEC 61260).

The primary analytical goal is to distinguish **continuous stationary mid-to-high-frequency noise** (e.g., $1\text{ kHz} - 12.5\text{ kHz}$) from **transient environmental noise** (vehicles, dogs, birds, wind) by recording both $L_{eq}$ and $L_{90}$ (10th percentile) per 1/3-octave band every minute.

### In Scope (Phases 1–3)
- Fixed-gain I2S audio capture from the onboard `ES7210` dual-microphone ADC.
- Real-time FFT via `esp-dsp` aggregated into 24 standard 1/3-octave bands.
- 1-minute statistical aggregation ($L_{eq}$, $L_{90}$, $L_{\text{peak}}$, clipping counter).
- Timekeeping via onboard `PCF85063` RTC synchronized via NTP.
- Minimal, low-refresh-rate status UI on the 4.2-inch monochrome `ST7305` RLCD ($400 \times 300$).
- Dual persistence: daily rotating CSV/JSONL files on MicroSD (`SD_MMC` 1-bit mode) and HTTP POST JSON delivery over Wi-Fi to a local server/NAS.

### Out of Scope (Deferred to Future Phase 4)
- Deep sleep / light sleep power cycling.
- Solar MPPT management or battery runtime optimization.
- Acoustic SPL calibration curve fitting (a single configurable `CALIBRATION_OFFSET_DB` constant is sufficient for now; default `0.0` = raw `dBFS`).
- Onboard `SHTC3` temp/humidity logging (optional nice-to-have only after Phase 3 is stable).

---

## 2. Target Hardware & Mandatory Pre-Flight Check

### 2.1 Board Summary (`Waveshare ESP32-S3-RLCD-4.2`)
- **MCU:** `ESP32-S3-WROOM-1-N16R8` (Dual-core LX7 @ $240\text{ MHz}$, $16\text{ MB}$ Flash, $8\text{ MB}$ OPI PSRAM)
- **Display:** 4.2-inch Monochrome Reflective LCD ($400 \times 300$), controller `ST7305`
- **Audio Input:** `ES7210` ADC connected to onboard dual-microphone array (board also includes `ES8311` codec — do not initialize speaker/playback path)
- **RTC:** `PCF85063` over I2C
- **Storage:** TF / MicroSD Card slot (`SD_MMC` 1-bit bus, FAT32)
- **Power / Battery:** USB Type-C 5V + 18650 holder + resistor-divider battery voltage ADC pin

### 2.2 Arduino IDE Build Configuration
All generated code MUST compile cleanly under Arduino IDE with the following settings:
- **Board:** `ESP32S3 Dev Module`
- **USB CDC On Boot:** `Enabled` (required for `Serial` output over native USB Type-C)
- **Flash Size:** `16MB (128Mb)`
- **Partition Scheme:** `16M Flash (3MB APP/9.9MB FATFS)` or `Huge APP`
- **PSRAM:** `OPI PSRAM`

### 2.3 MANDATORY PRE-FLIGHT RULE (DO NOT GUESS PINOUTS OR INIT SEQUENCES)
Before writing any hardware driver code:
1. Inspect `./reference/` (official Waveshare `ESP32-S3-RLCD-4.2` Arduino examples and/or schematic PDF).
2. Extract and verify every GPIO pin (`I2C`, `I2S`, `ES7210` I2C address, `ST7305` SPI/control pins, `PCF85063` I2C address, `SD_MMC` pins, Battery ADC pin) into `board_pins.h`.
3. Detect which `arduino-esp32` core version (`v2.x` legacy `driver/i2s.h` vs. `v3.x` `ESP_I2S.h` / `driver/i2s_std.h`) is targeted by the environment and reference demos.
4. For the `ST7305` RLCD (Phase 3), **reuse Waveshare's official initialization sequence** from `./reference/` (e.g., `ST7305_U8g2.cpp` or equivalent). Do NOT invent `ST7305` register init tables from memory.
5. If `./reference/` is missing any required pinout or driver file, **STOP and ask the user** instead of guessing.

---

## 3. Non-Negotiable Engineering & Privacy Guardrails

1. **STRICT PRIVACY-BY-DESIGN (Zero Audio Retention):**
   - **NEVER** record, save, cache to SD/Flash, or transmit raw PCM/WAV/MP3 audio over Wi-Fi or Serial.
   - Audio DMA buffers exist strictly in volatile RAM, are converted immediately into spectral/numeric metrics, and are overwritten/zeroed on the next frame.
2. **Acoustic Measurement Stability (`ES7210`):**
   - Initialize `ES7210` with a **fixed manual microphone gain** defined in `config.h` (e.g., `MIC_GAIN_DB = 24` or nearest valid ES7210 register step, verified against clipping in Phase 1).
   - **Explicitly disable** Automatic Gain Control (AGC), Echo Cancellation (AEC), and any hardware noise suppression.
   - Select a single primary microphone channel (or explicitly document dual-channel handling) consistently across all frames.
3. **Bus Collision Prevention:**
   - **NEVER** call `Wire.begin()` with default arguments. Always call `Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ_HZ)` once during startup and share the initialized I2C bus safely across `ES7210` and `PCF85063` (guarded by a mutex if accessed from multiple tasks).
   - Use `SD_MMC` in **1-bit mode** (`SD_MMC.begin("/sdcard", true)` after `SD_MMC.setPins(...)`). Do NOT use SPI `SD.h`.
4. **Real-Time RTOS Isolation & Memory Discipline:**
   - Pin the audio capture & DSP task to **Core 1** at high priority so Wi-Fi reconnects, HTTP timeouts, SD card writes, or RLCD refreshes on **Core 0** NEVER cause I2S DMA buffer overruns.
   - Keep real-time FFT working arrays in **internal SRAM** (16-byte aligned for ESP32-S3 SIMD). Place large display framebuffers or HTTP retry queues in **OPI PSRAM** (`ps_malloc`).
   - Avoid dynamic `String` heap fragmentation inside 1-second or real-time loops; prefer fixed-size `char[]` buffers with `snprintf`.

---

## 4. Directory & Modular Sketch Architecture

Arduino IDE compiles all `.h`/`.cpp` files inside the sketch folder automatically. Organize the workspace as follows:

```text
.
├── SPEC.md                          # This specification document
├── reference/                       # Official Waveshare ESP32-S3-RLCD-4.2 demos & schematic (read-only)
└── NoiseEventLogger/
    ├── NoiseEventLogger.ino         # Setup, FreeRTOS task orchestration, main loop watchdog
    ├── config.h                     # Sampling rate, FFT size, mic gain, intervals, calibration offset
    ├── secrets.example.h            # Template for Wi-Fi SSID/password and HTTP endpoint URL
    ├── secrets.h                    # Actual credentials (MUST be in .gitignore)
    ├── board_pins.h                 # Verified GPIO definitions extracted from ./reference/
    ├── audio_capture.h / .cpp       # Phase 1: I2C init, ES7210 fixed-gain driver, I2S DMA reader
    ├── dsp_octave.h / .cpp          # Phase 2: esp-dsp 4096-pt Hann FFT, 24 1/3-octave bands, 1m Leq/L90 stats
    ├── rtc_time.h / .cpp            # Phase 3: PCF85063 driver + NTP synchronization + ISO-8601 helper
    ├── display_ui.h / .cpp          # Phase 3: ST7305 RLCD low-refresh status screen
    ├── storage_log.h / .cpp         # Phase 3: SD_MMC 1-bit daily CSV/JSONL logger
    └── network_sync.h / .cpp        # Phase 3: Non-blocking Wi-Fi manager + HTTP POST queue
```

---

## 5. Acoustic & DSP Mathematical Specification

### 5.1 Sampling & FFT Parameters
- **Sampling Rate ($f_s$):** $32000\text{ Hz}$ (preferred) or $48000\text{ Hz}$.
  - Must satisfy $f_s \ge 32000\text{ Hz}$ so the Nyquist frequency ($f_N = f_s / 2 \ge 16000\text{ Hz}$) comfortably exceeds the upper edge of the $12.5\text{ kHz}$ 1/3-octave band ($14254.4\text{ Hz}$).
- **FFT Size ($N$) & Hop ($H$):** $N = 4096$ samples, $H = 2000$ samples (~51.2% overlap).
  - At $f_s = 32000\text{ Hz}$: frequency resolution $\Delta f = \frac{f_s}{N} = 7.8125\text{ Hz}$, window duration $T = 128\text{ ms}$, hop $H = 62.5\text{ ms}$ (exact $16$ FFT frames per $1.000\text{ s}$).
  - Provides $\ge 2$ FFT bins inside the lowest ($63\text{ Hz}$) 1/3-octave band for stable low-frequency estimation.
- **Bit Depth & Normalization:** Convert raw signed PCM samples (16-bit or 24/32-bit from ES7210) to normalized float32 $x[n] \in [-1.0, 1.0]$.
- **DC Offset Removal:** Subtract the frame mean $\bar{x} = \frac{1}{N}\sum_{n=0}^{N-1} x[n]$ before windowing to eliminate DC leakage into the $63\text{ Hz}$ band.
- **Clipping Detection:** Flag a clip whenever any raw sample reaches $\ge 99.5\%$ of full scale (e.g., $|s| \ge 32600$ for 16-bit signed).

### 5.2 Windowing & Power Spectrum (`esp-dsp`)
1. Apply a precomputed **Hann window**:
   $$w[n] = 0.5 \left(1 - \cos\left(\frac{2\pi n}{N - 1}\right)\right)$$
2. Run hardware-accelerated real/complex FFT using `esp-dsp` (`dsps_fft2r_fc32_aes3`).
3. Compute single-sided power per bin $k \in [1, N/2 - 1]$ with window energy normalization $W_{\text{ss}} = \sum_{n=0}^{N-1} w[n]^2$:
   $$P[k] = \frac{2 \cdot \left(\text{Re}(X[k])^2 + \text{Im}(X[k])^2\right)}{N \cdot W_{\text{ss}}}$$
   (Verify via unit test that a full-scale sine wave $x[n] = \sin(2\pi f_0 n / f_s)$ yields $\approx -3.01\text{ dBFS}$ RMS both in time domain and summed spectral domain — Parseval's theorem check.)

### 5.3 Standard 24 1/3-Octave Bands (IEC 61260 Base-2)
Center frequencies follow $f_{c,b} = 1000 \cdot 2^{(b - 12)/3}$ for $b \in \{0 \dots 23\}$ with band edges $f_{\text{low},b} = f_{c,b} \cdot 2^{-1/6}$ and $f_{\text{high},b} = f_{c,b} \cdot 2^{+1/6}$ (with $b=0$ lower edge clamped to $54.0\text{ Hz}$ so bins $k=7,8$ capture $63\text{ Hz}$):

| Band Index | Label | Exact $f_c$ (Hz) | Lower–Upper Edges (Hz) | FFT Bins ($32\text{ kHz}/4096$) |
| :---: | :---: | :---: | :---: | :---: |
| 0 | `63` | $62.5$ | $54.0 - 70.2$ | `7..8` |
| 1 | `80` | $78.7$ | $70.2 - 88.4$ | `9..11` |
| 2 | `100` | $99.2$ | $88.4 - 111.4$ | `12..14` |
| 3 | `125` | $125.0$ | $111.4 - 140.3$ | `15..17` |
| 4 | `160` | $157.5$ | $140.3 - 176.8$ | `18..22` |
| 5 | `200` | $198.4$ | $176.8 - 222.7$ | `23..28` |
| 6 | `250` | $250.0$ | $222.7 - 280.6$ | `29..35` |
| 7 | `315` | $315.0$ | $280.6 - 353.6$ | `36..45` |
| 8 | `400` | $396.9$ | $353.6 - 445.4$ | `46..57` |
| 9 | `500` | $500.0$ | $445.4 - 561.2$ | `58..71` |
| 10 | `630` | $630.0$ | $561.2 - 707.1$ | `72..90` |
| 11 | `800` | $793.7$ | $707.1 - 890.9$ | `91..114` |
| 12 | `1k` | $1000.0$ | $890.9 - 1122.5$ | `115..143` |
| 13 | `1.25k` | $1259.9$ | $1122.5 - 1414.2$ | `144..181` |
| 14 | `1.6k` | $1587.4$ | $1414.2 - 1781.8$ | `182..228` |
| 15 | `2k` | $2000.0$ | $1781.8 - 2244.9$ | `229..287` |
| 16 | `2.5k` | $2519.8$ | $2244.9 - 2828.4$ | `288..362` |
| 17 | `3.15k` | $3174.8$ | $2828.4 - 3563.6$ | `363..456` |
| 18 | `4k` | $4000.0$ | $3563.6 - 4489.8$ | `457..574` |
| 19 | `5k` | $5039.7$ | $4489.8 - 5656.9$ | `575..724` |
| 20 | `6.3k` | $6349.6$ | $5656.9 - 7127.2$ | `725..912` |
| 21 | `8k` | $8000.0$ | $7127.2 - 8979.7$ | `913..1149` |
| 22 | `10k` | $10079.4$ | $8979.7 - 11313.7$ | `1150..1448` |
| 23 | `12.5k` | $12699.2$ | $11313.7 - 14254.4$ | `1449..1824` |

For each band $b \in \{0\dots 23\}$, accumulate power over all bins $k$ whose center frequency $k \cdot \Delta f \in [f_{\text{low}, b}, f_{\text{high}, b})$:
$$P_{\text{band}}[b] = \sum_{k \in \text{band } b} P[k]$$

### 5.4 Temporal Aggregation (1-Second $\rightarrow$ 1-Minute)
- **1-Second Sub-Frame ($M_f$ FFT frames accumulated over $1\text{ s}$):**
  - Average linear power across all FFT frames in that second for broadband ($P_{\text{bb},1\text{s}}$) and each band ($P_{b,1\text{s}}$).
  - Convert to decibels (clamped with floor $\epsilon = 10^{-12}$ to prevent $-\infty$):
    $$L_{1\text{s}} = 10 \log_{10}(\max(P_{1\text{s}}, \epsilon)) + \text{CALIBRATION\_OFFSET\_DB}$$
  - Store the $60$ one-second values in fixed circular/linear arrays (`float bb_1s[60]`, `float band_1s[24][60]`).
- **1-Minute Summary Record (computed every $60\text{ s}$):**
  - **$L_{eq,1\text{m}}$ (Energy-equivalent mean):**
    $$L_{eq,1\text{m}} = 10 \log_{10}\left(\frac{1}{60}\sum_{i=0}^{59} 10^{L_{1\text{s}}[i] / 10}\right)$$
  - **$L_{90,1\text{m}}$ (Background / Stationary floor):**
    - Copy the $60$ values of $L_{1\text{s}}$ into a scratch buffer, sort ascending ($v_0 \le v_1 \le \dots \le v_{59}$).
    - Take the 10th percentile index ($90\%$ of samples exceed this level): $L_{90,1\text{m}} = v_5$ (or linear interpolation at rank $0.10 \times (60 - 1) = 5.9$).
  - **$L_{\text{peak},1\text{m}}$:** Maximum instantaneous sample peak ($\text{dBFS}$) observed during the minute.
  - **`clips`:** Total count of clipped samples during the minute.

### 5.5 Empirical Self-Noise Baseline (Anechoic Chamber Verified on `rlcd-noise-01`)
Measured over $120\text{ s}$ ($2$ consecutive 1-minute cycles at `MIC_GAIN_DB = 24.0`, $f_s = 32\text{ kHz}$, `FFT = 4096`, 1 Hz RLCD SPI DMA active, `SD_MMC` 1-bit active):
- **Broadband (`bb`):** $L_{eq} = -79.4\text{ dBFS}$ (`1s mean = -79.34 dBFS`, $\sigma = 0.167\text{ dB}$), $L_{90} = -79.6\text{ dBFS}$, $L_{\text{peak}} = -65.2\text{ dBFS}$, `clips = 0`.
- **Reliable Detection Floor:** Per-band levels $\ge \text{ANECHOIC\_OCT\_L90\_DBFS}[b] + 3.0\text{ dB}$ represent genuine external acoustic energy above hardware self-noise (stored in `config.h`).

| Band Index | Label | Anechoic $L_{90}$ (dBFS) | Anechoic $L_{eq}$ (dBFS) | $1\text{s}$ $\sigma$ (dB) |
| :---: | :---: | :---: | :---: | :---: |
| 0 | `63` | `-94.2` | `-92.8` | `1.07` |
| 1 | `80` | `-92.3` | `-91.2` | `0.91` |
| 2 | `100` | `-92.8` | `-91.6` | `0.83` |
| 3 | `125` | `-93.7` | `-92.5` | `0.92` |
| 4 | `160` | `-92.3` | `-91.2` | `0.74` |
| 5 | `200` | `-92.8` | `-92.0` | `0.59` |
| 6 | `250` | `-93.2` | `-92.5` | `0.60` |
| 7 | `315` | `-92.8` | `-92.2` | `0.49` |
| 8 | `400` | `-93.6` | `-93.0` | `0.47` |
| 9 | `500` | `-94.2` | `-93.6` | `0.41` |
| 10 | `630` | `-94.2` | `-93.7` | `0.38` |
| 11 | `800` | `-94.5` | `-94.0` | `0.35` |
| 12 | `1k` | `-95.0` | `-94.7` | `0.32` |
| 13 | `1.25k` | `-95.2` | `-94.9` | `0.27` |
| 14 | `1.6k` | `-95.5` | `-95.2` | `0.24` |
| 15 | `2k` | `-95.9` | `-95.6` | `0.23` |
| 16 | `2.5k` | `-95.8` | `-95.7` | `0.18` |
| 17 | `3.15k` | `-96.0` | `-95.8` | `0.15` |
| 18 | `4k` | `-96.0` | `-95.8` | `0.16` |
| 19 | `5k` | `-95.8` | `-95.6` | `0.13` |
| 20 | `6.3k` | `-95.6` | `-95.4` | `0.12` |
| 21 | `8k` | `-95.2` | `-95.1` | `0.10` |
| 22 | `10k` | `-94.7` | `-94.6` | `0.10` |
| 23 | `12.5k` | `-93.9` | `-93.8` | `0.08` |

---

## 6. Data Output Schemas

### 6.1 JSON Record (Serial in Phase 2, HTTP POST in Phase 3)
One compact JSON object per minute (no pretty-printing over network):

```json
{
  "device_id": "rlcd-noise-01",
  "ts": "2026-11-27T14:05:00+07:00",
  "uptime_s": 3600,
  "unit": "dBFS",
  "bb": { "leq": -42.3, "l90": -46.1, "peak": -18.4, "clips": 0 },
  "oct_leq": [-51.2, -50.8, -50.1, -49.8, "... 24 bands (63 Hz to 12.5 kHz) ..."],
  "oct_l90": [-55.0, -54.2, -53.6, -53.1, "... 24 bands (63 Hz to 12.5 kHz) ..."],
  "sys": { "vbat_mv": 4120, "rssi": -62, "sd_ok": true }
}
```
*(In Phase 2, `ts` may be omitted or set to `null` and `sys` omitted until RTC/Wi-Fi/SD are added in Phase 3.)*

### 6.2 MicroSD CSV Format (Phase 3)
- Path: `/logs/YYYY-MM-DD.csv` (create `/logs` directory automatically; write header row only if file is newly created).
- Header (24 `leq_*` + 24 `l90_*` columns):
  ```csv
  ts,uptime_s,bb_leq,bb_l90,bb_peak,clips,leq_63,leq_80,leq_100,leq_125,leq_160,leq_200,leq_250,leq_315,leq_400,leq_500,leq_630,leq_800,leq_1k,leq_1k25,leq_1k6,leq_2k,leq_2k5,leq_3k15,leq_4k,leq_5k,leq_6k3,leq_8k,leq_10k,leq_12k5,l90_63,l90_80,l90_100,l90_125,l90_160,l90_200,l90_250,l90_315,l90_400,l90_500,l90_630,l90_800,l90_1k,l90_1k25,l90_1k6,l90_2k,l90_2k5,l90_3k15,l90_4k,l90_5k,l90_6k3,l90_8k,l90_10k,l90_12k5,vbat_mv,rssi
  ```

---

## 7. Phased Implementation Plan & Acceptance Gates

> **STRICT GATING RULE:** Implement **ONLY** the currently active phase. Do **NOT** scaffold or enable code for future phases until the user confirms hardware verification of the current phase.

### Phase 1: Hardware Bring-up & Audio Capture (`ES7210` + `I2S`)
- **Files to create:** `NoiseEventLogger.ino`, `config.h`, `board_pins.h`, `audio_capture.h`, `audio_capture.cpp`.
- **Functional Requirements:**
  1. Initialize USB CDC Serial (`115200` baud) and wait briefly for host attach.
  2. Print chip info, PSRAM size, and verified pinout summary on boot.
  3. Initialize explicit I2C bus and scan for expected device addresses (confirm `ES7210` ACK).
  4. Configure `ES7210` with fixed manual gain and AGC disabled.
  5. Start I2S RX DMA capture at $f_s = 32000\text{ Hz}$ (or $48000\text{ Hz}$ if constrained by ES7210 clock dividers in `./reference/`).
  6. Every $1\text{ s}$, compute DC-removed broadband RMS ($\text{dBFS}$), Peak ($\text{dBFS}$), DC mean, and clip count; print one clean CSV/log line to Serial.
  7. Immediately discard raw audio buffer after metric calculation.
- **Must NOT include:** `esp-dsp` FFT, Display driver, RTC, Wi-Fi, SD card.
- **Phase 1 Acceptance Gate (User Verified on Hardware):**
  - Compiles and flashes cleanly via Arduino IDE.
  - Quiet room noise floor reads stably (e.g., between $-75$ and $-45\text{ dBFS}$ with no sudden AGC drift).
  - Speaking/clapping near the mic increases RMS/Peak predictably and returns immediately to the exact same floor afterward.
  - Zero I2S read timeouts or watchdog resets over a 5-minute burn-in test.

### Phase 2: Real-Time DSP (`esp-dsp` 24 1/3-Octave Bands + $L_{eq}$ / $L_{90}$)
- **Files to add/update:** `dsp_octave.h`, `dsp_octave.cpp`, `NoiseEventLogger.ino`.
- **Functional Requirements:**
  1. Integrate `esp-dsp` (`dsps_fft2r_init_fc32` + Hann window + 4096-pt FFT).
  2. Include a compile-time optional self-test (`RUN_DSP_SELF_TEST`) that feeds a synthetic $1000\text{ Hz}$ and $4000\text{ Hz}$ sine wave into the DSP pipeline on boot to verify bin mapping (`Band 12` = `1k`, `Band 18` = `4k`) and Parseval RMS accuracy within $\pm 0.5\text{ dB}$.
  3. Move audio capture + FFT processing into a dedicated FreeRTOS task pinned to **Core 1**.
  4. Print compact 1-second live telemetry to Serial (optional toggle via `VERBOSE_1S_SERIAL`) AND output the 1-minute JSON record (Section 6.1) every $60\text{ s}$.
  5. Report per-frame DSP execution time ($\mu\text{s}$) and free internal heap to confirm ample CPU headroom.
- **Must NOT include:** Display driver, RTC, Wi-Fi, SD card.
- **Phase 2 Acceptance Gate (User Verified on Hardware):**
  - DSP self-test passes within $\pm 0.5\text{ dB}$ for $1\text{ kHz}$ and $4\text{ kHz}$ synthetic tones.
  - Playing a $1\text{ kHz}$ test tone from a phone visibly spikes Band 12 (`1k`) relative to adjacent bands.
  - Bench test: continuous broadband noise (e.g., hairdryer / white noise) elevates both `oct_leq` and `oct_l90` in $1 - 12.5\text{ kHz}$, whereas intermittent clapping elevates `bb.peak` and `oct_leq` while keeping `oct_l90` flat.
  - Zero dropped audio frames over 15 minutes continuous operation.

### Phase 3: Full System Integration (RTC + RLCD + SD_MMC + Wi-Fi HTTP)
- **Files to add/update:** `secrets.example.h`, `rtc_time.h/.cpp`, `display_ui.h/.cpp`, `storage_log.h/.cpp`, `network_sync.h/.cpp`, `NoiseEventLogger.ino`.
- **Functional Requirements:**
  1. **RTC & NTP (`Core 0`):** Read `PCF85063` at boot so logging starts immediately with valid timestamps even without Wi-Fi. When Wi-Fi connects, sync NTP (`pool.ntp.org`, `UTC+7` / `ICT-7`) and write back to `PCF85063` once per boot and every 24 hours.
  2. **Queue Decoupling:** Core 1 pushes each completed 1-minute `MinuteLogRecord` struct into a FreeRTOS queue (e.g., depth 15) consumed by Core 0. Core 1 NEVER blocks on Wi-Fi, HTTP, SD, or LCD.
  3. **MicroSD Storage (`SD_MMC` 1-bit):** Append each 1-minute record to `/logs/YYYY-MM-DD.csv` and flush/close immediately to prevent FAT corruption on sudden power loss. If no SD card is inserted, degrade gracefully (`sd_ok: false`) without crashing.
  4. **Wi-Fi & HTTP POST:** Connect non-blockingly using credentials from `secrets.h`. POST the 1-minute JSON payload to `INGEST_URL` with a short timeout ($3000\text{ ms}$). If Wi-Fi/server is unreachable, log locally to SD and retry Wi-Fi periodically with exponential backoff.
  5. **RLCD Status Display (`ST7305` $400 \times 300$):**
     - Reuse Waveshare's verified `ST7305` driver from `./reference/`.
     - Configurable refresh rate (`RLCD_REFRESH_INTERVAL_MS`, default `1 s` live on USB power; can be increased to `15 s` / `60 s` in battery mode).
     - Display clean high-contrast text:
       - Top bar: `YYYY-MM-DD HH:MM:SS` | Wi-Fi RSSI | SD status | `VBAT` (mV)
       - Center: Large live 1s `bb_leq` and 1-minute `bb_l90` (`dBFS`)
       - Lower section: Clean 24-bar 1/3-octave spectrum bar graph (`63` to `12.5k`) showing solid 1-minute $L_{90}$ floor + live 1s level overlay box, plus last HTTP POST status (`OK` / `ERR` / `OFF`).
  6. **Battery ADC Reading:** Read battery voltage divider pin (verified in `board_pins.h`) using `analogReadMilliVolts()` multiplied by the schematic divider ratio, averaged over 16 samples.
- **Phase 3 Acceptance Gate (User Verified on Hardware):**
  - Unplugging Wi-Fi router does not interrupt 1-minute SD CSV logging or audio capture; reconnecting resumes HTTP POST automatically.
  - Rebooting without Wi-Fi preserves accurate timestamp from `PCF85063` RTC.
  - Display renders cleanly without artifacts or crashing PSRAM/heap over a 12-hour soak test.

---

## 8. Agent Operating Protocol (For Antigravity IDE)

When working in this workspace, the AI Agent MUST follow these rules:

1. **Read First, Code Second:** Always inspect `SPEC.md` and `./reference/` before proposing changes. Cite exact file paths in `./reference/` when defining `board_pins.h`.
2. **State the Active Phase:** At the start of each response, explicitly state which Phase is active (e.g., `Active Phase: Phase 1`) and verify no out-of-phase modules are being prematurely added.
3. **Explain Hardware Assumptions:** If any register setting (e.g., `ES7210` gain or I2S slot format) depends on board schematic details, list the exact assumption and register values clearly for review.
4. **Keep Diffs Minimal & Compilable:** Every checkpoint must leave `NoiseEventLogger/` in a clean, compilable state for Arduino IDE.
5. **Respect the Gate:** After finishing a phase's code, provide a short **Hardware Verification Checklist** for the user to run on the physical board, and **wait for user confirmation** before starting the next phase.