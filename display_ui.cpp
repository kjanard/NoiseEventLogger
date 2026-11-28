#include "display_ui.h"
#include "board_pins.h"
#include "config.h"

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_io.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

namespace {

constexpr uint8_t COLOR_BLACK = 0x00;
constexpr uint8_t COLOR_WHITE = 0xFF;
constexpr size_t  DISP_BUF_BYTES = (RLCD_WIDTH * RLCD_HEIGHT) / 8;  // 15,000 bytes

esp_lcd_panel_io_handle_t s_io_handle = nullptr;
uint8_t                  *s_disp_buf  = nullptr;
bool                      s_inited    = false;

// Standard 5x7 ASCII font (0x20..0x7E, column-major, LSB = top row)
constexpr uint8_t FONT_5X7[95][5] = {
  {0x00,0x00,0x00,0x00,0x00}, // ' ' 0x20
  {0x00,0x00,0x5F,0x00,0x00}, // '!'
  {0x00,0x07,0x00,0x07,0x00}, // '"'
  {0x14,0x7F,0x14,0x7F,0x14}, // '#'
  {0x24,0x2A,0x7F,0x2A,0x12}, // '$'
  {0x23,0x13,0x08,0x64,0x62}, // '%'
  {0x36,0x49,0x55,0x22,0x50}, // '&'
  {0x00,0x05,0x03,0x00,0x00}, // '''
  {0x00,0x1C,0x22,0x41,0x00}, // '('
  {0x00,0x41,0x22,0x1C,0x00}, // ')'
  {0x08,0x2A,0x1C,0x2A,0x08}, // '*'
  {0x08,0x08,0x3E,0x08,0x08}, // '+'
  {0x00,0x50,0x30,0x00,0x00}, // ','
  {0x08,0x08,0x08,0x08,0x08}, // '-'
  {0x00,0x60,0x60,0x00,0x00}, // '.'
  {0x20,0x10,0x08,0x04,0x02}, // '/'
  {0x3E,0x51,0x49,0x45,0x3E}, // '0' 0x30
  {0x00,0x42,0x7F,0x40,0x00}, // '1'
  {0x42,0x61,0x51,0x49,0x46}, // '2'
  {0x21,0x41,0x45,0x4B,0x31}, // '3'
  {0x18,0x14,0x12,0x7F,0x10}, // '4'
  {0x27,0x45,0x45,0x45,0x39}, // '5'
  {0x3C,0x4A,0x49,0x49,0x30}, // '6'
  {0x01,0x71,0x09,0x05,0x03}, // '7'
  {0x36,0x49,0x49,0x49,0x36}, // '8'
  {0x06,0x49,0x49,0x29,0x1E}, // '9'
  {0x00,0x36,0x36,0x00,0x00}, // ':'
  {0x00,0x56,0x36,0x00,0x00}, // ';'
  {0x08,0x14,0x22,0x41,0x00}, // '<'
  {0x14,0x14,0x14,0x14,0x14}, // '='
  {0x00,0x41,0x22,0x14,0x08}, // '>'
  {0x02,0x01,0x51,0x09,0x06}, // '?'
  {0x32,0x49,0x79,0x41,0x3E}, // '@' 0x40
  {0x7E,0x11,0x11,0x11,0x7E}, // 'A'
  {0x7F,0x49,0x49,0x49,0x36}, // 'B'
  {0x3E,0x41,0x41,0x41,0x22}, // 'C'
  {0x7F,0x41,0x41,0x22,0x1C}, // 'D'
  {0x7F,0x49,0x49,0x49,0x41}, // 'E'
  {0x7F,0x09,0x09,0x09,0x01}, // 'F'
  {0x3E,0x41,0x49,0x49,0x7A}, // 'G'
  {0x7F,0x08,0x08,0x08,0x7F}, // 'H'
  {0x00,0x41,0x7F,0x41,0x00}, // 'I'
  {0x20,0x40,0x41,0x3F,0x01}, // 'J'
  {0x7F,0x08,0x14,0x22,0x41}, // 'K'
  {0x7F,0x40,0x40,0x40,0x40}, // 'L'
  {0x7F,0x02,0x0C,0x02,0x7F}, // 'M'
  {0x7F,0x04,0x08,0x10,0x7F}, // 'N'
  {0x3E,0x41,0x41,0x41,0x3E}, // 'O'
  {0x7F,0x09,0x09,0x09,0x06}, // 'P'
  {0x3E,0x41,0x51,0x21,0x5E}, // 'Q'
  {0x7F,0x09,0x19,0x29,0x46}, // 'R'
  {0x46,0x49,0x49,0x49,0x31}, // 'S'
  {0x01,0x01,0x7F,0x01,0x01}, // 'T'
  {0x3F,0x40,0x40,0x40,0x3F}, // 'U'
  {0x1F,0x20,0x40,0x20,0x1F}, // 'V'
  {0x3F,0x40,0x38,0x40,0x3F}, // 'W'
  {0x63,0x14,0x08,0x14,0x63}, // 'X'
  {0x07,0x08,0x70,0x08,0x07}, // 'Y'
  {0x61,0x51,0x49,0x45,0x43}, // 'Z'
  {0x00,0x7F,0x41,0x41,0x00}, // '['
  {0x02,0x04,0x08,0x10,0x20}, // '\'
  {0x00,0x41,0x41,0x7F,0x00}, // ']'
  {0x04,0x02,0x01,0x02,0x04}, // '^'
  {0x40,0x40,0x40,0x40,0x40}, // '_'
  {0x00,0x01,0x02,0x04,0x00}, // '`' 0x60
  {0x20,0x54,0x54,0x54,0x78}, // 'a'
  {0x7F,0x48,0x44,0x44,0x38}, // 'b'
  {0x38,0x44,0x44,0x44,0x20}, // 'c'
  {0x38,0x44,0x44,0x48,0x7F}, // 'd'
  {0x38,0x54,0x54,0x54,0x18}, // 'e'
  {0x08,0x7E,0x09,0x01,0x02}, // 'f'
  {0x0C,0x52,0x52,0x52,0x3E}, // 'g'
  {0x7F,0x08,0x04,0x04,0x78}, // 'h'
  {0x00,0x44,0x7D,0x40,0x00}, // 'i'
  {0x20,0x40,0x44,0x3D,0x00}, // 'j'
  {0x7F,0x10,0x28,0x44,0x00}, // 'k'
  {0x00,0x41,0x7F,0x40,0x00}, // 'l'
  {0x7C,0x04,0x18,0x04,0x78}, // 'm'
  {0x7C,0x08,0x04,0x04,0x78}, // 'n'
  {0x38,0x44,0x44,0x44,0x38}, // 'o'
  {0x7C,0x14,0x14,0x14,0x08}, // 'p'
  {0x08,0x14,0x14,0x18,0x7C}, // 'q'
  {0x7C,0x08,0x04,0x04,0x08}, // 'r'
  {0x48,0x54,0x54,0x54,0x20}, // 's'
  {0x04,0x3F,0x44,0x40,0x20}, // 't'
  {0x3C,0x40,0x40,0x20,0x7C}, // 'u'
  {0x1C,0x20,0x40,0x20,0x1C}, // 'v'
  {0x3C,0x40,0x30,0x40,0x3C}, // 'w'
  {0x44,0x28,0x10,0x28,0x44}, // 'x'
  {0x0C,0x50,0x50,0x50,0x3C}, // 'y'
  {0x44,0x64,0x54,0x4C,0x44}, // 'z'
  {0x00,0x08,0x36,0x41,0x00}, // '{'
  {0x00,0x00,0x7F,0x00,0x00}, // '|'
  {0x00,0x41,0x36,0x08,0x00}, // '}'
  {0x08,0x04,0x08,0x10,0x08}  // '~' 0x7E
};

inline void rlcdCmd(uint8_t cmd) {
  esp_lcd_panel_io_tx_param(s_io_handle, cmd, nullptr, 0);
}

inline void rlcdData(uint8_t val) {
  esp_lcd_panel_io_tx_param(s_io_handle, -1, &val, 1);
}

void rlcdReset() {
  gpio_set_level(static_cast<gpio_num_t>(PIN_RLCD_RST), 1);
  delay(50);
  gpio_set_level(static_cast<gpio_num_t>(PIN_RLCD_RST), 0);
  delay(30);
  gpio_set_level(static_cast<gpio_num_t>(PIN_RLCD_RST), 1);
  delay(120);
}

// Exact Waveshare 400x300 landscape pixel bit mapping from reference display_bsp.cpp
inline bool pixelAddress(int x, int y, uint32_t *out_index, uint8_t *out_bit) {
  if (x < 0 || x >= RLCD_WIDTH || y < 0 || y >= RLCD_HEIGHT) {
    return false;
  }
  const uint16_t inv_y = static_cast<uint16_t>((RLCD_HEIGHT - 1) - y);
  *out_index = (static_cast<uint32_t>(x >> 1) * static_cast<uint32_t>(RLCD_HEIGHT >> 2)) +
               static_cast<uint32_t>(inv_y >> 2);
  *out_bit   = 7U - (((inv_y & 0x03U) << 1) | static_cast<uint8_t>(x & 0x01));
  return true;
}

inline void setPixel(int x, int y, uint8_t color) {
  uint32_t index = 0;
  uint8_t  bit   = 0;
  if (!pixelAddress(x, y, &index, &bit)) {
    return;
  }
  const uint8_t mask = static_cast<uint8_t>(1U << bit);
  if (color == COLOR_WHITE) {
    s_disp_buf[index] |= mask;
  } else {
    s_disp_buf[index] &= static_cast<uint8_t>(~mask);
  }
}

inline uint8_t getPixel(int x, int y) {
  uint32_t index = 0;
  uint8_t  bit   = 0;
  if (!pixelAddress(x, y, &index, &bit)) {
    return COLOR_WHITE;
  }
  return ((s_disp_buf[index] >> bit) & 0x01U) ? COLOR_WHITE : COLOR_BLACK;
}

void clearBuffer(uint8_t color) {
  memset(s_disp_buf, color, DISP_BUF_BYTES);
}

void drawHLine(int x, int y, int w, uint8_t color) {
  for (int i = 0; i < w; ++i) {
    setPixel(x + i, y, color);
  }
}

void drawVLine(int x, int y, int h, uint8_t color) {
  for (int i = 0; i < h; ++i) {
    setPixel(x, y + i, color);
  }
}

void drawDottedHLine(int x, int y, int w, uint8_t color, int step = 4) {
  for (int i = 0; i < w; i += step) {
    setPixel(x + i, y, color);
  }
}

void drawRect(int x, int y, int w, int h, uint8_t color) {
  if (w <= 0 || h <= 0) {
    return;
  }
  drawHLine(x, y, w, color);
  drawHLine(x, y + h - 1, w, color);
  drawVLine(x, y, h, color);
  drawVLine(x + w - 1, y, h, color);
}

void fillRect(int x, int y, int w, int h, uint8_t color) {
  for (int yy = y; yy < y + h; ++yy) {
    drawHLine(x, yy, w, color);
  }
}

void drawChar(int x, int y, char c, uint8_t color, uint8_t scale) {
  if (c < 0x20 || c > 0x7E) {
    c = '?';
  }
  const uint8_t *glyph = FONT_5X7[c - 0x20];
  for (int col = 0; col < 5; ++col) {
    const uint8_t bits = glyph[col];
    for (int row = 0; row < 7; ++row) {
      if ((bits >> row) & 0x01U) {
        if (scale == 1) {
          setPixel(x + col, y + row, color);
        } else {
          fillRect(x + (col * scale), y + (row * scale), scale, scale, color);
        }
      }
    }
  }
}

void drawText(int x, int y, const char *str, uint8_t color, uint8_t scale = 1) {
  if (str == nullptr) {
    return;
  }
  const int advance = 6 * scale;
  int cx = x;
  for (const char *p = str; *p != '\0'; ++p) {
    drawChar(cx, y, *p, color, scale);
    cx += advance;
  }
}

void flushFrame() {
  rlcdCmd(0x2A);
  rlcdData(0x12);
  rlcdData(0x2A);

  rlcdCmd(0x2B);
  rlcdData(0x00);
  rlcdData(0xC7);

  rlcdCmd(0x2C);
  esp_lcd_panel_io_tx_color(s_io_handle, -1, s_disp_buf, DISP_BUF_BYTES);
  delay(10);
}

int dbToBarHeightPx(float db_val, float db_min, float db_max, float px_per_db) {
  float clamped = db_val;
  if (clamped < db_min) clamped = db_min;
  if (clamped > db_max) clamped = db_max;
  return static_cast<int>(lroundf((clamped - db_min) * px_per_db));
}

const char *httpStateLabel(HttpSyncState_t st) {
  switch (st) {
    case HTTP_STATE_DISABLED: return "OFF";
    case HTTP_STATE_IDLE:     return "IDLE";
    case HTTP_STATE_OK:       return "OK";
    case HTTP_STATE_ERR:      return "ERR";
    default:                  return "?";
  }
}

uint8_t batteryPercentFromMv(uint32_t vbat_mv) {
  if (vbat_mv <= 3000U) {
    return 0U;
  }
  if (vbat_mv >= 4120U) {
    return 100U;
  }
  return static_cast<uint8_t>(((vbat_mv - 3000U) * 100U + 560U) / 1120U);
}

}  // namespace

bool DisplayUi_Init() {
  if (s_inited) {
    return true;
  }

  spi_bus_config_t buscfg = {};
  buscfg.miso_io_num     = -1;
  buscfg.mosi_io_num     = PIN_RLCD_MOSI;
  buscfg.sclk_io_num     = PIN_RLCD_SCK;
  buscfg.quadwp_io_num   = -1;
  buscfg.quadhd_io_num   = -1;
  buscfg.max_transfer_sz = RLCD_WIDTH * RLCD_HEIGHT;

  esp_err_t ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
    Serial.printf("[RLCD] spi_bus_initialize failed: %d\n", static_cast<int>(ret));
    return false;
  }

  esp_lcd_panel_io_spi_config_t io_config = {};
  io_config.dc_gpio_num       = PIN_RLCD_DC;
  io_config.cs_gpio_num       = PIN_RLCD_CS;
  io_config.pclk_hz           = 20 * 1000 * 1000;
  io_config.lcd_cmd_bits      = 8;
  io_config.lcd_param_bits    = 8;
  io_config.spi_mode          = 0;
  io_config.trans_queue_depth = 10;

  ret = esp_lcd_new_panel_io_spi(static_cast<esp_lcd_spi_bus_handle_t>(SPI3_HOST), &io_config, &s_io_handle);
  if (ret != ESP_OK || s_io_handle == nullptr) {
    Serial.printf("[RLCD] esp_lcd_new_panel_io_spi failed: %d\n", static_cast<int>(ret));
    return false;
  }

  gpio_config_t rst_cfg = {};
  rst_cfg.intr_type    = GPIO_INTR_DISABLE;
  rst_cfg.mode         = GPIO_MODE_OUTPUT;
  rst_cfg.pin_bit_mask = (1ULL << PIN_RLCD_RST);
  rst_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  rst_cfg.pull_up_en   = GPIO_PULLUP_ENABLE;
  gpio_config(&rst_cfg);

  gpio_config_t te_cfg = {};
  te_cfg.intr_type     = GPIO_INTR_DISABLE;
  te_cfg.mode          = GPIO_MODE_INPUT;
  te_cfg.pin_bit_mask  = (1ULL << PIN_RLCD_TE);
  te_cfg.pull_down_en  = GPIO_PULLDOWN_DISABLE;
  te_cfg.pull_up_en    = GPIO_PULLUP_DISABLE;
  gpio_config(&te_cfg);

  s_disp_buf = static_cast<uint8_t *>(heap_caps_malloc(DISP_BUF_BYTES, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (s_disp_buf == nullptr) {
    s_disp_buf = static_cast<uint8_t *>(malloc(DISP_BUF_BYTES));
  }
  if (s_disp_buf == nullptr) {
    Serial.println("[RLCD] Failed to allocate 15000 B framebuffer");
    return false;
  }

  // Exact ST7305 init sequence from Waveshare reference/examples/arduino/examples/09_LVGL_V9_Test/display_bsp.cpp
  rlcdReset();

  rlcdCmd(0xD6); rlcdData(0x17); rlcdData(0x02);
  rlcdCmd(0xD1); rlcdData(0x01);
  rlcdCmd(0xC0); rlcdData(0x11); rlcdData(0x04);

  rlcdCmd(0xC1); rlcdData(0x69); rlcdData(0x69); rlcdData(0x69); rlcdData(0x69);
  rlcdCmd(0xC2); rlcdData(0x19); rlcdData(0x19); rlcdData(0x19); rlcdData(0x19);
  rlcdCmd(0xC4); rlcdData(0x4B); rlcdData(0x4B); rlcdData(0x4B); rlcdData(0x4B);
  rlcdCmd(0xC5); rlcdData(0x19); rlcdData(0x19); rlcdData(0x19); rlcdData(0x19);

  rlcdCmd(0xD8); rlcdData(0x80); rlcdData(0xE9);
  rlcdCmd(0xB2); rlcdData(0x02);

  rlcdCmd(0xB3);
  rlcdData(0xE5); rlcdData(0xF6); rlcdData(0x05); rlcdData(0x46); rlcdData(0x77);
  rlcdData(0x77); rlcdData(0x77); rlcdData(0x77); rlcdData(0x76); rlcdData(0x45);

  rlcdCmd(0xB4);
  rlcdData(0x05); rlcdData(0x46); rlcdData(0x77); rlcdData(0x77); rlcdData(0x77);
  rlcdData(0x77); rlcdData(0x76); rlcdData(0x45);

  rlcdCmd(0x62); rlcdData(0x32); rlcdData(0x03); rlcdData(0x1F);
  rlcdCmd(0xB7); rlcdData(0x13);
  rlcdCmd(0xB0); rlcdData(0x64);

  rlcdCmd(0x11);  // Sleep Out
  delay(200);

  rlcdCmd(0xC9); rlcdData(0x00);
  rlcdCmd(0x36); rlcdData(0x48);
  rlcdCmd(0x3A); rlcdData(0x11);
  rlcdCmd(0xB9); rlcdData(0x20);
  rlcdCmd(0xB8); rlcdData(0x29);
  rlcdCmd(0x21);  // Display Inversion On

  rlcdCmd(0x2A); rlcdData(0x12); rlcdData(0x2A);
  rlcdCmd(0x2B); rlcdData(0x00); rlcdData(0xC7);
  rlcdCmd(0x35); rlcdData(0x00);
  rlcdCmd(0xD0); rlcdData(0xFF);
  rlcdCmd(0x38);
  rlcdCmd(0x29);  // Display On

  clearBuffer(COLOR_WHITE);
  flushFrame();

  s_inited = true;
  Serial.printf("[RLCD] ST7305 %dx%d initialized | PSRAM framebuffer=%u B\n",
                RLCD_WIDTH, RLCD_HEIGHT, static_cast<unsigned>(DISP_BUF_BYTES));
  return true;
}

bool DisplayUi_IsOk() {
  return s_inited;
}

void DisplayUi_Render(const DisplayUiState_t &state) {
  if (!s_inited || s_disp_buf == nullptr) {
    return;
  }

  clearBuffer(COLOR_WHITE);

  // --- 1. Top Black Header Bar (y = 0..23) ---
  fillRect(0, 0, RLCD_WIDTH, 24, COLOR_BLACK);

  const char *time_str = (state.time_hm != nullptr && state.time_hm[0] != '\0') ? state.time_hm : "0000-00-00 00:00:00";
  drawText(6, 5, time_str, COLOR_WHITE, 2);

  char top_r1[32];
  const char *clk_src = state.ntp_synced ? "NTP" : (state.rtc_ok ? "RTC" : "SYS");
  if (!state.wifi_configured) {
    snprintf(top_r1, sizeof(top_r1), "%s W:OFF  SD:%s", clk_src, state.sd_ok ? "OK" : "ERR");
  } else if (state.wifi_connected) {
    snprintf(top_r1, sizeof(top_r1), "%s W:%ddB SD:%s", clk_src, state.rssi, state.sd_ok ? "OK" : "ERR");
  } else if (state.wifi_sleeping) {
    snprintf(top_r1, sizeof(top_r1), "%s W:SLP  SD:%s", clk_src, state.sd_ok ? "OK" : "ERR");
  } else {
    snprintf(top_r1, sizeof(top_r1), "%s W:...  SD:%s", clk_src, state.sd_ok ? "OK" : "ERR");
  }
  drawText(238, 3, top_r1, COLOR_WHITE, 1);

  char top_r2[32];
  snprintf(top_r2, sizeof(top_r2), "BAT:%u%%  HTTP:%s",
           static_cast<unsigned>(batteryPercentFromMv(state.vbat_mv)),
           httpStateLabel(state.http_state));
  drawText(238, 13, top_r2, COLOR_WHITE, 1);

  // --- 2. Center Broadband Readout Cards (y = 28..111) ---
  // Left Card: Live 1s (LAeq when calibrated in dBA, else broadband Leq in dBFS)
  drawRect(6, 28, 191, 84, COLOR_BLACK);
  drawHLine(6, 43, 191, COLOR_BLACK);
  drawText(12, 32, state.calibrated ? "LIVE 1s LAeq (dBA)" : "LIVE 1s LEQ (dBFS)", COLOR_BLACK, 1);

  const bool live_ovl = (state.has_sec && state.sec.overload_1s) || (state.has_min && state.min.overload);
  if (live_ovl) {
    fillRect(168, 29, 28, 14, COLOR_BLACK);
    drawText(173, 32, "OVL", COLOR_WHITE, 1);
  }

  char leq_str[16];
  if (state.has_sec) {
    const float primary_1s = state.calibrated ? state.sec.laeq_1s : state.sec.bb_leq_1s;
    snprintf(leq_str, sizeof(leq_str), "%5.1f", primary_1s);
  } else {
    snprintf(leq_str, sizeof(leq_str), " --.-");
  }
  drawText(18, 52, leq_str, COLOR_BLACK, 3);

  char leq_sub1[36];
  if (state.has_sec) {
    if (state.calibrated) {
      snprintf(leq_sub1, sizeof(leq_sub1), "LAFmax:%5.1f LZ:%5.1f", state.sec.laf_max_1s, state.sec.bb_leq_1s);
    } else {
      snprintf(leq_sub1, sizeof(leq_sub1), "A-wt:%5.1f Pk:%5.1f", state.sec.laeq_1s, state.sec.bb_peak_1s);
    }
  } else {
    snprintf(leq_sub1, sizeof(leq_sub1), "A-wt: --.- Pk: --.-");
  }
  drawText(12, 83, leq_sub1, COLOR_BLACK, 1);

  char leq_sub2[36];
  if (state.has_min) {
    if (state.calibrated) {
      snprintf(leq_sub2, sizeof(leq_sub2), "1m LAeq:%5.1f LAS:%5.1f", state.min.laeq, state.min.las_max);
    } else {
      snprintf(leq_sub2, sizeof(leq_sub2), "1m Leq:%5.1f LAeq:%5.1f", state.min.bb_leq, state.min.laeq);
    }
  } else {
    snprintf(leq_sub2, sizeof(leq_sub2), "1m Avg: Warmup %02u/60s", static_cast<unsigned>(state.warmup_sec));
  }
  drawText(12, 96, leq_sub2, COLOR_BLACK, 1);

  // Right Card: 1m L90 Floor & HF Stationary Jet Nuisance Status
  drawRect(203, 28, 191, 84, COLOR_BLACK);
  const bool hf_alert = state.has_min && state.min.hf_alert;
  const uint8_t hf_idx = (state.has_min && state.min.hf_peak_band < OCTAVE_BAND_COUNT) ? state.min.hf_peak_band : 21;
  const char *hf_band_lbl = OCTAVE_BAND_LABELS[hf_idx];

  if (hf_alert) {
    fillRect(203, 28, 191, 16, COLOR_BLACK);
    char alert_hdr[32];
    snprintf(alert_hdr, sizeof(alert_hdr), "! HF JET ALERT (%sHz) !", hf_band_lbl);
    drawText(209, 32, alert_hdr, COLOR_WHITE, 1);
  } else {
    drawHLine(203, 43, 191, COLOR_BLACK);
    drawText(209, 32, state.calibrated ? "1m L90 FLOOR (dBA)" : "1m L90 FLOOR (dBFS)", COLOR_BLACK, 1);
  }

  char l90_str[16];
  if (state.has_min) {
    const float floor_1m = state.calibrated ? state.min.la90 : state.min.bb_l90;
    snprintf(l90_str, sizeof(l90_str), "%5.1f", floor_1m);
  } else {
    snprintf(l90_str, sizeof(l90_str), " --.-");
  }
  drawText(215, 52, l90_str, COLOR_BLACK, 3);

  char l90_sub1[36];
  if (state.has_min) {
    snprintf(l90_sub1, sizeof(l90_sub1), "HF(%s):%+4.1fdB Day:%lum",
             hf_band_lbl,
             state.min.hf_prom_db,
             static_cast<unsigned long>(state.hf_events_today));
  } else {
    snprintf(l90_sub1, sizeof(l90_sub1), "Waiting 1st minute...");
  }
  drawText(209, 83, l90_sub1, COLOR_BLACK, 1);

  char l90_sub2[36];
  const uint32_t clips = state.has_min ? state.min.clips : (state.has_sec ? state.sec.clips_1s : 0);
  if (state.has_min && state.min.overload) {
    snprintf(l90_sub2, sizeof(l90_sub2), "Sec:%02u/60 OVL!   Q:%u",
             static_cast<unsigned>(state.warmup_sec),
             static_cast<unsigned>(state.http_queued));
  } else if ((state.has_min && state.min.underrange) || (!state.has_min && state.has_sec && state.sec.underrange_1s)) {
    snprintf(l90_sub2, sizeof(l90_sub2), "Sec:%02u/60 UR     Q:%u",
             static_cast<unsigned>(state.warmup_sec),
             static_cast<unsigned>(state.http_queued));
  } else {
    snprintf(l90_sub2, sizeof(l90_sub2), "Sec:%02u/60 Clp:%lu Q:%u",
             static_cast<unsigned>(state.warmup_sec),
             static_cast<unsigned long>(clips),
             static_cast<unsigned>(state.http_queued));
  }
  drawText(209, 96, l90_sub2, COLOR_BLACK, 1);

  // --- 3. Lower 26-Band 1/3-Octave Spectrum Graph (y = 117..278) ---
  drawText(6, 118, "1/3-OCT (63Hz-20kHz)", COLOR_BLACK, 1);

  char gain_lbl[12];
  snprintf(gain_lbl, sizeof(gain_lbl), "G:+%.0fdB", state.gain_db);
  drawText(132, 118, gain_lbl, COLOR_BLACK, 1);

  if (state.marker_active) {
    fillRect(178, 116, 56, 11, COLOR_BLACK);
    drawText(188, 118, "*MARK*", COLOR_WHITE, 1);
  } else {
    drawText(188, 118, "M:OFF", COLOR_BLACK, 1);
  }

  drawText(244, 118, state.has_min ? "Bar=L90 Box=1s --=Floor" : "Bar=1s Live   --=Floor", COLOR_BLACK, 1);

  constexpr int PLOT_X = 32;
  constexpr int PLOT_Y = 131;
  constexpr int BAR_PITCH = 14;
  constexpr int BAR_WIDTH = 10;
  constexpr int PLOT_W = OCTAVE_BAND_COUNT * BAR_PITCH;  // 26 * 14 = 364 px (32+364=396 < 400)
  constexpr int PLOT_H = 120;  // 80 dB span => 1.5 px/dB
  constexpr int PLOT_BOTTOM = PLOT_Y + PLOT_H;
  constexpr float PX_PER_DB = 1.5f;

  const float axis_shift = state.calibrated ? (roundf(state.cal_offset_db / 10.0f) * 10.0f) : 0.0f;
  const float db_min = -100.0f + axis_shift;
  const float db_max = -20.0f + axis_shift;

  drawRect(PLOT_X - 1, PLOT_Y - 1, PLOT_W + 2, PLOT_H + 2, COLOR_BLACK);

  // Y-axis gridlines & dynamic dBFS/dBSPL labels
  const int db_offsets[] = { 80, 60, 40, 20, 0 };
  for (size_t i = 0; i < sizeof(db_offsets) / sizeof(db_offsets[0]); ++i) {
    const int db_val = static_cast<int>(lroundf(db_min)) + db_offsets[i];
    const int gy = PLOT_Y + ((80 - db_offsets[i]) * PLOT_H) / 80;
    if (i > 0 && i < 4) {
      drawDottedHLine(PLOT_X, gy, PLOT_W, COLOR_BLACK, 4);
    }
    char y_lbl[8];
    snprintf(y_lbl, sizeof(y_lbl), "%4d", db_val);
    drawText(2, gy - 3, y_lbl, COLOR_BLACK, 1);
  }

  // 26 x 1/3-octave bars (width 10 px, pitch 14 px)
  const bool have_bars = (state.has_min || state.has_sec);
  if (have_bars) {
    for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
      const int bar_x = PLOT_X + (static_cast<int>(b) * BAR_PITCH) + 2;

      if (!state.has_min) {
        const int live_h = dbToBarHeightPx(state.sec.oct_1s[b], db_min, db_max, PX_PER_DB);
        if (live_h > 0) {
          fillRect(bar_x, PLOT_BOTTOM - live_h, BAR_WIDTH, live_h, COLOR_BLACK);
        }
      } else {
        const int l90_h = dbToBarHeightPx(state.min.oct_l90[b], db_min, db_max, PX_PER_DB);
        if (l90_h > 0) {
          fillRect(bar_x, PLOT_BOTTOM - l90_h, BAR_WIDTH, l90_h, COLOR_BLACK);
        }

        if (state.has_sec) {
          const int live_h = dbToBarHeightPx(state.sec.oct_1s[b], db_min, db_max, PX_PER_DB);
          if (live_h > l90_h + 1) {
            drawRect(bar_x, PLOT_BOTTOM - live_h, BAR_WIDTH, (live_h - l90_h) + 1, COLOR_BLACK);
            fillRect(bar_x, PLOT_BOTTOM - live_h, BAR_WIDTH, 2, COLOR_BLACK);
          } else if (live_h >= 2 && live_h < l90_h) {
            drawHLine(bar_x + 1, PLOT_BOTTOM - live_h, BAR_WIDTH - 2, COLOR_WHITE);
          }
        }
      }

      // Small tick under each band
      drawVLine(bar_x + 4, PLOT_BOTTOM + 1, 3, COLOR_BLACK);
    }
  } else {
    drawText(130, PLOT_Y + 56, "Waiting for audio frames...", COLOR_BLACK, 1);
  }

  // Contrast-inverting dashed Anechoic Self-Noise Floor reference curve across all 26 bands
  for (uint8_t b = 0; b < OCTAVE_BAND_COUNT; ++b) {
    const int col_x   = PLOT_X + (static_cast<int>(b) * BAR_PITCH);
    const float ref_db = AnechoicOctL90Dbfs(b) + (state.calibrated ? state.cal_offset_db : 0.0f);
    const int floor_h = dbToBarHeightPx(ref_db, db_min, db_max, PX_PER_DB);
    const int floor_y = PLOT_BOTTOM - floor_h;
    if (floor_y > PLOT_Y && floor_y < PLOT_BOTTOM) {
      for (int x = col_x; x < col_x + BAR_PITCH; ++x) {
        if (((x - PLOT_X) & 0x03) < 2) {
          const uint8_t cur = getPixel(x, floor_y);
          setPixel(x, floor_y, (cur == COLOR_BLACK) ? COLOR_WHITE : COLOR_BLACK);
        }
      }
    }
  }

  // X-axis frequency labels (non-overlapping landmarks across 26 bands)
  struct AxisLabel { uint8_t idx; const char *txt; int offset_x; };
  static const AxisLabel kLabels[] = {
    { 0, "63",   -6 },
    { 3, "125",  -9 },
    { 6, "250",  -9 },
    { 9, "500",  -9 },
    {12, "1k",   -6 },
    {15, "2k",   -6 },
    {18, "4k",   -6 },
    {21, "8k",   -6 },
    {25, "20k", -12 }
  };
  for (size_t i = 0; i < sizeof(kLabels) / sizeof(kLabels[0]); ++i) {
    const int cx = PLOT_X + (static_cast<int>(kLabels[i].idx) * BAR_PITCH) + 7 + kLabels[i].offset_x;
    drawText(cx, PLOT_Y + PLOT_H + 7, kLabels[i].txt, COLOR_BLACK, 1);
  }

  // --- 4. Bottom Footer (y = 282..299) ---
  drawHLine(0, 281, RLCD_WIDTH, COLOR_BLACK);
  const uint32_t up_h = state.uptime_s / 3600UL;
  const uint32_t up_m = (state.uptime_s % 3600UL) / 60UL;
  const uint32_t up_s = state.uptime_s % 60UL;

  char footer[72];
  const char *sd_lbl = (state.sd_ok && state.sd_path != nullptr && state.sd_path[0] != '\0')
                         ? state.sd_path
                         : (state.sd_ok ? "/logs ready" : "NO_SD");
  snprintf(footer, sizeof(footer), "%s | Min#%lu | Up %02lu:%02lu:%02lu | SD:%s",
           DEVICE_ID,
           static_cast<unsigned long>(state.minute_count),
           static_cast<unsigned long>(up_h),
           static_cast<unsigned long>(up_m),
           static_cast<unsigned long>(up_s),
           sd_lbl);
  drawText(6, 287, footer, COLOR_BLACK, 1);

  flushFrame();
}

void DisplayUi_RenderLowBatterySleep(uint32_t vbat_mv, const char *time_hm) {
  if (!s_inited || s_disp_buf == nullptr) {
    return;
  }

  clearBuffer(COLOR_WHITE);

  // Header bar
  fillRect(0, 0, RLCD_WIDTH, 24, COLOR_BLACK);
  const char *t_str = (time_hm != nullptr && time_hm[0] != '\0') ? time_hm : "0000-00-00 00:00:00";
  drawText(6, 5, t_str, COLOR_WHITE, 2);

  char bat_hdr[24];
  snprintf(bat_hdr, sizeof(bat_hdr), "VBAT: %lu mV", static_cast<unsigned long>(vbat_mv));
  drawText(296, 8, bat_hdr, COLOR_WHITE, 1);

  // Center alert frame
  drawRect(20, 68, 360, 164, COLOR_BLACK);
  drawRect(22, 70, 356, 160, COLOR_BLACK);
  fillRect(20, 68, 360, 28, COLOR_BLACK);
  drawText(92, 78, "! LOW BATTERY PROTECTION !", COLOR_WHITE, 1);

  drawText(44, 118, "LOW BATTERY - SLEEP", COLOR_BLACK, 2);
  drawText(54, 154, "VBAT <= 3350 mV for 3 consecutive min", COLOR_BLACK, 1);
  drawText(54, 174, "MicroSD card safely flushed & unmounted.", COLOR_BLACK, 1);
  drawText(54, 194, "Charge 18650 or press KEY to wake.", COLOR_BLACK, 1);

  fillRect(0, 276, RLCD_WIDTH, 24, COLOR_BLACK);
  drawText(58, 284, "Deep Sleep Active - File System Protected", COLOR_WHITE, 1);

  flushFrame();
}
