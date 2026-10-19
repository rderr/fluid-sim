// SPI TFT driver for the two supported panels (GC9107 on the LOLIN S3 Mini
// Pro, GC9A01 on the Waveshare 1.28" round board), selected by board.h.
//
// Frames are pushed in horizontal bands: the renderer draws a band into one
// of two small buffers while the previous band streams out over DMA from the
// other. No full framebuffer is needed, which is what makes 240x240 fit next
// to the solver in internal SRAM.
#pragma once
#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "board.h"

namespace display {

constexpr int W = DISPLAY_W, H = DISPLAY_H;
constexpr int SPI_HZ = 40000000;

static spi_device_handle_t dev_ = nullptr;
static spi_transaction_t   tx_;
static bool inFlight_ = false;

// DC is driven from the transaction's user field just before each transfer.
static void IRAM_ATTR preTransfer(spi_transaction_t* t) {
  gpio_set_level((gpio_num_t)PIN_TFT_DC, (int)(intptr_t)t->user);
}

static void cmd(uint8_t c) {
  spi_transaction_t t = {};
  t.length = 8; t.tx_data[0] = c; t.flags = SPI_TRANS_USE_TXDATA; t.user = (void*)0;
  spi_device_polling_transmit(dev_, &t);
}
static void data(const uint8_t* d, size_t n) {
  spi_transaction_t t = {};
  t.length = n * 8; t.user = (void*)1;
  if (n <= 4) { memcpy(t.tx_data, d, n); t.flags = SPI_TRANS_USE_TXDATA; }
  else t.tx_buffer = d;
  spi_device_polling_transmit(dev_, &t);
}
static void d8(uint8_t v) { data(&v, 1); }
static void d16x2(uint16_t a, uint16_t b) {
  uint8_t d[4] = { (uint8_t)(a >> 8), (uint8_t)a, (uint8_t)(b >> 8), (uint8_t)b };
  data(d, 4);
}

// Command + data bytes, for the long vendor init tables.
static void cd(uint8_t c, std::initializer_list<uint8_t> d) {
  cmd(c);
  for (uint8_t v : d) d8(v);
}

static void initPanel() {
#if DISPLAY_GC9A01
  // GC9A01 vendor sequence (as used by TFT_eSPI and the Waveshare demos).
  cd(0xEF, {}); cd(0xEB, {0x14}); cd(0xFE, {}); cd(0xEF, {}); cd(0xEB, {0x14});
  cd(0x84, {0x40}); cd(0x85, {0xFF}); cd(0x86, {0xFF}); cd(0x87, {0xFF});
  cd(0x88, {0x0A}); cd(0x89, {0x21}); cd(0x8A, {0x00}); cd(0x8B, {0x80});
  cd(0x8C, {0x01}); cd(0x8D, {0x01}); cd(0x8E, {0xFF}); cd(0x8F, {0xFF});
  cd(0xB6, {0x00, 0x20});
  cd(0x3A, {0x05});                                   // 16 bpp
  cd(0x90, {0x08, 0x08, 0x08, 0x08});
  cd(0xBD, {0x06}); cd(0xBC, {0x00}); cd(0xFF, {0x60, 0x01, 0x04});
  cd(0xC3, {0x13}); cd(0xC4, {0x13}); cd(0xC9, {0x22}); cd(0xBE, {0x11});
  cd(0xE1, {0x10, 0x0E}); cd(0xDF, {0x21, 0x0C, 0x02});
  cd(0xF0, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A});
  cd(0xF1, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F});
  cd(0xF2, {0x45, 0x09, 0x08, 0x08, 0x26, 0x2A});
  cd(0xF3, {0x43, 0x70, 0x72, 0x36, 0x37, 0x6F});
  cd(0xED, {0x1B, 0x0B}); cd(0xAE, {0x77}); cd(0xCD, {0x63});
  cd(0x70, {0x07, 0x07, 0x04, 0x0E, 0x0F, 0x09, 0x07, 0x08, 0x03});
  cd(0xE8, {0x34});
  cd(0x62, {0x18, 0x0D, 0x71, 0xED, 0x70, 0x70, 0x18, 0x0F, 0x71, 0xEF, 0x70, 0x70});
  cd(0x63, {0x18, 0x11, 0x71, 0xF1, 0x70, 0x70, 0x18, 0x13, 0x71, 0xF3, 0x70, 0x70});
  cd(0x64, {0x28, 0x29, 0xF1, 0x01, 0xF1, 0x00, 0x07});
  cd(0x66, {0x3C, 0x00, 0xCD, 0x67, 0x45, 0x45, 0x10, 0x00, 0x00, 0x00});
  cd(0x67, {0x00, 0x3C, 0x00, 0x00, 0x00, 0x01, 0x54, 0x10, 0x32, 0x98});
  cd(0x74, {0x10, 0x85, 0x80, 0x00, 0x00, 0x4E, 0x00});
  cd(0x98, {0x3E, 0x07});
  cd(0x36, {DISPLAY_MADCTL});
  cmd(0x35);                                          // tearing-effect line on
  cmd(0x21);                                          // inversion on
  cmd(0x11); delay(120);                              // sleep out
  cmd(0x29); delay(20);                               // display on
#else
  // GC9107 accepts the ST7789-style sequence CircuitPython uses for this board.
  cmd(0x01); delay(150);                              // SWRESET
  cmd(0x11); delay(255);                              // SLPOUT
  cd(0x3A, {0x55}); delay(10);                        // COLMOD: 16 bpp
  cd(0x36, {DISPLAY_MADCTL});                         // MADCTL
  cmd(0x21); delay(10);                               // INVON
  cmd(0x13); delay(10);                               // NORON
  cmd(0x29); delay(255);                              // DISPON
#endif
}

inline void begin(size_t maxBandBytes) {
  pinMode(PIN_TFT_DC, OUTPUT);  digitalWrite(PIN_TFT_DC, HIGH);
  pinMode(PIN_TFT_RST, OUTPUT);
  pinMode(PIN_TFT_BL, OUTPUT);  digitalWrite(PIN_TFT_BL, LOW);

  spi_bus_config_t bus = {};
  bus.mosi_io_num = PIN_TFT_MOSI; bus.miso_io_num = -1; bus.sclk_io_num = PIN_TFT_SCK;
  bus.quadwp_io_num = -1; bus.quadhd_io_num = -1;
  bus.max_transfer_sz = (int)maxBandBytes;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

  spi_device_interface_config_t cfg = {};
  cfg.clock_speed_hz = SPI_HZ; cfg.mode = 0; cfg.spics_io_num = PIN_TFT_CS;
  cfg.queue_size = 2; cfg.pre_cb = preTransfer;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &cfg, &dev_));

  digitalWrite(PIN_TFT_RST, HIGH); delay(10);
  digitalWrite(PIN_TFT_RST, LOW);  delay(20);
  digitalWrite(PIN_TFT_RST, HIGH); delay(120);
  initPanel();
}

inline void setBacklight(bool on) { digitalWrite(PIN_TFT_BL, on ? HIGH : LOW); }

// Block until the previously queued band has been sent.
inline void waitBand() {
  if (!inFlight_) return;
  spi_transaction_t* r;
  spi_device_get_trans_result(dev_, &r, portMAX_DELAY);
  inFlight_ = false;
}

// Queue `rows` display rows starting at y0 (RGB565, byte-swapped) for DMA.
// Waits for the previous band first, so at most one buffer is in flight and
// the caller may draw into the other one meanwhile.
inline void pushBand(int y0, int rows, const uint16_t* buf) {
  waitBand();
  cmd(0x2A); d16x2(DISPLAY_COL_OFFSET, DISPLAY_COL_OFFSET + W - 1);              // CASET
  cmd(0x2B); d16x2(DISPLAY_ROW_OFFSET + y0, DISPLAY_ROW_OFFSET + y0 + rows - 1);  // RASET
  cmd(0x2C);                                                                       // RAMWR
  tx_ = {};
  tx_.length = (size_t)W * rows * 2 * 8;
  tx_.tx_buffer = buf;
  tx_.user = (void*)1;
  ESP_ERROR_CHECK(spi_device_queue_trans(dev_, &tx_, portMAX_DELAY));
  inFlight_ = true;
}

} // namespace display
