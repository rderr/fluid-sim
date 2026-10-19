// Minimal driver for the LOLIN S3 Mini Pro's 0.85" 128x128 TFT (GC9107).
// Init sequence and window offsets match the CircuitPython board definition
// for this board. Frames are streamed with the ESP-IDF SPI master driver
// using DMA, so a push runs in the background while the next frame is
// simulated (double-buffer on the caller's side).
#pragma once
#include <Arduino.h>
#include <driver/spi_master.h>
#include <driver/gpio.h>

namespace display {

constexpr int PIN_SCK  = 40;
constexpr int PIN_MOSI = 38;
constexpr int PIN_CS   = 35;
constexpr int PIN_DC   = 36;
constexpr int PIN_RST  = 34;
constexpr int PIN_BL   = 33;   // backlight, active high

constexpr int W = 128, H = 128;
constexpr int COL_OFFSET = 2;
constexpr int ROW_OFFSET = 1;
constexpr int SPI_HZ = 40000000;
constexpr size_t FRAME_BYTES = (size_t)W * H * 2;

static spi_device_handle_t dev_ = nullptr;
static spi_transaction_t frameTx_;
static bool frameInFlight_ = false;

// DC is driven from the transaction's user field just before each transfer.
static void IRAM_ATTR preTransfer(spi_transaction_t* t) {
  gpio_set_level((gpio_num_t)PIN_DC, (int)(intptr_t)t->user);
}

static void sendCmd(uint8_t c) {
  spi_transaction_t t = {};
  t.length = 8; t.tx_data[0] = c; t.flags = SPI_TRANS_USE_TXDATA; t.user = (void*)0;
  spi_device_polling_transmit(dev_, &t);
}
static void sendData(const uint8_t* d, size_t n) {
  spi_transaction_t t = {};
  t.length = n * 8; t.user = (void*)1;
  if (n <= 4) { memcpy(t.tx_data, d, n); t.flags = SPI_TRANS_USE_TXDATA; }
  else t.tx_buffer = d;
  spi_device_polling_transmit(dev_, &t);
}
static void sendData8(uint8_t d) { sendData(&d, 1); }
static void sendData16x2(uint16_t a, uint16_t b) {
  uint8_t d[4] = { (uint8_t)(a >> 8), (uint8_t)a, (uint8_t)(b >> 8), (uint8_t)b };
  sendData(d, 4);
}

inline void begin() {
  pinMode(PIN_DC, OUTPUT);  digitalWrite(PIN_DC, HIGH);
  pinMode(PIN_RST, OUTPUT);
  pinMode(PIN_BL, OUTPUT);  digitalWrite(PIN_BL, LOW);

  spi_bus_config_t bus = {};
  bus.mosi_io_num = PIN_MOSI; bus.miso_io_num = -1; bus.sclk_io_num = PIN_SCK;
  bus.quadwp_io_num = -1; bus.quadhd_io_num = -1;
  bus.max_transfer_sz = FRAME_BYTES;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

  spi_device_interface_config_t cfg = {};
  cfg.clock_speed_hz = SPI_HZ; cfg.mode = 0; cfg.spics_io_num = PIN_CS;
  cfg.queue_size = 2; cfg.pre_cb = preTransfer;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &cfg, &dev_));

  digitalWrite(PIN_RST, HIGH); delay(10);
  digitalWrite(PIN_RST, LOW);  delay(20);
  digitalWrite(PIN_RST, HIGH); delay(120);

  sendCmd(0x01); delay(150);                 // SWRESET
  sendCmd(0x11); delay(255);                 // SLPOUT
  sendCmd(0x3A); sendData8(0x55); delay(10); // COLMOD: 16 bpp
  sendCmd(0x36); sendData8(0x08);            // MADCTL: BGR order, no rotation
  sendCmd(0x21); delay(10);                  // INVON
  sendCmd(0x13); delay(10);                  // NORON
  sendCmd(0x29); delay(255);                 // DISPON
}

inline void setBacklight(bool on) { digitalWrite(PIN_BL, on ? HIGH : LOW); }

// Block until the previously queued frame has been sent.
inline void waitFrame() {
  if (!frameInFlight_) return;
  spi_transaction_t* r;
  spi_device_get_trans_result(dev_, &r, portMAX_DELAY);
  frameInFlight_ = false;
}

// Queue a full W*H RGB565 frame (byte-swapped, big-endian) for DMA transfer.
// The buffer must stay untouched until the next waitFrame()/pushFrame().
inline void pushFrame(const uint16_t* fb) {
  waitFrame();
  sendCmd(0x2A); sendData16x2(COL_OFFSET, COL_OFFSET + W - 1);   // CASET
  sendCmd(0x2B); sendData16x2(ROW_OFFSET, ROW_OFFSET + H - 1);   // RASET
  sendCmd(0x2C);                                                 // RAMWR
  frameTx_ = {};
  frameTx_.length = FRAME_BYTES * 8;
  frameTx_.tx_buffer = fb;
  frameTx_.user = (void*)1;
  ESP_ERROR_CHECK(spi_device_queue_trans(dev_, &frameTx_, portMAX_DELAY));
  frameInFlight_ = true;
}

} // namespace display
