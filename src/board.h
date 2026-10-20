// Board selection. Plain preprocessor so fluid.h / render.h stay free of
// Arduino and the desktop harness can build either configuration
// (c++ -DBOARD_WAVESHARE_LCD_128 ...).
//
// Everything a board decides lives here: simulation domain size and shape,
// display scale and physical size, pins, and which buttons exist.
#pragma once

#if defined(BOARD_WAVESHARE_LCD_128)
// ---------------------------------------------------------------------------
// Waveshare ESP32-S3-Touch-LCD-1.28: ESP32-S3R2, 16 MB flash, 2 MB PSRAM,
// 1.28" round 240x240 GC9A01 panel, QMI8658 IMU, CST816S touch (unused),
// one BOOT button, USB via a CH343P UART bridge.
// ---------------------------------------------------------------------------
#define BOARD_NAME        "Waveshare ESP32-S3-Touch-LCD-1.28"
#define FLUID_W           120        // simulation domain (px); display is 2x
#define FLUID_H           120
#define FLUID_CIRCLE      1          // round tank inscribed in the domain
#define DISPLAY_SCALE     2          // 120 -> 240 display pixels
#define DISPLAY_MM        32.4f      // active area across (1.28" round)
#define DISPLAY_GC9A01    1
#define DISPLAY_COL_OFFSET 0
#define DISPLAY_ROW_OFFSET 0
#define DISPLAY_MADCTL    0x08       // BGR, no rotation
#define DISPLAY_SPI_HZ    80000000   // GC9A01 takes 80 MHz; the 115 KB frame needs it
#define RENDER_BAND       20         // sim rows per band (40 display rows); must divide FLUID_H
#define DEFAULT_ORIENTATION 7        // verified: IMU +z toward the viewer, gravity on -x when upright

#define PIN_TFT_SCK       10
#define PIN_TFT_MOSI      11
#define PIN_TFT_CS        9
#define PIN_TFT_DC        8
#define PIN_TFT_RST       14
#define PIN_TFT_BL        2          // active high
#define PIN_IMU_SDA       6
#define PIN_IMU_SCL       7
#define PIN_BTN_MODE      0          // BOOT: the only button
#define HAS_THREE_BUTTONS 0
#define SERIAL_IS_USB_CDC 0          // UART bridge: Serial is HardwareSerial

#else
// ---------------------------------------------------------------------------
// LOLIN S3 Mini Pro (default): ESP32-S3FH4R2, 0.85" 128x128 GC9107 panel,
// QMI8658C IMU, three buttons, native USB.
// ---------------------------------------------------------------------------
#define BOARD_NAME        "LOLIN S3 Mini Pro"
#define FLUID_W           128
#define FLUID_H           128
#define FLUID_CIRCLE      0
#define DISPLAY_SCALE     1
#define DISPLAY_MM        15.3f      // 0.85" square panel
#define DISPLAY_GC9A01    0          // GC9107 with the ST7789-style init
#define DISPLAY_COL_OFFSET 2
#define DISPLAY_ROW_OFFSET 1
#define DISPLAY_MADCTL    0x08
#define DISPLAY_SPI_HZ    40000000
#define RENDER_BAND       32
#define DEFAULT_ORIENTATION 2        // verified on hardware

#define PIN_TFT_SCK       40
#define PIN_TFT_MOSI      38
#define PIN_TFT_CS        35
#define PIN_TFT_DC        36
#define PIN_TFT_RST       34
#define PIN_TFT_BL        33         // active high
#define PIN_IMU_SDA       12
#define PIN_IMU_SCL       11
#define PIN_BTN_MODE      0          // BOOT
#define PIN_BTN_ORIENT    47
#define PIN_BTN_RESET     48
#define HAS_THREE_BUTTONS 1
#define SERIAL_IS_USB_CDC 1
#endif

#define DISPLAY_W (FLUID_W * DISPLAY_SCALE)
#define DISPLAY_H (FLUID_H * DISPLAY_SCALE)
