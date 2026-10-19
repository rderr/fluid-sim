# Hardware notes

Target: **LOLIN S3 Mini Pro** by Wemos. ESP32-S3FH4R2 (4 MB flash, 2 MB
PSRAM), 0.85" 128×128 TFT (GC9107), QMI8658C 6-axis IMU, three buttons, RGB
LED, IR LED.

Sources: the [Wemos schematic (v1.0.0)](https://www.wemos.cc/en/latest/_static/files/sch_s3_mini_pro_v1.0.0.pdf)
and the CircuitPython board definition. All pins below were verified on
hardware.

## Pins

| Function | GPIO | Notes |
|---|---|---|
| TFT SCK | 40 | |
| TFT MOSI | 38 | |
| TFT CS | 35 | |
| TFT DC | 36 | |
| TFT RST | 34 | |
| TFT backlight | 33 | active high (NMOS low-side switch) |
| IMU SDA | 12 | 4.7 kΩ pull-ups on board |
| IMU SCL | 11 | |
| IMU INT1 / INT2 | 18 / 21 | unused |
| Button BOOT | 0 | to GND, use pull-up |
| Button | 47 | to GND, use pull-up (see gotcha below) |
| Button | 48 | to GND, use pull-up |
| RGB LED data / power | 8 / 7 | unused |
| IR LED | 9 | unused |

## Display

The panel accepts the ST7789-style sequence CircuitPython uses for this
board: `SWRESET`, `SLPOUT`, `COLMOD 0x55`, `MADCTL 0x08`, `INVON`, `NORON`,
`DISPON`, with a column offset of 2 and a row offset of 1. SPI at 40 MHz
works. Frames go out as one 32 KB DMA transaction from the ESP-IDF SPI
master driver; DC is driven from the transaction's pre-callback.

A full-frame push with the Arduino `SPI.writeBytes()` path costs about
15 ms because it polls in 64-byte chunks. DMA brings that to under 7 ms
and runs in the background.

## IMU

QMI8658C at I2C address 0x6B (0x6A is probed too). Configured for ±4 g
accel, ±512 dps gyro, ~235 Hz ODR, low-pass filters on. Only the
accelerometer is used.

After a soft reset the chip can be left mid-transaction and fail its first
WHO_AM_I read. `imu::begin()` retries for about a second with an I2C bus
recovery (clock SCL until SDA releases, then STOP) between attempts.

The chip reports its +z axis pointing down when the board lies screen-up,
so its frame is a pure rotation of the screen frame: no mirrored mapping is
needed. Mapping 2 (screen gravity = raw accel (x, y)) is correct for this
board revision.

## Gotchas

**Pin 47 and the Arduino core.** The `lolin_s3_mini` Arduino variant used
by PlatformIO defines `RGB_BUILTIN = 47`. In arduino-esp32 2.0.x,
`pinMode()` and `digitalWrite()` on that pin are redirected to
`RGB_BUILTIN - SOC_GPIO_PIN_COUNT`, which is a nonsense pin. So
`pinMode(47, INPUT_PULLUP)` silently does nothing and the pin floats.
The firmware drives all three buttons through the raw IDF `gpio_config` /
`gpio_get_level` API instead.

**Factory MicroPython and flashing.** The board ships with MicroPython on
USB-OTG CDC. esptool cannot reset it into the bootloader, and
`machine.bootloader()` keeps the USB session alive in a state esptool can't
talk to. Hold BOOT while plugging in for the first flash. Once the Arduino
firmware is on, it uses the USB-Serial/JTAG peripheral and auto-reset
works.

**Serial writes can stall the loop.** The USB CDC driver blocks writes for
up to 100 ms once a host has connected and stopped reading. The firmware
sets the TX timeout to zero and skips stats when no host is attached.

**Another ESP32-S3 on the same hub.** macOS names the USB JTAG/serial port
by hub location, so a different board in the same port gets the same
`/dev/cu.usbmodem…` name. Check the MAC in the esptool output.

## Memory

Internal SRAM is the limit, not flash. Approximate static budget at 780
particles:

| Item | KB |
|---|---|
| Two 128×128 RGB565 framebuffers | 64 |
| Pair cache (7168 × 16 B) | 112 |
| Particle arrays (832 × ~27 floats) | 90 |
| Render band fields (2 × 4 KB) | 8 |
| Grid, snapshot, misc | ~12 |

That lands around 88% of the 320 KB. PSRAM is not used because the
solver's random access patterns would be far slower through the cache.
