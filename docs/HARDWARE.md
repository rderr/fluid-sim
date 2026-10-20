# Hardware notes

Two boards are supported; `src/board.h` selects one per build flag and holds
everything board-specific. The second section covers the Waveshare board.

## LOLIN S3 Mini Pro

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

## Waveshare ESP32-S3-Touch-LCD-1.28

ESP32-S3R2 (16 MB flash, 2 MB PSRAM), 1.28" round 240×240 IPS panel
(GC9A01A), QMI8658 IMU, CST816S capacitive touch (not used yet), one BOOT
button, battery connector with charger, USB-C through a **CH343P UART
bridge** (not native USB).

Sources: the [Waveshare wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.28)
and TFT_eSPI's `Setup302_Waveshare_ESP32S3_GC9A01.h`, which agree on every
pin below. Verified on hardware: 60 fps in the liquid modes, ~50 fps in heat
mode, with 524 particles.

| Function | GPIO | Notes |
|---|---|---|
| TFT SCK | 10 | |
| TFT MOSI | 11 | |
| TFT MISO | 12 | unused |
| TFT CS | 9 | |
| TFT DC | 8 | |
| TFT RST | 14 | |
| TFT backlight | 2 | active high |
| I2C SDA | 6 | shared by IMU and touch |
| I2C SCL | 7 | |
| IMU INT1 / INT2 | 4 / 3 | unused |
| Touch INT / RST | 5 / 13 | unused |
| Battery ADC | 1 | 200k/100k divider |
| BOOT button | 0 | to GND |
| UART TX / RX | 43 / 44 | via CH343P: flashing and the serial log |

**Display.** GC9A01 with the usual vendor init table, 40 MHz SPI, no
offsets, `MADCTL 0x08`. The sim runs at 120×120 and is upsampled 2×
bilinearly to 240×240 on the way out, so particle size and tuning match the
LOLIN. The tank is a circle inscribed in the domain: the rim clamps
particles, contributes ghost density, and the heater is the arc at the
bottom of the rim along gravity.

**Memory.** Two full 240×240 framebuffers (230 KB) would not fit next to the
solver, so the renderer builds 20-row bands (40 display rows) and streams
each over DMA while drawing the next. The same band path is used on the
LOLIN, which freed 48 KB there.

**Serial.** With the UART bridge, `Serial` is `HardwareSerial`; the firmware
gives it a 1 KB TX buffer and only prints stats when there is room, since a
UART write that waits for the FIFO would stall the frame loop.

**Flashing on macOS.** Without WCH's CH34x driver approved in System
Settings, macOS drives the CH343 with its generic CDC driver, which does not
throttle the host: esptool's bulk writes overflow the bridge and fail with
"Failed to write to target RAM" at any baud rate, with or without the stub.
`tools/flash_small_packets.py` paces every serial write to the UART rate and
flashes reliably at 115200 (about 25 s for the app image). It takes esptool's
normal arguments:

```
B=.pio/build/waveshare_s3_lcd_1_28
~/.platformio/penv/bin/python tools/flash_small_packets.py \
  --chip esp32s3 --port /dev/cu.usbmodemXXXX --baud 115200 \
  --before default_reset --after hard_reset write_flash --flash_mode keep --flash_freq keep --flash_size keep \
  0x0 $B/bootloader.bin 0x8000 $B/partitions.bin \
  0xe000 ~/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
  0x10000 $B/firmware.bin
```

Once the WCH driver is approved, plain `pio run -e waveshare_s3_lcd_1_28 -t upload`
should work and the MAC guard applies.

**Core balance.** This board renders four times the pixels of the LOLIN, so
`HELPER_SHARE` in `board.h` hands core 0 only 30% of the solver's pair work
(50% on the LOLIN). Rows no splat touched are filled with background instead
of being upsampled, which is most of the screen for a half-full tank.

**IMU orientation.** The chip reads gravity on its -x axis when the board is upright (its +z reads toward the viewer, yet the non-mirrored mapping 1 is the one that matches on hardware); the
default mapping is the LOLIN's (2). If the liquid falls the wrong way, hold
BOOT for 4 s (or send `o` on serial) to step through the eight mappings;
the choice is saved to flash.

**Particle count.** The lower half of the round tank holds about 520
particles at the default spacing, so that is what spawns; the 780 setting
is a ceiling, not a target.
