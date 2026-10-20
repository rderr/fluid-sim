#!/usr/bin/env python3
"""Flash with esptool through a USB-UART bridge that drops bulk writes.

macOS's generic USB CDC driver does not throttle the host for some bridges
(seen with the CH343 on the Waveshare ESP32-S3-Touch-LCD-1.28 when WCH's
driver is not approved): bytes arrive at USB speed, the bridge's UART FIFO
overflows, and esptool fails with "Failed to write to target RAM". This
wrapper paces every serial write to the UART's byte rate in small pieces and
shrinks esptool's block sizes, then forwards all arguments to esptool.

Usage (board held in download mode: hold BOOT, tap RESET, release BOOT):
  ~/.platformio/penv/bin/python tools/flash_small_packets.py \
      --chip esp32s3 --port /dev/cu.usbmodemXXXX --baud 115200 \
      --before no_reset --after hard_reset write_flash ... (as usual)
"""
import sys, os, time
sys.path.insert(0, os.path.expanduser("~/.platformio/packages/tool-esptoolpy"))
import esptool
from esptool import loader

CHUNK = 64            # bytes per USB write (one full-speed bulk packet)
BAUD_GUESS = 115200   # pacing assumes this UART rate (the --baud value)
for i, a in enumerate(sys.argv):
    if a in ("--baud", "-b") and i + 1 < len(sys.argv):
        try: BAUD_GUESS = int(sys.argv[i + 1])
        except ValueError: pass
SECS_PER_CHUNK = CHUNK * 10.0 / BAUD_GUESS * 1.15   # 10 bits per byte, 15% margin

def paced_write(self, packet):
    buf = b"\xc0" + packet.replace(b"\xdb", b"\xdb\xdd").replace(b"\xc0", b"\xdb\xdc") + b"\xc0"
    for i in range(0, len(buf), CHUNK):
        self._port.write(buf[i:i + CHUNK])
        self._port.flush()
        time.sleep(SECS_PER_CHUNK)

loader.ESPLoader.write = paced_write
loader.ESPLoader.ESP_RAM_BLOCK = 0x400        # default 0x1800
try:
    from esptool.targets import esp32s3
    esp32s3.ESP32S3ROM.FLASH_WRITE_SIZE = 0x400
except Exception:
    pass

esptool.main(sys.argv[1:])
