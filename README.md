# Tilt fluid for the LOLIN S3 Mini Pro

A particle fluid that sloshes around the 128x128 display when you tilt or
shake the board. Gravity comes from the onboard QMI8658C accelerometer.

## Build and flash

PlatformIO (already installed in `~/.platformio`):

```
~/.platformio/penv/bin/pio run -t upload
~/.platformio/penv/bin/pio device monitor
```

The board enumerates as a USB JTAG/serial port and auto-resets for upload.
If a board is still running the factory MicroPython, hold BOOT (IO0) while
plugging it in for the first flash; after that plain uploads work.

## Controls

| Button | Action |
|---|---|
| IO47 | Cycle the accelerometer-to-screen axis mapping. Saved to flash. |
| IO48 | Short press: respawn the fluid. Hold 1.5 s with the board flat on a level table: calibrate the accelerometer bias ("level"), saved to flash. |
| IO0 (BOOT) | Cycle mode: water, lava, slime, mercury themes, then heat convection. Saved to flash. Typing `m` on the serial port does the same; `p` toggles per-phase timing output. |

## Heat convection mode

The last mode in the cycle gives every particle a temperature. The wall that
gravity points at is a heater, the opposite wall is a cold sink, exposed
surface particles cool toward ambient, and heat diffuses between neighbours.
Hot fluid feels less gravity than the pool's average and rises; cool fluid
sinks, so plumes and rolling cells form, and they lean with the board when
you tilt it. Colours run deep blue (cold) through red and orange to white
(hot). The knobs (`heatRate`, `coolRate`, `coolBulk`, `diffusion`,
`buoyancy`) are documented in `src/fluid.h`; the harness previews it with
`HEAT=1 HOLD=1 TILT=0`.

**Orientation:** the default IMU-to-screen mapping (2) was checked on
hardware. If the liquid ever runs sideways or uphill, press IO47 to cycle:
mappings 0-3 are the four rotations, 4-7 the same four mirrored. The choice
is saved to flash.

Note for anyone changing the button code: the LOLIN S3 Mini Arduino variant
defines pin 47 as `RGB_BUILTIN`, and the core's `pinMode`/`digitalRead`
silently redirect it, so the buttons use the raw IDF GPIO API.

## How it works

- `src/fluid.h` is a Clavet-style particle fluid (double density relaxation
  with a near-pressure term) on a 22x22 spatial hash. 640 particles, two
  fixed 1/120 s substeps per frame (the relaxation scheme is only stable at
  a fixed step, so a sub-60 fps frame plays slightly slow rather than wobble). Neighbour pairs are found once per substep into a compact
  list that the viscosity, density and relaxation passes all reuse. Pair
  discovery, density, displacement and geometry refresh are split between
  both cores (a helper task on core 0 takes half the grid rows / pairs), and
  heat diffusion runs on core 0 alongside the viscosity pass.
- `src/render.h` splats particles into an 8-bit density field and maps it
  through a colour LUT, giving a metaball look with a bright surface rim.
  It runs on core 0 from a position snapshot, overlapping the physics.
- `src/display.h` is a tiny GC9107 driver streaming the 32 KB framebuffer over
  SPI at 40 MHz with DMA, double-buffered.
- `src/imu.h` is a register-level QMI8658C driver (I2C on SDA 12 / SCL 11).
- `src/main.cpp` wires it together. Effective gravity is minus the measured
  proper acceleration, so shaking the board moves the liquid the right way.

## Desktop test

The physics and renderer have no Arduino dependencies and can be run on the
host to tune parameters or check stability:

```
c++ -O2 -std=c++17 -Isrc tools/sim_test.cpp -o /tmp/sim_test
/tmp/sim_test /tmp/frames        # writes BMP frames + stats
```

Parameters can be overridden with env vars (`RHO0`, `K`, `KN`, `BETA`,
`SIGMA`, `THR_RIM`, `THR_BODY`, `THR_DEEP`).
