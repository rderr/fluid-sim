# Tuning the fluid

All knobs live in `src/fluid.h` (`struct Params`), with a comment on each
describing what you see when it goes up or down. The IMU smoothing and shake
cap are in `src/main.cpp`; the look (splat size, thresholds, colours) is in
`src/render.h` and the theme table in `src/main.cpp`.

## Describing what you want

The parameters map onto everyday descriptions:

| You'd say… | Knob | Direction |
|---|---|---|
| "more like water" / "more like honey" | `viscSigma` | down / up |
| "splashes are too violent" | `viscBeta` | up |
| "it bounces / it squishes against the wall" | `stiffness`, `nearStiffness` | up / down |
| "it beads up and clings" / "it shatters into spray" | `restDensity` | down / up |
| "feels like a thimble" / "feels like a big tank" | `gravity` | down / up |
| "it twitches when my hand shakes" | `ACCEL_ALPHA` (main.cpp) | down |
| "a film sticks to the wall" | `wallDensity` | up |
| "twisting the board should swirl it more / less" | `spinGain` (1.0 = real life at `displayMm` scale) | up / down |
| "surface shimmers when still" | `viscSigma` up, or `stiffness` down | |
| "droplets are too big" | `RADIUS`, particle count | down, up |
| "edges too blobby" | `KR`, `thrBody` (render.h) | down |

## Previewing on the desktop

The solver and renderer have no Arduino dependencies, so the harness runs
them on your machine with a scripted tilt-and-shake sequence and writes BMP
frames you can look at:

```
c++ -O2 -std=c++11 -Isrc tools/sim_test.cpp -o /tmp/sim_test
mkdir -p /tmp/frames
SIGMA=5 BETA=0.6 /tmp/sim_test /tmp/frames 780
```

Environment overrides: `RHO0 K KN SIGMA BETA GRAV WALLD` for the fluid,
`HEAT=1 HEATR COOLR COOLB DIFF BUOY` for convection, `SUBSTEPS FPS DISC`
for the integrator, `SPIN=deg/s SPING=gain` for a scripted spin (ramps up
at 2 s, holds, stops at 5 s), scenarios `HOLD=1 TILT=0.15` (steady tilt) or
`SLOSH=1` (hard tilt then settle), `SWAY=A` for a gentle rocking tilt, and
`SECONDS`, `START`, `DUMP=N` to write every Nth frame for animations. The
last argument is the particle count.

The stats line per half-second is the quickest read:

```
t= 4.00 g=( 0.00, 1.00) meanRho=1.85 maxV=27.3 meanV=7.7 lowerHalf=100% leftWall=10 pairs=5780 depth=43px
```

- `meanV` at rest is the jitter. Under ~10 px/s looks still; over ~40 visibly
  boils.
- `meanRho` versus `restDensity` shows how compressed the pool is under
  gravity. Much above rest density means a squashed pool; raise stiffness.
- `pairs` is the main cost driver on the board; `depth` is how tall the
  resting pool stands.
- `leftWall` counts particles touching the left wall, useful for the
  clinging-film check.

## Frame-rate budget

The board does about 20 ms of physics per frame at 780 particles with the
current packing. Things that cost time, in order: neighbour discovery,
displacement, viscosity, geometry refresh, density. Type `p` on the serial
monitor to see the per-phase breakdown live.

Rules of thumb:

- Pair count scales with particle count and with how compressed the pool is.
  Softer stiffness packs the pool tighter and costs more pairs; firmer
  stiffness is nearly free performance until it starts to jitter.
- Each 60 particles is roughly 5 fps at this density.
- The physics step is a fixed 1/120 s. Below 60 fps the fluid plays slightly
  slow rather than wobbling; do not make the step follow the frame time (it
  feeds back on itself).

## Heat convection

The thermal mode's knobs are at the bottom of `Params`: `heatRate`,
`coolRate`, `coolBulk`, `diffusion`, `buoyancy`, `ambient`. `buoyancy` sets
how vigorously plumes roll; `diffusion` trades sharp thin plumes for broad
warm regions; `coolBulk` sets the equilibrium so the pool doesn't saturate.
The thermal palette is stretched over `heatLo..heatHi` in `render.h` so the
contrast is visible; if the whole pool looks orange, widen that range or
raise `coolBulk`.

Preview with `HEAT=1 HOLD=1 TILT=0 /tmp/sim_test /tmp/frames`.
