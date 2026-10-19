# Architecture

## Pipeline

```
core 1 (Arduino loop)                        core 0
───────────────────────────────────────      ─────────────────────────────────
read IMU, map axes, filter ──► gravity
physics substep 1 ──┐
physics substep 2 ──┤ pair passes split ───► helper task: half of discovery,
                    │                        density, displacement, refresh;
                    ▼                        heat diffusion during viscosity
copy position (+temperature) snapshot ─────► render task: splat bands,
pace to 60 fps                               color LUT, queue DMA frame
```

The loop never waits on the display: `submitFrame()` waits only for the
previous *draw* to finish (not the DMA), copies a snapshot of positions, and
signals the render task. The render task waits for the last DMA before
queueing the next, so the two framebuffers alternate safely.

## Solver (`src/fluid.h`)

Clavet, Beaudoin & Poulin 2005, *Particle-based viscoelastic fluid
simulation*, in 2D with pixel units:

1. Gravity (per particle, scaled by buoyancy in heat mode), plus the
   fictitious forces of the board's spin about the screen normal, taken
   about the box center: centrifugal ω²r, Euler −α×r, Coriolis −2ω×v.
   ω comes from the gyro's z axis, α from its filtered derivative. The
   forces are evaluated at real scale (`displayMm` gives meters per pixel)
   and converted to sim units through the gravity scale, so `spinGain`
   1.0 is physically faithful.
2. Viscosity impulses over the neighbor list (sequential, in place).
3. Clamp speed, save positions, advect.
4. Neighbor discovery on a 22×22 grid (cell = interaction radius) using a
   half stencil so each pair is visited once. Runs every other substep; in
   between, the previous list's geometry is refreshed from current positions.
5. Density and near-density per particle, pressure from the gap to rest
   density, then Jacobi-style pair displacements, capped per particle.
   Walls contribute ghost density and push back.
6. Walls clamp positions; velocity is re-derived from the position change.
7. Heat mode: heater on the wall gravity points at, cold sink opposite,
   surface and bulk cooling, pair-wise diffusion.

Numerics that matter on an ESP32-S3:

- No hardware sqrt. A Quake-style inverse sqrt with one Newton step is used
  everywhere; this alone halved the step time.
- The relaxation displacement scales with dt², so the step is fixed at
  1/120 s. An adaptive step produced a feedback loop (longer step → pool
  expands → fewer pairs → faster frame → shorter step → …).
- A NaN guard respawns any particle whose position goes non-finite.

### Dual-core split

The pair list is one array filled from both ends: core 1 writes pairs from
grid rows `[0, rowMid)` forwards, the helper writes rows `[rowMid, GH)`
backwards. `rowMid` is chosen each discovery so the estimated pair work
(`count_r × (count_r + count_r+1)` per row) balances. Density,
displacement and refresh then split the same way (each core takes its own
range, partial sums are merged). Viscosity stays sequential because it
updates velocities in place; heat diffusion, which touches only
temperatures, runs on the helper at the same time.

Hand-off is two binary semaphores. The helper task has a higher priority
than the render task so it preempts drawing; the render has enough slack.

## Renderer (`src/render.h`)

Particles are splatted with a 7×7 kernel into an 8-bit density field,
which goes through a 256-entry color LUT: below a rim threshold is
background, a band above it is the bright rim, then body color darkening
to a deep color. In heat mode a second field accumulates temperature ×
weight; the per-pixel ratio indexes a thermal palette.

Everything is done per horizontal band: the field for one band (plus one
extra row) is built, converted to RGB565 (bilinearly upsampled when the
display is 2× the sim, as on the round board), and handed to a sink. The
firmware's sink queues the band for DMA and returns; the renderer then
draws the next band into its other buffer. There is no full framebuffer,
which is what lets a 240×240 panel fit next to the solver. Pixels are
pre-swapped to big-endian so DMA streams them unmodified.

## Boards (`src/board.h`)

A board is a set of defines: domain size and whether the tank is round,
display scale and physical size, panel type and offsets, pins, and which
buttons exist. Everything else is shared. `static_assert`s keep the band
height dividing the domain height.

## Frame budget (780 particles, measured)

| Phase | ms / frame |
|---|---|
| Discovery (per core, every other substep) | ~3 |
| Displacement (split) | ~3.5 |
| Viscosity (sequential) | ~3.5 |
| Refresh + misc loops | ~3 |
| Density (split) | ~1.5 |
| IMU read | 0.5 |
| Draw (core 0, overlapped) | 4–5 |
| DMA push (background) | ~7 |

## Things tried and dropped

- **TFT_eSPI.** No setup file exists upstream for this board; a 90-line
  driver was simpler and faster.
- **Merging the viscosity pass into the relaxation pass.** Stale geometry
  made the fluid foam.
- **Verlet list with a skin.** Rest jitter and convection move particles
  more than any affordable skin every step, so the list rebuilt anyway at a
  larger radius.
- **Packed 12-byte pairs.** Saved 25% of the pair cache but the fixed-point
  conversions cost ~30% of the step time.
- **Adaptive timestep.** See above.
- **Joining the two discovery halves with memmove.** 3 ms per frame; the
  passes now iterate both ranges instead.
