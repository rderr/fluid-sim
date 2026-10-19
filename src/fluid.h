// Particle fluid simulation (Clavet et al. 2005, "Particle-based Viscoelastic
// Fluid Simulation": double density relaxation). Pure C++, no Arduino deps,
// so it can be unit-tested on the desktop (see tools/sim_test.cpp).
//
// Units: pixels and seconds. The domain is W x H pixels.
#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "board.h"

namespace fluid {

constexpr int  W = FLUID_W;
constexpr int  H = FLUID_H;
constexpr bool CIRCLE = FLUID_CIRCLE != 0;   // round tank inscribed in the domain
constexpr int MAX_P = 832;

constexpr float RADIUS = 6.0f;     // interaction radius h (px)
constexpr int   CELL = 6;          // spatial hash cell size, must be >= RADIUS
constexpr int   GW = (W + CELL - 1) / CELL;
constexpr int   GH = (H + CELL - 1) / CELL;
constexpr int   CELLS = GW * GH;
constexpr float WALL = 1.5f;       // wall margin (px)
constexpr float CX = W * 0.5f, CY = H * 0.5f;          // domain center
constexpr float CR = (W < H ? W : H) * 0.5f - WALL;    // round-tank radius

// Quake-style inverse square root with one Newton step (~0.2% error);
// the ESP32-S3 has no hardware sqrt and this runs ~10x faster than sqrtf.
static inline float fastInvSqrt(float x) {
  union { float f; uint32_t i; } u; u.f = x;
  u.i = 0x5f375a86 - (u.i >> 1);
  float y = u.f;
  return y * (1.5f - 0.5f * x * y * y);
}

// Tuning knobs. Each comment says what you see on the screen when the value
// goes up or down. Changes can be previewed with tools/sim_test.cpp before
// flashing (env vars RHO0, K, KN, SIGMA, BETA, GRAV).
struct Params {
  // Cohesion / surface tension. Sum of (1-q)^2 over neighbors at rest.
  //   up:   liquid beads up, holds together, clings to walls, fewer stray
  //         droplets; too high and it clumps into lumps and looks lumpy.
  //   down: falls apart easily into spray, wets walls less, flatter surface;
  //         too low and it never gathers back into one body.
  float restDensity   = 1.25f;

  // Pressure stiffness. How hard the liquid resists being compressed.
  //   up:   firmer, more incompressible, bounces off walls, slosh waves
  //         travel faster; too high and the surface shimmers or boils.
  //   down: squishier, packs down against walls when tilted, waves are
  //         slower and softer; too low and the volume visibly shrinks.
  float stiffness     = 3500.f;

  // Near-pressure (anti-clustering). Keeps particles from stacking on top
  // of each other; usually kept at ~2x stiffness.
  //   up:   more even spacing, smoother interior, slightly more jitter.
  //   down: particles bunch into dense knots and holes open in the body.
  float nearStiffness = 7000.f;

  // Linear viscosity. Thickness: water vs honey.
  //   up:   thicker, settles faster, surface calms quickly, small tilts
  //         produce slow smooth flow; too high and it crawls like syrup.
  //   down: more watery, livelier, surface ripples longer; too low and it
  //         jitters at rest and over-reacts to tiny movements.
  float viscSigma     = 2.0f;

  // Quadratic viscosity. Only acts on fast collisions, so it damps splashes
  // without thickening gentle flow.
  //   up:   hard shakes land with a dull thud, fewer and lower droplets.
  //   down: violent splashes, droplets fly high and bounce.
  float viscBeta      = 0.5f;

  // Gravity scale, px/s^2 per 1 g measured by the IMU.
  //   up:   heavier and faster, behaves like a thimble: everything happens
  //         at once and small tilts already pour it to one side.
  //   down: lighter and slower, behaves like a large tank: lazy sloshing,
  //         gentle tilts only tip the surface.
  float gravity       = 1000.f;

  // Neighbor discovery every substep (true) or every other substep with a
  // geometry refresh in between (false). Halving discovery saves ~15% of the
  // frame; particles move well under a pixel per substep so little is missed.
  bool discoverEveryStep = false;

  // Wall ghost density. The wall counts as a layer of neighbors so particles
  // touching it feel normal pressure instead of the negative (cohesive)
  // pressure that makes a film cling to the wall.
  //   up:   walls shed liquid cleanly, no stuck lines; too high and the
  //         liquid hovers a pixel or two off the walls.
  //   down: liquid wets and clings to the walls (0 = old behavior).
  float wallDensity   = 1.0f;

  // Speed cap, px/s. Safety limit so no particle skips a grid cell in one
  // step; it should rarely be reached. Lower only if shakes look wrong.
  float maxSpeed      = 0.9f * CELL * 60.f;

  // Physical size of the display's active area (mm across the W pixels).
  // Sets the real-world scale of a pixel, which the spin forces need: they
  // grow with the real radius, so a 15 mm panel swirls far less than a
  // meter-wide tank would. LOLIN S3 Mini Pro: 0.85" square panel.
  float displayMm     = DISPLAY_MM;

  // Spin response. When the board rotates about the screen normal the
  // fluid, in the board's frame, feels the Euler force (lag when the spin
  // starts or stops: this is the swirl), the centrifugal force (pushes
  // outward into the walls while spinning) and the Coriolis force (curves
  // the flow). They are computed at real scale from displayMm and brought
  // into sim units through the same factor as gravity, so 1.0 means
  // "as real life"; this gain exaggerates or mutes that. 0 turns them off.
  //   up:   a flick of the wrist sends the pool around the box, steady
  //         spinning plasters it against the walls.
  //   down: only fast spins have a visible effect.
  float spinGain      = 15.0f;  // exaggerated on purpose: 1.0 (real life) is too tame to enjoy on a 15 mm panel

  // ---- Heat convection (only used when heat is enabled) ----------------
  // Temperatures run 0 (cold) to 1 (heater). The wall that gravity points
  // at is the heater, the opposite wall is the cold sink, and the free
  // surface also cools toward ambient.
  float ambient       = 0.25f;  // temperature particles cool toward at the surface
  // Heater strength, 1/s. How fast particles touching the hot wall heat up.
  //   up: plumes form sooner and hotter.  down: gentle, sparse plumes.
  float heatRate      = 2.5f;
  // Cooling strength, 1/s, at the cold wall and the free surface.
  //   up: hot plumes die quickly at the top, strong sinking flow.
  //   down: heat accumulates and the whole pool warms up.
  float coolRate      = 1.5f;
  // Thermal diffusion between neighbors, 1/s.
  //   up: smooth temperature, plumes blur into broad warm regions.
  //   down: sharp thin plumes that keep their identity longer.
  float diffusion     = 2.0f;
  // Bulk heat loss to the air, 1/s, applied to every particle. Sets the
  // equilibrium: heater input vs this loss gives a stable bottom-hot,
  // top-cool gradient instead of the whole pool saturating.
  //   up: cooler pool, stronger contrast, plumes fade faster.
  //   down: pool warms through; too low and everything goes white-hot.
  float coolBulk      = 0.6f;
  // Buoyancy: effective gravity = gravity * (1 - buoyancy * (T - Tmean)),
  // relative to the pool's mean temperature so the pool's overall weight
  // stays the same and only temperature differences drive the flow.
  //   up: hot fluid rises hard, vigorous rolling convection.
  //   down: lazy, barely visible circulation.
  float buoyancy      = 2.5f;
};

class Sim {
public:
  int n = 0;
  float px[MAX_P], py[MAX_P];
  float vx[MAX_P], vy[MAX_P];
  float temp[MAX_P];       // 0..1, only meaningful when heat is on
  bool  heat = false;      // enable convection
  Params p;

  // Optional second-core helper. When set, the density and displacement
  // passes are split: this core does pairs [0, mid), the helper runs
  // runJob(job) for [mid, nPairs). start() must return immediately and
  // wait() must block until the job has finished.
  // Optional profiling clock (microseconds). When set, per-phase times
  // accumulate in prof[] (grid, pairs, density, displace, viscosity, heat, rest).
  uint32_t (*clock)() = nullptr;
  uint32_t prof[7] = {0, 0, 0, 0, 0, 0, 0};
  uint32_t profOwn[2] = {0, 0}, profWait[2] = {0, 0};   // discovery, displacement: own work / wait for helper

  struct Helper { void (*start)(void*, int job); void (*wait)(void*); void* ctx; };
  Helper helper { nullptr, nullptr, nullptr };
  enum { JOB_DENSITY = 0, JOB_DISPLACE = 1, JOB_DISCOVER = 2, JOB_REFRESH = 3, JOB_DIFFUSE = 4 };
  void runJob(int job) {
    switch (job) {
      case JOB_DENSITY:  densityRange(b0_, MAX_PAIRS, rhoB_, rhoNB_); break;
      case JOB_DISPLACE: displaceRange(b0_, MAX_PAIRS, dxB_, dyB_); break;
      case JOB_DISCOVER: discoverRows(rowMid_, GH, true); break;
      case JOB_REFRESH:  refreshRange(b0_, MAX_PAIRS); break;
      case JOB_DIFFUSE:  diffuse(); break;
    }
  }

  // Spawn `count` particles on a lattice filling the lower `fill` fraction of the box.
  void reset(int count, float spacing = 3.3f, float fill = 0.5f) {
    if (count > MAX_P) count = MAX_P;
    n = 0; nPairs_ = 0; b0_ = MAX_PAIRS;
    int cols = (int)((W - 2 * WALL) / spacing);
    float x0 = (W - cols * spacing) * 0.5f + spacing * 0.5f;
    float y = H - WALL - spacing * 0.5f;
    float yMin = H * (1.0f - fill);
    while (n < count && y > yMin) {
      for (int c = 0; c < cols && n < count; ++c) {
        float x = x0 + c * spacing + ((int)(y / spacing) & 1 ? spacing * 0.25f : 0.f);
        if (CIRCLE) {
          float dx = x - CX, dy = y - CY;
          if (dx * dx + dy * dy > (CR - spacing * 0.5f) * (CR - spacing * 0.5f)) continue;
        }
        px[n] = x;
        py[n] = y;
        vx[n] = vy[n] = 0.f;
        temp[n] = p.ambient;
        ++n;
      }
      y -= spacing * 0.87f;
    }
  }

  // One simulation step. (gx, gy) is the effective gravity in g units
  // (screen coordinates: +x right, +y down).
  // omega: board spin rate about the screen normal (rad/s, positive =
  // clockwise on screen); alpha: its rate of change (rad/s^2).
  void step(float dt, float gx, float gy, float omega = 0.f, float alpha = 0.f) {
    const float h = RADIUS;
    uint32_t t0 = clock ? clock() : 0, t1;
    auto lap = [&](int slot) { if (clock) { t1 = clock(); prof[slot] += t1 - t0; t0 = t1; } };

    // 1. gravity (scaled per particle by buoyancy when heat is on)
    const float ax = gx * p.gravity * dt, ay = gy * p.gravity * dt;
    if (heat) {
      float tm = meanTemp();
      for (int i = 0; i < n; ++i) {
        float sc = 1.f - p.buoyancy * (temp[i] - tm);
        if (sc < -0.5f) sc = -0.5f; else if (sc > 2.f) sc = 2.f;
        vx[i] += ax * sc; vy[i] += ay * sc;
      }
    } else {
      for (int i = 0; i < n; ++i) { vx[i] += ax; vy[i] += ay; }
    }
    // 1b. fictitious forces from the board spinning about the screen normal,
    //     taken about the box center: centrifugal w^2 r, Euler -alpha x r,
    //     Coriolis -2 w x v (screen frame: x right, y down, z into screen)
    if (p.spinGain > 0.f && (omega != 0.f || alpha != 0.f)) {
      // real meters per pixel, then physical m/s^2 -> sim px/s^2 via the
      // gravity scale (p.gravity px/s^2 per 9.81 m/s^2)
      const float mPerPx = p.displayMm * 1e-3f / W;
      const float g = p.spinGain * mPerPx * (p.gravity / 9.81f) * dt;
      const float w2 = omega * omega, cx = CX, cy = CY;
      for (int i = 0; i < n; ++i) {
        float rx = px[i] - cx, ry = py[i] - cy;
        float fx = w2 * rx + alpha * ry + 2.f * omega * vy[i];
        float fy = w2 * ry - alpha * rx - 2.f * omega * vx[i];
        vx[i] += fx * g; vy[i] += fy * g;
      }
    }

    // 2. viscosity impulses, using the pair list found in the previous step
    //    (positions have moved by at most a fraction of a pixel since).
    //    Heat diffusion uses the same list and runs on the helper meanwhile.
    bool diffusing = false;
    if (heat && nPairs_ > 0) {
      diffK_ = p.diffusion * dt; if (diffK_ > 0.04f) diffK_ = 0.04f;
      if (helper.start) { helper.start(helper.ctx, JOB_DIFFUSE); diffusing = true; }
      else diffuse();
    }
    if (p.viscSigma > 0.f || p.viscBeta > 0.f) {
      viscosityRange(0, nPairs_, dt);
      viscosityRange(b0_, MAX_PAIRS, dt);
    }

    if (diffusing) helper.wait(helper.ctx);
    lap(4);   // gravity + viscosity (+ diffusion on the helper)
    // 3. clamp speed, save old positions, advect
    const float vmax2 = p.maxSpeed * p.maxSpeed;
    for (int i = 0; i < n; ++i) {
      float s2 = vx[i] * vx[i] + vy[i] * vy[i];
      if (s2 > vmax2) { float s = p.maxSpeed * fastInvSqrt(s2); vx[i] *= s; vy[i] *= s; }
      ox_[i] = px[i]; oy_[i] = py[i];
      px[i] += vx[i] * dt; py[i] += vy[i] * dt;
    }

    // 4. find neighbor pairs once, then double density relaxation
    lap(6);
    if (p.discoverEveryStep || (stepCount_++ & 1) == 0 || nPairs_ + (MAX_PAIRS - b0_) == 0) {
      buildGrid();
      lap(0);
      findPairs();
    } else {
      refreshPairs();     // reuse last step's list; particles moved < 1 px since
    }
    lap(1);
    for (int i = 0; i < n; ++i) { rho_[i] = 0.f; rhoN_[i] = 0.f; }
    if (helper.start) {
      for (int i = 0; i < n; ++i) { rhoB_[i] = 0.f; rhoNB_[i] = 0.f; }
      helper.start(helper.ctx, JOB_DENSITY);
      densityRange(0, nPairs_, rho_, rhoN_);
      helper.wait(helper.ctx);
      for (int i = 0; i < n; ++i) { rho_[i] += rhoB_[i]; rhoN_[i] += rhoNB_[i]; }
    } else {
      densityRange(0, nPairs_, rho_, rhoN_);
    }
    lap(2);
    const float invH = 1.0f / h;
    if (heat) heatStep(dt, gx, gy, invH);
    lap(5);
    if (CIRCLE && p.wallDensity > 0.f) {
      for (int i = 0; i < n; ++i) {
        float dx = px[i] - CX, dy = py[i] - CY;
        float r2 = dx * dx + dy * dy;
        if (r2 < 1e-6f) continue;
        float invR = fastInvSqrt(r2);
        float q = (CR - r2 * invR) * invH;          // distance to the rim, as q
        if (q < 1.f) {
          float a = 1.f - (q < 0.f ? 0.f : q);
          rho_[i]  += a * a * p.wallDensity;
          rhoN_[i] += a * a * a * p.wallDensity;
          wn_[i] = a;                               // remembered for the push below
        } else wn_[i] = 0.f;
      }
    } else if (p.wallDensity > 0.f) {
      for (int i = 0; i < n; ++i) {
        // nearest distance to each wall, as q = d/h
        float qs[4] = { (px[i] - WALL) * invH, (W - WALL - px[i]) * invH,
                        (py[i] - WALL) * invH, (H - WALL - py[i]) * invH };
        for (int w = 0; w < 4; ++w) {
          if (qs[w] < 1.f) {
            float a = 1.f - (qs[w] < 0.f ? 0.f : qs[w]);
            rho_[i]  += a * a * p.wallDensity;
            rhoN_[i] += a * a * a * p.wallDensity;
          }
        }
      }
    }
    for (int i = 0; i < n; ++i) {
      pr_[i]  = p.stiffness * (rho_[i] - p.restDensity);
      prN_[i] = p.nearStiffness * rhoN_[i];
      dx_[i] = dy_[i] = 0.f;
    }
    const float dt2 = dt * dt;
    if (CIRCLE && p.wallDensity > 0.f) {
      // the rim pushes back toward the center like an immovable neighbor
      for (int i = 0; i < n; ++i) {
        float a = wn_[i];
        if (a <= 0.f) continue;
        float dx = px[i] - CX, dy = py[i] - CY;
        float r2 = dx * dx + dy * dy;
        if (r2 < 1e-6f) continue;
        float invR = fastInvSqrt(r2);
        float D = dt2 * (pr_[i] * a + prN_[i] * a * a) * p.wallDensity * 0.5f;
        if (D < 0.f) D = 0.f;
        dx_[i] -= D * dx * invR; dy_[i] -= D * dy * invR;
      }
    } else if (p.wallDensity > 0.f) {
      // the wall pushes back like an immovable neighbor
      for (int i = 0; i < n; ++i) {
        float qs[4] = { (px[i] - WALL) * invH, (W - WALL - px[i]) * invH,
                        (py[i] - WALL) * invH, (H - WALL - py[i]) * invH };
        static const float sx[4] = { 1.f, -1.f, 0.f, 0.f };
        static const float sy[4] = { 0.f, 0.f, 1.f, -1.f };
        for (int w = 0; w < 4; ++w) {
          if (qs[w] < 1.f) {
            float a = 1.f - (qs[w] < 0.f ? 0.f : qs[w]);
            float D = dt2 * (pr_[i] * a + prN_[i] * a * a) * p.wallDensity * 0.5f;
            if (D < 0.f) D = 0.f;              // walls only push, never pull
            dx_[i] += D * sx[w]; dy_[i] += D * sy[w];
          }
        }
      }
    }
    dt2_ = dt2;
    if (helper.start) {
      for (int i = 0; i < n; ++i) { dxB_[i] = 0.f; dyB_[i] = 0.f; }
      uint32_t a = clock ? clock() : 0;
      helper.start(helper.ctx, JOB_DISPLACE);
      displaceRange(0, nPairs_, dx_, dy_);
      uint32_t b = clock ? clock() : 0;
      helper.wait(helper.ctx);
      if (clock) { profOwn[1] += b - a; profWait[1] += clock() - b; }
      for (int i = 0; i < n; ++i) { dx_[i] += dxB_[i]; dy_[i] += dyB_[i]; }
    } else {
      displaceRange(0, nPairs_, dx_, dy_);
    }
    lap(3);
    const float maxD = 0.5f * h, maxD2 = maxD * maxD;
    for (int i = 0; i < n; ++i) {
      float d2 = dx_[i] * dx_[i] + dy_[i] * dy_[i];
      if (d2 > maxD2) { float s = maxD * fastInvSqrt(d2); dx_[i] *= s; dy_[i] *= s; }
      px[i] += dx_[i]; py[i] += dy_[i];
    }

    // 5. walls + 6. velocity from positions
    const float invDt = 1.0f / dt;
    for (int i = 0; i < n; ++i) {
      if (!(px[i] == px[i]) || !(py[i] == py[i])) {   // NaN guard: respawn at the center
        px[i] = W * 0.5f; py[i] = H * 0.5f; ox_[i] = px[i]; oy_[i] = py[i];
      }
      if (CIRCLE) {
        float dx = px[i] - CX, dy = py[i] - CY;
        float r2 = dx * dx + dy * dy;
        if (r2 > CR * CR) { float s = CR * fastInvSqrt(r2); px[i] = CX + dx * s; py[i] = CY + dy * s; }
      } else {
        if (px[i] < WALL) px[i] = WALL; else if (px[i] > W - WALL) px[i] = W - WALL;
        if (py[i] < WALL) py[i] = WALL; else if (py[i] > H - WALL) py[i] = H - WALL;
      }
      vx[i] = (px[i] - ox_[i]) * invDt;
      vy[i] = (py[i] - oy_[i]) * invDt;
    }

    // 7. refresh pair geometry from the final positions so the next step's
    //    viscosity pass sees current directions and distances
    refreshPairs();
    lap(6);
  }

  int pairCount() const { return nPairs_ + (MAX_PAIRS - b0_); }
  float meanTemp() const { float t = 0; for (int i = 0; i < n; ++i) t += temp[i]; return n ? t / n : 0.f; }

private:
  // Heater on the wall gravity points at, cold sink on the opposite wall,
  // surface cooling where the neighborhood is sparse, diffusion over pairs.
  void heatStep(float dt, float gx, float gy, float invH) {
    // The heater is the whole wall whose outward direction is closest to
    // gravity (switches at 45 degrees); the cold sink is the opposite wall.
    // Distances are measured perpendicular to those walls.
    float ax = gx < 0 ? -gx : gx, ay = gy < 0 ? -gy : gy;
    bool horizontal = ax > ay;                      // hot wall is left/right
    float hotEdge, coldEdge;
    if (horizontal) { hotEdge = gx > 0 ? W - WALL : WALL; coldEdge = gx > 0 ? WALL : W - WALL; }
    else            { hotEdge = gy > 0 ? H - WALL : WALL; coldEdge = gy > 0 ? WALL : H - WALL; }
    // Round tank: measure along gravity instead; the heater is the arc at
    // the bottom of the rim, the cold sink the arc at the top.
    float gm2 = gx * gx + gy * gy; if (gm2 < 1e-4f) gm2 = 1e-4f;
    float ginv = fastInvSqrt(gm2), ux = gx * ginv, uy = gy * ginv;
    float meanRho = 0; for (int i = 0; i < n; ++i) meanRho += rho_[i];
    meanRho = n ? meanRho / n : 1.f;
    const float surfaceRho = 0.7f * meanRho;        // sparse neighborhood = exposed surface
    const float bulk = p.coolBulk * dt;
    for (int i = 0; i < n; ++i) {
      float qh, qc;
      if (CIRCLE) {
        float d = (px[i] - CX) * ux + (py[i] - CY) * uy;   // position along gravity
        qh = (CR - d) * invH;                              // 0 at the bottom of the rim
        qc = (CR + d) * invH;                              // 0 at the top
      } else {
        float c = horizontal ? px[i] : py[i];
        float dh = hotEdge - c, dc = coldEdge - c;
        qh = (dh < 0 ? -dh : dh) * invH;      // 0 at the hot wall, 1 at h away
        qc = (dc < 0 ? -dc : dc) * invH;
      }
      float T = temp[i];
      if (qh < 1.f) { float a = 1.f - (qh < 0 ? 0 : qh); T += p.heatRate * dt * a * (1.f - T); }
      if (qc < 1.f) { float a = 1.f - (qc < 0 ? 0 : qc); T += p.coolRate * dt * a * (0.f - T); }
      if (rho_[i] < surfaceRho) {                   // exposed: cool toward ambient
        float a = 1.f - rho_[i] / surfaceRho;
        T += p.coolRate * dt * a * (p.ambient - T);
      }
      T += bulk * (p.ambient - T);
      temp[i] = T;
    }
  }

  // Explicit diffusion over the pair list (k*dt*neighbors stays well below 1).
  // Touches only temp[], so it can run on the helper while this core runs
  // the viscosity pass (which touches only velocities).
  float diffK_ = 0.f;
  void diffuse() {
    const float k = diffK_;
    for (int c = 0; c < MAX_PAIRS; ++c) {
      if (c == nPairs_) { c = b0_; if (c >= MAX_PAIRS) break; }
      const Pair& pr = pairs_[c];
      float w = 1.f - pr.q();
      if (w <= 0.f) continue;
      float f = k * w * (temp[pr.j] - temp[pr.i]);
      temp[pr.i] += f; temp[pr.j] -= f;
    }
  }

public:

  // Average neighbor density (diagnostic).
  float meanDensity() const {
    float s = 0; for (int i = 0; i < n; ++i) s += rho_[i];
    return n ? s / n : 0.f;
  }

private:
  float ox_[MAX_P], oy_[MAX_P];
  float rho_[MAX_P], rhoN_[MAX_P], pr_[MAX_P], prN_[MAX_P];
  float dx_[MAX_P], dy_[MAX_P];
  float rhoB_[MAX_P], rhoNB_[MAX_P], dxB_[MAX_P], dyB_[MAX_P];   // helper's partial sums
  float wn_[MAX_P];                  // round tank: rim ghost-density weight per particle
  float dt2_ = 0.f;
  uint32_t stepCount_ = 0;


  void viscosityRange(int a, int b, float dt) {
    const float sig = p.viscSigma, beta = p.viscBeta;
    for (int k = a; k < b; ++k) {
      const Pair& pr = pairs_[k];
      int i = pr.i, j = pr.j;
      float u = (vx[i] - vx[j]) * pr.nx() + (vy[i] - vy[j]) * pr.ny();
      if (u > 0.f && pr.q() < 1.f) {
        float I = dt * (1.f - pr.q()) * (sig * u + beta * u * u) * 0.5f;
        if (I > u * 0.5f) I = u * 0.5f;   // never reverse the relative velocity
        vx[i] -= I * pr.nx(); vy[i] -= I * pr.ny();
        vx[j] += I * pr.nx(); vy[j] += I * pr.ny();
      }
    }
  }

  void densityRange(int a, int b, float* rho, float* rhoN) const {
    for (int k = a; k < b; ++k) {
      const Pair& pr = pairs_[k];
      float w = 1.f - pr.q();
      if (w <= 0.f) continue;                 // in the skin, not in range
      float w2 = w * w, w3 = w2 * w;
      rho[pr.i] += w2; rhoN[pr.i] += w3;
      rho[pr.j] += w2; rhoN[pr.j] += w3;
    }
  }
  void displaceRange(int a, int b, float* dx, float* dy) const {
    const float dt2 = dt2_;
    for (int k = a; k < b; ++k) {
      const Pair& pr = pairs_[k];
      int i = pr.i, j = pr.j;
      float w = 1.f - pr.q();
      if (w <= 0.f) continue;
      float D = dt2 * ((pr_[i] + pr_[j]) * w + (prN_[i] + prN_[j]) * w * w) * 0.5f;
      float Dx = D * pr.nx(), Dy = D * pr.ny();
      dx[i] -= Dx; dy[i] -= Dy;
      dx[j] += Dx; dy[j] += Dy;
    }
  }

  // Neighbor pair cache: unit vector from i to j and q = r/h.
  struct Pair {
    uint16_t i, j;
    float nx_, ny_, q_;
    inline float nx() const { return nx_; }
    inline float ny() const { return ny_; }
    inline float q()  const { return q_; }
    inline void set(float x, float y, float qq) { nx_ = x; ny_ = y; q_ = qq; }
  };
  static constexpr int MAX_PAIRS = 7168;
  Pair pairs_[MAX_PAIRS];
  int  nPairs_ = 0;

  void refreshPairs() {
    if (helper.start) {
      helper.start(helper.ctx, JOB_REFRESH);
      refreshRange(0, nPairs_);
      helper.wait(helper.ctx);
    } else {
      refreshRange(0, nPairs_);
    }
  }
  void refreshRange(int a, int b) {
    const float invH = 1.0f / RADIUS;
    for (int k = a; k < b; ++k) {
      Pair& pr = pairs_[k];
      float rx = px[pr.j] - px[pr.i], ry = py[pr.j] - py[pr.i];
      float r2 = rx * rx + ry * ry;
      if (r2 < 1e-6f) { pr.q_ = 1.f; continue; }       // coincident: skip in viscosity
      float invR = fastInvSqrt(r2);
      pr.set(rx * invR, ry * invR, r2 * invR * invH);
    }
  }

public:
  int rowMid_ = 0;                 // first grid row handled by the helper
private:
  // The pair list is two ranges so both cores can fill it concurrently:
  // range A = [0, nPairs_) written by this core, range B = [b0_, MAX_PAIRS)
  // written backwards by the helper. b0_ == MAX_PAIRS when B is empty.
  int b0_ = MAX_PAIRS;

  // Discover pairs in grid rows [r0, r1). The helper writes backwards from
  // the end of pairs_ so both halves can fill the same array concurrently.
  void discoverRows(int r0, int r1, bool back) {
    const int cap = helper.start ? MAX_PAIRS / 2 : MAX_PAIRS;
    int cnt = 0;
    forEachPairRows(r0, r1, [&](int i, int j, float rx, float ry, float invR, float q) {
      if (cnt >= cap) return;              // degrade gracefully when over-packed
      Pair& pr = back ? pairs_[MAX_PAIRS - 1 - cnt] : pairs_[cnt];
      cnt++;
      pr.i = (uint16_t)i; pr.j = (uint16_t)j;
      pr.set(rx * invR, ry * invR, q);
    });
    if (back) b0_ = MAX_PAIRS - cnt; else nPairs_ = cnt;
  }

  void findPairs() {
    if (!helper.start) { discoverRows(0, GH, false); b0_ = MAX_PAIRS; return; }
    // split rows where the estimated pair work (count_r * (count_r + count_r+1))
    // is balanced, since the fluid pools densely at one side of the box
    uint32_t work[GH], total = 0;
    for (int r = 0; r < GH; ++r) {
      uint32_t c  = cellStart_[(r + 1) * GW] - cellStart_[r * GW];
      uint32_t c1 = (r + 1 < GH) ? cellStart_[(r + 2) * GW] - cellStart_[(r + 1) * GW] : 0;
      work[r] = c * (c + c1) + 1;
      total += work[r];
    }
    uint32_t acc = 0; rowMid_ = GH / 2;
    for (int r = 0; r < GH; ++r) {
      acc += work[r];
      if (acc * 2 >= total) { rowMid_ = r + 1; break; }
    }
    if (rowMid_ >= GH) rowMid_ = GH - 1;
    if (rowMid_ < 1) rowMid_ = 1;
    b0_ = MAX_PAIRS;
    uint32_t a = clock ? clock() : 0;
    helper.start(helper.ctx, JOB_DISCOVER);
    discoverRows(0, rowMid_, false);
    uint32_t b = clock ? clock() : 0;
    helper.wait(helper.ctx);
    if (clock) { profOwn[0] += b - a; profWait[0] += clock() - b; }
  }
  uint16_t cellStart_[CELLS + 1];
  uint16_t sorted_[MAX_P];
  float spx_[MAX_P], spy_[MAX_P];   // positions in cell-sorted order (sequential reads)
  uint16_t cellOf_[MAX_P];

  static inline int cellCoord(float v, int max) {
    int c = (int)(v * (1.0f / CELL));
    return c < 0 ? 0 : (c >= max ? max - 1 : c);
  }

  void buildGrid() {
    uint16_t count[CELLS];
    memset(count, 0, sizeof(count));
    for (int i = 0; i < n; ++i) {
      int c = cellCoord(py[i], GH) * GW + cellCoord(px[i], GW);
      cellOf_[i] = (uint16_t)c;
      count[c]++;
    }
    uint16_t acc = 0;
    for (int c = 0; c < CELLS; ++c) { cellStart_[c] = acc; acc += count[c]; count[c] = cellStart_[c]; }
    cellStart_[CELLS] = acc;
    for (int i = 0; i < n; ++i) sorted_[count[cellOf_[i]]++] = (uint16_t)i;
    for (int a = 0; a < n; ++a) { int i = sorted_[a]; spx_[a] = px[i]; spy_[a] = py[i]; }
  }

  // Visit every unordered pair (i<j) closer than RADIUS exactly once.
  // Callback receives (i, j, rx, ry, 1/r, q = r/h) with r = (rx, ry) from i to j.
  template <typename F>
  void forEachPair(F&& f) { forEachPairRows(0, GH, f); }

  template <typename F>
  void forEachPairRows(int r0, int r1, F&& f) {
    const float h2 = RADIUS * RADIUS, invH = 1.0f / RADIUS;
    for (int cy = r0; cy < r1; ++cy) {
      for (int cx = 0; cx < GW; ++cx) {
        int c = cy * GW + cx;
        int s0 = cellStart_[c], s1 = cellStart_[c + 1];
        if (s0 == s1) continue;
        // Half-stencil: self cell plus 4 forward neighbors -> each pair once.
        static const int8_t NX[5] = {0, 1, -1, 0, 1};
        static const int8_t NY[5] = {0, 0,  1, 1, 1};
        for (int k = 0; k < 5; ++k) {
          int nx = cx + NX[k], ny = cy + NY[k];
          if (nx < 0 || nx >= GW || ny >= GH) continue;
          int nc = ny * GW + nx;
          int t0 = cellStart_[nc], t1 = cellStart_[nc + 1];
          for (int a = s0; a < s1; ++a) {
            const float xi = spx_[a], yi = spy_[a];
            int bStart = (k == 0) ? a + 1 : t0;
            for (int b = bStart; b < t1; ++b) {
              float rx = spx_[b] - xi, ry = spy_[b] - yi;
              float r2 = rx * rx + ry * ry;
              if (r2 >= h2 || r2 < 1e-6f) continue;
              float invR = fastInvSqrt(r2);
              f(sorted_[a], sorted_[b], rx, ry, invR, r2 * invR * invH);
            }
          }
        }
      }
    }
  }
};

} // namespace fluid
