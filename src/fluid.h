// Particle fluid simulation (Clavet et al. 2005, "Particle-based Viscoelastic
// Fluid Simulation": double density relaxation). Pure C++, no Arduino deps,
// so it can be unit-tested on the desktop (see tools/sim_test.cpp).
//
// Units: pixels and seconds. The domain is W x H pixels.
#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>

namespace fluid {

constexpr int W = 128;
constexpr int H = 128;
constexpr int MAX_P = 800;

constexpr float RADIUS = 6.0f;     // interaction radius h (px)
constexpr int   CELL = 6;          // spatial hash cell size, must be >= RADIUS
constexpr int   GW = (W + CELL - 1) / CELL;
constexpr int   GH = (H + CELL - 1) / CELL;
constexpr int   CELLS = GW * GH;
constexpr float WALL = 1.5f;       // wall margin (px)

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
  // Cohesion / surface tension. Sum of (1-q)^2 over neighbours at rest.
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
  float stiffness     = 1000.f;

  // Near-pressure (anti-clustering). Keeps particles from stacking on top
  // of each other; usually kept at ~2x stiffness.
  //   up:   more even spacing, smoother interior, slightly more jitter.
  //   down: particles bunch into dense knots and holes open in the body.
  float nearStiffness = 2000.f;

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

  // Wall ghost density. The wall counts as a layer of neighbours so particles
  // touching it feel normal pressure instead of the negative (cohesive)
  // pressure that makes a film cling to the wall.
  //   up:   walls shed liquid cleanly, no stuck lines; too high and the
  //         liquid hovers a pixel or two off the walls.
  //   down: liquid wets and clings to the walls (0 = old behaviour).
  float wallDensity   = 1.0f;

  // Speed cap, px/s. Safety limit so no particle skips a grid cell in one
  // step; it should rarely be reached. Lower only if shakes look wrong.
  float maxSpeed      = 0.9f * CELL * 60.f;
};

class Sim {
public:
  int n = 0;
  float px[MAX_P], py[MAX_P];
  float vx[MAX_P], vy[MAX_P];
  Params p;

  // Spawn `count` particles on a lattice filling the lower `fill` fraction of the box.
  void reset(int count, float spacing = 3.3f, float fill = 0.5f) {
    if (count > MAX_P) count = MAX_P;
    n = 0;
    int cols = (int)((W - 2 * WALL) / spacing);
    float x0 = (W - cols * spacing) * 0.5f + spacing * 0.5f;
    float y = H - WALL - spacing * 0.5f;
    float yMin = H * (1.0f - fill);
    while (n < count && y > yMin) {
      for (int c = 0; c < cols && n < count; ++c) {
        px[n] = x0 + c * spacing + ((int)(y / spacing) & 1 ? spacing * 0.25f : 0.f);
        py[n] = y;
        vx[n] = vy[n] = 0.f;
        ++n;
      }
      y -= spacing * 0.87f;
    }
  }

  // One simulation step. (gx, gy) is the effective gravity in g units
  // (screen coordinates: +x right, +y down).
  void step(float dt, float gx, float gy) {
    const float h = RADIUS;

    // 1. gravity
    const float ax = gx * p.gravity * dt, ay = gy * p.gravity * dt;
    for (int i = 0; i < n; ++i) { vx[i] += ax; vy[i] += ay; }

    // 2. viscosity impulses, using the pair list found in the previous step
    //    (positions have moved by at most a fraction of a pixel since)
    if (p.viscSigma > 0.f || p.viscBeta > 0.f) {
      for (int k = 0; k < nPairs_; ++k) {
        const Pair& pr = pairs_[k];
        int i = pr.i, j = pr.j;
        float u = (vx[i] - vx[j]) * pr.nx + (vy[i] - vy[j]) * pr.ny;
        if (u > 0.f && pr.q < 1.f) {
          float I = dt * (1.f - pr.q) * (p.viscSigma * u + p.viscBeta * u * u) * 0.5f;
          if (I > u * 0.5f) I = u * 0.5f;   // never reverse the relative velocity
          vx[i] -= I * pr.nx; vy[i] -= I * pr.ny;
          vx[j] += I * pr.nx; vy[j] += I * pr.ny;
        }
      }
    }

    // 3. clamp speed, save old positions, advect
    const float vmax2 = p.maxSpeed * p.maxSpeed;
    for (int i = 0; i < n; ++i) {
      float s2 = vx[i] * vx[i] + vy[i] * vy[i];
      if (s2 > vmax2) { float s = p.maxSpeed * fastInvSqrt(s2); vx[i] *= s; vy[i] *= s; }
      ox_[i] = px[i]; oy_[i] = py[i];
      px[i] += vx[i] * dt; py[i] += vy[i] * dt;
    }

    // 4. find neighbour pairs once, then double density relaxation
    buildGrid();
    findPairs();
    for (int i = 0; i < n; ++i) { rho_[i] = 0.f; rhoN_[i] = 0.f; }
    for (int k = 0; k < nPairs_; ++k) {
      const Pair& pr = pairs_[k];
      float a = 1.f - pr.q, a2 = a * a, a3 = a2 * a;
      rho_[pr.i] += a2; rhoN_[pr.i] += a3;
      rho_[pr.j] += a2; rhoN_[pr.j] += a3;
    }
    const float invH = 1.0f / h;
    if (p.wallDensity > 0.f) {
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
    if (p.wallDensity > 0.f) {
      // the wall pushes back like an immovable neighbour
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
    for (int k = 0; k < nPairs_; ++k) {
      const Pair& pr = pairs_[k];
      int i = pr.i, j = pr.j;
      float a = 1.f - pr.q;
      float D = dt2 * ((pr_[i] + pr_[j]) * a + (prN_[i] + prN_[j]) * a * a) * 0.5f;
      float Dx = D * pr.nx, Dy = D * pr.ny;
      dx_[i] -= Dx; dy_[i] -= Dy;
      dx_[j] += Dx; dy_[j] += Dy;
    }
    const float maxD = 0.5f * h, maxD2 = maxD * maxD;
    for (int i = 0; i < n; ++i) {
      float d2 = dx_[i] * dx_[i] + dy_[i] * dy_[i];
      if (d2 > maxD2) { float s = maxD * fastInvSqrt(d2); dx_[i] *= s; dy_[i] *= s; }
      px[i] += dx_[i]; py[i] += dy_[i];
    }

    // 5. walls + 6. velocity from positions
    const float invDt = 1.0f / dt;
    for (int i = 0; i < n; ++i) {
      if (!(px[i] == px[i]) || !(py[i] == py[i])) {   // NaN guard: respawn at the centre
        px[i] = W * 0.5f; py[i] = H * 0.5f; ox_[i] = px[i]; oy_[i] = py[i];
      }
      if (px[i] < WALL) px[i] = WALL; else if (px[i] > W - WALL) px[i] = W - WALL;
      if (py[i] < WALL) py[i] = WALL; else if (py[i] > H - WALL) py[i] = H - WALL;
      vx[i] = (px[i] - ox_[i]) * invDt;
      vy[i] = (py[i] - oy_[i]) * invDt;
    }

    // 7. refresh pair geometry from the final positions so the next step's
    //    viscosity pass sees current directions and distances
    refreshPairs();
  }

  int pairCount() const { return nPairs_; }

  // Average neighbour density (diagnostic).
  float meanDensity() const {
    float s = 0; for (int i = 0; i < n; ++i) s += rho_[i];
    return n ? s / n : 0.f;
  }

private:
  float ox_[MAX_P], oy_[MAX_P];
  float rho_[MAX_P], rhoN_[MAX_P], pr_[MAX_P], prN_[MAX_P];
  float dx_[MAX_P], dy_[MAX_P];

  // Neighbour pair cache: unit vector from i to j and q = r/h.
  struct Pair { uint16_t i, j; float nx, ny, q; };
  static constexpr int MAX_PAIRS = 6144;
  Pair pairs_[MAX_PAIRS];
  int  nPairs_ = 0;

  void refreshPairs() {
    const float invH = 1.0f / RADIUS;
    for (int k = 0; k < nPairs_; ++k) {
      Pair& pr = pairs_[k];
      float rx = px[pr.j] - px[pr.i], ry = py[pr.j] - py[pr.i];
      float r2 = rx * rx + ry * ry;
      if (r2 < 1e-6f) { pr.q = 1.f; continue; }      // coincident: skip in viscosity
      float invR = fastInvSqrt(r2);
      pr.nx = rx * invR; pr.ny = ry * invR; pr.q = r2 * invR * invH;
    }
  }

  void findPairs() {
    nPairs_ = 0;
    forEachPair([&](int i, int j, float rx, float ry, float invR, float q) {
      if (nPairs_ >= MAX_PAIRS) return;      // degrade gracefully when over-packed
      Pair& pr = pairs_[nPairs_++];
      pr.i = (uint16_t)i; pr.j = (uint16_t)j;
      pr.nx = rx * invR; pr.ny = ry * invR; pr.q = q;
    });
  }
  uint16_t cellStart_[CELLS + 1];
  uint16_t sorted_[MAX_P];
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
  }

  // Visit every unordered pair (i<j) closer than RADIUS exactly once.
  // Callback receives (i, j, rx, ry, 1/r, q = r/h) with r = (rx, ry) from i to j.
  template <typename F>
  void forEachPair(F&& f) {
    const float h2 = RADIUS * RADIUS, invH = 1.0f / RADIUS;
    for (int cy = 0; cy < GH; ++cy) {
      for (int cx = 0; cx < GW; ++cx) {
        int c = cy * GW + cx;
        int s0 = cellStart_[c], s1 = cellStart_[c + 1];
        if (s0 == s1) continue;
        // Half-stencil: self cell plus 4 forward neighbours -> each pair once.
        static const int8_t NX[5] = {0, 1, -1, 0, 1};
        static const int8_t NY[5] = {0, 0,  1, 1, 1};
        for (int k = 0; k < 5; ++k) {
          int nx = cx + NX[k], ny = cy + NY[k];
          if (nx < 0 || nx >= GW || ny >= GH) continue;
          int nc = ny * GW + nx;
          int t0 = cellStart_[nc], t1 = cellStart_[nc + 1];
          for (int a = s0; a < s1; ++a) {
            int i = sorted_[a];
            int bStart = (k == 0) ? a + 1 : t0;
            for (int b = bStart; b < t1; ++b) {
              int j = sorted_[b];
              float rx = px[j] - px[i], ry = py[j] - py[i];
              float r2 = rx * rx + ry * ry;
              if (r2 >= h2 || r2 < 1e-6f) continue;
              float invR = fastInvSqrt(r2);
              f(i, j, rx, ry, invR, r2 * invR * invH);
            }
          }
        }
      }
    }
  }
};

} // namespace fluid
