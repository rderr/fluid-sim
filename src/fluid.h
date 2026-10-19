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
constexpr int MAX_P = 512;

constexpr float RADIUS = 8.0f;     // interaction radius h (px)
constexpr int   CELL = 8;          // spatial hash cell size, must be >= RADIUS
constexpr int   GW = W / CELL;
constexpr int   GH = H / CELL;
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

struct Params {
  float restDensity   = 1.8f;     // rho0 (sum of (1-q)^2 over neighbours)
  float stiffness     = 6000.f;   // k
  float nearStiffness = 12000.f;  // k_near (anti-clustering)
  float viscSigma     = 3.0f;     // linear viscosity: damps rest jitter and small sloshes
  float viscBeta      = 0.3f;     // quadratic viscosity: tames hard impacts
  float gravity       = 500.f;    // px/s^2 per 1 g (lower = behaves like a bigger tank)
  float maxSpeed      = 0.9f * CELL * 120.f; // px/s; cap so particles never skip a cell
};

class Sim {
public:
  int n = 0;
  float px[MAX_P], py[MAX_P];
  float vx[MAX_P], vy[MAX_P];
  Params p;

  // Spawn `count` particles on a lattice filling the lower `fill` fraction of the box.
  void reset(int count, float spacing = 4.4f, float fill = 0.5f) {
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

    // 2. viscosity impulses (uses last frame's neighbourhood)
    buildGrid();
    if (p.viscSigma > 0.f || p.viscBeta > 0.f) {
      forEachPair([&](int i, int j, float rx, float ry, float invR, float q) {
        float nx = rx * invR, ny = ry * invR;
        float u = (vx[i] - vx[j]) * nx + (vy[i] - vy[j]) * ny;
        if (u > 0.f) {
          float I = dt * (1.f - q) * (p.viscSigma * u + p.viscBeta * u * u) * 0.5f;
          if (I > u * 0.5f) I = u * 0.5f;   // never reverse the relative velocity
          vx[i] -= I * nx; vy[i] -= I * ny;
          vx[j] += I * nx; vy[j] += I * ny;
        }
      });
    }

    // 3. clamp speed, save old positions, advect
    const float vmax2 = p.maxSpeed * p.maxSpeed;
    for (int i = 0; i < n; ++i) {
      float s2 = vx[i] * vx[i] + vy[i] * vy[i];
      if (s2 > vmax2) { float s = p.maxSpeed * fastInvSqrt(s2); vx[i] *= s; vy[i] *= s; }
      ox_[i] = px[i]; oy_[i] = py[i];
      px[i] += vx[i] * dt; py[i] += vy[i] * dt;
    }

    // 4. double density relaxation (Jacobi form: each pair visited once)
    buildGrid();
    for (int i = 0; i < n; ++i) { rho_[i] = 0.f; rhoN_[i] = 0.f; }
    forEachPair([&](int i, int j, float, float, float, float q) {
      float a = 1.f - q, a2 = a * a, a3 = a2 * a;
      rho_[i] += a2; rhoN_[i] += a3;
      rho_[j] += a2; rhoN_[j] += a3;
    });
    for (int i = 0; i < n; ++i) {
      pr_[i]  = p.stiffness * (rho_[i] - p.restDensity);
      prN_[i] = p.nearStiffness * rhoN_[i];
      dx_[i] = dy_[i] = 0.f;
    }
    const float dt2 = dt * dt;
    forEachPair([&](int i, int j, float rx, float ry, float invR, float q) {
      float a = 1.f - q;
      float D = dt2 * ((pr_[i] + pr_[j]) * a + (prN_[i] + prN_[j]) * a * a) * 0.5f;
      float nx = rx * invR, ny = ry * invR;   // unit vector from i to j
      dx_[i] -= D * nx; dy_[i] -= D * ny;
      dx_[j] += D * nx; dy_[j] += D * ny;
    });
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
  }

  // Average neighbour density (diagnostic).
  float meanDensity() const {
    float s = 0; for (int i = 0; i < n; ++i) s += rho_[i];
    return n ? s / n : 0.f;
  }

private:
  float ox_[MAX_P], oy_[MAX_P];
  float rho_[MAX_P], rhoN_[MAX_P], pr_[MAX_P], prN_[MAX_P];
  float dx_[MAX_P], dy_[MAX_P];
  uint16_t cellStart_[CELLS + 1];
  uint16_t sorted_[MAX_P];
  uint8_t  cellOf_[MAX_P];

  static inline int cellCoord(float v, int max) {
    int c = (int)(v * (1.0f / CELL));
    return c < 0 ? 0 : (c >= max ? max - 1 : c);
  }

  void buildGrid() {
    uint16_t count[CELLS];
    memset(count, 0, sizeof(count));
    for (int i = 0; i < n; ++i) {
      int c = cellCoord(py[i], GH) * GW + cellCoord(px[i], GW);
      cellOf_[i] = (uint8_t)c;
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
