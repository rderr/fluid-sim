// Renders the particle field into a 128x128 RGB565 framebuffer.
// Particles are splatted into an 8-bit density field, which is then mapped
// through a colour LUT: this gives a metaball-style "liquid" look with a
// bright rim at the surface. Pure C++, no Arduino deps.
#pragma once
#include <stdint.h>
#include <string.h>
#include "fluid.h"

namespace render {

constexpr int W = fluid::W;
constexpr int H = fluid::H;
constexpr int KR = 3;                 // splat kernel radius (px)
constexpr int KS = 2 * KR + 1;
constexpr float KAMP = 60.f;          // splat kernel peak weight

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
// Byte-swapped so the buffer can be streamed to the panel as-is (big-endian).
static inline uint16_t swap16(uint16_t v) { return (uint16_t)((v << 8) | (v >> 8)); }

struct RGB { uint8_t r, g, b; };

struct Theme {
  RGB bg, deep, body, rim;          // background, dense interior, body, surface rim
  uint8_t thrRim, thrBody, thrDeep; // density thresholds
  Theme() : Theme({0, 0, 0}, {10, 40, 150}, {20, 90, 220}, {120, 220, 255}, 35, 75, 200) {}
  Theme(RGB bg_, RGB deep_, RGB body_, RGB rim_, uint8_t tr, uint8_t tb, uint8_t td)
    : bg(bg_), deep(deep_), body(body_), rim(rim_), thrRim(tr), thrBody(tb), thrDeep(td) {}
};

class Renderer {
public:
  uint16_t fbs[2][W * H];    // RGB565, byte-swapped for SPI; double-buffered
  uint16_t* fb = fbs[0];     // buffer holding the most recently drawn frame
  uint8_t  field[W * H];
  uint8_t  tfield[W * H];    // temperature * kernel weight, accumulated (heat mode);
                             // fits a byte because each term is <= its weight
  // Palette stretch: temperatures from heatLo..heatHi span the full thermal
  // palette (the pool rarely reaches 0 or 1).
  float heatLo = 0.3f, heatHi = 0.85f;

  Renderer() { setTheme(Theme()); buildKernel(); buildHeatLut(); }

  void setTheme(const Theme& t) {
    thrRim_ = t.thrRim;
    bg_ = swap16(rgb565(t.bg.r, t.bg.g, t.bg.b));
    for (int d = 0; d < 256; ++d) {
      uint8_t r, g, b;
      if (d < t.thrRim) {
        r = t.bg.r; g = t.bg.g; b = t.bg.b;   // solid background below the rim threshold
      } else if (d < t.thrBody) {
        float f = (float)(d - t.thrRim) / (t.thrBody - t.thrRim);
        r = lerp(t.rim.r, t.body.r, f);
        g = lerp(t.rim.g, t.body.g, f);
        b = lerp(t.rim.b, t.body.b, f);
      } else {
        float f = (float)(d - t.thrBody) / (t.thrDeep - t.thrBody);
        if (f > 1.f) f = 1.f;
        r = lerp(t.body.r, t.deep.r, f);
        g = lerp(t.body.g, t.deep.g, f);
        b = lerp(t.body.b, t.deep.b, f);
      }
      lut_[d] = swap16(rgb565(r, g, b));
    }
  }

  void draw(const fluid::Sim& s) { draw(s.px, s.py, s.n, s.heat ? s.temp : nullptr); }

  // temps may be null (plain density rendering).
  void draw(const float* xs, const float* ys, int n, const float* temps) {
    fb = (fb == fbs[0]) ? fbs[1] : fbs[0];   // draw into the buffer not being sent
    memset(field, 0, sizeof(field));
    if (!temps) {
      for (int i = 0; i < n; ++i) splat(xs[i], ys[i]);
      const uint8_t* f = field;
      uint16_t* o = fb;
      for (int i = 0; i < W * H; ++i) o[i] = lut_[f[i]];
      return;
    }
    memset(tfield, 0, sizeof(tfield));
    for (int i = 0; i < n; ++i) splatT(xs[i], ys[i], temps[i]);
    const uint8_t* f = field; const uint8_t* tf = tfield;
    uint16_t* o = fb;
    for (int i = 0; i < W * H; ++i) {
      int d = f[i];
      if (d < thrRim_) { o[i] = bg_; continue; }
      int t = (tf[i] << 8) / d;                  // weighted mean temperature, 0..255
      if (t > 255) t = 255;
      o[i] = heatLut_[t];
    }
  }

private:
  uint16_t lut_[256];
  uint16_t heatLut_[256];
  uint16_t bg_ = 0;
  uint8_t  thrRim_ = 35;
  uint8_t  kernel_[KS * KS];

  // Thermal palette: deep blue -> purple -> red -> orange -> yellow -> white.
  void buildHeatLut() {
    static const uint8_t stops[6][3] = { {10,20,120}, {110,20,160}, {220,30,30}, {255,140,0}, {255,230,60}, {255,255,255} };
    for (int i = 0; i < 256; ++i) {
      float f = i / 255.f * 5.f;
      int k = (int)f; if (k > 4) k = 4;
      float u = f - k;
      uint8_t r = lerp(stops[k][0], stops[k + 1][0], u);
      uint8_t g = lerp(stops[k][1], stops[k + 1][1], u);
      uint8_t b = lerp(stops[k][2], stops[k + 1][2], u);
      heatLut_[i] = swap16(rgb565(r, g, b));
    }
  }

  inline void splatT(float fx, float fy, float temp) {
    int cx = (int)(fx + 0.5f), cy = (int)(fy + 0.5f);
    int x0 = cx - KR, y0 = cy - KR;
    int t8 = (int)((temp - heatLo) / (heatHi - heatLo) * 255.f + 0.5f);
    if (t8 < 0) t8 = 0; if (t8 > 255) t8 = 255;
    for (int ky = 0; ky < KS; ++ky) {
      int y = y0 + ky;
      if (y < 0 || y >= H) continue;
      uint8_t* row = field + y * W; uint8_t* trow = tfield + y * W;
      const uint8_t* krow = kernel_ + ky * KS;
      for (int kx = 0; kx < KS; ++kx) {
        int x = x0 + kx;
        if (x < 0 || x >= W) continue;
        int w = krow[kx];
        int v = row[x] + w;
        if (v > 255) { w -= v - 255; v = 255; }   // keep the two fields consistent
        row[x] = (uint8_t)v;
        trow[x] += (uint8_t)((w * t8) >> 8);
      }
    }
  }

  static inline uint8_t lerp(uint8_t a, uint8_t b, float f) {
    return (uint8_t)(a + (b - a) * f + 0.5f);
  }

  void buildKernel() {
    for (int y = 0; y < KS; ++y)
      for (int x = 0; x < KS; ++x) {
        float dx = x - KR, dy = y - KR;
        float d = sqrtf(dx * dx + dy * dy) / (KR + 0.5f);
        float w = d >= 1.f ? 0.f : (1.f - d * d);
        w = w * w;                       // smooth falloff
        kernel_[y * KS + x] = (uint8_t)(w * KAMP);
      }
  }

  inline void splat(float fx, float fy) {
    int cx = (int)(fx + 0.5f), cy = (int)(fy + 0.5f);
    int x0 = cx - KR, y0 = cy - KR;
    for (int ky = 0; ky < KS; ++ky) {
      int y = y0 + ky;
      if (y < 0 || y >= H) continue;
      uint8_t* row = field + y * W;
      const uint8_t* krow = kernel_ + ky * KS;
      for (int kx = 0; kx < KS; ++kx) {
        int x = x0 + kx;
        if (x < 0 || x >= W) continue;
        int v = row[x] + krow[kx];
        row[x] = v > 255 ? 255 : (uint8_t)v;
      }
    }
  }
};

} // namespace render
