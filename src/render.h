// Renders the particle field into RGB565 display bands.
// Particles are splatted into an 8-bit density field, which is then mapped
// through a color LUT: this gives a metaball-style "liquid" look with a
// bright rim at the surface. The field is built one horizontal band at a
// time and handed to a sink per band, so no full framebuffer is needed.
// With DISPLAY_SCALE 2 the field is bilinearly upsampled on the way out.
// Pure C++, no Arduino deps.
#pragma once
#include <stdint.h>
#include <string.h>
#include "board.h"
#include "fluid.h"

namespace render {

constexpr int W = fluid::W;                 // simulation (field) size
constexpr int H = fluid::H;
constexpr int SCALE = DISPLAY_SCALE;
constexpr int OUT_W = W * SCALE;            // display size
constexpr int OUT_H = H * SCALE;
constexpr int BAND = RENDER_BAND;           // field rows per band
static_assert(H % BAND == 0, "RENDER_BAND must divide FLUID_H");
constexpr int OUT_BAND = BAND * SCALE;      // display rows per band
constexpr int KR = 3;                       // splat kernel radius (px)
constexpr int KS = 2 * KR + 1;
constexpr float KAMP = 60.f;                // splat kernel peak weight

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

// Color themes. Index 0 is used for the thermal mode's background.
static const Theme THEMES[] = {
  // bg            deep            body            rim             thresholds
  Theme({  0,  0,  0}, { 10, 40,150}, { 20, 90,220}, {120,220,255}, 35, 75, 200),   // water
  Theme({  0,  0,  0}, {150, 20,  0}, {240,110,  0}, {255,235,120}, 35, 75, 200),   // lava
  Theme({  0,  0,  0}, { 10,120, 40}, { 40,220, 90}, {200,255,200}, 35, 75, 200),   // slime
  Theme({  0,  0,  0}, { 90, 90,110}, {170,170,190}, {255,255,255}, 35, 75, 200),   // mercury
};
static const char* const THEME_NAMES[] = { "water", "lava", "slime", "mercury" };
constexpr int THEME_COUNT = sizeof(THEMES) / sizeof(THEMES[0]);

// Receives finished display bands: y0 and rows in display pixels, buf is
// RGB565 byte-swapped, OUT_W wide. The renderer alternates two buffers, so a
// sink that streams asynchronously must finish buf before the band after next.
typedef void (*BandSink)(int y0, int rows, const uint16_t* buf, void* ctx);

class Renderer {
public:
  // Two band buffers so one can stream while the next is drawn.
  uint16_t bands[2][OUT_W * OUT_BAND];
  // Field rows for one band plus one extra row for the bilinear upsample.
  uint8_t  field[(BAND + 1) * W];
  uint8_t  tfield[(BAND + 1) * W];   // temperature * kernel weight (heat mode);
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

  void draw(const fluid::Sim& s, BandSink sink, void* ctx) {
    draw(s.px, s.py, s.n, s.heat ? s.temp : nullptr, sink, ctx);
  }

  // temps may be null (plain density rendering). Calls sink once per band.
  void draw(const float* xs, const float* ys, int n, const float* temps, BandSink sink, void* ctx) {
    int which = 0;
    for (int y0 = 0; y0 < H; y0 += BAND) {
      memset(field, 0, sizeof(field));
      if (temps) memset(tfield, 0, sizeof(tfield));
      const float lo = y0 - KR - 1.f, hi = y0 + BAND + KR + 2.f;
      for (int i = 0; i < n; ++i) {
        float y = ys[i];
        if (y < lo || y > hi) continue;        // does not touch this band
        if (temps) splatT(xs[i], y - y0, temps[i]); else splat(xs[i], y - y0);
      }
      uint16_t* out = bands[which];
      which ^= 1;
      if (SCALE == 1) {
        if (!temps) {
          for (int i = 0; i < W * BAND; ++i) out[i] = lut_[field[i]];
        } else {
          for (int i = 0; i < W * BAND; ++i) out[i] = heatPixel(field[i], tfield[i]);
        }
      } else {
        upsample(out, temps != nullptr);
      }
      sink(y0 * SCALE, OUT_BAND, out, ctx);
    }
  }

private:
  uint16_t lut_[256];
  uint16_t heatLut_[256];
  uint16_t bg_ = 0;
  uint8_t  thrRim_ = 35;
  uint8_t  kernel_[KS * KS];

  static inline uint8_t lerp(uint8_t a, uint8_t b, float f) {
    return (uint8_t)(a + (b - a) * f + 0.5f);
  }

  inline uint16_t heatPixel(int d, int tf) const {
    if (d < thrRim_) return bg_;
    int t = (tf << 8) / d;                   // weighted mean temperature, 0..255
    if (t > 255) t = 255;
    return heatLut_[t];
  }

  // Bilinear upsample of the band's field (and temperature field) by SCALE.
  // Output row oy samples field row (oy + 0.5) / SCALE - 0.5; the band's extra
  // field row makes the last output rows interpolate correctly.
  void upsample(uint16_t* out, bool heat) {
    for (int oy = 0; oy < OUT_BAND; ++oy) {
      float sy = (oy + 0.5f) / SCALE - 0.5f;
      int y0 = (int)sy; if (sy < 0) { y0 = 0; sy = 0; }
      int y1 = y0 + 1; if (y1 > BAND) y1 = BAND;
      int fy = (int)((sy - y0) * 256.f);
      const uint8_t* r0 = field + y0 * W; const uint8_t* r1 = field + y1 * W;
      const uint8_t* t0 = tfield + y0 * W; const uint8_t* t1 = tfield + y1 * W;
      uint16_t* o = out + oy * OUT_W;
      for (int ox = 0; ox < OUT_W; ++ox) {
        float sx = (ox + 0.5f) / SCALE - 0.5f;
        int x0 = (int)sx; if (sx < 0) { x0 = 0; sx = 0; }
        int x1 = x0 + 1; if (x1 >= W) x1 = W - 1;
        int fx = (int)((sx - x0) * 256.f);
        int top = r0[x0] * (256 - fx) + r0[x1] * fx;
        int bot = r1[x0] * (256 - fx) + r1[x1] * fx;
        int d = (top * (256 - fy) + bot * fy) >> 16;
        if (!heat) { o[ox] = lut_[d]; continue; }
        int ttop = t0[x0] * (256 - fx) + t0[x1] * fx;
        int tbot = t1[x0] * (256 - fx) + t1[x1] * fx;
        int tf = (ttop * (256 - fy) + tbot * fy) >> 16;
        o[ox] = heatPixel(d, tf);
      }
    }
  }

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

  inline void splat(float fx, float fy) {       // fy relative to the band
    int cx = (int)(fx + 0.5f), cy = (int)floorf(fy + 0.5f);
    int x0 = cx - KR, y0 = cy - KR;
    for (int ky = 0; ky < KS; ++ky) {
      int y = y0 + ky;
      if (y < 0 || y > BAND) continue;        // band rows plus the extra row
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

  inline void splatT(float fx, float fy, float temp) {   // fy relative to the band
    int cx = (int)(fx + 0.5f), cy = (int)floorf(fy + 0.5f);
    int x0 = cx - KR, y0 = cy - KR;
    int t8 = (int)((temp - heatLo) / (heatHi - heatLo) * 255.f + 0.5f);
    if (t8 < 0) t8 = 0; if (t8 > 255) t8 = 255;
    for (int ky = 0; ky < KS; ++ky) {
      int y = y0 + ky;
      if (y < 0 || y > BAND) continue;
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
};

} // namespace render
