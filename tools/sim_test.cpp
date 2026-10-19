// Desktop harness: runs the simulation with a scripted tilt sequence, checks
// stability, times it, and writes BMP frames for eyeballing.
//   c++ -O2 -std=c++17 -I src tools/sim_test.cpp -o /tmp/sim_test && /tmp/sim_test out_dir
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <string>
#include "fluid.h"
#include "render.h"

static void writeBmp(const char* path, const uint16_t* fb) {
  const int W = render::W, H = render::H;
  FILE* f = fopen(path, "wb");
  if (!f) return;
  int rowBytes = (W * 3 + 3) & ~3;
  int dataSize = rowBytes * H;
  uint8_t hdr[54] = {0};
  hdr[0] = 'B'; hdr[1] = 'M';
  uint32_t fileSize = 54 + dataSize;
  memcpy(hdr + 2, &fileSize, 4);
  uint32_t off = 54; memcpy(hdr + 10, &off, 4);
  uint32_t dib = 40; memcpy(hdr + 14, &dib, 4);
  int32_t w = W, h = H; memcpy(hdr + 18, &w, 4); memcpy(hdr + 22, &h, 4);
  uint16_t planes = 1, bpp = 24; memcpy(hdr + 26, &planes, 2); memcpy(hdr + 28, &bpp, 2);
  memcpy(hdr + 34, &dataSize, 4);
  fwrite(hdr, 1, 54, f);
  std::string row(rowBytes, '\0');
  for (int y = H - 1; y >= 0; --y) {
    for (int x = 0; x < W; ++x) {
      uint16_t v = render::swap16(fb[y * W + x]);
      uint8_t r = (v >> 11) << 3, g = ((v >> 5) & 0x3F) << 2, b = (v & 0x1F) << 3;
      row[x * 3 + 0] = b; row[x * 3 + 1] = g; row[x * 3 + 2] = r;
    }
    fwrite(row.data(), 1, rowBytes, f);
  }
  fclose(f);
}

int main(int argc, char** argv) {
  const char* out = argc > 1 ? argv[1] : ".";
  int count = argc > 2 ? atoi(argv[2]) : 400;
  static fluid::Sim sim;
  static render::Renderer rend;
  sim.reset(count);
  auto envf = [](const char* k, float d) { const char* v = getenv(k); return v ? (float)atof(v) : d; };
  sim.p.restDensity   = envf("RHO0", sim.p.restDensity);
  sim.p.stiffness     = envf("K", sim.p.stiffness);
  sim.p.nearStiffness = envf("KN", sim.p.nearStiffness);
  sim.p.viscBeta      = envf("BETA", sim.p.viscBeta);
  sim.p.viscSigma     = envf("SIGMA", sim.p.viscSigma);
  sim.p.gravity       = envf("GRAV", sim.p.gravity);
  sim.p.wallDensity   = envf("WALLD", sim.p.wallDensity);
  float tiltG         = envf("TILT", 0.8f);
  int substeps        = (int)envf("SUBSTEPS", 2);
  bool hold           = envf("HOLD", 0) > 0;   // HOLD=1: constant gentle tilt the whole run
  bool slosh          = envf("SLOSH", 0) > 0;
  sim.heat            = envf("HEAT", 0) > 0;   // HEAT=1: convection on
  sim.p.heatRate      = envf("HEATR", sim.p.heatRate);
  sim.p.coolRate      = envf("COOLR", sim.p.coolRate);
  sim.p.diffusion     = envf("DIFF", sim.p.diffusion);
  sim.p.buoyancy      = envf("BUOY", sim.p.buoyancy);
  sim.p.coolBulk      = envf("COOLB", sim.p.coolBulk);  // SLOSH=1: hard tilt left 2-3 s, then slight right tilt
  float fps = envf("FPS", 60.f);            // frame rate the substeps are sized for
  const float dt = 1.f / (fps * substeps);
  render::Theme th;
  th.thrRim  = (uint8_t)envf("THR_RIM", th.thrRim);
  th.thrBody = (uint8_t)envf("THR_BODY", th.thrBody);
  th.thrDeep = (uint8_t)envf("THR_DEEP", th.thrDeep);
  rend.setTheme(th);
  printf("particles: %d rho0=%.2f k=%.0f kn=%.0f beta=%.2f\n", sim.n, sim.p.restDensity, sim.p.stiffness, sim.p.nearStiffness, sim.p.viscBeta);

  const int frames = 600;               // 10 s at 60 fps
  double simTime = 0, renderTime = 0;
  int bad = 0;
  for (int fr = 0; fr < frames; ++fr) {
    // scripted tilt: rest, tilt right, tilt up (upside down), shake, rest
    float gx = 0, gy = 1;
    float t = fr / 60.f;
    if (slosh)               { gx = (t > 2 && t < 3) ? -0.7f : 0.1f; gy = sqrtf(1.f - gx * gx); }
    else if (hold)           { gx = tiltG; gy = sqrtf(1.f - tiltG * tiltG); }
    else if (t > 2 && t < 4) { gx = tiltG; gy = sqrtf(1.f - tiltG * tiltG); }
    else if (!hold && !slosh && t > 4 && t < 6) { gx = 0; gy = -1; }
    else if (!hold && !slosh && t > 6 && t < 8) { gx = 3.f * sinf(t * 40.f); gy = 1; }

    auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < substeps; ++k) sim.step(dt, gx, gy);
    auto t1 = std::chrono::steady_clock::now();
    rend.draw(sim);
    auto t2 = std::chrono::steady_clock::now();
    simTime += std::chrono::duration<double>(t1 - t0).count();
    renderTime += std::chrono::duration<double>(t2 - t1).count();

    float maxV = 0;
    for (int i = 0; i < sim.n; ++i) {
      if (!std::isfinite(sim.px[i]) || !std::isfinite(sim.py[i])) bad++;
      if (sim.px[i] < 0 || sim.px[i] > fluid::W || sim.py[i] < 0 || sim.py[i] > fluid::H) bad++;
      float v = sqrtf(sim.vx[i] * sim.vx[i] + sim.vy[i] * sim.vy[i]);
      if (v > maxV) maxV = v;
    }
    if (fr % 30 == 0 || fr == frames - 1) {
      // fluid height estimate: fraction of particles in lower half
      int lower = 0, leftWall = 0;
      for (int i = 0; i < sim.n; ++i) { if (sim.py[i] > fluid::H / 2) lower++; if (sim.px[i] < fluid::WALL + 0.5f) leftWall++; }
      float sumV = 0, tmin = 1, tmax = 0;
      for (int i = 0; i < sim.n; ++i) { sumV += sqrtf(sim.vx[i]*sim.vx[i] + sim.vy[i]*sim.vy[i]); if (sim.temp[i] < tmin) tmin = sim.temp[i]; if (sim.temp[i] > tmax) tmax = sim.temp[i]; }
      printf("t=%5.2f g=(%5.2f,%5.2f) meanRho=%.2f maxV=%7.1f meanV=%6.1f lowerHalf=%3d%% leftWall=%3d pairs=%d T=%.2f [%.2f..%.2f]\n",
             t, gx, gy, sim.meanDensity(), maxV, sumV / sim.n, lower * 100 / sim.n, leftWall, sim.pairCount(), sim.meanTemp(), tmin, tmax);
      char path[512]; snprintf(path, sizeof path, "%s/frame_%03d.bmp", out, fr);
      writeBmp(path, rend.fb);
    }
  }
  printf("bad samples: %d\n", bad);
  printf("avg per frame: sim(%d substeps) %.3f ms, render %.3f ms (desktop)\n", substeps,
         simTime / frames * 1e3, renderTime / frames * 1e3);
  return bad ? 1 : 0;
}
