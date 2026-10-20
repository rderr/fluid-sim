// Tilt-reactive fluid simulation for the LOLIN S3 Mini Pro
// (ESP32-S3, 128x128 GC9107 TFT, QMI8658C IMU).
//
// The accelerometer reading is the board's proper acceleration; in the board
// frame the liquid feels an effective gravity equal to minus that vector, so
// tilting AND shaking the board both slosh the fluid correctly.
//
// Controls (LOLIN, three buttons):
//   IO47  cycle the accelerometer-to-screen axis mapping (saved to flash).
//   IO48  short press: respawn. Hold 1.5 s flat and still: calibrate level
//         and gyro zero (saved to flash).
//   IO0   cycle mode: water, lava, slime, mercury themes, then heat convection.
// Controls (Waveshare, one button):
//   BOOT  press: next mode. Hold 1.5 s: calibrate. Hold 4 s: next axis mapping.
// Serial (both): 'm' mode, 'o' orientation, 'r' respawn, 'c' calibrate,
//   'p' toggle per-phase profiling output.
#include <Arduino.h>
#include <Preferences.h>
#include <driver/gpio.h>
#include "board.h"
#include "fluid.h"
#include "render.h"
#include "display.h"
#include "imu.h"

constexpr int BTN_THEME  = PIN_BTN_MODE;
#if HAS_THREE_BUTTONS
constexpr int BTN_ORIENT = PIN_BTN_ORIENT;
constexpr int BTN_RESET  = PIN_BTN_RESET;
#endif

constexpr int   PARTICLES   = 780;
constexpr int   SUBSTEPS    = 2;             // physics substeps per rendered frame
constexpr float SUB_DT      = 1.f / 120.f;   // fixed: the relaxation scheme's stability depends
                                             // on dt, so it must not follow the frame rate
constexpr float ACCEL_ALPHA = 0.2f;          // IMU low-pass (per frame); lower = smoother
constexpr float ACCEL_MAX   = 2.5f;          // clamp shakes to +/-2.5 g

static fluid::Sim      sim;
static render::Renderer rend;
static Preferences     prefs;

// Rendering runs on core 0 from a snapshot of particle positions while the
// physics for the next frame runs on core 1.
static float snapX[fluid::MAX_P], snapY[fluid::MAX_P], snapT[fluid::MAX_P];
static int   snapN = 0;
static bool  snapHeat = false;
static SemaphoreHandle_t renderGo, renderDone;
static uint32_t drawUsAcc = 0;

static uint32_t pushUsAcc = 0, bandsAcc = 0;
static void bandSink(int y0, int rows, const uint16_t* buf, void*) {
  uint32_t t0 = micros();
  display::pushBand(y0, rows, buf);       // waits for the previous band's DMA, queues this one
  pushUsAcc += micros() - t0; bandsAcc++;
}

static void renderTask(void*) {
  for (;;) {
    xSemaphoreTake(renderGo, portMAX_DELAY);
    uint32_t t0 = micros();
    rend.draw(snapX, snapY, snapN, snapHeat ? snapT : nullptr, bandSink, nullptr);
    drawUsAcc += micros() - t0;
    xSemaphoreGive(renderDone);
  }
}

// Second-core helper for the solver's split passes (see fluid::Sim::Helper).
static SemaphoreHandle_t jobGo, jobDone;
static volatile int jobId = 0;
static uint32_t helperUs[5] = {0, 0, 0, 0, 0};   // time spent per job type on core 0
static void helperTask(void*) {
  Serial.printf("helper task on core %d\n", xPortGetCoreID());
  for (;;) {
    xSemaphoreTake(jobGo, portMAX_DELAY);
    uint32_t t = micros();
    sim.runJob(jobId);
    helperUs[jobId] += micros() - t;
    xSemaphoreGive(jobDone);
  }
}
static void helperStart(void*, int job) { jobId = job; xSemaphoreGive(jobGo); }
static void helperWait(void*)           { xSemaphoreTake(jobDone, portMAX_DELAY); }

static void submitFrame() {
  xSemaphoreTake(renderDone, portMAX_DELAY); // previous frame fully drawn and queued
  memcpy(snapX, sim.px, sim.n * sizeof(float));
  memcpy(snapY, sim.py, sim.n * sizeof(float));
  if (sim.heat) memcpy(snapT, sim.temp, sim.n * sizeof(float));
  snapHeat = sim.heat;
  snapN = sim.n;
  xSemaphoreGive(renderGo);
}

static bool  imuOk = false;
static bool  profiling = false;     // 'p' on serial toggles per-phase timing output
static int   orientation = DEFAULT_ORIENTATION;   // 0..7: 4 rotations (0-3) x mirror (4-7)
static int   theme = 0;
static float fAx = 0.f, fAy = 0.f;  // filtered chip-frame accel (g)
static float biasX = 0.f, biasY = 0.f; // accel bias (chip frame, g), subtracted
static float biasW = 0.f;              // gyro z bias (deg/s), subtracted
static float rawW  = 0.f;              // bias-corrected gyro z (deg/s), kept for calibration
static float fW = 0.f, fAlpha = 0.f;   // filtered spin rate (rad/s) and its derivative (rad/s^2)
constexpr float GYRO_ALPHA  = 0.5f;    // gyro low-pass (per frame)
constexpr float ALPHA_ALPHA = 0.3f;    // angular-acceleration low-pass (per frame)

using render::THEMES;
using render::THEME_COUNT;
constexpr int MODE_COUNT  = THEME_COUNT + 1;      // last mode = heat convection
constexpr int MODE_HEAT   = THEME_COUNT;

// Map chip-frame (x, y) to screen-frame (+x right, +y down).
static void mapAxes(int mode, float x, float y, float& ox, float& oy) {
  if (mode & 4) x = -x;                       // mirror
  for (int r = 0; r < (mode & 3); ++r) {      // rotate 90 deg
    float t = x; x = -y; y = t;
  }
  ox = x; oy = y;
}

static void buttonInit(int pin) {
  // Raw IDF call: the Arduino variant maps pin 47 to RGB_BUILTIN and treats
  // it specially, so pinMode/digitalRead cannot be trusted for it.
  gpio_config_t c = {};
  c.pin_bit_mask = 1ULL << pin; c.mode = GPIO_MODE_INPUT;
  c.pull_up_en = GPIO_PULLUP_ENABLE; c.pull_down_en = GPIO_PULLDOWN_DISABLE;
  c.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&c);
}
static inline int buttonRead(int pin) { return gpio_get_level((gpio_num_t)pin); }

struct Button {
  int pin; uint32_t lowSince; bool armed;
  // armed starts false: a press only counts after the button has been seen
  // released once, so a pin that is low at boot cannot fire.
  explicit Button(int p) : pin(p), lowSince(0), armed(false) {}
  // True once per press. The input must read LOW continuously for HOLD_MS
  // before it counts, so glitches from handling the board or coupling from
  // the SPI lines cannot register as presses.
  static constexpr uint32_t HOLD_MS = 50;
  bool pressed() {
    uint32_t t = millis();
    if (buttonRead(pin) == HIGH) { lowSince = 0; armed = true; longFired = longFired2 = false; return false; }
    if (lowSince == 0) { lowSince = t; return false; }
    if (armed && t - lowSince >= HOLD_MS) { armed = false; return true; }
    return false;
  }
  // True once when the button has been held for `ms` (after pressed() fired).
  bool heldFor(uint32_t ms) {
    if (longFired || lowSince == 0 || buttonRead(pin) == HIGH) return false;
    if (millis() - lowSince >= ms) { longFired = true; return true; }
    return false;
  }
  // A second, longer threshold on the same hold.
  bool heldFor2(uint32_t ms) {
    if (longFired2 || lowSince == 0 || buttonRead(pin) == HIGH) return false;
    if (millis() - lowSince >= ms) { longFired2 = true; return true; }
    return false;
  }
  bool longFired = false, longFired2 = false;
};
static Button btnTheme{BTN_THEME};
#if HAS_THREE_BUTTONS
static Button btnOrient{BTN_ORIENT}, btnReset{BTN_RESET};
#endif

static void applyTheme() {
  bool wasHeat = sim.heat;
  sim.heat = (theme == MODE_HEAT);
  if (sim.heat && !wasHeat)
    for (int i = 0; i < sim.n; ++i) sim.temp[i] = sim.p.ambient;
  rend.setTheme(THEMES[sim.heat ? 0 : theme]);
}

void setup() {
#if SERIAL_IS_USB_CDC
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);                 // never block the frame loop on a host that isn't reading
#else
  Serial.setTxBufferSize(1024);             // UART bridge: buffer so stats never block the loop
  Serial.begin(115200);
#endif
  delay(200);                               // let the panel's supply settle after reset
  buttonInit(BTN_THEME);
#if HAS_THREE_BUTTONS
  buttonInit(BTN_ORIENT);
  buttonInit(BTN_RESET);
#endif
  sim.p.displayMm = DISPLAY_MM;
  sim.p.helperShare = HELPER_SHARE;

  prefs.begin("fluid", false);
  orientation = prefs.getInt("orientation2", DEFAULT_ORIENTATION) & 7;
  biasX = prefs.getFloat("biasX", 0.f);
  biasY = prefs.getFloat("biasY", 0.f);
  biasW = prefs.getFloat("biasW", 0.f);
  theme       = prefs.getInt("theme", 0) % MODE_COUNT;

  display::begin(sizeof(rend.bands[0]));
  imuOk = imu::begin();

  sim.reset(PARTICLES);
  applyTheme();
  renderGo   = xSemaphoreCreateBinary();
  renderDone = xSemaphoreCreateBinary();
  xSemaphoreGive(renderDone);
  xTaskCreatePinnedToCore(renderTask, "render", 4096, nullptr, 2, nullptr, 0);
  jobGo   = xSemaphoreCreateBinary();
  jobDone = xSemaphoreCreateBinary();
  xTaskCreatePinnedToCore(helperTask, "simhelper", 4096, nullptr, 3, nullptr, 0);
  sim.helper = { helperStart, helperWait, nullptr };
  sim.clock = []() -> uint32_t { return (uint32_t)micros(); };
  rend.clock = sim.clock;
  submitFrame();
  delay(50);
  display::setBacklight(true);

  Serial.printf("loop on core %d\n", xPortGetCoreID());
  Serial.printf("fluid sim on %s: %d particles, %dx%d%s domain -> %dx%d display, IMU %s (0x%02X), orientation %d (saved %s), bias (%+.3f,%+.3f) g, theme %d\n",
                BOARD_NAME, sim.n, fluid::W, fluid::H, fluid::CIRCLE ? " round" : "", display::W, display::H,
                imuOk ? "ok" : "NOT FOUND - demo mode", imu::address(), orientation,
                prefs.isKey("orientation2") ? "present" : "absent", biasX, biasY, theme);
}

void loop() {
  static uint32_t lastFrame = micros();
  static uint32_t statT = millis();
  static int frames = 0;
  static uint32_t simUs = 0;
  static uint32_t maxFrameUs = 0, maxImuUs = 0, maxSimUs = 0, maxSubmitUs = 0;
  uint32_t frameStart = micros();

  // --- input -------------------------------------------------------------
  // Serial: 'm' mode, 'o' orientation, 'r' respawn, 'c' calibrate, 'p' profiling.
  bool doMode = false, doOrient = false, doRespawn = false, doCalib = false;
  while (Serial.available()) {
    int c = Serial.read();
    if (c == 'm') doMode = true;
    if (c == 'o') doOrient = true;
    if (c == 'r') doRespawn = true;
    if (c == 'c') doCalib = true;
    if (c == 'p') profiling = !profiling;
  }
#if HAS_THREE_BUTTONS
  if (btnOrient.pressed()) doOrient = true;
  if (btnReset.pressed())  doRespawn = true;
  if (btnReset.heldFor(1500)) doCalib = true;
  if (btnTheme.pressed())  doMode = true;
#else
  // One button: press = next mode, hold 1.5 s = calibrate, hold 4 s = next orientation.
  if (btnTheme.pressed())      doMode = true;
  if (btnTheme.heldFor(1500))  doCalib = true;
  if (btnTheme.heldFor2(4000)) doOrient = true;
#endif
  if (doOrient) {
    orientation = (orientation + 1) & 7;
    prefs.putInt("orientation2", orientation);
    Serial.printf("orientation -> %d\n", orientation);
  }
  if (doRespawn) sim.reset(PARTICLES);
  if (doCalib) {
    // Board is assumed flat and still on a level surface: x/y and spin read zero.
    biasX = fAx + biasX; biasY = fAy + biasY;   // fAx/fAy are already bias-corrected
    biasW = rawW + biasW;
    prefs.putFloat("biasX", biasX); prefs.putFloat("biasY", biasY); prefs.putFloat("biasW", biasW);
    Serial.printf("calibrated: bias x=%+.3f y=%+.3f g, gyro z %+.2f dps\n", biasX, biasY, biasW);
    sim.reset(PARTICLES);
  }
  if (doMode) {
    theme = (theme + 1) % MODE_COUNT;
    prefs.putInt("theme", theme);
    applyTheme();
    Serial.printf("mode -> %s\n", theme == MODE_HEAT ? "heat convection" : render::THEME_NAMES[theme]);
  }

  // --- gravity from the IMU ---------------------------------------------
  float gx, gy;
  float ax, ay, az, wx, wy, wz;
  uint32_t imuT0 = micros();
  bool imuRead = imuOk && imu::read(ax, ay, az, wx, wy, wz);
  uint32_t imuUs = micros() - imuT0; if (imuUs > maxImuUs) maxImuUs = imuUs;
  if (imuRead) {
    fAx += ((ax - biasX) - fAx) * ACCEL_ALPHA;
    fAy += ((ay - biasY) - fAy) * ACCEL_ALPHA;
    float sx, sy;
    mapAxes(orientation, fAx, fAy, sx, sy);
    gx = -sx; gy = -sy;                       // effective gravity = -proper acceleration
    float m = sqrtf(gx * gx + gy * gy);
    if (m > ACCEL_MAX) { gx *= ACCEL_MAX / m; gy *= ACCEL_MAX / m; }
    // spin about the screen normal: chip z is the screen normal; mirrored
    // mappings (4-7) flip the handedness, so the sign flips with them
    rawW = wz - biasW;
    float w = rawW * (3.14159265f / 180.f) * ((orientation & 4) ? -1.f : 1.f);
    float prevW = fW;
    fW += (w - fW) * GYRO_ALPHA;
    fAlpha += ((fW - prevW) * 60.f - fAlpha) * ALPHA_ALPHA;   // per-frame derivative at ~60 fps
  } else {
    fW = fAlpha = 0.f;
    float t = millis() * 0.001f;              // demo: slowly rotating gravity
    gx = sinf(t * 0.7f); gy = cosf(t * 0.7f);
  }

  // --- simulate + draw ---------------------------------------------------
  uint32_t t0 = micros();
  for (int s = 0; s < SUBSTEPS; ++s) sim.step(SUB_DT, gx, gy, fW, fAlpha);
  uint32_t thisSim = micros() - t0; simUs += thisSim; if (thisSim > maxSimUs) maxSimUs = thisSim;
  uint32_t subT0 = micros();
  submitFrame();                            // hands the frame to core 0
  uint32_t subUs = micros() - subT0; if (subUs > maxSubmitUs) maxSubmitUs = subUs;

  // --- stats -------------------------------------------------------------
  frames++;
  uint32_t thisFrame = micros() - frameStart; if (thisFrame > maxFrameUs) maxFrameUs = thisFrame;
  uint32_t now = millis();
#if SERIAL_IS_USB_CDC
  bool canPrint = (bool)Serial;              // only when a host is listening
#else
  bool canPrint = Serial.availableForWrite() > 400;   // never block on a full UART buffer
#endif
  if (now - statT >= 2000 && canPrint) {
    Serial.printf("%.1f fps  sim %.1f ms  draw %.1f ms (push %.1f, dma wait %.1f, splat %.1f, up %.1f, helper cpu %.1f)  mode %d  pairs %d  worst: frame %.1f imu %.1f sim %.1f submit %.1f ms  orient %d  g=(%+.2f,%+.2f)  raw=(%+.2f,%+.2f,%+.2f)  spin %+.2f rad/s\n",
                  frames * 1000.f / (now - statT), simUs / 1000.f / frames, drawUsAcc / 1000.f / frames,
                  pushUsAcc / 1000.f / frames, display::takeWaitUs() / 1000.f / frames,
                  rend.splatUs / 1000.f / frames, rend.upUs / 1000.f / frames,
                  (helperUs[0] + helperUs[1] + helperUs[2] + helperUs[3] + helperUs[4]) / 1000.f / frames, theme, sim.pairCount(),
                  maxFrameUs / 1000.f, maxImuUs / 1000.f, maxSimUs / 1000.f, maxSubmitUs / 1000.f,
                  orientation, gx, gy, fAx, fAy, imuOk ? az : 0.f, fW);
    maxFrameUs = maxImuUs = maxSimUs = maxSubmitUs = 0;
    if (profiling) {
    Serial.printf("   per frame: grid %.1f pairs %.1f dens %.1f disp %.1f visc %.1f heat %.1f rest %.1f ms, pairs %d\n",
                  sim.prof[0] / 1000.f / frames, sim.prof[1] / 1000.f / frames, sim.prof[2] / 1000.f / frames,
                  sim.prof[3] / 1000.f / frames, sim.prof[4] / 1000.f / frames, sim.prof[5] / 1000.f / frames,
                  sim.prof[6] / 1000.f / frames, sim.pairCount());
    Serial.printf("   discover own %.1f wait %.1f helper %.1f | displace own %.1f wait %.1f helper %.1f | helper dens %.1f refresh %.1f diffuse %.1f ms, rowMid %d\n",
                  sim.profOwn[0] / 1000.f / frames, sim.profWait[0] / 1000.f / frames, helperUs[2] / 1000.f / frames,
                  sim.profOwn[1] / 1000.f / frames, sim.profWait[1] / 1000.f / frames, helperUs[1] / 1000.f / frames,
                  helperUs[0] / 1000.f / frames, helperUs[3] / 1000.f / frames, helperUs[4] / 1000.f / frames, sim.rowMid_);
    }
    for (int i = 0; i < 5; ++i) helperUs[i] = 0;
    (void)0;
    for (int i = 0; i < 7; ++i) sim.prof[i] = 0;
    sim.profOwn[0] = sim.profOwn[1] = sim.profWait[0] = sim.profWait[1] = 0;
    statT = now; frames = 0; simUs = 0; drawUsAcc = 0; pushUsAcc = 0; rend.splatUs = rend.upUs = 0;
  }

  // Pace to ~60 fps so the physics timestep stays honest.
  uint32_t frameUs = micros() - lastFrame;
  if (frameUs < 16667) delayMicroseconds(16667 - frameUs);
  lastFrame = micros();
}
