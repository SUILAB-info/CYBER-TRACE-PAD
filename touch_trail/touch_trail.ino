// ============================================================================
// touch_trail — 12x12 LED matrix + 4x4 touch pad
//   "Finger position glows bright; the path behind fades out as a trail."
//
// MCU:    ATtiny1616 (VQFN-20) @ 20 MHz internal, megaTinyCore
// Driver: IS31FL3731 (U2), I2C addr 0x74, cross-plex 144 LEDs
// Pins (extracted from the routed board, led_touch_matrix.kicad_pcb):
//   PA1=SDA  PA2=SCL   (TWI0 ALTERNATE position -> Wire.swap(1))
//   PA3=SDB  (driver enable, board has 100k pulldown -> drive HIGH)
//   PA4..PA7 = TOUCH_X0..X3 (rows),  PC0..PC3 = TOUCH_Y0..Y3 (columns)
//   PB0..PB5 unused (pulled up in software)
//
// Touch is SELF-CAPACITANCE (see touch4x4.h; mutual-cap fails on this
// hardware). Robustness: 2 s watchdog + I2C error tracking / bus recovery.
// All position/effect math is 8.8/4.4 fixed point — keep float out, the
// soft-float runtime costs ~1.3 KB of flash.
//
// Boot: 1.4 s raster sweep (one LED at a time, left->right / top->bottom).
//       Clean raster order = LED mapping verified on hardware.
// ============================================================================

#include <Wire.h>

// ---------------- tuning ----------------------------------------------------
#define TOUCH_ACC       16     // ADC charge-share cycles per electrode
#define TOUCH_THRESHOLD 80     // touch-on threshold (sum of TOUCH_ACC readings,
#define TOUCH_KEEP      32     // so scale both with TOUCH_ACC); KEEP = touch-off
#define TOUCH_STUCK_FRAMES 900 // continuous-touch frames (~15 s) before a
                               // forced recalibration (shifted-baseline latch)
#define AXIS_AMBIG_256  192    // reject an axis whose peak is rivalled (>=0.75)
                               // by an electrode 2+ nodes away: two contact
                               // points, or noise — position is meaningless
#define BASE_STUCK_FRAMES 600  // per-electrode frames (~10 s) parked above
                               // +TOUCH_KEEP before its baseline is snapped
#define CANVAS_MAX_MS  30000   // hard cap on how long the canvas may hold
                               // without fading, however busy the pad looks
#define HOLD_MS        800     // keep the drawing lit this long after the last
                               // touch; a new touch within this window keeps
                               // drawing on the same canvas (multi-stroke text)
#define DECAY_NUM      247     // fade-out speed once HOLD_MS expires:
                               //   247 ~ 2 s fade / 242 ~ 1 s / 251 ~ 3 s
#define FRAME_MS        16     // ~60 fps
#define BRIGHT_CAP     153     // 60% of full; lower if too bright/hot
#define TOUCH_DEBUG      0     // 1 = show raw electrode deltas on the matrix

#include "led_map.h"           // LED_REG[144], GAMMA8[256]  (auto-generated)
#include "is31fl3731.h"
#include "touch4x4.h"

static uint8_t fb[144];        // framebuffer, linear = y*12 + x
static uint8_t pwm[144];       // IS31FL3731 register-order output buffer

// All position math is 8.8 fixed point (1 LED cell = 256): no float library.
static void addPix(int8_t x, int8_t y, uint16_t w) {   // w = weight, 0..256
  if (x < 0 || x > 11 || y < 0 || y > 11) return;
  uint8_t v = (uint8_t)(((uint32_t)w * BRIGHT_CAP) >> 8);
  uint8_t *p = &fb[y * 12 + x];
  if (v > *p) *p = v;          // max-blend so the trail never darkens the head
}

// Bilinear splat: the finger "dot" moves smoothly between LED cells.
static void splat(int16_t fx, int16_t fy) {
  int8_t x0 = (int8_t)(fx >> 8), y0 = (int8_t)(fy >> 8);
  uint16_t wx = (uint8_t)fx, wy = (uint8_t)fy;         // fractional parts
  addPix(x0,     y0,     (uint16_t)(((uint32_t)(256 - wx) * (256 - wy)) >> 8));
  addPix(x0 + 1, y0,     (uint16_t)(((uint32_t)wx         * (256 - wy)) >> 8));
  addPix(x0,     y0 + 1, (uint16_t)(((uint32_t)(256 - wx) * wy)         >> 8));
  addPix(x0 + 1, y0 + 1, (uint16_t)(((uint32_t)wx         * wy)         >> 8));
}

static void render() {
  for (uint8_t i = 0; i < 144; i++) pwm[LED_REG[i]] = GAMMA8[fb[i]];
  is31Frame(pwm);
}

#include "shapefx.h"           // shape gestures + effects (needs fb/render)

static void bootSweep() {                 // visual proof of the LED mapping
  for (uint8_t i = 0; i < 144; i++) {
    memset(pwm, 0, sizeof(pwm));
    pwm[LED_REG[i]] = GAMMA8[200];
    is31Frame(pwm);
    delay(6);
  }
  memset(pwm, 0, sizeof(pwm));
  is31Frame(pwm);
}

void setup() {
  uint8_t rstf = RSTCTRL.RSTFR;           // why did we reset?
  RSTCTRL.RSTFR = rstf;                   // (clear flags)

  pinMode(PIN_PA3, OUTPUT);               // SDB: enable the LED driver
  digitalWrite(PIN_PA3, HIGH);
  // unused pins: pull up so they don't float. NOTE: on 20-pin parts the
  // PORTB Arduino numbers run in reverse (PIN_PB5=4 .. PIN_PB0=9), so a
  // PIN_PB0..PIN_PB5 for-loop never executes — list them explicitly.
  const uint8_t unusedPins[] = { PIN_PB0, PIN_PB1, PIN_PB2,
                                 PIN_PB3, PIN_PB4, PIN_PB5 };
  for (uint8_t i = 0; i < sizeof(unusedPins); i++)
    pinMode(unusedPins[i], INPUT_PULLUP);

  touchPins();

  i2cBusClear();                          // the IS31 may still hold SDA from a
                                          // transaction cut off by the reset
  if (!Wire.swap(1))                      // SDA=PA1 / SCL=PA2 (alt position)
    PORTMUX.CTRLB |= PORTMUX_TWI0_bm;     // fallback for older core versions
  Wire.begin();
  Wire.setClock(400000);
  delay(10);

  is31Init();
  if (!(rstf & RSTCTRL_WDRF_bm))          // skip the sweep on watchdog resets
    bootSweep();
  touchCalibrate();                       // don't touch the pad during power-up

  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_2KCLK_gc);   // ~2 s watchdog
}

void loop() {
  __asm__ __volatile__("wdr");            // feed the watchdog every frame
  uint32_t t0 = millis();

  touchScan();

#if TOUCH_DEBUG
  // Raw electrode deltas, for tuning TOUCH_THRESHOLD without a serial port:
  //   upper half = 4 vertical bars  = COLUMN deltas (left -> right)
  //   lower half = 4 vertical bars  = ROW deltas    (top row shown leftmost)
  // Bars should stay dark untouched and light up strongly under a finger.
  memset(fb, 0, sizeof(fb));
  for (uint8_t i = 0; i < 4; i++) {
    int16_t dc = tDelta[4 + i], dr = tDelta[i];
    uint8_t vc = (dc <= 0) ? 0 : (dc >= 128 ? 255 : dc * 2);
    uint8_t vr = (dr <= 0) ? 0 : (dr >= 128 ? 255 : dr * 2);
    for (uint8_t y = 0; y < 5; y++)
      for (uint8_t x = 0; x < 3; x++) {
        fb[y * 12 + (i * 3 + x)]       = vc;   // columns, y = 0..4
        fb[(y + 7) * 12 + (i * 3 + x)] = vr;   // rows,    y = 7..11
      }
  }
#else
  static int16_t sfx, sfy;
  static int16_t rawPX, rawPY;                      // last RAW touch position
  static bool hadTouch = false;
  static uint32_t lastTouch = 0;
  static uint8_t spike = 0;
  int16_t fx, fy;
  bool on = touchPos(&fx, &fy);
  bool stroke = false;                              // continuing a stroke?
  int16_t pfx = 0, pfy = 0;                         // previous pen position
  if (on && hadTouch) {
    // Spike rejection: a RAW-position jump > 2.5 cells in ONE frame is far
    // more likely noise than a finger (the segment fill below would paint a
    // streak across untouched LEDs). Hold for one frame; if the next raw
    // position stays out there it is a genuine fast swipe — accept it.
    // Measure raw-vs-raw: measuring against the SMOOTHED position (which
    // deliberately lags by ~one frame of velocity) halves the effective
    // threshold and mangles fast X strokes.
    int16_t jx = fx - rawPX, jy = fy - rawPY;
    bool far = (int32_t)jx * jx + (int32_t)jy * jy > 640L * 640L;
    rawPX = fx; rawPY = fy;
    if (!spike && far) { spike = 1; fx = sfx; fy = sfy; }
    else spike = 0;
  } else if (on) {
    spike = 0;
    rawPX = fx; rawPY = fy;                         // first contact
  }
  if (on) {
    if (!hadTouch) { sfx = fx; sfy = fy; }          // jump to first contact
    else {
      stroke = true; pfx = sfx; pfy = sfy;
      sfx += (fx - sfx) / 2; sfy += (fy - sfy) / 2; // de-jitter
    }
    strokeAdd(sfx, sfy);                            // record for shape gestures
    lastTouch = t0;
  }
  if (!on && hadTouch) {                            // pen-up: classify & reward
    // The recorded stroke ends at the SMOOTHED position, which lags the
    // finger by ~one frame of velocity — enough to shave a fast X stroke
    // under its minimum-length test. Append the last RAW position first
    // (unless that frame was a held noise spike).
    if (!spike) strokeAdd(rawPX, rawPY);
    onRelease(t0);
    spike = 0;
  }
  hadTouch = on;

  // Canvas behaviour: everything drawn stays lit while touching and for
  // HOLD_MS after the last touch (so multi-stroke characters accumulate);
  // then the whole drawing fades out smoothly.
  // CANVAS_MAX_MS is a backstop: drawing is max-blended and never dims while
  // a touch is being reported, so a false touch arriving more often than once
  // per HOLD_MS would keep resetting lastTouch and freeze the fade forever —
  // lit LEDs then only ever accumulate. Past the cap the canvas drains no
  // matter what the pad claims.
  static uint32_t canvasT = 0;
  bool drain = !on && (uint32_t)(t0 - lastTouch) > HOLD_MS;
  if (drain) canvasT = t0;                          // draining normally: fresh
  if (drain || (uint32_t)(t0 - canvasT) > CANVAS_MAX_MS) {
    for (uint8_t i = 0; i < 144; i++) {             // exponential fade-out
      uint8_t o = fb[i];
      uint8_t v = (uint8_t)(((uint16_t)o * DECAY_NUM) >> 8);
      fb[i] = (v == o && v) ? v - 1 : v;            // -1 only when the multiply
    }                                               // stalls, so it still hits 0
  }
  if (on) {
    if (stroke) {                                   // fill the whole segment
      int16_t dx = sfx - pfx, dy = sfy - pfy;       // since the last frame, so
      uint16_t dist = isqrt32((int32_t)dx * dx + (int32_t)dy * dy);
      uint8_t steps = (uint8_t)(dist >> 7) + 1;     // ~0.5 cell per step
      for (uint8_t s = 1; s <= steps; s++)
        splat(pfx + (int16_t)((int32_t)dx * s / steps),
              pfy + (int16_t)((int32_t)dy * s / steps));
    } else {
      splat(sfx, sfy);                              // pen-down dot
    }
  }
#endif

  render();
  if (is31Err >= 6) is31Recover();                  // bus wedged? unstick it

  static uint8_t cfgTick = 0;                       // silent register corruption
  if (++cfgTick >= 32) { cfgTick = 0; is31Refresh(); } // heals within ~0.5 s

  while (millis() - t0 < FRAME_MS) {}               // frame pacing
}
