// Shape gestures + reward effects.
// Include from touch_trail.ino AFTER fb[] / render() are defined.
//
// Gestures (recognition is deliberately forgiving of hand-drawn wobble):
//   X  (two crossing diagonal strokes, within 2.5 s)
//        -> firework: shell launches from where you drew the X, bursts
//   closed loop (square-ish, drawn in one stroke)
//        -> zoom tunnel of square rings
//   vertical zigzag (tall stroke with >=2 left/right reversals)
//        -> lightning strike
//   V (down-then-up, exactly one vertical reversal)
//        -> heavy rain: a dense downpour of vertical drops with floor
//           splashes (distinct from the ^ meteor shower's diagonal streaks)
//   ^ (up-then-down, exactly one vertical reversal)
//        -> whirlpool: 8 glowing arms spiral into the centre and the eye
//           pulses out; spin direction follows the drawn stroke
//   long press (stationary >= ~1 s, fires on release)
//        -> water-drop ripples from the pressed point
//   full-length straight swipe, horizontal OR vertical
//        -> wave crest sweeps across in the drawn direction
//   double tap (same spot, within 0.6 s, away from corners)
//        -> starburst: a spinning 16-spike star at the tapped point
//   4 corner taps (any order, within 3 s)
//        -> grand finale
//
// Detection is on pen-up, from the recorded stroke:
//   closed:  endpoints near each other AND the stroke encloses real area
//            (shoelace formula) - a retraced line has ~zero area and is
//            rejected, so no false tunnels from scribbling back and forth.
//   zigzag:  tall bounding box + at least 2 horizontal direction reversals.
//   X:       straight-ish diagonal stroke; two of them with opposite slopes
//            and nearby midpoints within X_WINDOW_MS fire the firework.
//
// All math is integer fixed point (positions 8.8 from the touch layer,
// stroke storage 4.4) — keep float out, its runtime costs ~2 KB of flash.
// The stroke buffer decimates 2:1 when full so BIG squares record their
// whole perimeter (a dropped tail would hide the closure from the detector).
#pragma once

// ---- tuning ----------------------------------------------------------------
// Coordinates: stroke points are stored in 4.4 fixed point (1 cell = 16),
// so "cells" constants below are value/16 cells; _256 suffix = x/256 ratios.
#define STROKE_MAX      80     // recorded points; decimated 2:1 when full
#define STROKE_SEG_SQ   52     // min dist^2 between recorded points ((0.45c)^2)
#define SHAPE_MIN_SIZE  64     // closed loop: min bounding box (4.0 cells)
#define SHAPE_CLOSE_256 115    // "closed" if start-end < 0.45 * bbox diagonal
#define LOOP_AREA_256   141    // enclosed area >= 0.55 * bbox area
#define SQ_CORNER_256   77     // square test: stroke must pass within 0.30 *
                               // min(bw,bh) of ALL 4 bbox corners. Rejects
                               // triangles/diamonds (>=1 corner far away);
                               // forgiving of rounded hand-drawn corners.
#define ZIG_MIN_H       80     // zigzag: min height (5.0 cells)
#define ZIG_MIN_W       29     // zigzag: min width (1.8 cells)
#define VEE_MIN_LEG     32     // V / ^: min vertical travel per leg (2.0 cells)
#define VEE_MIN_W       24     // V / ^: min HORIZONTAL start-end separation
                               // (1.5 cells) — a real V/^ opens sideways;
                               // rejects any line retraced back on itself
                               // (its endpoints coincide)
#define X_MIN_LEN       52     // X: min diagonal stroke length (3.25 cells)
#define X_STRAIGHT_256  410    // X: path length <= 1.6 * endpoint distance
#define X_WINDOW_MS     2500   // X: max time between the two strokes
#define X_MID_DIST_SQ   4096   // X: max midpoint distance^2 ((4.0 cells)^2)
#define TAP_MAX_FRAMES  24     // a "tap" is shorter than this (~0.4 s)
#define TAP_WINDOW_MS   3000   // collect the 4 corner taps within this window
#define HOLD_FRAMES     60     // long press: stationary touch >= ~1 s
#define DTAP_MS         600    // double tap: max gap between the two taps
#define DTAP_DIST_SQ    1024   // double tap: max distance^2 ((2.0 cells)^2)
#define SWIPE_MIN_W     144    // swipe: min width (9.0 cells — near full row)
#define SWIPE_MAX_H     24     // swipe: max height (1.5 cells)
#define TUNNEL_FRAMES   120    // tunnel length (~1.9 s, rings do ~1.5 cycles);
                               // 77 = one full cycle (~1.2 s) if it ever needs
                               // shortening again
#define FX_BRIGHT       153    // effect peak brightness, 0..255 = 60% of full
                               // (canvas has its own BRIGHT_CAP)

// Integer sqrt: isqrt32 of a sum of squares of 8.8 values is an 8.8 length
// (and of 4.4 values a 4.4 length). Replaces sqrtf + the float libs.
static uint16_t isqrt32(uint32_t v) {
  uint32_t r = 0, b = 0x40000000UL;
  while (b > v) b >>= 2;
  while (b) {
    if (v >= r + b) { v -= r + b; r = (r >> 1) + b; }
    else            r >>= 1;
    b >>= 2;
  }
  return (uint16_t)r;
}

// ---- tiny PRNG -------------------------------------------------------------
static uint16_t rnd_s = 1;
static uint8_t rnd8() { rnd_s = rnd_s * 2053u + 13849u; return (uint8_t)(rnd_s >> 8); }

// ---- stroke recording ------------------------------------------------------
static uint8_t  stX[STROKE_MAX], stY[STROKE_MAX];  // positions, 4.4 (x16)
static uint8_t  stN = 0;
static uint16_t strokeFrames = 0;
static uint16_t stSegSq = STROKE_SEG_SQ;

static void strokeReset() { stN = 0; strokeFrames = 0; stSegSq = STROKE_SEG_SQ; }

static void strokeAdd(int16_t x, int16_t y) {      // x,y = 8.8 LED coords
  strokeFrames++;
  uint8_t xi = (uint8_t)((x + 8) >> 4), yi = (uint8_t)((y + 8) >> 4);  // -> 4.4
  if (stN == 0) { stX[0] = xi; stY[0] = yi; stN = 1; return; }
  if (stN >= STROKE_MAX) {                 // full: decimate 2:1, keep recording
    for (uint8_t i = 0; i < STROKE_MAX / 2; i++) {   // (so a LARGE square's
      stX[i] = stX[2 * i];                           //  whole perimeter fits;
      stY[i] = stY[2 * i];                           //  dropping the tail hid
    }                                                //  the closure from the
    stN = STROKE_MAX / 2;                            //  detector)
    stSegSq <<= 2;                                   // min segment doubles
  }
  int16_t dx = (int16_t)xi - stX[stN - 1], dy = (int16_t)yi - stY[stN - 1];
  uint16_t d2 = (uint16_t)(dx * dx) + (uint16_t)(dy * dy);
  if (d2 >= stSegSq) { stX[stN] = xi; stY[stN] = yi; stN++; }
}

// ---- effect plumbing -------------------------------------------------------
static void fxClear() { memset(fb, 0, sizeof(fb)); }

static void fxFade(uint8_t num) {          // fb *= num/256
  for (uint8_t i = 0; i < 144; i++) fb[i] = (uint8_t)(((uint16_t)fb[i] * num) >> 8);
}

static uint8_t fxV(uint8_t v) { return (uint8_t)(((uint16_t)v * FX_BRIGHT) >> 8); }

static void addPixI(int8_t x, int8_t y, uint8_t v) {  // FX_BRIGHT applied here,
  if (x < 0 || x > 11 || y < 0 || y > 11) return;     // the single choke point
  v = fxV(v);                                         // for all effect drawing
  uint8_t *p = &fb[y * 12 + x];
  if (v > *p) *p = v;
}

static void fxSplat(int16_t x, int16_t y, uint8_t v) {  // bilinear, max-blend
  int8_t x0 = (int8_t)(x >> 8), y0 = (int8_t)(y >> 8);  // >>8 floors negatives
  uint16_t wx = (uint8_t)x, wy = (uint8_t)y;            // fractional parts
  addPixI(x0,     y0,     (uint8_t)(((((uint32_t)(256 - wx) * (256 - wy)) >> 8) * v) >> 8));
  addPixI(x0 + 1, y0,     (uint8_t)(((((uint32_t)wx         * (256 - wy)) >> 8) * v) >> 8));
  addPixI(x0,     y0 + 1, (uint8_t)(((((uint32_t)(256 - wx) * wy)         >> 8) * v) >> 8));
  addPixI(x0 + 1, y0 + 1, (uint8_t)(((((uint32_t)wx         * wy)         >> 8) * v) >> 8));
}

static void fxShow(uint8_t ms) {
  uint32_t t0 = millis();
  render();
  __asm__ __volatile__("wdr");             // effects outlive the 2 s watchdog
  if (is31Err >= 6) is31Recover();         // self-heal even mid-effect
  static uint8_t cfg = 0;                  // effects stream I2C for seconds:
  if (++cfg >= 32) { cfg = 0; is31Refresh(); }  // re-assert function page too
  // Deadline pacing, same as the main loop (which never stuttered — effects
  // did, because render-then-delay APPENDED every variable I2C cost — chunk
  // retries, a recovery, a bounded bus stall — to the frame, and ring-motion
  // effects like the tunnel show each wobble). The frame runs a fixed
  // ms + 5 (nominal render cost, preserving the old pace); variable costs
  // are absorbed by the remaining wait unless they exceed the whole budget.
  uint16_t budget = (uint16_t)ms + 5;
  while ((uint16_t)(millis() - t0) < budget) {}
}

// cos/sin * 64, 16 directions
static const int8_t DIRX[16] = {64,59,45,24,0,-24,-45,-59,-64,-59,-45,-24,0,24,45,59};
static const int8_t DIRY[16] = {0,24,45,59,64,59,45,24,0,-24,-45,-59,-64,-59,-45,-24};

// ---- effects ---------------------------------------------------------------
static void fxFirework(int16_t bx) {       // X: shell launch + burst + twinkle
  if (bx < 512)  bx = 512;                 // clamp 2.0 .. 9.5 (8.8)
  if (bx > 2432) bx = 2432;
  fxClear();
  int16_t y = 2944, vy = -128;             // 11.5, -0.50 cells/frame
  while (y > 717) {                        // decelerating ascent to 2.8
    fxFade(196);
    fxSplat(bx, y, 255);
    if (rnd8() & 1) addPixI((int8_t)(bx >> 8) + (rnd8() & 1), (int8_t)(y >> 8) + 1, 80);
    y += vy; vy = (int16_t)(((int32_t)vy * 244) >> 8);   // *= 0.955
    fxShow(13);
  }
  int16_t px[16], py[16], vx[16], vyp[16]; // the burst (8.8)
  for (uint8_t i = 0; i < 16; i++) {
    int16_t sp = 61 + (rnd8() & 15) * 2;   // speed 0.24..0.36 (8.8)
    px[i] = bx; py[i] = 717;
    vx[i]  = (int16_t)(((int32_t)DIRX[i] * sp) >> 6);
    vyp[i] = (int16_t)(((int32_t)DIRY[i] * sp * 218) >> 14);  // * 0.85 / 64
  }
  for (uint8_t f = 0; f < 80; f++) {
    fxFade(214);
    uint8_t v = (f < 22) ? 255 : (uint8_t)(255 - (f - 22) * 4);
    for (uint8_t i = 0; i < 16; i++) {
      px[i] += vx[i]; py[i] += vyp[i]; vyp[i] += 3;      // gravity ~0.012
      if (f > 30 && rnd8() < 80) continue;               // falling twinkle
      fxSplat(px[i], py[i], v);
    }
    if (f < 6) fxSplat(bx, 717, 255);      // core flash
    fxShow(13);
  }
  fxClear();
}

static void fxTunnel() {                             // closed loop: zoom tunnel
  for (uint8_t f = 0; f < TUNNEL_FRAMES; f++) {
    fxClear();
    for (uint8_t w = 0; w < 3; w++) {
      uint16_t r = (uint16_t)f * 23 + (uint16_t)w * 589;  // 0.09 cells/frame
                                                     // (8.8); ring spacing 2.3
      while (r >= 1766) r -= 1766;                   // wraps 0..6.9
      uint16_t amp = 64 + (uint16_t)(((uint32_t)r * 192) / 1766);  // 0.25..1.0
      for (int8_t y = 0; y < 12; y++)
        for (int8_t x = 0; x < 12; x++) {
          int16_t ax = x * 256 - 1408; if (ax < 0) ax = -ax;   // |x - 5.5|
          int16_t ay = y * 256 - 1408; if (ay < 0) ay = -ay;
          int16_t ch = (ax > ay) ? ax : ay;          // Chebyshev = square ring
          int32_t d = ((int32_t)(ch - (int16_t)r) * 341) >> 8;   // / 0.75
          if (d <= -256 || d >= 256) continue;       // a = 1 - d^2 <= 0
          uint16_t a = 256 - (uint16_t)((d * d) >> 8);
          addPixI(x, y, (uint8_t)(((uint32_t)a * amp * 255) >> 16));
        }
    }
    fxShow(11);
  }
  fxClear();
}

static void fxLightning() {                          // zigzag: lightning strike
  for (uint8_t bolt = 0; bolt < 2; bolt++) {
    int8_t bx[12];                                   // jagged path, top->bottom
    int8_t x = 3 + (rnd8() % 6);
    for (int8_t yy = 0; yy < 12; yy++) {
      bx[yy] = x;
      x += (int8_t)(rnd8() % 5) - 2;
      if (x < 0) x = 0;
      if (x > 11) x = 11;
    }
    for (uint8_t i = 0; i < 144; i++) fb[i] = fxV(80); // sky pre-flash
    fxShow(20);
    fxClear();
    fxShow(30);
    for (int8_t yy = 0; yy < 12; yy++) {             // the strike races down
      addPixI(bx[yy], yy, 255);
      addPixI(bx[yy] - 1, yy, 60);
      addPixI(bx[yy] + 1, yy, 60);
      fxShow(7);
    }
    for (uint8_t k = 0; k < 3; k++) {                // channel flicker
      fxFade(80);  fxShow(30);
      for (int8_t yy = 0; yy < 12; yy++) addPixI(bx[yy], yy, 255);
      fxShow(45);
    }
    for (uint8_t f = 0; f < 20; f++) { fxFade(196); fxShow(12); }  // afterglow
    fxClear();
    if (bolt == 0) fxShow(90);                       // beat between bolts
  }
}

static void fxFinale() {                             // 4 corners: grand finale
  fxClear();
  for (uint8_t f = 0; f <= 20; f++) {                // comets rush to centre
    fxFade(190);
    int16_t a = f * 70;                              // 0 -> 5.5 (8.8)
    int16_t b = 2816 - a;                            // 11.0 -> 5.5
    fxSplat(a, a, 255); fxSplat(b, a, 255); fxSplat(a, b, 255); fxSplat(b, b, 255);
    fxShow(12);
  }
  memset(fb, FX_BRIGHT, sizeof(fb));                 // impact flash
  fxShow(35);
  fxFade(120); fxShow(25);
  for (uint8_t f = 0; f < 110; f++) {                // shockwave + glitter
    fxFade(205);
    uint16_t r = (uint16_t)f * 44;                   // 0.17 cells/frame (8.8)
    if (r < 2560)                                    // 10.0
      for (int8_t y = 0; y < 12; y++)
        for (int8_t x = 0; x < 12; x++) {
          int16_t dx = x * 256 - 1408, dy = y * 256 - 1408;
          int32_t d = (int32_t)isqrt32((int32_t)dx * dx + (int32_t)dy * dy) - r;
          if (d <= -256 || d >= 256) continue;
          uint16_t a = 256 - (uint16_t)((d * d) >> 8);
          addPixI(x, y, (uint8_t)(((uint32_t)a * 255) >> 8));
        }
    if (f < 85)
      for (uint8_t s = 0; s < 5; s++) {              // glitter rain
        uint8_t p = rnd8();
        if (p < 144) { uint8_t v = fxV(140 + (rnd8() & 0x73)); if (v > fb[p]) fb[p] = v; }
      }
    fxShow(12);
  }
  fxClear();
}

static void fxRain() {                     // V: heavy downpour, ~1.3 s
  fxClear();
  int16_t rx[10], ry[10], rv[10];          // drop pool: pos + fall speed
  uint16_t alive = 0;
  for (uint8_t f = 0; f < 120; f++) {
    fxFade(178);                           // short afterglow = rain streaks
    if (f < 75) {                          // spawn phase: keep the pool full
      uint8_t born = 0;
      for (uint8_t i = 0; i < 10 && born < 2; i++) {
        if (alive & (1 << i)) continue;
        alive |= (uint16_t)(1 << i);
        rx[i] = (int16_t)(rnd8() % 12) << 8;
        ry[i] = -256 - (int16_t)(rnd8() & 255);
        rv[i] = 300 + (rnd8() & 127);      // 1.17..1.67 cells/frame
        born++;
      }
    }
    bool any = false;
    for (uint8_t i = 0; i < 10; i++) {
      if (!(alive & (1 << i))) continue;
      any = true;
      fxSplat(rx[i], ry[i], 220);          // drop head...
      fxSplat(rx[i], ry[i] - rv[i], 90);   // ...with a streak above it
      ry[i] += rv[i];
      if (ry[i] >= 2816) {                 // floor splash, both sides
        alive &= (uint16_t)~(1 << i);
        fxSplat(rx[i] - 200, 2816, 110);
        fxSplat(rx[i] + 200, 2816, 110);
      }
    }
    if (f >= 75 && !any) break;            // drained after the spawn phase
    fxShow(11);
  }
  for (uint8_t f = 0; f < 12; f++) { fxFade(185); fxShow(12); }  // dry up
  fxClear();
}

static void fxVortex(bool cw) {            // ^: glowing whirlpool — 8 arms in
  fxClear();                               //    two rings spiral into the
  int16_t ox[8], oy[8];                    //    centre; spin direction follows
  for (uint8_t i = 0; i < 8; i++) {        //    the drawn stroke
    uint8_t a = i * 2;                     // evenly spaced round the circle
    int16_t r = (i & 1) ? 17 : 26;         // layered radii: 4.3 / 6.5 cells
    ox[i] = DIRX[a] * r;
    oy[i] = DIRY[a] * r;
  }
  int16_t s = cw ? 60 : -60;               // ~13 deg of rotation per frame
  for (uint8_t f = 0; f < 78; f++) {
    fxFade(215);                           // arms leave a luminous swirl
    uint8_t b = 170 + f;                   // brighten as they near the eye
    for (uint8_t i = 0; i < 8; i++) {
      fxSplat(1408 + ox[i], 1408 + oy[i], b);
      int16_t nx = (int16_t)(((int32_t)ox[i] * 244 - (int32_t)oy[i] * s) >> 8);
      int16_t ny = (int16_t)(((int32_t)ox[i] * s + (int32_t)oy[i] * 244) >> 8);
      ox[i] = (int16_t)(((int32_t)nx * 252) >> 8);   // rotate + pull inward
      oy[i] = (int16_t)(((int32_t)ny * 252) >> 8);   // (~3.4%/frame)
    }
    if (rnd8() < 60) {                     // stray sparkle caught in the flow
      uint8_t i = rnd8() & 7;
      fxSplat(1408 + ox[i] * 2, 1408 + oy[i] * 2, 90);
    }
    fxShow(13);
  }
  for (uint8_t k = 0; k < 2; k++) {        // the eye pulses and winks out
    addPixI(5, 5, 255); addPixI(6, 5, 255);
    addPixI(5, 6, 255); addPixI(6, 6, 255);
    addPixI(4, 5, 110); addPixI(7, 6, 110);
    addPixI(5, 4, 110); addPixI(6, 7, 110);
    fxShow(30);
    fxFade(150);
    fxShow(25);
  }
  for (uint8_t f = 0; f < 15; f++) { fxFade(200); fxShow(12); }
  fxClear();
}

static void fxRipple(int16_t cx, int16_t cy) {   // hold: water-drop ripples —
  fxClear();                                     // 3 soft rings spread slowly
  for (uint8_t f = 0; f < 70; f++) {             // from the pressed point
    fxFade(215);
    int16_t r0 = (int16_t)f * 40;                // ~0.16 cells/frame
    for (int8_t y = 0; y < 12; y++)
      for (int8_t x = 0; x < 12; x++) {
        int16_t dx = x * 256 - cx, dy = y * 256 - cy;
        int32_t dist = isqrt32((int32_t)dx * dx + (int32_t)dy * dy);
        for (uint8_t w = 0; w < 3; w++) {
          int16_t r = r0 - (int16_t)w * 700;     // rings staggered 2.7 cells
          if (r < 0) break;                      // later rings not born yet
          int32_t d = dist - r;
          if (d <= -192 || d >= 192) continue;   // thin ring (0.75 cell)
          d = (d * 341) >> 8;                    // normalize to +-256
          uint16_t a = 256 - (uint16_t)((d * d) >> 8);
          uint8_t amp = (uint8_t)(200 - (uint16_t)((uint32_t)r * 130 / 2816));
          addPixI(x, y, (uint8_t)(((uint32_t)a * amp) >> 8));
        }
      }
    fxShow(13);
  }
  for (uint8_t f = 0; f < 12; f++) { fxFade(200); fxShow(12); }
  fxClear();
}

static void fxWave(bool vert, bool fwd) {        // swipe: a living wave crest
  fxClear();                                     // sweeps the drawn direction
  for (uint8_t f = 0; f < 28; f++) {             // (either axis, either way)
    fxFade(205);                                 // water trail behind
    int16_t h = fwd ? ((int16_t)f * 128 - 256)   // head crosses in ~26 frames
                    : (3072 - (int16_t)f * 128);
    for (int8_t c = 0; c < 12; c++) {            // c runs along the crest
      int16_t wob = DIRY[(uint8_t)(f + c * 3) & 15];   // +-0.5 cell shimmer
      int16_t a = h + wob * 2;                   // sweep-axis position
      int16_t fa = h + wob - (fwd ? 300 : -300); // foam trails the crest
      if (vert) {
        fxSplat(c * 256, a, 240);
        if (rnd8() < 70) fxSplat(c * 256, fa, 90);
      } else {
        fxSplat(a, c * 256, 240);
        if (rnd8() < 70) fxSplat(fa, c * 256, 90);
      }
    }
    fxShow(12);
  }
  for (uint8_t f = 0; f < 12; f++) { fxFade(190); fxShow(12); }
  fxClear();
}

static void fxBurst(int16_t cx, int16_t cy) {    // double tap: starburst —
  fxClear();                                     // 16 spikes, long and short
  for (uint8_t f = 0; f < 26; f++) {             // alternating, spinning out
    fxFade(196);                                 // spikes persist as trails
    uint8_t v = (f < 10) ? 255 : (uint8_t)(255 - (f - 10) * 15);
    for (uint8_t a = 0; a < 16; a++) {
      uint8_t d = (uint8_t)((a + (f >> 3)) & 15);   // long spikes spin as the
      int16_t r = (int16_t)f * ((a & 1) ? 44 : 78) + 128;   // star grows
      fxSplat(cx + (int16_t)(((int32_t)DIRX[d] * r) >> 6),
              cy + (int16_t)(((int32_t)DIRY[d] * r) >> 6), v);
    }
    if (f < 4) fxSplat(cx, cy, 255);             // white-hot core
    if (f > 5) {                                 // sparks thrown clear
      uint8_t p = rnd8();
      if (p < 144) { uint8_t s = fxV(150 + (rnd8() & 0x5F)); if (s > fb[p]) fb[p] = s; }
    }
    fxShow(11);
  }
  for (uint8_t f = 0; f < 14; f++) { fxFade(188); fxShow(11); }
  fxClear();
}

// ---- 4-corner tap tracker ----------------------------------------------------
static uint8_t  tapMask = 0;
static uint32_t tapT0 = 0;

static bool cornerTap(uint8_t mx, uint8_t my, uint32_t now) {  // 4.4 coords
  int8_t q = -1;
  if      (mx < 56  && my < 56)  q = 0;              // corner = outer 3.5 cells
  else if (mx > 120 && my < 56)  q = 1;
  else if (mx < 56  && my > 120) q = 2;
  else if (mx > 120 && my > 120) q = 3;
  if (q < 0) { tapMask = 0; return false; }          // tap away from corners
  if (tapMask == 0 || (uint32_t)(now - tapT0) > TAP_WINDOW_MS) { tapMask = 0; tapT0 = now; }
  tapMask |= (uint8_t)(1 << q);
  if (tapMask == 0x0F) { tapMask = 0; return true; }
  return false;
}

// ---- X-stroke tracker ---------------------------------------------------------
static uint8_t  xSlope = 0;      // 0 = none pending, 1 = "\", 2 = "/"
static int16_t  xMx, xMy;        // midpoint of the pending stroke (4.4)
static uint32_t xT = 0;

// ---- double-tap tracker -------------------------------------------------------
static int16_t  dtX, dtY;        // last non-corner tap position (4.4)
static uint32_t dtT = 0;         // ...and when (0 = none pending)

// ---- release hook (call on pen-up) --------------------------------------------
static void onRelease(uint32_t now) {
  rnd_s ^= (uint16_t)now | 1;
  if (stN && stN <= 3 && strokeFrames >= HOLD_FRAMES) {
    // stationary long press (nothing else stays put this long): ripples
    // spread from where the finger was held
    fxRipple((int16_t)stX[stN - 1] << 4, (int16_t)stY[stN - 1] << 4);
    strokeReset();
    return;
  }
  if (strokeFrames <= TAP_MAX_FRAMES && stN <= 3) {  // a tap, not a stroke
    if (stN) {
      uint8_t mx = stX[0], my = stY[0];
      if ((mx < 56 || mx > 120) && (my < 56 || my > 120)) {
        dtT = 0;                                   // corner zones belong to
        if (cornerTap(mx, my, now)) fxFinale();    // the finale collection
      } else {
        tapMask = 0;                               // stray tap still cancels
                                                   // the finale collection
        if (dtT && (uint32_t)(now - dtT) <= DTAP_MS) {
          int16_t ddx = (int16_t)mx - dtX, ddy = (int16_t)my - dtY;
          if ((int32_t)ddx * ddx + (int32_t)ddy * ddy <= DTAP_DIST_SQ) {
            dtT = 0;                               // double tap: pop!
            fxBurst((int16_t)mx << 4, (int16_t)my << 4);
            strokeReset();
            return;
          }
        }
        dtX = mx; dtY = my; dtT = now;             // arm / re-arm
      }
    }
    strokeReset();
    return;
  }
  if (stN < 7) { strokeReset(); return; }  // (7: a short fast X diagonal,
                                           //  trimmed by the 2-frame touch-on
                                           //  debounce, must still qualify)

  int16_t sx = stX[0], sy = stY[0];                  // all geometry in 4.4
  int16_t ex = stX[stN - 1], ey = stY[stN - 1];
  uint8_t x0 = 255, x1 = 0, y0 = 255, y1 = 0;
  uint16_t pathLen = 0;                              // 4.4
  for (uint8_t i = 0; i < stN; i++) {
    if (stX[i] < x0) x0 = stX[i];
    if (stX[i] > x1) x1 = stX[i];
    if (stY[i] < y0) y0 = stY[i];
    if (stY[i] > y1) y1 = stY[i];
    if (i) {
      int16_t dx = (int16_t)stX[i] - stX[i - 1], dy = (int16_t)stY[i] - stY[i - 1];
      pathLen += isqrt32((int32_t)dx * dx + (int32_t)dy * dy);
    }
  }
  uint8_t bw = x1 - x0, bh = y1 - y0;
  uint16_t endDist = isqrt32((int32_t)(ex - sx) * (ex - sx) +
                             (int32_t)(ey - sy) * (ey - sy));
  uint16_t diag = isqrt32((uint32_t)bw * bw + (uint32_t)bh * bh);
  bool closed = (uint32_t)endDist * 256 < (uint32_t)diag * SHAPE_CLOSE_256;

  if (closed && bw >= SHAPE_MIN_SIZE && bh >= SHAPE_MIN_SIZE) {
    // SQUARE: a real loop (shoelace area) that also passes near all 4 corners
    // of its bounding box. Triangles/diamonds leave >= 1 corner far away and
    // are rejected; near-circular loops can pass — for this gadget a generous
    // square (rounded hand-drawn corners still fire) beats a strict one.
    int32_t a2 = 0;                                  // shoelace, (4.4)^2 units
    for (uint8_t i = 0; i < stN; i++) {
      uint8_t j = (i + 1) % stN;
      a2 += (int32_t)stX[i] * stY[j] - (int32_t)stX[j] * stY[i];
    }
    if (a2 < 0) a2 = -a2;                            // |2 * area|
    if ((uint32_t)a2 * 128 >= (uint32_t)LOOP_AREA_256 * bw * bh) {
      uint16_t thr = ((uint16_t)((bw < bh) ? bw : bh) * SQ_CORNER_256) >> 8;
      if (thr < 14) thr = 14;                        // >= 0.9 cell
      uint32_t t2 = (uint32_t)thr * thr;
      uint32_t c00 = ~0UL, c10 = ~0UL, c01 = ~0UL, c11 = ~0UL;
      for (uint8_t i = 0; i < stN; i++) {
        uint16_t dx0 = stX[i] - x0, dx1 = x1 - stX[i];
        uint16_t dy0 = stY[i] - y0, dy1 = y1 - stY[i];
        uint32_t d;
        d = (uint32_t)dx0 * dx0 + (uint32_t)dy0 * dy0; if (d < c00) c00 = d;
        d = (uint32_t)dx1 * dx1 + (uint32_t)dy0 * dy0; if (d < c10) c10 = d;
        d = (uint32_t)dx0 * dx0 + (uint32_t)dy1 * dy1; if (d < c01) c01 = d;
        d = (uint32_t)dx1 * dx1 + (uint32_t)dy1 * dy1; if (d < c11) c11 = d;
      }
      if (c00 <= t2 && c10 <= t2 && c01 <= t2 && c11 <= t2) {
        fxTunnel();
        strokeReset();
        return;
      }
      strokeReset();       // a genuine loop, just not square-cornered: consume
      return;              // it (don't let a circle leak into the V detector)
    }
  }
  // "closed" but enclosing no real area (thin / retraced stroke, or a bbox
  // too small): fall through — a narrow deep V looks "closed" because its
  // endpoints nearly meet, yet it must still reach the V detector below.

  // open stroke: direction reversals on BOTH axes (0.7-cell quantized moves).
  // Horizontal reversals -> zigzag; exactly one vertical reversal -> V / ^.
  uint8_t revsX = 0, revsY = 0;
  int8_t dirX = 0, dirY = 0, firstY = 0;
  int16_t accx = 0, accy = 0;
  for (uint8_t i = 1; i < stN; i++) {
    accx += (int16_t)stX[i] - stX[i - 1];
    accy += (int16_t)stY[i] - stY[i - 1];
    if (accx >= 11 || accx <= -11) {                 // 0.7 cell
      int8_t d = (accx > 0) ? 1 : -1;
      if (dirX && d != dirX) revsX++;
      dirX = d;
      accx = 0;
    }
    if (accy >= 11 || accy <= -11) {
      int8_t d = (accy > 0) ? 1 : -1;
      if (dirY && d != dirY) revsY++;
      if (!firstY) firstY = d;
      dirY = d;
      accy = 0;
    }
  }
  if (bh >= ZIG_MIN_H && bw >= ZIG_MIN_W && revsX >= 2) {
    fxLightning();
    xSlope = 0;
    strokeReset();
    return;
  }

  // full-length straight swipe (either axis) -> wave in the drawn direction.
  // No overlap with anything else: X needs >= 2 cells on BOTH axes, zigzag
  // needs 2 horizontal reversals, V/^ needs a vertical reversal (which also
  // excludes retraced lines here via revsX/revsY == 0).
  if ((uint32_t)pathLen * 256 <= (uint32_t)endDist * 320) {
    if (bw >= SWIPE_MIN_W && bh <= SWIPE_MAX_H && revsX == 0) {
      fxWave(false, ex > sx);                      // horizontal sweep
      xSlope = 0;
      strokeReset();
      return;
    }
    if (bh >= SWIPE_MIN_W && bw <= SWIPE_MAX_H && revsY == 0) {
      fxWave(true, ey > sy);                       // vertical sweep
      xSlope = 0;
      strokeReset();
      return;
    }
  }

  // straight diagonal stroke? two opposite ones crossing = X -> firework.
  // Checked BEFORE V/^: a diagonal with a small hook at pen-up can register
  // one vertical reversal and must still count as an X stroke, not a V/^.
  // (No overlap the other way: a real V/^ has pathLen ~2x its endpoint
  // distance and fails the straightness test.)
  int16_t dx = ex - sx, dy = ey - sy;
  int16_t adx = (dx < 0) ? -dx : dx, ady = (dy < 0) ? -dy : dy;
  if (endDist >= X_MIN_LEN &&
      (uint32_t)pathLen * 256 <= (uint32_t)endDist * X_STRAIGHT_256 &&
      adx >= 32 && ady >= 32) {                      // both components >= 2.0
    uint8_t slope = ((int32_t)dx * dy > 0) ? 1 : 2;  // "\" : "/"  (y grows down)
    int16_t mx = (sx + ex) / 2, my = (sy + ey) / 2;
    int16_t mdx = mx - xMx, mdy = my - xMy;
    if (xSlope && slope != xSlope &&
        (uint32_t)(now - xT) <= X_WINDOW_MS &&
        (int32_t)mdx * mdx + (int32_t)mdy * mdy <= X_MID_DIST_SQ) {
      int16_t cxr = (mx + xMx) << 3;                 // mid of mids, 4.4 -> 8.8
      xSlope = 0;
      fxFirework(cxr);
      strokeReset();
      return;
    }
    xSlope = slope; xMx = mx; xMy = my; xT = now;
    strokeReset();
    return;
  }

  // V / ^: exactly ONE vertical reversal, mostly-monotonic horizontally
  // (revsX <= 1 also keeps thin circle-ish strokes out: a circle reverses
  // horizontally twice), both legs with real vertical travel, and endpoints
  // opened sideways (kills retraced lines).
  if (revsY == 1 && revsX <= 1 && adx >= VEE_MIN_W) {
    if (firstY > 0 &&                                // down first = V
        (int16_t)y1 - sy >= VEE_MIN_LEG && (int16_t)y1 - ey >= VEE_MIN_LEG) {
      fxRain();
      xSlope = 0;
      strokeReset();
      return;
    }
    if (firstY < 0 &&                                // up first = ^
        sy - y0 >= VEE_MIN_LEG && ey - y0 >= VEE_MIN_LEG) {
      fxVortex(ex > sx);                             // spin the drawn direction
      xSlope = 0;
      strokeReset();
      return;
    }
  }
  strokeReset();
}
