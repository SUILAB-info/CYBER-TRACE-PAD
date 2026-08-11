// 4x4 touch pad position sensing — SELF-CAPACITANCE via ADC charge-sharing.
//
// Mutual capacitance (drive column / sense row) could not separate COLUMNS: a
// finger capacitively loads the whole sensed ROW electrode, and that self-cap
// effect (identical for all 4 column measurements) swamped the tiny per-column
// mutual dip -> X centroid sat near the middle and jittered. Self-capacitance
// senses each of the 8 electrodes directly, so both axes get the same strong,
// symmetric signal.
//
// Electrodes:  rows X0..X3 = PA4..PA7 -> ADC0 AIN4..AIN7  (top -> bottom)
//              cols Y0..Y3 = PC0..PC3 -> ADC1 AIN6..AIN9  (left -> right)
//
// Measurement (repeated TOUCH_ACC times per electrode):
//   1. drive the electrode HIGH  -> its capacitance Ce charges to VDD
//   2. ADC conversion on the internal GND channel -> S/H cap left near 0 V
//   3. float the electrode, convert its channel   -> Ce shares charge with
//      the discharged S/H cap; result ~ Ce/(Ce+Csh)*1023.
//   A finger adds to Ce -> the reading RISES above baseline.
// All other electrodes stay driven LOW (grounded shield) meanwhile.
//
// Requires TOUCH_ACC / TOUCH_THRESHOLD / TOUCH_KEEP / TOUCH_STUCK_FRAMES
// #defined before include.
#pragma once
#include <Arduino.h>

typedef struct { ADC_t *adc; uint8_t mux; PORT_t *port; uint8_t bm; } t_elec;

static const t_elec T_EL[8] = {
  { &ADC0, ADC_MUXPOS_AIN4_gc, &PORTA, PIN4_bm },  // 0: row0 (top)
  { &ADC0, ADC_MUXPOS_AIN5_gc, &PORTA, PIN5_bm },  // 1: row1
  { &ADC0, ADC_MUXPOS_AIN6_gc, &PORTA, PIN6_bm },  // 2: row2
  { &ADC0, ADC_MUXPOS_AIN7_gc, &PORTA, PIN7_bm },  // 3: row3 (bottom)
  { &ADC1, ADC_MUXPOS_AIN6_gc, &PORTC, PIN0_bm },  // 4: col0 (left)
  { &ADC1, ADC_MUXPOS_AIN7_gc, &PORTC, PIN1_bm },  // 5: col1
  { &ADC1, ADC_MUXPOS_AIN8_gc, &PORTC, PIN2_bm },  // 6: col2
  { &ADC1, ADC_MUXPOS_AIN9_gc, &PORTC, PIN3_bm },  // 7: col3 (right)
};

static uint16_t tBase[8];   // per-electrode no-touch baseline
static int16_t  tDelta[8];  // raw - baseline (positive on touch)
static uint16_t tHi[8];     // frames this electrode has been stuck high

static void adcSetup(ADC_t *a) {
  a->CTRLA    = 0;
  a->CTRLB    = 0;                                       // we accumulate ourselves
  a->CTRLC    = ADC_PRESC_DIV16_gc | ADC_REFSEL_VDDREF_gc; // SAMPCAP=0: big S/H cap
  a->SAMPCTRL = 2;
  a->INTCTRL  = 0;
  a->CTRLA    = ADC_ENABLE_bm;                           // 10-bit
}

static uint16_t adcConv(ADC_t *a, uint8_t mux) {
  a->MUXPOS  = mux;
  a->COMMAND = ADC_STCONV_bm;
  while (!(a->INTFLAGS & ADC_RESRDY_bm)) {}
  return a->RES;                                         // read clears RESRDY
}

static void touchPins() {
  // digital input buffers off on all electrode pins (cleaner ADC, less leakage)
  PORTA.PIN4CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN5CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN6CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.PIN7CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTC.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;
  PORTA.OUTCLR = PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm;  // idle: grounded
  PORTA.DIRSET = PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm;
  PORTC.OUTCLR = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;
  PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;
  adcSetup(&ADC0);
  adcSetup(&ADC1);
}

static uint16_t elecRaw(const t_elec *e) {
  uint16_t acc = 0;
  for (uint8_t k = 0; k < TOUCH_ACC; k++) {
    e->port->OUTSET = e->bm;                 // charge electrode to VDD
    e->port->DIRSET = e->bm;
    delayMicroseconds(2);
    (void)adcConv(e->adc, ADC_MUXPOS_GND_gc); // S/H cap <- ~0 V
    e->port->DIRCLR = e->bm;                 // float (electrode holds its charge)
    e->port->OUTCLR = e->bm;
    acc += adcConv(e->adc, e->mux);          // charge share -> ~Ce/(Ce+Csh)
  }
  e->port->DIRSET = e->bm;                   // park grounded (OUT already 0)
  return acc;
}

static void touchCalibrate() {
  uint32_t sum[8] = {0};
  for (uint8_t s = 0; s < 32; s++)
    for (uint8_t i = 0; i < 8; i++)
      sum[i] += elecRaw(&T_EL[i]);
  for (uint8_t i = 0; i < 8; i++) { tBase[i] = sum[i] / 32; tHi[i] = 0; }
}

static void touchScan() {
  static uint8_t drift = 0;
  bool doDrift = (++drift & 0x07) == 0;      // slow baseline tracking
  for (uint8_t i = 0; i < 8; i++) {
    uint16_t v = elecRaw(&T_EL[i]);
    tDelta[i] = (int16_t)v - (int16_t)tBase[i];
    if (tDelta[i] < -(int16_t)TOUCH_KEEP) {
      tBase[i] += tDelta[i] / 2;             // raw far BELOW baseline can never
      tHi[i] = 0;                            // be a touch (touch raises it) ->
                                             // converge fast; without this a
                                             // recal done mid-touch would dead-
                                             // lock the pad after release
    } else if (tDelta[i] > (int16_t)TOUCH_KEEP) {
      // Above +TOUCH_KEEP the slow tracking below is frozen, so an upward
      // baseline shift (self-heating in the closed case, humidity, a hand near
      // the pad during boot calibration) is a ONE-WAY TRAP: the electrode sits
      // permanently elevated, eating the touch margin until the pad fires
      // without contact. No finger rests on one electrode for ten seconds, so
      // treat that as drift and snap the baseline to the current reading.
      if (++tHi[i] >= BASE_STUCK_FRAMES) { tBase[i] = v; tHi[i] = 0; }
    } else {
      tHi[i] = 0;
      if (doDrift) tBase[i] += (v > tBase[i]) ? 1 : ((v < tBase[i]) ? -1 : 0);
    }
  }
}

// Position of one 4-electrode axis -> 0..11 LED coordinate in 8.8 fixed point
// (i.e. 0..2816; all position math is integer — no float library needed).
// Peak-plus-neighbours ratio estimator (the standard touch-slider method):
// take the strongest electrode and interpolate toward its larger neighbour by
// t = neighbour/(peak+neighbour). Unlike a centroid over all electrodes this
// needs NO noise-floor subtraction, so it stays linear even for a light touch
// where the neighbour signal is small (a floor-based centroid discarded that
// neighbour and snapped to node centres -> intermittent L-shaped diagonals,
// depending on how firmly the finger pressed).
// The SMALLEST of all four electrodes is the pedestal estimate (common
// far-field coupling: a nearby hand, thermal drift) and is subtracted before
// the ratio. Using the global minimum rather than the smaller neighbour also
// fixes the edge nodes, where one neighbour is forced to 0 and the pedestal
// was therefore never removed at all.
static bool axisBad;                         // set when the profile is ambiguous

static int16_t axisPos(const int16_t *d) {
  int8_t ip = 0;
  int16_t mx = d[0], fl = d[0];
  for (uint8_t i = 1; i < 4; i++) {
    if (d[i] > mx) { mx = d[i]; ip = i; }
    if (d[i] < fl) fl = d[i];
  }
  if (fl < 0) fl = 0;
  int16_t c = mx - fl;                       // peak above the pedestal
  if (c < 1) c = 1;

  // Ambiguity guard. This estimator is smooth when the peak moves between
  // ADJACENT electrodes, but jumps the full width of the pad when the peak
  // flips between two that are far apart — a 1-count noise flip on a corner
  // touch moved the reported position from 1.0 to 10.0 cells and the segment
  // fill painted that as a solid line across the panel. A rival two or more
  // nodes from the peak means two contact points (self-capacitance cannot
  // separate them) or a signal too weak to localize: report nothing instead.
  for (uint8_t i = 0; i < 4; i++) {
    int8_t dd = (int8_t)i - ip;
    if (dd < 0) dd = -dd;
    if (dd >= 2 && (int32_t)(d[i] - fl) * 256 >= (int32_t)c * AXIS_AMBIG_256)
      axisBad = true;
  }

  int16_t l = (ip > 0) ? (int16_t)(d[ip - 1] - fl) : 0;
  int16_t r = (ip < 3) ? (int16_t)(d[ip + 1] - fl) : 0;
  if (l < 0) l = 0;
  if (r < 0) r = 0;
  int16_t t = (r >= l)                       // 8.8: toward ip+1 (0..128)
      ? (int16_t)(((int32_t)r << 8) / (c + r))
      : (int16_t)(-(((int32_t)l << 8) / (c + l)));
  return (int16_t)((((int32_t)ip * 256 + t) * 11) / 3);  // nodes 0..3 -> 0..11
}

// Interpolated finger position in LED coordinates (8.8 fixed point). A real
// touch must show on BOTH axes; hysteresis on the weaker one.
// Ghost-touch guard: turning ON needs 2 consecutive frames over threshold
// (independent single-frame noise spikes on one row + one col otherwise
// manifest as a phantom touch at their intersection). OFF stays immediate.
// Stuck-touch guard: a touch "held" for TOUCH_STUCK_FRAMES (~15 s) is not a
// finger — the baseline has shifted under it (humidity / skin residue) and
// drift tracking is frozen while ON, so it would latch forever and the canvas
// would never fade. Force a recalibration; a real finger released after that
// recovers via the fast negative-drift path in touchScan().
static bool touchPos(int16_t *fx, int16_t *fy) {
  static bool on = false;
  static uint8_t  onCnt = 0;
  static uint16_t onFrames = 0;
  int16_t mr = 0, mc = 0;
  for (uint8_t i = 0; i < 4; i++) if (tDelta[i]     > mr) mr = tDelta[i];
  for (uint8_t i = 0; i < 4; i++) if (tDelta[4 + i] > mc) mc = tDelta[4 + i];
  int16_t m = (mr < mc) ? mr : mc;
  if (on ? (m < TOUCH_KEEP) : (m < TOUCH_THRESHOLD)) {
    on = false; onCnt = 0; onFrames = 0;
    return false;
  }
  axisBad = false;
  int16_t py = axisPos(tDelta);              // rows  -> vertical
  int16_t px = axisPos(tDelta + 4);          // cols  -> horizontal
  if (axisBad) {                             // profile says the position is
    on = false; onCnt = 0; onFrames = 0;     // not trustworthy -> no touch
    return false;                            // (better a dropped frame than a
  }                                          //  line painted across the panel)
  if (!on && ++onCnt < 2) return false;      // 2-frame confirmation
  if (++onFrames >= TOUCH_STUCK_FRAMES) {
    touchCalibrate();
    on = false; onCnt = 0; onFrames = 0;
    return false;
  }
  on = true;
  *fy = py; *fx = px;
  return true;
}
