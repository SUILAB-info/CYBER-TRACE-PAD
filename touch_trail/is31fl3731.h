// Minimal IS31FL3731 driver (picture mode, frame 0) — no external library.
// Board: U2 @ I2C 0x74 (AD=GND), SDB on PA3 (100k pulldown on board).
//
// Robustness: every transaction result is tracked in is31Err. Finger contact
// is an ESD/noise source and can glitch the bus; call is31Recover() when
// is31Err accumulates (frees a slave stuck holding SDA, re-inits the driver).
#pragma once
#include <Arduino.h>
#include <Wire.h>

#define IS31_ADDR      0x74
#define IS31_BANK_FUNC 0x0B

static uint8_t is31Err = 0;   // consecutive failed transactions

static void is31EndTx() {
  if (Wire.endTransmission() != 0) { if (is31Err < 250) is31Err++; }
  else is31Err = 0;
}

static void is31Bank(uint8_t bank) {
  Wire.beginTransmission(IS31_ADDR);
  Wire.write(0xFD);            // command register: select page
  Wire.write(bank);
  is31EndTx();
}

static void is31Reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(IS31_ADDR);
  Wire.write(reg);
  Wire.write(val);
  is31EndTx();
}

// n consecutive registers set to the same value in ONE transaction. Keep n+1
// within the 32-byte Wire buffer.
static void is31Fill(uint8_t reg, uint8_t val, uint8_t n) {
  Wire.beginTransmission(IS31_ADDR);
  Wire.write(reg);
  while (n--) Wire.write(val);
  is31EndTx();
}

// Boot-time init: full configuration through a software-shutdown cycle, and
// the PWM registers cleared (after a watchdog reset the driver can still be
// displaying the pre-reset frame). Registers are written as bulk
// transactions. BOOT ONLY — never call this from the recovery path: the
// shutdown dance blanks the display for 10+ ms.
static void is31Init() {
  is31Bank(IS31_BANK_FUNC);
  is31Reg(0x0A, 0x00);         // software shutdown while configuring
  delay(10);
  is31Reg(0x00, 0x00);         // picture mode
  is31Reg(0x01, 0x00);         // display frame 0
  is31Reg(0x06, 0x00);         // audio sync off
  is31Bank(0);                 // frame 0
  is31Fill(0x00, 0xFF, 0x12);  // LED control: all 144 enabled
  is31Fill(0x12, 0x00, 0x12);  // no blink
  for (uint8_t o = 0; o < 144; o += 24) is31Fill(0x24 + o, 0x00, 24);
  is31Bank(IS31_BANK_FUNC);
  is31Reg(0x0A, 0x01);         // normal operation
  is31Bank(0);                 // leave frame 0 selected for PWM bursts
}

// Push a full 144-byte PWM frame (chunked so each I2C transaction stays small).
// The page select and the LED-control block are re-asserted EVERY frame: a
// glitched-but-ACKed write (finger ESD) can land PWM data in the LED-control
// area 0x00-0x11 and turn most scan rows off; re-asserting here bounds that
// to a single frame (~16 ms, invisible) instead of the ~0.5 s the periodic
// is31Refresh() allowed. Costs 23 bytes of I2C (~0.6 ms @ 400 kHz) per frame.
static void is31Frame(const uint8_t *pwm) {
  uint32_t t0 = millis();
  is31Bank(0);
  is31Fill(0x00, 0xFF, 0x12);             // LED control: all 144 LEDs enabled
  for (uint8_t o = 0; o < 144; o += 24) {
    // Frame time budget. Wire's timeout only aborts a stalled transaction
    // after ~10 ms (20k polling iterations, reset per byte), so a noise
    // burst could chain stalls + retries into a 10-30 ms frame — past the
    // deadline slack in fxShow, i.e. a visible random stutter. A full clean
    // frame costs ~4.6 ms; once we are late, stop streaming and never
    // retry: every register is rewritten next frame anyway, and two briefly
    // stale rows are invisible next to a time hitch.
    if ((uint8_t)(millis() - t0) >= 6) break;
    // One retry per chunk: a transient NACK otherwise drops this 24-pixel
    // slice for a whole frame and the animation stumbles.
    for (uint8_t tries = 0; tries < 2; tries++) {
      Wire.beginTransmission(IS31_ADDR);
      Wire.write(0x24 + o);
      Wire.write(pwm + o, 24);
      if (Wire.endTransmission() == 0) { is31Err = 0; break; }
      if (is31Err < 250) is31Err++;
      if ((uint8_t)(millis() - t0) >= 6) break;
    }
  }
}

// Re-assert function-page configuration that a corrupted-but-ACKed write may
// have hit (mode / display frame / shutdown). The frame-0 page select and the
// LED-control block are re-asserted every frame in is31Frame() instead.
static void is31Refresh() {
  is31Bank(IS31_BANK_FUNC);
  is31Reg(0x00, 0x00);         // picture mode
  is31Reg(0x01, 0x00);         // display frame 0
  is31Reg(0x05, 0x00);         // display option: no blink
  is31Reg(0x06, 0x00);         // audio sync off
  is31Reg(0x0A, 0x01);         // normal operation
  is31Bank(0);                 // frame 0
}

// Free a wedged bus by bit-banging: 9 SCL clocks release a slave stuck
// mid-bit holding SDA, then a manual STOP. Uses no Wire calls, so it is also
// safe (and REQUIRED) at boot BEFORE Wire.begin(): after a watchdog reset the
// IS31 can still be holding SDA from the interrupted transaction, and calling
// is31Init() on that wedged bus hangs -> watchdog -> hang again, forever.
static void i2cBusClear() {
  pinMode(PIN_PA1, INPUT_PULLUP);              // SDA released
  pinMode(PIN_PA2, OUTPUT);
  for (uint8_t i = 0; i < 9; i++) {
    digitalWrite(PIN_PA2, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_PA2, HIGH); delayMicroseconds(5);
  }
  pinMode(PIN_PA1, OUTPUT);                    // STOP: SDA low -> high
  digitalWrite(PIN_PA1, LOW);  delayMicroseconds(5);   // (while SCL high)
  pinMode(PIN_PA1, INPUT_PULLUP);              // board 4.7k pulls SDA high
  delayMicroseconds(5);
}

// Full recovery: clear the bus, re-init Wire and re-assert the driver config.
// Deliberately does NOT go through is31Init(): its software-shutdown cycle
// (display off + 10 ms settle) made every recovery a visible dark blink, and
// a noise burst that trips the error counter several times in quick
// succession turned that into rapid flicker — the "stutter got MORE frequent"
// field report. is31Refresh() re-asserts the function page with the display
// kept running, and is31Frame() rewrites LED control + PWM every frame
// anyway, so everything a glitched write can corrupt heals within a frame.
// Total cost < 1.5 ms: invisible even when it fires repeatedly.
static void is31Recover() {
  Wire.end();
  i2cBusClear();
  Wire.swap(1);
  Wire.begin();
  Wire.setClock(400000);
  // Clear the TWI master's sticky error flags (write-1-to-clear). Wire never
  // clears ARBLOST, and masterTransmit() returns TWI_ERR_BUS_ARB *before*
  // writing MADDR (which would clear it), so a single noise-induced ARBLOST
  // fails every later transaction; Wire.end()/begin() only force BUSSTATE.
  TWI0.MSTATUS = TWI_RIF_bm | TWI_WIF_bm | TWI_ARBLOST_bm | TWI_BUSERR_bm |
                 TWI_BUSSTATE_IDLE_gc;
  is31Refresh();
  is31Err = 0;
}
