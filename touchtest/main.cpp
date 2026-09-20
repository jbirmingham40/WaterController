// Standalone MPR121 / CST816 touch monitor.
//
// Deliberately minimal: no display, no radio, no WiFi/web portal, no relay,
// no EEPROM, no FreeRTOS task, no event queue. Everything runs in loop() so
// there is nothing to starve, block or drop. If touches do not register
// here, the cause is the MPR121, the CST816, the I2C bus or the wiring -
// not the WaterController firmware.
//
// Build and run:
//   pio run -e touchtest -t upload && pio device monitor -e touchtest
//
// Output per pad, once a second and on every event:
//   pad0 f=  14 base=  15 d= +1 peak=  2 [       ]
// f    = raw filteredData()
// base = software-tracked idle baseline
// d    = base - f (positive = classic touch direction)
// peak = largest |d| seen since the last line for that pad
//
// Commands over serial (single key + Enter, or just the key):
//   r  re-init the MPR121 (cap.begin) - does it restore sensitivity?
//   b  reset all software baselines to the current readings
//   d  dump MPR121 registers
//   s  rescan the I2C bus

#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_MPR121.h"

// Same pins as the WaterController board.
static const uint8_t I2C_SDA_PIN = 18;
static const uint8_t I2C_SCL_PIN = 8;

#define MPR121_ADDR 0x5A
#define CST816_ADDR 0x15

// Pads wired on this board.
static const uint8_t ACTIVE_PADS[] = {0, 1, 2, 3};
static const uint8_t NUM_ACTIVE_PADS = sizeof(ACTIVE_PADS) / sizeof(ACTIVE_PADS[0]);

// Detection thresholds, matching the main firmware's small-signal operating
// point (idle ~15, a working touch pulls the reading toward 1-7).
static const int16_t TOUCH_DELTA = 5;
static const int16_t RELEASE_DELTA = 3;
static const uint8_t DEBOUNCE_SAMPLES = 2;
// No real button press lasts this long; past it, assume the pad is stuck
// latched against a stale baseline rather than actually being touched.
static const uint32_t MAX_HELD_MS = 3000;
// Movement that marks a pad as "possibly being pressed". Above this the
// baseline stops adapting so the excursion can develop without being chased.
// Must sit above the idle noise (~1-3 counts) but below TOUCH_DELTA.
static const int16_t ARM_DELTA = 3;
// Cap on how long the baseline may be held still while armed, so a genuine
// environmental shift is still absorbed eventually (~2s at 20ms polls).
static const uint16_t ARM_MAX_POLLS = 100;

static const uint32_t POLL_MS = 20;

Adafruit_MPR121 cap = Adafruit_MPR121();

// Per-pad state. Baseline is held at 4x the reading for sub-count precision.
uint32_t padBaseline[12] = {0};
bool baselineReady[12] = {false};
uint16_t baselineInit[12] = {0};
bool touchedState[12] = {false};
uint8_t touchCandidate[12] = {0};
uint8_t releaseCandidate[12] = {0};
int16_t peakDelta[12] = {0};
uint16_t peakFiltered[12] = {0};
uint32_t pressCount[12] = {0};
// millis() when a pad latched, 0 when not held. Used to break out of a
// stuck HELD state - see the force-release in loop().
uint32_t heldSince[12] = {0};
// Last baseline value seen while the pad was clearly idle. A press is
// measured and released against this, never against a baseline that may
// have drifted while the finger was approaching.
uint32_t preTouchBaseline[12] = {0};
// Consecutive polls a pad has been above ARM_DELTA without latching.
uint16_t armedFor[12] = {0};

// Bus/health counters, reported once a second.
uint32_t polls = 0;
uint32_t mprFails = 0;
uint32_t cstPolls = 0, cstFails = 0, cstFingers = 0;
uint32_t mprReadUsTotal = 0, mprReadUsMax = 0, mprReadCount = 0;

bool cstPresent = false;
bool screenTouchPrev = false;
uint32_t screenWakeCount = 0;

void i2cScan() {
  Serial.println("--- I2C scan ---");
  uint8_t found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X found%s\n", addr,
                    addr == MPR121_ADDR ? "  <- MPR121"
                    : addr == CST816_ADDR ? "  <- CST816 touchscreen"
                                          : "");
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  NOTHING FOUND - check SDA/SCL/VCC/GND wiring");
  }
  Serial.println("--- end scan ---");
}

void dumpRegisters() {
  uint8_t ecr = cap.readRegister8(MPR121_ECR);
  uint8_t cfg1 = cap.readRegister8(MPR121_CONFIG1);
  uint8_t cfg2 = cap.readRegister8(MPR121_CONFIG2);
  uint8_t ac0 = cap.readRegister8(MPR121_AUTOCONFIG0);
  uint8_t ac1 = cap.readRegister8(MPR121_AUTOCONFIG1);
  uint16_t touchBits = cap.touched();

  Serial.println("--- MPR121 registers ---");
  Serial.printf("  ECR       = 0x%02X  (run=%s, electrodes=%u)\n",
                ecr, (ecr & 0x3F) ? "YES" : "NO - STOPPED", ecr & 0x0F);
  Serial.printf("  CONFIG1   = 0x%02X  (charge current bits)\n", cfg1);
  Serial.printf("  CONFIG2   = 0x%02X  (charge time / sample interval)\n", cfg2);
  Serial.printf("  AUTOCFG0  = 0x%02X   AUTOCFG1 = 0x%02X%s\n", ac0, ac1,
                (ac1 & 0x01) ? "  <-- ACFF: autoconfig FAILED" : "");
  Serial.printf("  touched() = 0x%03X\n", touchBits);
  Serial.print("  per-electrode filtered/baseline: ");
  for (uint8_t ch = 0; ch < 12; ch++) {
    Serial.printf("e%u:%u/%u ", ch, cap.filteredData(ch), cap.baselineData(ch));
  }
  Serial.println();
  Serial.println("--- end registers ---");
}

void resetBaselines() {
  for (uint8_t i = 0; i < NUM_ACTIVE_PADS; i++) {
    uint8_t pad = ACTIVE_PADS[i];
    baselineReady[pad] = false;
    baselineInit[pad] = 0;
    touchedState[pad] = false;
    touchCandidate[pad] = 0;
    releaseCandidate[pad] = 0;
    heldSince[pad] = 0;
    preTouchBaseline[pad] = 0;
    armedFor[pad] = 0;
  }
  Serial.println(">>> baselines reset");
}

bool mprInit() {
  // Autoconfig stays OFF. Enabling it on this board raises idle readings
  // from ~15 to ~710 but flattens touch response to a few counts.
  bool ok = cap.begin(MPR121_ADDR);
  Serial.printf(">>> cap.begin(0x%02X) = %s\n", MPR121_ADDR, ok ? "OK" : "FAILED");
  if (ok) {
    resetBaselines();
  }
  return ok;
}

// Polls the CST816 touchscreen the same way the main firmware does.
bool screenTouched() {
  cstPolls++;
  Wire.beginTransmission(CST816_ADDR);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission(false) != 0) {
    cstFails++;
    return false;
  }
  if (Wire.requestFrom((int)CST816_ADDR, 3) < 3) {
    cstFails++;
    return false;
  }
  uint8_t buf[3];
  for (uint8_t i = 0; i < 3; i++) {
    buf[i] = Wire.read();
  }
  if (buf[2] != 0) {
    cstFingers++;
    return true;
  }
  return false;
}

void handleSerialCommand() {
  if (!Serial.available()) {
    return;
  }
  int c = Serial.read();
  switch (c) {
    case 'r': case 'R': mprInit(); break;
    case 'b': case 'B': resetBaselines(); break;
    case 'd': case 'D': dumpRegisters(); break;
    case 's': case 'S': i2cScan(); break;
    default: break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(3000); // let USB CDC enumerate
  Serial.println();
  Serial.println("===== standalone touch monitor =====");
  Serial.println("keys: r=re-init MPR121  b=reset baselines  d=dump regs  s=I2C scan");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setTimeOut(50);

  i2cScan();

  // Wake the CST816 into normal mode, as the main firmware does.
  Wire.beginTransmission(CST816_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.write((uint8_t)0x00);
  cstPresent = (Wire.endTransmission() == 0);
  Serial.printf("CST816 touchscreen: %s\n", cstPresent ? "present" : "NOT RESPONDING");

  if (!mprInit()) {
    Serial.println("!! MPR121 not responding - check wiring and the ADD pin");
  }
  dumpRegisters();

  Serial.println();
  Serial.println("Touch the pads. A press prints immediately; a status line");
  Serial.println("per pad prints once a second showing the peak excursion.");
  Serial.println();
}

void loop() {
  handleSerialCommand();

  static uint32_t lastPoll = 0;
  if (millis() - lastPoll < POLL_MS) {
    return;
  }
  lastPoll = millis();
  polls++;

  // --- touchscreen, edge triggered ---
  if (cstPresent) {
    bool now = screenTouched();
    if (now && !screenTouchPrev) {
      screenWakeCount++;
      Serial.printf("  [SCREEN] touch #%lu\n", (unsigned long)screenWakeCount);
    }
    screenTouchPrev = now;
  }

  // --- capacitive pads ---
  for (uint8_t i = 0; i < NUM_ACTIVE_PADS; i++) {
    uint8_t pad = ACTIVE_PADS[i];

    uint32_t t0 = micros();
    uint16_t filtered = cap.filteredData(pad);
    uint32_t dt = micros() - t0;
    mprReadUsTotal += dt;
    mprReadCount++;
    if (dt > mprReadUsMax) {
      mprReadUsMax = dt;
    }

    // 0xFFFF means the I2C read itself failed.
    if (filtered == 0xFFFF) {
      mprFails++;
      continue;
    }

    if (!baselineReady[pad]) {
      padBaseline[pad] = (uint32_t)filtered << 2;
      preTouchBaseline[pad] = padBaseline[pad];
      if (++baselineInit[pad] >= 25) {
        baselineReady[pad] = true;
      }
      continue;
    }

    uint16_t baseline = (uint16_t)(padBaseline[pad] >> 2);
    int16_t delta = (int16_t)baseline - (int16_t)filtered;
    int16_t mag = delta < 0 ? (int16_t)-delta : delta;

    if (mag > peakDelta[pad]) {
      peakDelta[pad] = mag;
      peakFiltered[pad] = filtered;
    }

    if (!touchedState[pad]) {
      // Baseline tracking, with the rate chosen so a developing press is not
      // absorbed before it can latch.
      //
      // The problem this solves: a press needs DEBOUNCE_SAMPLES consecutive
      // in-window samples to latch, but while the reading is still climbing
      // toward TOUCH_DELTA the magnitude is small, so a fast-adapting
      // baseline chases the finger and shrinks the very delta being
      // measured. Rapid presses were lost that way; spacing presses out
      // worked only because the baseline had settled in between.
      //
      // Once the reading has moved at least ARM_DELTA the pad is treated as
      // "possibly being pressed" and the baseline is held still, so the
      // excursion can develop cleanly. Well below that, track normally.
      bool arming = mag >= ARM_DELTA;
      if (!arming) {
        int32_t err = (int32_t)((uint32_t)filtered << 2) - (int32_t)padBaseline[pad];
        int32_t step = err >> 6;
        if (step == 0 && err != 0) {
          step = (err > 0) ? 1 : -1;
        }
        padBaseline[pad] += step;
        // Remember the settled baseline so a latched press can be measured
        // against the pre-touch value rather than a contaminated one.
        preTouchBaseline[pad] = padBaseline[pad];
        armedFor[pad] = 0;
      } else {
        // Hold the baseline while a press develops, but not forever - a
        // genuine environmental shift must still be absorbed eventually.
        if (++armedFor[pad] > ARM_MAX_POLLS) {
          int32_t err = (int32_t)((uint32_t)filtered << 2) - (int32_t)padBaseline[pad];
          int32_t step = err >> 11;
          if (step == 0 && err != 0) {
            step = (err > 0) ? 1 : -1;
          }
          padBaseline[pad] += step;
        }
      }

      if (mag > TOUCH_DELTA) {
        if (++touchCandidate[pad] >= DEBOUNCE_SAMPLES) {
          touchedState[pad] = true;
          touchCandidate[pad] = 0;
          releaseCandidate[pad] = 0;
          pressCount[pad]++;
          heldSince[pad] = millis();
          // Latch the baseline to the last settled pre-touch value. Release
          // is judged against this, so a baseline that drifted during the
          // press cannot leave the pad stuck HELD.
          padBaseline[pad] = preTouchBaseline[pad];
          Serial.printf("  *** PAD %u PRESSED  (f=%u base=%u d=%+d)  total=%lu\n",
                        pad, filtered, (uint16_t)(padBaseline[pad] >> 2), delta,
                        (unsigned long)pressCount[pad]);
        }
      } else {
        touchCandidate[pad] = 0;
      }
    } else {
      if (mag < RELEASE_DELTA) {
        if (++releaseCandidate[pad] >= DEBOUNCE_SAMPLES) {
          touchedState[pad] = false;
          releaseCandidate[pad] = 0;
          heldSince[pad] = 0;
          armedFor[pad] = 0;
          Serial.printf("      pad %u released\n", pad);
        }
      } else {
        releaseCandidate[pad] = 0;
      }

      // Safety net: a pad must never latch forever. The baseline is frozen
      // while held, so if it was stale when the press latched, mag can stay
      // above RELEASE_DELTA indefinitely and the pad sticks in HELD with no
      // finger on it. Force a release and re-seed the baseline from the
      // current reading, which is by definition the real resting value.
      if (heldSince[pad] != 0 && millis() - heldSince[pad] > MAX_HELD_MS) {
        touchedState[pad] = false;
        releaseCandidate[pad] = 0;
        heldSince[pad] = 0;
        armedFor[pad] = 0;
        padBaseline[pad] = (uint32_t)filtered << 2;
        preTouchBaseline[pad] = padBaseline[pad];
        Serial.printf("      pad %u force-released after %lums held "
                      "(baseline re-seeded to %u)\n",
                      pad, (unsigned long)MAX_HELD_MS, filtered);
      }
    }
  }

  // --- once-a-second status ---
  static uint32_t lastReport = 0;
  if (millis() - lastReport >= 1000) {
    uint32_t elapsed = millis() - lastReport;
    lastReport = millis();

    for (uint8_t i = 0; i < NUM_ACTIVE_PADS; i++) {
      uint8_t pad = ACTIVE_PADS[i];
      uint16_t baseline = (uint16_t)(padBaseline[pad] >> 2);
      uint16_t filtered = cap.filteredData(pad);
      // A crude bar so a live excursion is visible at a glance.
      char bar[16];
      uint8_t n = peakDelta[pad] > 15 ? 15 : (uint8_t)peakDelta[pad];
      memset(bar, ' ', sizeof(bar));
      memset(bar, '#', n);
      bar[15] = '\0';
      int16_t liveDelta = (int16_t)baseline - (int16_t)filtered;
      Serial.printf("pad%u f=%4u base=%4u d=%+3d peak=%2d [%s] presses=%lu%s\n",
                    pad, filtered, baseline, liveDelta, peakDelta[pad], bar,
                    (unsigned long)pressCount[pad],
                    touchedState[pad] ? "  HELD" : "");
      peakDelta[pad] = 0;
    }

    uint32_t avgUs = mprReadCount ? (mprReadUsTotal / mprReadCount) : 0;
    Serial.printf("      bus: %lu polls/s, mpr read avg=%luus max=%luus fails=%lu | "
                  "cst polls=%lu fails=%lu fingers=%lu\n",
                  (unsigned long)(polls * 1000UL / (elapsed ? elapsed : 1)),
                  (unsigned long)avgUs, (unsigned long)mprReadUsMax,
                  (unsigned long)mprFails, (unsigned long)cstPolls,
                  (unsigned long)cstFails, (unsigned long)cstFingers);
    Serial.println();

    polls = 0;
    mprReadUsTotal = 0; mprReadUsMax = 0; mprReadCount = 0;
    cstPolls = 0; cstFails = 0; cstFingers = 0;
  }
}
