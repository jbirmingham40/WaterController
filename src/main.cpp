#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <time.h>
#include <EEPROM.h>
#include "Adafruit_MPR121.h"
#include <Arduino_GFX_Library.h>
#include "WebPortal.h"

// ===================== WaterControllerV8 port: configurable settings =====================
#define MAX_BATTERY_VOLTAGE 3.3f
#define HOLE_DEPTH 12.5f
#define MAX_FILL_WITHOUT_SENSOR_TIME_MIN 15    // stop filling if sensor silent this long
#define MAX_WAIT_WITHOUT_SENSOR_BEFORE_RESTART_MIN 60 // restart if sensor silent this long
#define AUTO_RESTART_TIME_MIN 1440             // restart every 24h regardless
#define METRIC_UPDATE_FREQ_MS 60000
#define CHECK_FILLING_FREQ_MS 60000
#define SENSOR_READING_UNKNOWN 99
#define MAX_READING_SAMPLES 45
#define MAX_READING_DEVIATION 0.5f
#define CONTROLLER_NODE_ID 2 // matches WaterControllerV8.ino's CONTROLLER_NODE

// Another device is currently the active controller acking the sensor; leave
// this false until this board is the one actually deployed, to avoid two
// controllers both acking the same sensor packet.
bool radioAckEnabled = false;

// ===================== WiFi =====================
// AP provisioning, STA connect/confirm, and NTP sync are all owned by
// WebPortal (see WebPortal.h/.cpp) - it also runs the settings web server.
bool wifiConnected = false;

// ===================== Metrics (Graphite/Carbon Cache line protocol, matches WaterControllerV8) =====================
// Destination host/port are configurable from the settings web page and
// persisted in NVS by WebPortal; CARBON_CACHE_*_DEFAULT (WebPortal.h) are
// only the fallback values for a freshly-provisioned device.
WiFiClient metricsClient;

// Everything below runs on the main loop, so it must never block for long:
// a stalled loop drops MPR121 samples and makes touches unresponsive.
// The default WiFiClient connect timeout is 3s and updateMetrics() sends 7
// metrics, so an unreachable carbon host used to be able to block for >20s.
static const int32_t METRIC_CONNECT_TIMEOUT_MS = 300;
// After a failed connect, don't try again until this long has passed. Without
// it every updateMetrics() cycle pays the timeout on all 7 metrics.
static const uint32_t METRIC_RETRY_BACKOFF_MS = 60000;
static uint32_t metricNextConnectAttemptMs = 0;
// Set once per updateMetrics() cycle if the connection is unusable, so the
// remaining metrics in that cycle skip straight out instead of each retrying.
static bool metricCycleFailed = false;

// Resolve + connect if needed. Returns false (fast) when metrics can't be
// sent right now. Called at the start of each updateMetrics() cycle.
bool metricsEnsureConnected() {
  if (!wifiConnected) {
    return false;
  }
  if (metricsClient.connected()) {
    return true;
  }
  if (millis() < metricNextConnectAttemptMs) {
    return false; // still backing off from a recent failure
  }

  static String lastResolvedHost;
  static IPAddress carbonCacheIp;
  static bool carbonCacheIpResolved = false;

  String host = WebPortal::getCarbonHost();
  if (!carbonCacheIpResolved || host != lastResolvedHost) {
    // hostByName() blocks on DNS; back off on failure so a bad host doesn't
    // re-resolve every cycle.
    if (!WiFi.hostByName(host.c_str(), carbonCacheIp)) {
      Serial.println("Metrics: DNS lookup failed");
      metricNextConnectAttemptMs = millis() + METRIC_RETRY_BACKOFF_MS;
      return false;
    }
    carbonCacheIpResolved = true;
    lastResolvedHost = host;
  }

  uint16_t port = WebPortal::getCarbonPort();
  if (!metricsClient.connect(carbonCacheIp, port, METRIC_CONNECT_TIMEOUT_MS)) {
    Serial.println("Failed to connect to metrics server");
    metricNextConnectAttemptMs = millis() + METRIC_RETRY_BACKOFF_MS;
    return false;
  }
  // Bounds the retry loop in WiFiClient::write(), which otherwise blocks for
  // up to 10 x 1s against a half-open socket.
  metricsClient.setTimeout(1);
  return true;
}

void sendMetric(const char *key, float value) {
  if (metricCycleFailed || !metricsClient.connected()) {
    return;
  }
  char line[128];
  snprintf(line, sizeof(line), "%s %.2f %lu\n", key, value, (unsigned long)time(nullptr));
  if (metricsClient.print(line) == 0) {
    // Write failed - the socket is half-open. Drop it and stop trying for
    // the rest of this cycle; the next cycle will reconnect.
    Serial.println("Metrics: write failed, dropping connection");
    metricsClient.stop();
    metricCycleFailed = true;
  }
}

// ===================== Onboard I2C bus =====================
// Shared with the CST816 touch controller, QMI8658 IMU, and TCA9554 expander
static const uint8_t I2C_SDA_PIN = 18;
static const uint8_t I2C_SCL_PIN = 8;

// The touch task and the main loop both drive this bus (MPR121/CST816 from
// the task, TCA9554 relay from the loop), so every access is serialized.
// Created in setup() before the touch task starts; the lock helpers no-op
// until then so early setup() I2C work needs no special casing.
static SemaphoreHandle_t i2cMutex = nullptr;

static inline void i2cLock() {
  if (i2cMutex != nullptr) {
    xSemaphoreTake(i2cMutex, portMAX_DELAY);
  }
}

static inline void i2cUnlock() {
  if (i2cMutex != nullptr) {
    xSemaphoreGive(i2cMutex);
  }
}

// ===================== TCA9554 I2C GPIO expander (found at 0x20) =====================
// EX0 (P0) drives the water valve relay. Add more EX pins here as needed -
// all others stay inputs.
#define TCA9554_ADDR 0x20
#define TCA9554_REG_INPUT 0x00
#define TCA9554_REG_OUTPUT 0x01
#define TCA9554_REG_CONFIG 0x03
static const uint8_t RELAY_EXIO_BIT = 0; // EX0 = P0

void tca9554WriteReg(uint8_t reg, uint8_t value) {
  i2cLock();
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
  i2cUnlock();
}

void relaySet(bool on) {
  // Config register: 1=input, 0=output. Only the relay's bit is an output;
  // the rest stay inputs.
  uint8_t config = 0xFF & ~(1 << RELAY_EXIO_BIT);
  tca9554WriteReg(TCA9554_REG_CONFIG, config);
  tca9554WriteReg(TCA9554_REG_OUTPUT, on ? (1 << RELAY_EXIO_BIT) : 0x00);
}

void relayInit() {
  relaySet(false); // start with the relay off
}

// ===================== Persistent state (EEPROM) =====================
#define CURRENT_EEPROM_VERSION 105 // bumped to force freeze-protect off now that its toggle comes from an MPR121 touch pad instead of the removed TCA9554 switch
struct EepromData {
  uint8_t version;
  float preferredWaterLevel;
  uint8_t inFreezeProtect;
};
EepromData eData;

void saveEeprom() {
  EEPROM.put(0, eData);
  EEPROM.commit();
}

void loadEeprom() {
  EEPROM.begin(sizeof(EepromData));
  EEPROM.get(0, eData);
  if (eData.version != CURRENT_EEPROM_VERSION) {
    eData.version = CURRENT_EEPROM_VERSION;
    eData.preferredWaterLevel = 0.0f;
    eData.inFreezeProtect = 0;
    saveEeprom();
  }
}

// Small accessors so WebPortal.cpp can read eData's fields for /api/status
// without needing the EepromData struct definition duplicated there.
float getPreferredWaterLevel() { return eData.preferredWaterLevel; }
bool getFreezeProtectState() { return eData.inFreezeProtect; }

// ===================== Sensor/filling state (matches WaterControllerV8) =====================
float sensorVoltage = -1.0f;
float sensorPercentage = -1.0f;
float sensorWaterLevel = -1.0f;
bool completedFirstSensorReading = false;
unsigned long lastHeardFromSensorTime = millis();

bool isFilling = false;
bool fillingPaused = false; // true when checkFilling() is holding off because sensor readings look bad
unsigned long fillingStartTime = millis();
unsigned long fillingEndTime = millis();

float previousSensorReadings[MAX_READING_SAMPLES];

void startFilling() {
  relaySet(true);
  isFilling = true;
  fillingStartTime = millis();
  Serial.println("Start filling");
}

void stopFilling() {
  relaySet(false);
  isFilling = false;
  fillingEndTime = millis();
  Serial.println("Stop filling");
}

bool checkPauseFilling() {
  if (!completedFirstSensorReading) {
    return false; // assume it is working on startup
  }

  for (int i = MAX_READING_SAMPLES - 1; i > 0; i--) {
    previousSensorReadings[i] = previousSensorReadings[i - 1];
  }
  previousSensorReadings[0] = sensorWaterLevel;

  float minValue = sensorWaterLevel - MAX_READING_DEVIATION;
  float maxValue = sensorWaterLevel + MAX_READING_DEVIATION;

  for (int i = 0; i < MAX_READING_SAMPLES; i++) {
    if (previousSensorReadings[i] < 0) {
      return true; // invalid reading, pause filling
    } else if (previousSensorReadings[i] == SENSOR_READING_UNKNOWN) {
      return false; // reached unknown values, assume fine
    } else if (previousSensorReadings[i] < minValue || previousSensorReadings[i] > maxValue) {
      return true; // deviated too far from the median
    }
  }
  return false;
}

void checkFilling() {
  if (eData.inFreezeProtect) {
    startFilling();
    fillingPaused = false;
  } else if (sensorWaterLevel == -1) {
    stopFilling(); // haven't heard from sensor
    fillingPaused = false;
  } else if (checkPauseFilling()) { // has side effects (shifts previousSensorReadings) - call exactly once per checkFilling()
    stopFilling();
    fillingPaused = true;
  } else if (eData.preferredWaterLevel == 0) {
    stopFilling(); // lowest level = never fill
    fillingPaused = false;
  } else if (isFilling) {
    fillingPaused = false;
    unsigned long minsSinceHeard = (millis() - lastHeardFromSensorTime) / 60000;
    if (minsSinceHeard > MAX_FILL_WITHOUT_SENSOR_TIME_MIN) {
      stopFilling();
    } else if (sensorWaterLevel >= eData.preferredWaterLevel) {
      stopFilling();
    }
  } else {
    fillingPaused = false;
    unsigned long minsSinceHeard = (millis() - lastHeardFromSensorTime) / 60000;
    if (sensorWaterLevel < eData.preferredWaterLevel && minsSinceHeard <= MAX_FILL_WITHOUT_SENSOR_TIME_MIN) {
      startFilling();
    }
  }
}

// Toggled by touching the freeze-protect pad (see FREEZE_PROTECT_PAD), the
// same way the up/down water-level pads toggle their own state.
void toggleFreezeProtect() {
  eData.inFreezeProtect = !eData.inFreezeProtect;
  saveEeprom();
  Serial.printf("Freeze protect %s\n", eData.inFreezeProtect ? "On" : "Off");
}

void adjustDesiredWaterLevel(float delta) {
  eData.preferredWaterLevel += delta;
  if (eData.preferredWaterLevel < 0) {
    eData.preferredWaterLevel = 0;
  }
  if (eData.preferredWaterLevel > 12) {
    eData.preferredWaterLevel = 12;
  }
  saveEeprom();
  Serial.printf("Desired water level: %.1f\n", eData.preferredWaterLevel);
}

// ===================== Auto-restart safety net =====================
// Original AVR firmware used a hardware watchdog timer; this ports the
// functional behavior (restart if the sensor's gone silent too long, or
// every 24h regardless) using simple software timers instead.
unsigned long uptimeStartMs = millis();

void checkAutoRestart() {
  if (completedFirstSensorReading) {
    unsigned long minsSinceHeard = (millis() - lastHeardFromSensorTime) / 60000;
    if (minsSinceHeard > MAX_WAIT_WITHOUT_SENSOR_BEFORE_RESTART_MIN) {
      Serial.println("Restarting: haven't heard from sensor in too long");
      ESP.restart();
    }
  }
  if ((millis() - uptimeStartMs) / 60000 > AUTO_RESTART_TIME_MIN) {
    Serial.println("Restarting: scheduled 24h restart");
    ESP.restart();
  }
}

// ===================== MPR121 capacitive touch (HW-017 board: pads repurposed as desired-water-level +/- and freeze-protect toggle) =====================
static const uint32_t POLL_MS = 20;
static const uint32_t SCREEN_TIMEOUT_MS = 120000; // turn backlight off after this long with no touches

// Touch detection compares filteredData() against a per-pad baseline that is
// tracked in software (see touchTask), so environmental drift can't walk the
// signal out from under a fixed threshold the way the old hardcoded BASELINE
// constant allowed.
//
// These pads run at a small-signal operating point: idle filtered readings
// around 15, with a touch pulling them down by a handful of counts. That is
// why the original thresholds were 5/3 and why they worked. Enabling MPR121
// autoconfig moves the idle point to ~710 but flattens the touch response to
// 5-12 counts, which is worse - see mpr121Init(), autoconfig stays off.
//
// Keep these in proportion to the actual signal. If the operating point ever
// changes, re-measure with a -DMPR121_DIAGNOSTICS build rather than guessing:
// a usable pad's deflection should be several times the idle noise, which is
// about +/-3 counts here.
// Measured on this board (2026-09-19, -DSENS_TRACE, steady pressing over
// 50s): a touched pad peaks at 4-10 counts, while untouched pads never
// exceed 3. TOUCH_DELTA sits in that gap.
//
// It was previously 5, which is inside the touch range rather than below it:
// only 2 of 8 sample windows cleared it even while the pad was being pressed
// steadily. That is what made the buttons feel responsive right after boot
// and unreliable a few seconds later - a freshly seeded baseline gives the
// full deflection, and losing 1-2 counts of margin as it settles is decisive
// when the signal is only 4-6 counts. It is not a WiFi or CPU-load effect;
// measured sensitivity is the same with the radio disabled.
static const int16_t TOUCH_DELTA = 4;   // excursion from baseline to call it touched
static const int16_t RELEASE_DELTA = 2; // excursion below this to call it released
static const uint8_t DEBOUNCE_SAMPLES = 2; // consecutive in-window polls before latching

// Movement that marks a pad as "possibly being pressed". Above this the
// baseline stops adapting, so a developing press is not chased by its own
// baseline before it can latch. Must sit above the idle noise (about +/-2
// counts here) but below TOUCH_DELTA.
//
// This is what fixed presses being missed roughly 1 in 10, and missed more
// often when pressed in quick succession: a press needs DEBOUNCE_SAMPLES
// consecutive samples over TOUCH_DELTA, but while the reading was still
// climbing the excursion was small, so a fast-adapting baseline absorbed it.
// Spacing presses out used to help only because the baseline had settled
// back in between.
// Must stay below TOUCH_DELTA, or the baseline keeps adapting while a press
// is still rising and absorbs it: at ARM_DELTA 3 with TOUCH_DELTA 4, a
// simulated 5-count press was caught 0 times out of 5; at 2, all 5 latched
// with no false positives on measured idle noise.
static const int16_t ARM_DELTA = 2;
// Cap on how long the baseline may be held still while armed, so a genuine
// environmental shift is still absorbed eventually (~2s at 20ms polls).
static const uint16_t ARM_MAX_POLLS = 100;

// Baseline tracking. While a pad reads clearly idle its baseline drifts
// toward the current reading (shift of 6 => ~1/64 per 20ms poll); while a
// pad is armed or latched the baseline is held still so a press cannot be
// absorbed into it.
static const uint8_t BASELINE_SHIFT = 6;
// Recovery rate once a pad has been armed longer than ARM_MAX_POLLS. Slow
// enough that a genuine press latches long before the baseline absorbs it,
// but non-zero so a stale baseline always recovers instead of freezing.
static const uint8_t BASELINE_SHIFT_SLOW = 11;
static const uint16_t BASELINE_INIT_SAMPLES = 25; // ~0.5s of settling before arming a pad
// No real button press lasts this long; past it, assume the pad is stuck
// latched against a stale baseline rather than actually being touched.
static const uint32_t MAX_HELD_MS = 3000;

// Only these MPR121 electrodes are wired up right now; add more pad numbers
// here as additional pins get connected.
static const uint8_t ACTIVE_PADS[] = {0, 1, 2, 3};
static const uint8_t NUM_ACTIVE_PADS = sizeof(ACTIVE_PADS) / sizeof(ACTIVE_PADS[0]);
static const uint8_t FREEZE_PROTECT_PAD = 2; // touching this pad toggles freeze protect on/off
static const uint8_t WIFI_RESET_PAD = 3;     // double-press within the window below resets WiFi
static const uint32_t WIFI_RESET_CONFIRM_WINDOW_MS = 5000;

// 0 = no reset pending. Set to millis()+WIFI_RESET_CONFIRM_WINDOW_MS on the
// first press of WIFI_RESET_PAD; a second press before this expires confirms
// the reset. Cleared back to 0 (no action taken) once the window elapses.
uint32_t wifiResetPendingUntilMs = 0;

// Pin 0 increases the desired water level, pin 1 decreases it (0.1in per touch)
float padLevelDelta[12] = {0};

Adafruit_MPR121 cap = Adafruit_MPR121();
bool touchedState[12] = {false};
uint8_t touchCandidate[12] = {0};
uint8_t releaseCandidate[12] = {0};
// Software baseline per pad, held at 4x the filtered reading so the slow
// drift below keeps sub-count precision. baselineReady gates detection until
// a pad has seen BASELINE_INIT_SAMPLES settling samples.
uint32_t padBaseline[12] = {0};
bool baselineReady[12] = {false};
uint16_t baselineInitCount[12] = {0};
// Last baseline seen while the pad was clearly idle. A press is measured and
// released against this, never against a baseline that may have drifted
// while the finger was approaching - which is what used to leave pads stuck
// reporting touched with nothing on them.
uint32_t preTouchBaseline[12] = {0};
// Consecutive polls a pad has read above ARM_DELTA without latching.
uint16_t armedFor[12] = {0};
// millis() when a pad latched, 0 when not held. Backstop against a pad
// latching forever if its baseline was stale when the press landed.
uint32_t heldSince[12] = {0};
#ifdef SENS_TRACE
// Peak deflection per pad within the current reporting window.
int16_t sensPeak[12] = {0};
uint32_t sensLatches[12] = {0};
#endif

bool screenOn = true;
uint32_t lastActivityMs = 0;

// ===================== Touch sampling task =====================
// Sampling used to live in loop(), which meant any blocking call there (the
// metrics TCP connect was the worst offender) starved the debouncer: a press
// only latches after DEBOUNCE_SAMPLES *consecutive* in-window samples, and a
// single missed poll resets the counter. Sampling now runs in its own task so
// loop() stalls can no longer drop samples.
//
// The task only samples and debounces. Acting on a press (relay via the
// TCA9554, EEPROM writes, display SPI) stays on the main loop, which it
// reaches through touchEventQueue - the relay shares this I2C bus and the
// display isn't thread-safe.
static QueueHandle_t touchEventQueue = nullptr;
static TaskHandle_t touchTaskHandle = nullptr;
static const uint32_t TOUCH_TASK_STACK = 3072;
static const UBaseType_t TOUCH_TASK_PRIORITY = 2; // above loopTask (priority 1)

// Most recent filtered reading per pad, and the tracked baseline it is being
// compared against. Both are surfaced on /api/status so the pads' margin can
// be watched live instead of inferred from whether presses happen to work.
volatile uint16_t lastFiltered[12] = {0};
volatile uint16_t lastBaseline[12] = {0};

// Consecutive failed MPR121 reads. A shared I2C bus can wedge (the CST816,
// TCA9554 relay and MPR121 all live on it), and a wedged bus used to mean
// every read returned 0xFFFF forever with only a power cycle to recover.
// After I2C_FAIL_LIMIT consecutive failures the bus and the MPR121 are
// re-initialised in place.
static const uint16_t I2C_FAIL_LIMIT = 50; // ~1s at POLL_MS
volatile uint16_t touchI2cFailStreak = 0;
volatile uint32_t touchI2cRecoveryCount = 0;

// A screen touch (CST816) carries no pad, so it uses this sentinel to mean
// "wake the display only".
static const uint8_t TOUCH_EVENT_SCREEN = 0xFF;

// Brings up the MPR121 exactly as the long-working version of this firmware
// did: autoconfig OFF, library defaults.
//
// Autoconfig was tried on 2026-09-19 and made things worse. It reports
// success (no ACFF) and lifts idle filtered readings from ~15 to ~710, but
// the charge settings it picks leave almost no touch sensitivity: measured
// deflection fell to 5-12 counts, so presses stopped registering. With
// autoconfig off the pads return to their original small-signal operating
// point, where the touch/release deltas below are sized to work.
// Callers must not hold the I2C lock.
bool mpr121Init() {
  i2cLock();
  bool ok = cap.begin(0x5A);
  i2cUnlock();
  return ok;
}

#ifdef MPR121_DIAGNOSTICS
// Continuous MPR121 monitor across ALL 12 channels.
//
// Reading this output: a touch should pull an electrode's filtered value
// DOWN by clearly more than the idle noise. On this board the pads run at a
// small-signal point (idle ~15 with autoconfig off), so judge deflection
// relative to the idle jitter, not against an absolute count.
//
// History worth knowing: enabling autoconfig on 2026-09-19 raised the idle
// readings to ~710 but flattened touch deflection to 5-12 counts, and the
// buttons largely stopped responding. Electrodes 0-3 are wired and working;
// an earlier reading of this output wrongly concluded they were unconnected
// because only a 10s window was sampled. All 12 channels are watched here
// since ACTIVE_PADS assumes 0-3.
//
// AUTOCONFIG1 bit0 (ACFF) flags an autoconfig failure, in which case the
// per-pad charge settings fell back to defaults.
// Sized for the small-signal operating point (idle ~15, noise ~+/-1).
// Raise this if the pads are ever run at a higher idle point.
static const int16_t DIAG_NOTE_DELTA = 3; // report excursions beyond this

void mpr121DumpRegisters() {
  i2cLock();
  uint8_t ecr = cap.readRegister8(MPR121_ECR);
  uint8_t cfg1 = cap.readRegister8(MPR121_CONFIG1);
  uint8_t cfg2 = cap.readRegister8(MPR121_CONFIG2);
  uint8_t ac0 = cap.readRegister8(MPR121_AUTOCONFIG0);
  uint8_t ac1 = cap.readRegister8(MPR121_AUTOCONFIG1);
  uint16_t touchBits = cap.touched();
  i2cUnlock();

  Serial.println("--- MPR121 diagnostics (all 12 channels) ---");
  Serial.printf("ECR=0x%02X (run=%s, electrodes=%u)  CONFIG1=0x%02X  CONFIG2=0x%02X\n",
                ecr, (ecr & 0x3F) ? "yes" : "NO", ecr & 0x0F, cfg1, cfg2);
  Serial.printf("AUTOCONFIG0=0x%02X AUTOCONFIG1=0x%02X%s\n", ac0, ac1,
                (ac1 & 0x01) ? "  <-- ACFF: AUTOCONFIG FAILED" : "");
  Serial.printf("touched() bitmap=0x%03X\n", touchBits);

  Serial.print("idle filtered: ");
  for (uint8_t ch = 0; ch < 12; ch++) {
    i2cLock();
    uint16_t f = cap.filteredData(ch);
    uint16_t b = cap.baselineData(ch);
    i2cUnlock();
    Serial.printf("e%u:%u/%u ", ch, f, b);
  }
  Serial.println();
}

// Never returns. Runs the live monitor as the sole activity so nothing else
// on the loop can interfere with or delay the sampling.
void mpr121MonitorForever() {
  uint16_t settled[12];
  uint16_t lo[12], hi[12];
  bool reported[12] = {false};

  for (uint8_t ch = 0; ch < 12; ch++) {
    i2cLock();
    uint16_t f = cap.filteredData(ch);
    i2cUnlock();
    settled[ch] = f;
    lo[ch] = f;
    hi[ch] = f;
  }

  Serial.println("Monitoring all 12 electrodes. Touch each pad in turn;");
  Serial.println("excursions are printed immediately, summary every 5s.");
  Serial.println("Press the pads you expect to be Level+/Level-/Freeze/WiFi.");

  uint32_t lastSummary = millis();
  uint16_t prevTouch = 0;

  for (;;) {
    i2cLock();
    uint16_t touchBits = cap.touched();
    i2cUnlock();
    if (touchBits != prevTouch) {
      Serial.printf("[touched() changed: 0x%03X]\n", touchBits);
      prevTouch = touchBits;
    }

    for (uint8_t ch = 0; ch < 12; ch++) {
      i2cLock();
      uint16_t f = cap.filteredData(ch);
      i2cUnlock();
      if (f == 0xFFFF) {
        continue;
      }
      if (f < lo[ch]) lo[ch] = f;
      if (f > hi[ch]) hi[ch] = f;

      int16_t delta = (int16_t)settled[ch] - (int16_t)f;
      if (delta > DIAG_NOTE_DELTA || delta < -DIAG_NOTE_DELTA) {
        Serial.printf("  >>> e%u MOVED: %u -> %u (delta %+d)\n",
                      ch, settled[ch], f, -delta);
        reported[ch] = true;
      }
    }

    if (millis() - lastSummary >= 5000) {
      lastSummary = millis();
      Serial.print("summary min/max/span: ");
      for (uint8_t ch = 0; ch < 12; ch++) {
        uint16_t span = hi[ch] - lo[ch];
        Serial.printf("e%u:%u-%u(%u)%s ", ch, lo[ch], hi[ch], span,
                      reported[ch] ? "*" : "");
      }
      Serial.println();
    }

    delay(50);
  }
}
#endif // MPR121_DIAGNOSTICS

// Drops and re-opens the shared I2C bus, then re-inits the MPR121. Called
// from the touch task when reads have been failing long enough to look like
// a wedged bus rather than an isolated NAK.
void i2cRecover() {
  i2cLock();
  Wire.end();
  delay(5);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setTimeOut(50);
  i2cUnlock();
  mpr121Init();
  touchI2cRecoveryCount = touchI2cRecoveryCount + 1;
  Serial.println("I2C: bus wedged, re-initialised MPR121");
}

// ===================== Onboard CST816 touchscreen (used only to wake the display) =====================
// Its IRQ/RST pins are not wired on this board per Waveshare's docs, so it
// has to be polled over I2C like the MPR121. Register map per Waveshare's
// own touch_bsp.c: burst-read 7 bytes from reg 0x00; byte[2] is finger count.
#define CST816_ADDR 0x15

#ifdef TOUCH_TRACE
// I2C bus health counters. The MPR121 pads and this CST816 touchscreen are
// separate chips sharing only the I2C bus, so both degrading at once points
// at the bus rather than at either chip's electrodes.
volatile uint32_t cstPolls = 0, cstAddrFail = 0, cstShortRead = 0, cstFingers = 0;
// MPR121 read timing. A bus that has gone slow (clock stretching, marginal
// pull-ups, noise causing retries) throttles the sample rate the debouncer
// depends on, which looks exactly like insensitive pads.
volatile uint32_t mprReads = 0, mprReadUsTotal = 0, mprReadUsMax = 0;
volatile uint32_t touchLoopIters = 0;
#endif

bool screenTouchDetected() {
  i2cLock();
#ifdef TOUCH_TRACE
  cstPolls++;
#endif
  Wire.beginTransmission(CST816_ADDR);
  Wire.write((uint8_t)0x00);
  uint8_t txResult = Wire.endTransmission(false);
  if (txResult != 0) {
#ifdef TOUCH_TRACE
    cstAddrFail++;
#endif
    i2cUnlock();
    return false;
  }
  uint8_t n = Wire.requestFrom((int)CST816_ADDR, 3);
  if (n < 3) {
#ifdef TOUCH_TRACE
    cstShortRead++;
#endif
    i2cUnlock();
    return false;
  }
  uint8_t buf[3];
  for (uint8_t i = 0; i < 3; i++) {
    buf[i] = Wire.read();
  }
  i2cUnlock();
#ifdef TOUCH_TRACE
  if (buf[2] != 0) {
    cstFingers++;
  }
#endif
  return buf[2] != 0;
}

// ===================== RFM69HWCW radio =====================
// Wired to the only free GPIOs left on the header. The ESP32-C6 has exactly
// one general-purpose hardware SPI peripheral and the LCD already owns it on
// a different set of pins, so this bus is bit-banged in software rather than
// fighting over the one hardware SPI controller.
static const uint8_t RFM_SCLK = 1;
static const uint8_t RFM_MISO = 2;
static const uint8_t RFM_MOSI = 22;
static const uint8_t RFM_CS = 23;
static const uint8_t RFM_RST = 9;

#define RFM_REG_FIFO 0x00
#define RFM_REG_OPMODE 0x01
#define RFM_REG_DATAMODUL 0x02
#define RFM_REG_BITRATEMSB 0x03
#define RFM_REG_BITRATELSB 0x04
#define RFM_REG_FDEVMSB 0x05
#define RFM_REG_FDEVLSB 0x06
#define RFM_REG_FRFMSB 0x07
#define RFM_REG_FRFMID 0x08
#define RFM_REG_FRFLSB 0x09
#define RFM_REG_VERSION 0x10
#define RFM_REG_RXBW 0x19
#define RFM_REG_IRQFLAGS1 0x27
#define RFM_REG_IRQFLAGS2 0x28
#define RFM_REG_RSSIVALUE 0x24
#define RFM_REG_PREAMBLEMSB 0x2C
#define RFM_REG_PREAMBLELSB 0x2D
#define RFM_REG_SYNCCONFIG 0x2E
#define RFM_REG_SYNCVALUE1 0x2F
#define RFM_REG_SYNCVALUE2 0x30
#define RFM_REG_RSSITHRESH 0x29
#define RFM_REG_PACKETCONFIG1 0x37
#define RFM_REG_PAYLOADLENGTH 0x38
#define RFM_REG_FIFOTHRESH 0x3C
#define RFM_REG_PACKETCONFIG2 0x3D
#define RFM_REG_AESKEY1 0x3E
#define RFM_REG_TESTDAGC 0x6F

#define RFM_MODE_SLEEP 0x00
#define RFM_MODE_STANDBY 0x04
#define RFM_MODE_TX 0x0C
#define RFM_MODE_RX 0x10

#define RFM_CTL_SENDACK 0x80
#define RFM_CTL_REQACK 0x40
#define RFM_IRQFLAGS1_MODEREADY 0x80
#define RFM_IRQFLAGS2_PACKETSENT 0x08

// Bit-banged SPI mode 0 (idle clock low, sample on rising edge)
uint8_t rfmTransfer(uint8_t out) {
  uint8_t in = 0;
  for (int8_t bit = 7; bit >= 0; bit--) {
    digitalWrite(RFM_MOSI, (out >> bit) & 0x01);
    delayMicroseconds(2);
    digitalWrite(RFM_SCLK, HIGH);
    delayMicroseconds(2);
    in = (in << 1) | digitalRead(RFM_MISO);
    digitalWrite(RFM_SCLK, LOW);
    delayMicroseconds(2);
  }
  return in;
}

uint8_t rfmReadReg(uint8_t addr) {
  digitalWrite(RFM_CS, LOW);
  rfmTransfer(addr & 0x7F);
  uint8_t val = rfmTransfer(0x00);
  digitalWrite(RFM_CS, HIGH);
  return val;
}

void rfmWriteReg(uint8_t addr, uint8_t value) {
  digitalWrite(RFM_CS, LOW);
  rfmTransfer(addr | 0x80);
  rfmTransfer(value);
  digitalWrite(RFM_CS, HIGH);
}

void rfmSetMode(uint8_t mode) {
  rfmWriteReg(RFM_REG_OPMODE, (rfmReadReg(RFM_REG_OPMODE) & 0xE3) | mode);
}

bool rfmReady = false;

bool rfmInit() {
  pinMode(RFM_SCLK, OUTPUT);
  pinMode(RFM_MOSI, OUTPUT);
  pinMode(RFM_MISO, INPUT);
  pinMode(RFM_CS, OUTPUT);
  pinMode(RFM_RST, OUTPUT);
  digitalWrite(RFM_SCLK, LOW);
  digitalWrite(RFM_CS, HIGH);

  // Hardware reset: RST high >=100us, then low and wait for the chip to boot
  digitalWrite(RFM_RST, HIGH);
  delayMicroseconds(150);
  digitalWrite(RFM_RST, LOW);
  delay(10);

  uint8_t version = rfmReadReg(RFM_REG_VERSION);
  if (version != 0x24) {
    Serial.printf("RFM69 not found (VERSION reg read 0x%02X, expected 0x24)\n", version);
    return false;
  }

  rfmWriteReg(RFM_REG_OPMODE, RFM_MODE_STANDBY);

  // FSK, no shaping, packet mode
  rfmWriteReg(RFM_REG_DATAMODUL, 0x00);

  // Bitrate 55555bps, matching LowPowerLab RFM69 library's actual default
  // register values (RF_BITRATEMSB/LSB_55555 = 0x02,0x40 - its own code
  // comment claims "4.8kbps" but that's stale; the real value is 55555bps)
  rfmWriteReg(RFM_REG_BITRATEMSB, 0x02);
  rfmWriteReg(RFM_REG_BITRATELSB, 0x40);

  // Frequency deviation 50kHz, matching LowPowerLab's RF_FDEVMSB/LSB_50000
  rfmWriteReg(RFM_REG_FDEVMSB, 0x03);
  rfmWriteReg(RFM_REG_FDEVLSB, 0x33);

  // Carrier frequency 915.0MHz: 915000000 / 61.03515625 = 14991360 = 0xE4C000
  rfmWriteReg(RFM_REG_FRFMSB, 0xE4);
  rfmWriteReg(RFM_REG_FRFMID, 0xC0);
  rfmWriteReg(RFM_REG_FRFLSB, 0x00);

  // RX bandwidth 125kHz, matching LowPowerLab's RXBW (Mant=16, Exp=2)
  rfmWriteReg(RFM_REG_RXBW, 0x42);

  // 3-byte preamble — LowPowerLab's init doesn't touch this register, so it
  // stays at the chip's own power-on-reset default
  rfmWriteReg(RFM_REG_PREAMBLEMSB, 0x00);
  rfmWriteReg(RFM_REG_PREAMBLELSB, 0x03);

  // 2-byte sync word {0x2D, networkID}. The transmitter calls
  // radio.initialize(RF69_915MHZ, CONTROLLER_NODE, 0) -> networkID=0
  rfmWriteReg(RFM_REG_SYNCCONFIG, 0x88);
  rfmWriteReg(RFM_REG_SYNCVALUE1, 0x2D);
  rfmWriteReg(RFM_REG_SYNCVALUE2, 0x00);

  // Variable-length packets with CRC enabled, no address filtering
  rfmWriteReg(RFM_REG_PACKETCONFIG1, 0x90);
  rfmWriteReg(RFM_REG_PAYLOADLENGTH, 66); // matches LowPowerLab's max frame size
  rfmWriteReg(RFM_REG_FIFOTHRESH, 0x8F);
  rfmWriteReg(RFM_REG_RSSITHRESH, 220);
  rfmWriteReg(RFM_REG_TESTDAGC, 0x30); // improved fading margin, matches LowPowerLab

  // AES-128 enabled with the transmitter's key: radio.encrypt("TOPSECRETPASSWRD")
  const char aesKey[] = "TOPSECRETPASSWRD"; // 16 chars + null terminator
  digitalWrite(RFM_CS, LOW);
  rfmTransfer(RFM_REG_AESKEY1 | 0x80);
  for (uint8_t i = 0; i < 16; i++) {
    rfmTransfer((uint8_t)aesKey[i]);
  }
  digitalWrite(RFM_CS, HIGH);
  rfmWriteReg(RFM_REG_PACKETCONFIG2, 0x10 | 0x01); // RxRestartDelay=2bits, AES on

  rfmSetMode(RFM_MODE_RX);

  Serial.println("RFM69 ready, listening at 915.0MHz");
  return true;
}

// Sends a LowPowerLab-compatible ACK frame (3-byte header, no payload) back
// to the given node, then resumes listening. Only called when
// radioAckEnabled is true.
void rfmSendAck(uint8_t toAddress) {
  // Avoid RX deadlock, same as LowPowerLab's sendACK()
  rfmWriteReg(RFM_REG_PACKETCONFIG2, (rfmReadReg(RFM_REG_PACKETCONFIG2) & 0xFB) | 0x04);

  rfmSetMode(RFM_MODE_STANDBY);
  uint32_t start = millis();
  while (!(rfmReadReg(RFM_REG_IRQFLAGS1) & RFM_IRQFLAGS1_MODEREADY) && millis() - start < 50) {
    delay(1);
  }

  digitalWrite(RFM_CS, LOW);
  rfmTransfer(RFM_REG_FIFO | 0x80);
  rfmTransfer(3); // header-only frame: bufferSize(0) + 3 header bytes
  rfmTransfer(toAddress);
  rfmTransfer(CONTROLLER_NODE_ID);
  rfmTransfer(RFM_CTL_SENDACK);
  digitalWrite(RFM_CS, HIGH);

  rfmSetMode(RFM_MODE_TX);
  start = millis();
  while (!(rfmReadReg(RFM_REG_IRQFLAGS2) & RFM_IRQFLAGS2_PACKETSENT) && millis() - start < 100) {
    delay(1);
  }

  rfmSetMode(RFM_MODE_STANDBY);
  rfmSetMode(RFM_MODE_RX);
}

// Matches the sender's PayloadStruct (WaterControllerV8.ino)
struct WaterPayload {
  float waterLevel;
  float batteryVoltage;
};

void updateMetrics();

void handleReceivedPacket(uint8_t senderId, uint8_t ctl, const WaterPayload &payload) {
  if (payload.batteryVoltage < 0 || payload.batteryVoltage > MAX_BATTERY_VOLTAGE ||
      payload.waterLevel < -1 || payload.waterLevel > HOLE_DEPTH) {
    Serial.println("Last sensor reading was bad. Ignoring.");
    return;
  }

  lastHeardFromSensorTime = millis();
  completedFirstSensorReading = true;
  sensorVoltage = payload.batteryVoltage;
  sensorPercentage = min((sensorVoltage / MAX_BATTERY_VOLTAGE) * 100.0f, 100.0f);
  sensorWaterLevel = payload.waterLevel;
  Serial.printf("Received an update from the sensor. voltage=%.2f waterLevel=%.2f\n",
                sensorVoltage, sensorWaterLevel);

  if (radioAckEnabled && (ctl & RFM_CTL_REQACK)) {
    rfmSendAck(senderId);
    Serial.printf("Sent ACK to node %d\n", senderId);
  }

  updateMetrics();
  checkFilling();
}

void updateMetrics() {
  // One connect attempt per cycle, not one per metric.
  metricCycleFailed = !metricsEnsureConnected();
  if (metricCycleFailed) {
    return;
  }
  sendMetric("autofill.sensor.battery.voltage", sensorVoltage);
  sendMetric("autofill.sensor.battery.percentage", sensorPercentage);
  sendMetric("autofill.sensor.water_level", sensorWaterLevel);
  sendMetric("autofill.sensor.filling", isFilling ? 1.0f : 0.0f);
  sendMetric("autofill.controller.sensor_last_heard", (millis() - lastHeardFromSensorTime) / 1000.0f);
  sendMetric("autofill.controller.freeze_protect", eData.inFreezeProtect ? 1.0f : 0.0f);
  sendMetric("autofill.controller.desired_water_level", eData.preferredWaterLevel);
}

// Polls IRQFLAGS2's PayloadReady bit instead of using DIO0, since that pin
// isn't wired up.
void rfmPollReceive() {
  if (!rfmReady) {
    return;
  }

  uint8_t irqflags2 = rfmReadReg(RFM_REG_IRQFLAGS2);
  if (!(irqflags2 & 0x04)) {
    return;
  }

  int8_t rssi = -(int8_t)(rfmReadReg(RFM_REG_RSSIVALUE) / 2);

  digitalWrite(RFM_CS, LOW);
  rfmTransfer(RFM_REG_FIFO & 0x7F);
  uint8_t len = rfmTransfer(0x00);
  if (len > 66) {
    len = 66;
  }
  uint8_t buf[66];
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = rfmTransfer(0x00);
  }
  digitalWrite(RFM_CS, HIGH);

  if (len == 0) {
    return;
  }

  Serial.printf("RFM69 packet: len=%d rssi=%ddBm data=", len, rssi);
  for (uint8_t i = 0; i < len; i++) {
    Serial.printf("%02X ", buf[i]);
  }
  Serial.println();

  // LowPowerLab header: [targetID, senderID, CTLbyte], then the payload
  static const uint8_t HEADER_LEN = 3;
  if (len >= HEADER_LEN + sizeof(WaterPayload)) {
    uint8_t senderId = buf[1];
    uint8_t ctl = buf[2];

    WaterPayload payload;
    memcpy(&payload, &buf[HEADER_LEN], sizeof(payload));

    handleReceivedPacket(senderId, ctl, payload);
  }
}

// ===================== Display (GFX) =====================
// LCD pins/offsets per Waveshare's own ESP32-C6-Touch-LCD-1.9 GFX example.
// rotation=3 turns the 170x320 portrait panel into a 320x170 landscape view,
// flipped 180 degrees from rotation=1.
#define GFX_BL 15
Arduino_DataBus *bus = new Arduino_HWSPI(6 /* DC */, 7 /* CS */, 5 /* SCK */, 4 /* MOSI */);
Arduino_GFX *gfx = new Arduino_ST7789(bus, 14 /* RST */, 3 /*rotation*/, 1 /*IPS*/,
                                      170 /*w*/, 320 /*h*/, 35, 0, 35, 0);

// ---- Dashboard layout (320x170 landscape): a title bar, a water-tank gauge
// on the left, and a column of stat rows/badges on the right. ----
#define TITLEBAR_H 22
#define TANK_X 8
#define TANK_Y (TITLEBAR_H + 4)
#define TANK_W 64
#define TANK_H 120
#define PANEL_X (TANK_X + TANK_W + 12)
#define PANEL_W (320 - PANEL_X - 4)
#define ROW_H 20
#define ROW_Y(n) (TITLEBAR_H + 4 + (n) * ROW_H)
// Battery row layout, left to right: WiFi icon, Yes/No text, a wider gap
// (so the two metrics read as visually distinct), then the battery icon+voltage.
#define WIFI_ICON_X PANEL_X
#define WIFI_TEXT_X (WIFI_ICON_X + 17)
#define WIFI_TEXT_CHARS 3
#define BATTERY_ICON_X (WIFI_TEXT_X + WIFI_TEXT_CHARS * 12 + 28)

uint16_t batteryColor(float percentage) {
  if (percentage > 50) return RGB565_GREEN;
  if (percentage >= 20) return RGB565_YELLOW;
  return RGB565_RED;
}

// Draws opaque text (fg over bg in one pass, via the glcd font's per-pixel
// background fill) padded/truncated to a fixed character count, so a shorter
// new string fully overwrites a longer old one without a separate black-fill
// "erase" step first. That preceding erase-then-redraw was what caused every
// row to visibly flash black once a second, regardless of whether its value
// had actually changed.
void printPadded(int x, int y, uint16_t fg, uint16_t bg, int chars, const char *text) {
  char buf[24];
  snprintf(buf, sizeof(buf), "%-*.*s", chars, chars, text);
  gfx->setTextSize(2);
  gfx->setTextColor(fg, bg);
  gfx->setCursor(x, y);
  gfx->print(buf);
}

// Fixed-width status chip: the label stays constant and only its color
// changes with state, so text never needs re-centering for a new string length.
void drawBadge(int x, int y, int w, int h, const char *label, uint16_t bgColor, uint16_t fgColor) {
  gfx->fillRoundRect(x, y, w, h, 4, bgColor);
  gfx->setTextSize(2);
  gfx->setTextColor(fgColor, bgColor);
  int textW = strlen(label) * 12;
  gfx->setCursor(x + (w - textW) / 2, y + (h - 16) / 2);
  gfx->print(label);
}

// Geometry that never changes shape or position - only its fill color/size
// does - gets drawn once here instead of every refresh: the title bar
// background+label, the tank's outer border, the desired-level triangle
// marker, and the battery icon's outline+nub.
void drawDashboardChrome() {
  gfx->fillRect(0, 0, gfx->width(), TITLEBAR_H, RGB565_TEAL);
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_WHITE, RGB565_TEAL);
  gfx->setCursor(4, 3);
  gfx->print("AUTOFILL");

  gfx->drawRect(TANK_X, TANK_Y, TANK_W, TANK_H, RGB565_WHITE);

  int yDesired = ROW_Y(2);
  gfx->fillTriangle(PANEL_X, yDesired + 4, PANEL_X, yDesired + 14, PANEL_X + 8, yDesired + 9, RGB565_YELLOW);

  int yBatt = ROW_Y(0);
  const int bodyX = BATTERY_ICON_X, bodyY = yBatt + 4, bodyW = 28, bodyH = 12;
  gfx->drawRect(bodyX, bodyY, bodyW, bodyH, RGB565_WHITE);
  gfx->fillRect(bodyX + bodyW, bodyY + 3, 3, bodyH - 6, RGB565_WHITE);
}

void drawBatteryRow(int y) {
  static float lastVoltage = -999, lastPercentage = -999;
  if (sensorVoltage == lastVoltage && sensorPercentage == lastPercentage) {
    return; // unchanged since last redraw - nothing to repaint
  }
  lastVoltage = sensorVoltage;
  lastPercentage = sensorPercentage;

  const int bodyX = BATTERY_ICON_X, bodyY = y + 4, bodyW = 28, bodyH = 12;
  const int textX = bodyX + bodyW + 8;

  if (sensorVoltage < 0) {
    gfx->fillRect(bodyX + 2, bodyY + 2, bodyW - 4, bodyH - 4, RGB565_DARKGREY);
    printPadded(textX, y + 2, RGB565_GRAY, RGB565_BLACK, 8, "?");
    return;
  }

  uint16_t color = batteryColor(sensorPercentage);
  int fillW = (int)((bodyW - 4) * (sensorPercentage / 100.0f));
  gfx->fillRect(bodyX + 2, bodyY + 2, bodyW - 4, bodyH - 4, RGB565_DARKGREY); // clear the old fill width first
  gfx->fillRect(bodyX + 2, bodyY + 2, fillW, bodyH - 4, color);

  char buf[16];
  snprintf(buf, sizeof(buf), "%.2fV", sensorVoltage);
  printPadded(textX, y + 2, RGB565_WHITE, RGB565_BLACK, 8, buf);
}

void drawWaterRow(int y) {
  static float lastWaterLevel = -999;
  if (sensorWaterLevel == lastWaterLevel) {
    return;
  }
  lastWaterLevel = sensorWaterLevel;

  char buf[24];
  if (sensorWaterLevel < 0) {
    printPadded(PANEL_X, y + 2, RGB565_GRAY, RGB565_BLACK, 19, "Water: ?");
  } else {
    snprintf(buf, sizeof(buf), "Water: %.2f in", sensorWaterLevel);
    printPadded(PANEL_X, y + 2, RGB565_CYAN, RGB565_BLACK, 19, buf);
  }
}

void drawDesiredRow(int y) {
  static float lastDesired = -999;
  if (eData.preferredWaterLevel == lastDesired) {
    return;
  }
  lastDesired = eData.preferredWaterLevel;

  char buf[24];
  snprintf(buf, sizeof(buf), "Desired: %.1f in", eData.preferredWaterLevel);
  printPadded(PANEL_X + 14, y + 2, RGB565_YELLOW, RGB565_BLACK, 17, buf);
}

void drawFillingPausedRow(int y) {
  static int8_t lastFilling = -1, lastPaused = -1;

  if ((int8_t)isFilling != lastFilling) {
    drawBadge(PANEL_X, y, 96, 18, "FILLING", isFilling ? RGB565_GREEN : RGB565_DARKGREY,
              isFilling ? RGB565_BLACK : RGB565_LIGHTGREY);
    lastFilling = isFilling;
  }
  if ((int8_t)fillingPaused != lastPaused) {
    drawBadge(PANEL_X + 104, y, 90, 18, "PAUSED", fillingPaused ? RGB565_RED : RGB565_DARKGREY,
              fillingPaused ? RGB565_WHITE : RGB565_LIGHTGREY);
    lastPaused = fillingPaused;
  }
}

void drawFreezeRow(int y) {
  static int8_t lastFreeze = -1;
  if ((int8_t)eData.inFreezeProtect == lastFreeze) {
    return;
  }
  lastFreeze = eData.inFreezeProtect;
  drawBadge(PANEL_X, y, 96, 18, "FREEZE", eData.inFreezeProtect ? RGB565_CYAN : RGB565_DARKGREY,
            eData.inFreezeProtect ? RGB565_BLACK : RGB565_LIGHTGREY);
}

// The device's own address, in the strip below the stat rows - this is how you
// find the settings portal without a serial console. Shares its footprint with
// the WiFi-reset banner (which takes precedence while pending), so both track
// footerNeedsRedraw to know when the other has clobbered them.
bool footerNeedsRedraw = true;

void drawIpFooter() {
  static char lastText[24] = "";

  // The banner owns this strip while it's pending; drawing under it would just
  // be overpainted every second.
  if (wifiResetPendingUntilMs != 0 && millis() < wifiResetPendingUntilMs) {
    return;
  }

  char text[24];
  if (wifiConnected) {
    snprintf(text, sizeof(text), "%s", WiFi.localIP().toString().c_str());
  } else if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
    snprintf(text, sizeof(text), "AP %s", WiFi.softAPIP().toString().c_str());
  } else {
    snprintf(text, sizeof(text), "no network");
  }

  if (!footerNeedsRedraw && strcmp(text, lastText) == 0) {
    return; // same address, and nothing has painted over us
  }
  strncpy(lastText, text, sizeof(lastText) - 1);
  lastText[sizeof(lastText) - 1] = '\0';
  footerNeedsRedraw = false;

  // 15 chars covers the widest case: "AP 192.168.4.1".
  const int chars = 15;
  int footerY = ROW_Y(6);
  int footerH = gfx->height() - footerY;
  printPadded(4, footerY + (footerH - 16) / 2, RGB565_GRAY, RGB565_BLACK, chars, text);
}

// First press of WIFI_RESET_PAD shows this banner in the otherwise-unused
// strip below the stat rows; a second press before it clears confirms the
// reset. Only redraws on a shown/hidden transition (not every second) since
// nothing else ever draws in this footprint.
void drawWifiResetBanner() {
  static bool lastShown = false;
  bool shown = wifiResetPendingUntilMs != 0 && millis() < wifiResetPendingUntilMs;
  if (shown == lastShown) {
    return;
  }
  lastShown = shown;

  int bannerY = ROW_Y(6);
  int bannerH = gfx->height() - bannerY;
  if (!shown) {
    gfx->fillRect(0, bannerY, gfx->width(), bannerH, RGB565_BLACK);
    footerNeedsRedraw = true; // we just erased the IP footer - let it repaint
    return;
  }
  gfx->fillRect(0, bannerY, gfx->width(), bannerH, RGB565_RED);
  gfx->setTextSize(2);
  gfx->setTextColor(RGB565_WHITE, RGB565_RED);
  gfx->setCursor(6, bannerY + (bannerH - 16) / 2);
  gfx->print("Press again: reset WiFi");
}

void drawHeardRow(int y) {
  // Ticks every second once a reading exists, so there's no value to gate on
  // here - opaque printPadded() is what keeps this flicker-free, not skipping.
  char buf[24];
  if (!completedFirstSensorReading) {
    printPadded(PANEL_X, y + 2, RGB565_GRAY, RGB565_BLACK, 19, "Heard: never");
    return;
  }
  unsigned long secsSinceHeard = (millis() - lastHeardFromSensorTime) / 1000;
  uint16_t color = RGB565_WHITE;
  if (secsSinceHeard > (unsigned long)MAX_FILL_WITHOUT_SENSOR_TIME_MIN * 60) {
    color = RGB565_RED;
  } else if (secsSinceHeard > (unsigned long)MAX_FILL_WITHOUT_SENSOR_TIME_MIN * 30) {
    color = RGB565_ORANGE;
  }
  snprintf(buf, sizeof(buf), "Heard: %lus ago", secsSinceHeard);
  printPadded(PANEL_X, y + 2, color, RGB565_BLACK, 19, buf);
}

// Shares the battery row, to its left: three ascending bars (green when
// connected, gray with a red slash through them when not - a distinct icon,
// not just a duller copy) followed by a Yes/No readout, then a wide gap
// before the battery icon so the two metrics read as visually distinct.
void drawWifiIcon(int y) {
  static int8_t lastWifi = -1;
  if ((int8_t)wifiConnected == lastWifi) {
    return;
  }
  lastWifi = wifiConnected;

  const int x = WIFI_ICON_X, top = y + 4;
  gfx->fillRect(x, top, 13, 12, RGB565_BLACK); // clear - bars vs. bars+slash occupy different footprints

  uint16_t barColor = wifiConnected ? RGB565_GREEN : RGB565_DARKGREY;
  gfx->fillRect(x, top + 8, 3, 4, barColor);
  gfx->fillRect(x + 5, top + 4, 3, 8, barColor);
  gfx->fillRect(x + 10, top, 3, 12, barColor);

  if (!wifiConnected) {
    gfx->drawLine(x, top, x + 12, top + 11, RGB565_RED);
    gfx->drawLine(x + 1, top, x + 12, top + 10, RGB565_RED); // thicken the slash to 2px
  }

  printPadded(WIFI_TEXT_X, y + 2, RGB565_WHITE, RGB565_BLACK, WIFI_TEXT_CHARS, wifiConnected ? "Yes" : "No");
}

void drawTankGauge() {
  static float lastWaterLevel = -999, lastDesired = -999;
  if (sensorWaterLevel == lastWaterLevel && eData.preferredWaterLevel == lastDesired) {
    return;
  }
  lastWaterLevel = sensorWaterLevel;
  lastDesired = eData.preferredWaterLevel;

  const int innerX = TANK_X + 2, innerY = TANK_Y + 2, innerW = TANK_W - 4, innerH = TANK_H - 4;

  if (sensorWaterLevel < 0) {
    gfx->fillRect(innerX, innerY, innerW, innerH, RGB565_DARKGREY);
    gfx->setTextSize(2);
    gfx->setTextColor(RGB565_LIGHTGREY, RGB565_DARKGREY);
    gfx->setCursor(innerX + innerW / 2 - 6, innerY + innerH / 2 - 8);
    gfx->print("?");
  } else {
    float proportion = constrain(sensorWaterLevel / HOLE_DEPTH, 0.0f, 1.0f);
    int fillH = (int)(innerH * proportion);
    gfx->fillRect(innerX, innerY, innerW, innerH - fillH, RGB565_NAVY);
    gfx->fillRect(innerX, innerY + innerH - fillH, innerW, fillH, RGB565_BLUE);
  }

  // Desired-level marker line, extending slightly past the tank's own border.
  // Redrawn together with the fill above so a moved marker always paints over
  // a freshly-repainted tank interior instead of needing its own erase step.
  float desiredProportion = constrain(eData.preferredWaterLevel / HOLE_DEPTH, 0.0f, 1.0f);
  int markerY = innerY + innerH - (int)(innerH * desiredProportion);
  gfx->drawFastHLine(TANK_X - 4, markerY, TANK_W + 8, RGB565_YELLOW);
}

void drawTitleBarClock() {
  if (!wifiConnected) {
    return;
  }
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo, 0)) {
    return;
  }

  static char lastTimeBuf[24] = "";
  char tbuf[24];
  strftime(tbuf, sizeof(tbuf), "%m/%d %H:%M %Z", &timeinfo);
  if (strcmp(tbuf, lastTimeBuf) == 0) {
    return; // still the same minute - nothing to repaint
  }
  strncpy(lastTimeBuf, tbuf, sizeof(lastTimeBuf) - 1);
  lastTimeBuf[sizeof(lastTimeBuf) - 1] = '\0';

  const int chars = 15; // "MM/DD HH:MM ZZZ" is exactly 15 chars
  printPadded(gfx->width() - chars * 12 - 4, 3, RGB565_WHITE, RGB565_TEAL, chars, tbuf);
}

void updateStatsDisplay() {
  static bool chromeDrawn = false;
  if (!chromeDrawn) {
    drawDashboardChrome();
    chromeDrawn = true;
  }

  drawTitleBarClock();
  drawTankGauge();
  drawWifiIcon(ROW_Y(0));
  drawBatteryRow(ROW_Y(0));
  drawWaterRow(ROW_Y(1));
  drawDesiredRow(ROW_Y(2));
  drawHeardRow(ROW_Y(3));
  drawFillingPausedRow(ROW_Y(4));
  drawFreezeRow(ROW_Y(5));
  drawIpFooter();
  drawWifiResetBanner();
}

// ===================== Touch task =====================
// Samples the MPR121 pads and the CST816 screen on a fixed cadence that no
// longer depends on how long loop() takes. Debounce state lives here; only
// completed press events cross to the main loop.
void touchTask(void *param) {
  (void)param;
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(POLL_MS);
  // Previous CST816 finger state, for edge detection below.
  bool screenTouchPrev = false;

  for (;;) {
    // Edge-triggered, not level-triggered. This used to queue an event on
    // every poll where a finger was present, so holding or tapping the
    // screen enqueued ~50 events/second. Each one makes the main loop run
    // checkFilling() and a full updateStatsDisplay() SPI redraw, which takes
    // far longer than the 20ms poll period - so the 8-deep queue filled and
    // xQueueSend (timeout 0) silently dropped everything after it, including
    // genuine MPR121 pad presses. That is why pad touches were ignored and
    // why the screen itself needed many taps to wake.
    bool screenTouchNow = screenTouchDetected();
    if (screenTouchNow && !screenTouchPrev) {
      uint8_t ev = TOUCH_EVENT_SCREEN;
      xQueueSend(touchEventQueue, &ev, 0);
    }
    screenTouchPrev = screenTouchNow;

    for (uint8_t i = 0; i < NUM_ACTIVE_PADS; i++) {
      uint8_t pad = ACTIVE_PADS[i];

      i2cLock();
#ifdef TOUCH_TRACE
      uint32_t readStartUs = micros();
#endif
      uint16_t filtered = cap.filteredData(pad);
#ifdef TOUCH_TRACE
      uint32_t readUs = micros() - readStartUs;
      mprReadUsTotal += readUs;
      mprReads++;
      if (readUs > mprReadUsMax) {
        mprReadUsMax = readUs;
      }
#endif
      i2cUnlock();

      // filteredData() returns 0xFFFF when the underlying I2C read fails
      // (Adafruit_BusIO_Register::read() returns -1). Treat that as "no
      // sample" rather than letting it read as a release, but count it: a
      // long run of these means the bus has wedged and needs recovering.
      if (filtered == 0xFFFF) {
        uint16_t streak = (uint16_t)(touchI2cFailStreak + 1);
        touchI2cFailStreak = streak;
        if (streak >= I2C_FAIL_LIMIT) {
          touchI2cFailStreak = 0;
          i2cRecover();
          // Baselines captured before the reset no longer describe the pads.
          for (uint8_t p = 0; p < 12; p++) {
            baselineReady[p] = false;
            baselineInitCount[p] = 0;
            touchedState[p] = false;
            touchCandidate[p] = 0;
            releaseCandidate[p] = 0;
            preTouchBaseline[p] = 0;
            armedFor[p] = 0;
            heldSince[p] = 0;
          }
        }
        continue;
      }
      touchI2cFailStreak = 0;
      lastFiltered[pad] = filtered;

      // Seed the baseline from the first samples after boot or recovery, and
      // hold off detection until it has settled.
      if (!baselineReady[pad]) {
        padBaseline[pad] = (uint32_t)filtered << 2;
        preTouchBaseline[pad] = padBaseline[pad];
        lastBaseline[pad] = filtered;
        if (++baselineInitCount[pad] >= BASELINE_INIT_SAMPLES) {
          baselineReady[pad] = true;
        }
        continue;
      }

      uint16_t baseline = (uint16_t)(padBaseline[pad] >> 2);
      lastBaseline[pad] = baseline;
      // Magnitude of the excursion from baseline, either direction.
      // Textbook MPR121 wiring pulls the filtered value DOWN on touch, but
      // measured on this board (2026-09-19, -DMPR121_DIAGNOSTICS) the pads
      // move UP: e2 read 14 idle and 18 while touched. A one-directional
      // test misses those entirely, which is why presses did not register.
      int16_t signedDelta = (int16_t)baseline - (int16_t)filtered;
      int16_t delta = signedDelta < 0 ? (int16_t)-signedDelta : signedDelta;

#ifdef TOUCH_TRACE
      // Traces the values the detector actually computes, once per second per
      // pad plus on every threshold crossing. Diagnosing this from a separate
      // monitor loop proved misleading - this is the real code path.
      {
        // Report the peak excursion seen per pad each second, not just a
        // point sample - a brief press between samples was otherwise
        // invisible and made the pads look less sensitive than they are.
        static uint32_t lastTrace[12] = {0};
        static int16_t peakDelta[12] = {0};
        static uint16_t peakFiltered[12] = {0};
        if (delta > peakDelta[pad]) {
          peakDelta[pad] = delta;
          peakFiltered[pad] = filtered;
        }
        if (delta > TOUCH_DELTA || millis() - lastTrace[pad] >= 1000) {
          lastTrace[pad] = millis();
          Serial.printf("[t] pad%u f=%u base=%u |d|=%d PEAK|d|=%d(f=%u) cand=%u %s\n",
                        pad, filtered, baseline, delta,
                        peakDelta[pad], peakFiltered[pad],
                        touchCandidate[pad],
                        touchedState[pad] ? "HELD" : "");
          peakDelta[pad] = 0;
        }
      }
#endif

      if (!touchedState[pad]) {
        // Baseline tracking, with the rate chosen so a developing press is
        // not absorbed before it can latch.
        //
        // Below ARM_DELTA the pad is clearly idle: track normally and keep a
        // snapshot of the settled baseline. At or above it the pad may be
        // being pressed, so hold the baseline still and let the excursion
        // develop - otherwise the baseline chases the finger and shrinks the
        // very delta being measured, which is what made roughly 1 in 10
        // presses vanish and made quick successive presses much worse.
        //
        // ARM_MAX_POLLS bounds the hold so a genuine environmental shift is
        // still absorbed rather than freezing the pad permanently.
        bool arming = delta >= ARM_DELTA;
        if (!arming) {
          int32_t err = (int32_t)((uint32_t)filtered << 2) - (int32_t)padBaseline[pad];
          int32_t step = err >> BASELINE_SHIFT;
          // At this signal scale the shift alone truncates to zero, which
          // would stall tracking entirely. Always move at least one
          // quarter-count in the right direction.
          if (step == 0 && err != 0) {
            step = (err > 0) ? 1 : -1;
          }
          padBaseline[pad] += step;
          preTouchBaseline[pad] = padBaseline[pad];
          armedFor[pad] = 0;
        } else if (++armedFor[pad] > ARM_MAX_POLLS) {
          int32_t err = (int32_t)((uint32_t)filtered << 2) - (int32_t)padBaseline[pad];
          int32_t step = err >> BASELINE_SHIFT_SLOW;
          if (step == 0 && err != 0) {
            step = (err > 0) ? 1 : -1;
          }
          padBaseline[pad] += step;
        }

#ifdef SENS_TRACE
        // Track peak deflection per pad so a drop in sensitivity is visible
        // against the boot clock, alongside whatever else is starting up.
        if (delta > sensPeak[pad]) {
          sensPeak[pad] = delta;
        }
#endif

        if (delta > TOUCH_DELTA) {
          if (++touchCandidate[pad] >= DEBOUNCE_SAMPLES) {
            touchedState[pad] = true;
            touchCandidate[pad] = 0;
            releaseCandidate[pad] = 0;
            heldSince[pad] = millis();
#ifdef SENS_TRACE
            sensLatches[pad]++;
#endif
            // Judge the release against the last settled pre-touch value, so
            // a baseline that drifted during the press cannot leave the pad
            // stuck reporting touched.
            padBaseline[pad] = preTouchBaseline[pad];
            // Hand the press to loop(); it owns the relay, EEPROM and display.
            if (xQueueSend(touchEventQueue, &pad, 0) != pdTRUE) {
              // A dropped press is a real lost button action, so make it
              // visible rather than silently discarding it.
              Serial.printf("Touch queue full - dropped press on pad %u\n", pad);
            }
          }
        } else {
          touchCandidate[pad] = 0;
        }
      } else {
        if (delta < RELEASE_DELTA) {
          if (++releaseCandidate[pad] >= DEBOUNCE_SAMPLES) {
            touchedState[pad] = false;
            releaseCandidate[pad] = 0;
            touchCandidate[pad] = 0;
            armedFor[pad] = 0;
            heldSince[pad] = 0;
          }
        } else {
          releaseCandidate[pad] = 0;
        }

        // Backstop: a pad must never latch forever. If its baseline was
        // stale when the press landed, the excursion can stay above
        // RELEASE_DELTA indefinitely with no finger present.
        if (heldSince[pad] != 0 && millis() - heldSince[pad] > MAX_HELD_MS) {
          touchedState[pad] = false;
          releaseCandidate[pad] = 0;
          touchCandidate[pad] = 0;
          armedFor[pad] = 0;
          heldSince[pad] = 0;
          padBaseline[pad] = (uint32_t)filtered << 2;
          preTouchBaseline[pad] = padBaseline[pad];
          Serial.printf("Touch pad %u force-released after %lums held\n",
                        pad, (unsigned long)MAX_HELD_MS);
        }
      }
    }

#ifdef TOUCH_TRACE
    // Bus-health report. Both touch chips share only the I2C bus, so if the
    // MPR121 pads and the CST816 screen are failing together, this is where
    // the common cause shows up: slow reads, address NAKs or short reads.
    touchLoopIters++;
    {
      static uint32_t lastBusReport = 0;
      if (millis() - lastBusReport >= 5000) {
        uint32_t elapsed = millis() - lastBusReport;
        lastBusReport = millis();
        uint32_t avgUs = mprReads ? (mprReadUsTotal / mprReads) : 0;
        // Read the chip's mode back live. All four pads losing sensitivity
        // together points at chip state, not at four separate electrodes -
        // ECR leaving Run Mode, or the electrode count being reset, would do
        // exactly that while every I2C read still succeeds.
        i2cLock();
        uint8_t ecrNow = cap.readRegister8(MPR121_ECR);
        uint8_t cfg1Now = cap.readRegister8(MPR121_CONFIG1);
        uint8_t cfg2Now = cap.readRegister8(MPR121_CONFIG2);
        uint16_t touchNow = cap.touched();
        i2cUnlock();
        Serial.printf("[chip] ECR=0x%02X(run=%s,ele=%u) CFG1=0x%02X CFG2=0x%02X touched=0x%03X\n",
                      ecrNow, (ecrNow & 0x3F) ? "yes" : "NO", ecrNow & 0x0F,
                      cfg1Now, cfg2Now, touchNow);
        Serial.printf("[bus] %lums: touchIters=%lu (%.1f/s) mprRead avg=%luus max=%luus | "
                      "cst polls=%lu addrFail=%lu shortRead=%lu fingers=%lu | i2cRecov=%lu\n",
                      (unsigned long)elapsed, (unsigned long)touchLoopIters,
                      touchLoopIters * 1000.0f / elapsed,
                      (unsigned long)avgUs, (unsigned long)mprReadUsMax,
                      (unsigned long)cstPolls, (unsigned long)cstAddrFail,
                      (unsigned long)cstShortRead, (unsigned long)cstFingers,
                      (unsigned long)touchI2cRecoveryCount);
        touchLoopIters = 0;
        mprReads = 0; mprReadUsTotal = 0; mprReadUsMax = 0;
        cstPolls = 0; cstAddrFail = 0; cstShortRead = 0; cstFingers = 0;
      }
    }
#endif

#ifdef SENS_TRACE
    {
      static uint32_t lastSens = 0;
      if (millis() - lastSens >= 5000) {
        lastSens = millis();
        Serial.printf("[sens] t=%lus peak:", (unsigned long)(millis() / 1000));
        for (uint8_t i = 0; i < NUM_ACTIVE_PADS; i++) {
          uint8_t p = ACTIVE_PADS[i];
          Serial.printf(" p%u=%d(%lu)", p, sensPeak[p], (unsigned long)sensLatches[p]);
          sensPeak[p] = 0;
        }
        Serial.println();
      }
    }
#endif

    vTaskDelayUntil(&lastWake, period);
  }
}

// Touch diagnostics for /api/status (WebPortal.cpp). Reports the live
// filtered reading, the tracked baseline and the resulting margin for each
// wired pad, so a pad drifting toward unresponsiveness is visible before it
// stops working - and so a wedged I2C bus is distinguishable from drift.
// Writes a JSON array fragment; returns the number of chars written.
size_t formatTouchDiagnostics(char *out, size_t outLen) {
  size_t used = 0;
  used += snprintf(out + used, outLen > used ? outLen - used : 0, "[");
  for (uint8_t i = 0; i < NUM_ACTIVE_PADS; i++) {
    uint8_t pad = ACTIVE_PADS[i];
    uint16_t filtered = lastFiltered[pad];
    uint16_t baseline = lastBaseline[pad];
    used += snprintf(out + used, outLen > used ? outLen - used : 0,
                     "%s{\"pad\":%u,\"filtered\":%u,\"baseline\":%u,\"delta\":%d,"
                     "\"touched\":%s,\"ready\":%s}",
                     i ? "," : "", pad, filtered, baseline,
                     (int)baseline - (int)filtered,
                     touchedState[pad] ? "true" : "false",
                     baselineReady[pad] ? "true" : "false");
  }
  used += snprintf(out + used, outLen > used ? outLen - used : 0, "]");
  return used;
}

uint32_t getTouchI2cRecoveryCount() { return touchI2cRecoveryCount; }
int16_t getTouchTouchDelta() { return TOUCH_DELTA; }

// Runs on the main loop: applies one debounced press from the touch task.
void handleTouchEvent(uint8_t pad) {
  lastActivityMs = millis();
  if (!screenOn) {
    digitalWrite(GFX_BL, LOW); // wake: backlight is active-low
    screenOn = true;
  }

  if (pad == TOUCH_EVENT_SCREEN) {
    return; // screen touch only wakes the display
  }

  if (pad == FREEZE_PROTECT_PAD) {
    toggleFreezeProtect();
  } else if (pad == WIFI_RESET_PAD) {
    if (wifiResetPendingUntilMs != 0 && millis() < wifiResetPendingUntilMs) {
      WebPortal::resetWifiAndReboot(); // confirming second press - does not return
    }
    wifiResetPendingUntilMs = millis() + WIFI_RESET_CONFIRM_WINDOW_MS;
  } else {
    adjustDesiredWaterLevel(padLevelDelta[pad]);
  }
  checkFilling();
  updateStatsDisplay();
}

void setup() {
  Serial.begin(115200);
  // Make serial writes non-blocking.
  //
  // With ARDUINO_USB_CDC_ON_BOOT this Serial is USB CDC, whose write()
  // blocks for up to tx_timeout_ms (250ms by default) whenever the USB host
  // is attached but nothing is draining the port - exactly the case when the
  // board is plugged in for power with no serial monitor open. Once the CDC
  // buffer fills, every Serial.print in loop() stalls it for a quarter of a
  // second, and since the touch queue is drained at the end of the loop,
  // button presses came through late or not at all. That is why the device
  // behaved perfectly while a monitor was attached and went laggy without
  // one. A zero timeout makes write() drop output instead of waiting.
  Serial.setTxTimeoutMs(0);
  delay(10000);
  Serial.println("Setting up WaterController...");

  loadEeprom();
  for (int i = 0; i < MAX_READING_SAMPLES; i++) {
    previousSensorReadings[i] = SENSOR_READING_UNKNOWN;
  }

  padLevelDelta[0] = 0.1f;
  padLevelDelta[1] = -0.1f;

  i2cMutex = xSemaphoreCreateMutex();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  // Bound each transaction so a stuck device can't hold the I2C mutex - and with
  // it the touch task and the main loop's relay writes - indefinitely. Failed
  // reads surface as 0xFFFF, which the touch task counts toward recovery.
  Wire.setTimeOut(50);

  relayInit();

  // Switch the onboard CST816 touch controller into normal mode
  Wire.beginTransmission(CST816_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();

  if (!mpr121Init()) {
    Serial.println("MPR121 not found on I2C bus (addr 0x5A) — check ADD pin wiring");
  }
  // Build with -DMPR121_DIAGNOSTICS for a touch-wiring diagnostic image.
  // This takes the device over completely: it dumps chip state, then monitors
  // all 12 electrodes forever and never starts the display, radio, web portal
  // or relay. Flash a normal build to restore operation.
#ifdef MPR121_DIAGNOSTICS
  mpr121DumpRegisters();
  mpr121MonitorForever(); // never returns
#endif

  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(RGB565_BLACK);
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, LOW); // this panel's backlight is active-low
  updateStatsDisplay();
  lastActivityMs = millis();

  rfmReady = rfmInit();

  // Build with -DNO_WIFI to leave WiFi and the web portal down. Touch
  // responsiveness degrades a few seconds after the AP comes up, and this
  // isolates whether the radio is the cause.
#ifndef NO_WIFI
  WebPortal::begin();
#else
  Serial.println("NO_WIFI build: WiFi and web portal disabled");
#endif

  checkFilling();

  touchEventQueue = xQueueCreate(8, sizeof(uint8_t));
  if (touchEventQueue == nullptr) {
    Serial.println("Failed to create touch event queue");
  } else if (xTaskCreate(touchTask, "touch", TOUCH_TASK_STACK, nullptr,
                         TOUCH_TASK_PRIORITY, &touchTaskHandle) != pdPASS) {
    Serial.println("Failed to start touch task");
    touchTaskHandle = nullptr;
  }

  Serial.println("WaterController ready");
}

void loop() {

#ifdef LOOP_TRACE
  // Times each blocking call in the loop. The touch queue is drained at the
  // end of the loop, so anything slow here directly delays button response.
  uint32_t tStart = millis();
  static uint32_t worstIter = 0, iterCount = 0, lastLoopReport = 0;
  uint32_t tMark = tStart, dRfm = 0, dWeb = 0, dAuto = 0, dMetric = 0,
           dFill = 0, dDraw = 0, dTouch = 0;
#define LOOP_MARK(var) do { uint32_t now = millis(); var = now - tMark; tMark = now; } while (0)
#else
#define LOOP_MARK(var) do { } while (0)
#endif

  // The once-a-second "loop tick" heartbeat that used to print here is gone:
  // it was the main thing filling the USB CDC buffer when no monitor was
  // attached. Build with -DLOOP_HEARTBEAT if you want it back for debugging.
#ifdef LOOP_HEARTBEAT
  static uint32_t lastLoopPrintMs = 0;
  if (millis() - lastLoopPrintMs >= 1000UL) {
    lastLoopPrintMs = millis();
    Serial.println("WaterController loop tick");
  }
#endif

  static uint32_t lastMetricUpdateMs = 0;
  static uint32_t lastCheckFillingMs = 0;
  static uint32_t lastStatsRedrawMs = 0;

  rfmPollReceive();
  LOOP_MARK(dRfm);

#ifndef NO_WIFI
  WebPortal::poll();
#endif
  LOOP_MARK(dWeb);

  checkAutoRestart();
  LOOP_MARK(dAuto);

  if (millis() - lastMetricUpdateMs > METRIC_UPDATE_FREQ_MS) {
    lastMetricUpdateMs = millis();
    updateMetrics();
  }
  LOOP_MARK(dMetric);

  if (millis() - lastCheckFillingMs > CHECK_FILLING_FREQ_MS) {
    lastCheckFillingMs = millis();
    checkFilling();
  }
  LOOP_MARK(dFill);

  if (millis() - lastStatsRedrawMs > 1000) {
    lastStatsRedrawMs = millis();
    if (screenOn) {
      updateStatsDisplay();
    }
  }
  LOOP_MARK(dDraw);


  // Touch sampling/debouncing happens in touchTask; drain whatever presses it
  // has queued since the last pass. Bounded so a burst can't stall the loop.
  if (touchEventQueue != nullptr) {
    uint8_t pad;
    for (uint8_t drained = 0; drained < 8 && xQueueReceive(touchEventQueue, &pad, 0) == pdTRUE; drained++) {
      handleTouchEvent(pad);
    }
  }
  LOOP_MARK(dTouch);

#ifdef LOOP_TRACE
  {
    uint32_t iter = millis() - tStart;
    iterCount++;
    if (iter > worstIter) {
      worstIter = iter;
    }
    // Report the slowest iteration seen per window, plus where that time
    // went. Anything here above ~100ms delays the touch drain by that much.
    if (millis() - lastLoopReport >= 5000) {
      uint32_t elapsed = millis() - lastLoopReport;
      lastLoopReport = millis();
      UBaseType_t queued = touchEventQueue ? uxQueueMessagesWaiting(touchEventQueue) : 0;
      Serial.printf("[loop] %lums: iters=%lu (%.1f/s) worst=%lums | rfm=%lu web=%lu auto=%lu "
                    "metric=%lu fill=%lu draw=%lu touch=%lu | queued=%u\n",
                    (unsigned long)elapsed, (unsigned long)iterCount,
                    iterCount * 1000.0f / elapsed, (unsigned long)worstIter,
                    (unsigned long)dRfm, (unsigned long)dWeb, (unsigned long)dAuto,
                    (unsigned long)dMetric, (unsigned long)dFill,
                    (unsigned long)dDraw, (unsigned long)dTouch,
                    (unsigned)queued);
      worstIter = 0;
      iterCount = 0;
    }
  }
#endif

  if (wifiResetPendingUntilMs != 0 && millis() >= wifiResetPendingUntilMs) {
    wifiResetPendingUntilMs = 0; // confirm window elapsed with no second press - clear the banner
    updateStatsDisplay();
  }

  if (screenOn && millis() - lastActivityMs > SCREEN_TIMEOUT_MS) {
    digitalWrite(GFX_BL, HIGH); // active-low backlight: HIGH turns it off
    screenOn = false;
  }
}
