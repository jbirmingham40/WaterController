#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <time.h>
#include <EEPROM.h>
#include "Adafruit_MPR121.h"
#include <Arduino_GFX_Library.h>

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

// ===================== WiFi / NTP =====================
static const char *WIFI_SSID = "***REMOVED***";
static const char *WIFI_PASSWORD = "***REMOVED***";
static const char *NTP_SERVER = "pool.ntp.org";
// US Central time with automatic DST (CST6CDT, DST starts 2nd Sun in March, ends 1st Sun in November)
static const char *TZ_INFO = "CST6CDT,M3.2.0,M11.1.0";
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;

bool wifiConnected = false;

void connectWifiAndSyncTime() {
  Serial.printf("Connecting to WiFi \"%s\"...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMs < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  wifiConnected = WiFi.status() == WL_CONNECTED;
  if (!wifiConnected) {
    Serial.println("WiFi connection failed, skipping NTP sync");
    return;
  }
  Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());

  configTzTime(TZ_INFO, NTP_SERVER); // syncs over NTP, converts to local time per TZ_INFO

  struct tm timeinfo;
  if (getLocalTime(&timeinfo, 10000)) {
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S %Z", &timeinfo);
    Serial.printf("Time synced: %s\n", buf);
  } else {
    Serial.println("Failed to obtain time from NTP server");
  }
}

// ===================== Metrics (Graphite/Grafana, matches WaterControllerV8) =====================
#define GRAFANA_HOSTNAME "grafana.jbirmingham.linkpc.net"
#define METRICS_SERVER_PORT 2003
WiFiClient metricsClient;
IPAddress grafanaIp;
bool grafanaIpResolved = false;

void sendMetric(const char *key, float value) {
  if (!wifiConnected) {
    return;
  }
  if (!grafanaIpResolved) {
    if (!WiFi.hostByName(GRAFANA_HOSTNAME, grafanaIp)) {
      return;
    }
    grafanaIpResolved = true;
  }
  if (!metricsClient.connected() && !metricsClient.connect(grafanaIp, METRICS_SERVER_PORT)) {
    return;
  }
  char line[128];
  snprintf(line, sizeof(line), "%s %.2f %lu\n", key, value, (unsigned long)time(nullptr));
  metricsClient.print(line);
}

// ===================== Onboard I2C bus =====================
// Shared with the CST816 touch controller, QMI8658 IMU, and TCA9554 expander
static const uint8_t I2C_SDA_PIN = 18;
static const uint8_t I2C_SCL_PIN = 8;

// ===================== TCA9554 I2C GPIO expander (found at 0x20) =====================
// EX0 (P0) drives the water valve relay; EX1 (P1) reads the freeze-protect
// switch. Add more EX pins here as needed - all others stay inputs.
#define TCA9554_ADDR 0x20
#define TCA9554_REG_INPUT 0x00
#define TCA9554_REG_OUTPUT 0x01
#define TCA9554_REG_CONFIG 0x03
static const uint8_t RELAY_EXIO_BIT = 0;          // EX0 = P0
static const uint8_t FREEZE_PROTECT_EXIO_BIT = 1; // EX1 = P1

void tca9554WriteReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void relaySet(bool on) {
  // Config register: 1=input, 0=output. Only the relay's bit is an output;
  // the rest (including the freeze-protect input) stay inputs.
  uint8_t config = 0xFF & ~(1 << RELAY_EXIO_BIT);
  tca9554WriteReg(TCA9554_REG_CONFIG, config);
  tca9554WriteReg(TCA9554_REG_OUTPUT, on ? (1 << RELAY_EXIO_BIT) : 0x00);
}

void relayInit() {
  relaySet(false); // start with the relay off
}

bool tca9554ReadBit(uint8_t bit) {
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write((uint8_t)TCA9554_REG_INPUT);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((int)TCA9554_ADDR, 1) < 1) {
    return false;
  }
  uint8_t val = Wire.read();
  return (val >> bit) & 0x01;
}

// ===================== Persistent state (EEPROM) =====================
#define CURRENT_EEPROM_VERSION 103 // bumped once to clear a stale freeze-protect=on left by an earlier bug
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

bool lastFreezeProtectPin = false;

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

void checkFreezeProtectPressed() {
  bool pressed = tca9554ReadBit(FREEZE_PROTECT_EXIO_BIT);
  if (pressed && !lastFreezeProtectPin) {
    eData.inFreezeProtect = !eData.inFreezeProtect;
    saveEeprom();
    Serial.printf("Freeze protect %s\n", eData.inFreezeProtect ? "On" : "Off");
  }
  lastFreezeProtectPin = pressed;
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

// ===================== MPR121 capacitive touch (pins repurposed as desired-water-level +/-) =====================
static const uint32_t POLL_MS = 20;
static const uint32_t SCREEN_TIMEOUT_MS = 90000; // turn backlight off after this long with no touches

// This chip's onboard baseline register stays stuck at 0, so cap.touched()'s
// internal (baseline - filtered) comparison never trips even though filteredData()
// clearly reacts to touch. Compare filteredData() against a fixed baseline instead.
#define BASELINE 15 // re-measured idle reading (was 12, drifted/recalibrated)
static const int16_t TOUCH_DELTA = 5;    // filtered drop below BASELINE to call it touched
static const int16_t RELEASE_DELTA = 3;  // drop below BASELINE to call it released
static const uint8_t DEBOUNCE_SAMPLES = 4; // consecutive polls required before latching

// Only these MPR121 electrodes are wired up right now; add more pad numbers
// here as additional pins get connected (up to 4 planned).
static const uint8_t ACTIVE_PADS[] = {0, 1};
static const uint8_t NUM_ACTIVE_PADS = sizeof(ACTIVE_PADS) / sizeof(ACTIVE_PADS[0]);

// Pin 0 increases the desired water level, pin 1 decreases it (0.1in per touch)
float padLevelDelta[12] = {0};

Adafruit_MPR121 cap = Adafruit_MPR121();
bool touchedState[12] = {false};
uint8_t touchCandidate[12] = {0};
uint8_t releaseCandidate[12] = {0};

bool screenOn = true;
uint32_t lastActivityMs = 0;

// ===================== Onboard CST816 touchscreen (used only to wake the display) =====================
// Its IRQ/RST pins are not wired on this board per Waveshare's docs, so it
// has to be polled over I2C like the MPR121. Register map per Waveshare's
// own touch_bsp.c: burst-read 7 bytes from reg 0x00; byte[2] is finger count.
#define CST816_ADDR 0x15

bool screenTouchDetected() {
  Wire.beginTransmission(CST816_ADDR);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  uint8_t n = Wire.requestFrom((int)CST816_ADDR, 3);
  if (n < 3) {
    return false;
  }
  uint8_t buf[3];
  for (uint8_t i = 0; i < 3; i++) {
    buf[i] = Wire.read();
  }
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
// rotation=1 turns the 170x320 portrait panel into a 320x170 landscape view.
#define GFX_BL 15
Arduino_DataBus *bus = new Arduino_HWSPI(6 /* DC */, 7 /* CS */, 5 /* SCK */, 4 /* MOSI */);
Arduino_GFX *gfx = new Arduino_ST7789(bus, 14 /* RST */, 1 /*rotation*/, 1 /*IPS*/,
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
#define BATTERY_ICON_X (WIFI_TEXT_X + WIFI_TEXT_CHARS * 12 + 14)

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

  int yDesired = ROW_Y(5);
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
  printPadded(textX, y + 2, color, RGB565_BLACK, 8, buf);
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

  printPadded(WIFI_TEXT_X, y + 2, wifiConnected ? RGB565_GREEN : RGB565_GRAY, RGB565_BLACK,
              WIFI_TEXT_CHARS, wifiConnected ? "Yes" : "No");
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
  drawHeardRow(ROW_Y(2));
  drawFillingPausedRow(ROW_Y(3));
  drawFreezeRow(ROW_Y(4));
  drawDesiredRow(ROW_Y(5));
}

void setup() {
  Serial.begin(115200);
  delay(200); // let native USB-CDC enumerate before first prints

  loadEeprom();
  for (int i = 0; i < MAX_READING_SAMPLES; i++) {
    previousSensorReadings[i] = SENSOR_READING_UNKNOWN;
  }

  padLevelDelta[0] = 0.1f;
  padLevelDelta[1] = -0.1f;

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  relayInit();
  // Prime the edge-detector with whatever the pin currently reads (floats
  // HIGH via the TCA9554's internal pull-up until a real switch is wired),
  // so we don't mistake "unwired" for "just pressed" on the first boot.
  lastFreezeProtectPin = tca9554ReadBit(FREEZE_PROTECT_EXIO_BIT);

  // Switch the onboard CST816 touch controller into normal mode
  Wire.beginTransmission(CST816_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();

  if (!cap.begin(0x5A)) {
    Serial.println("MPR121 not found on I2C bus (addr 0x5A) — check ADD pin wiring");
    while (1) {
      delay(1000);
    }
  }

  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(RGB565_BLACK);
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, LOW); // this panel's backlight is active-low
  updateStatsDisplay();
  lastActivityMs = millis();

  rfmReady = rfmInit();

  connectWifiAndSyncTime();

  checkFilling();

  Serial.println("WaterController ready");
}

void loop() {
  static uint32_t lastPollMs = 0;
  static uint32_t lastMetricUpdateMs = 0;
  static uint32_t lastCheckFillingMs = 0;
  static uint32_t lastStatsRedrawMs = 0;
  static uint16_t lastFiltered[12];

  rfmPollReceive();

  checkAutoRestart();

  if (millis() - lastMetricUpdateMs > METRIC_UPDATE_FREQ_MS) {
    lastMetricUpdateMs = millis();
    updateMetrics();
  }

  if (millis() - lastCheckFillingMs > CHECK_FILLING_FREQ_MS) {
    lastCheckFillingMs = millis();
    checkFilling();
  }

  if (millis() - lastStatsRedrawMs > 1000) {
    lastStatsRedrawMs = millis();
    if (screenOn) {
      updateStatsDisplay();
    }
  }


  if (millis() - lastPollMs > POLL_MS) {
    lastPollMs = millis();

    checkFreezeProtectPressed();

    if (screenTouchDetected()) {
      lastActivityMs = millis();
      if (!screenOn) {
        digitalWrite(GFX_BL, LOW); // wake: backlight is active-low
        screenOn = true;
      }
    }

    for (uint8_t i = 0; i < NUM_ACTIVE_PADS; i++) {
      uint8_t pad = ACTIVE_PADS[i];
      uint16_t filtered = cap.filteredData(pad);
      lastFiltered[pad] = filtered;
      int16_t delta = (int16_t)BASELINE - (int16_t)filtered;

      if (!touchedState[pad]) {
        if (delta > TOUCH_DELTA) {
          if (++touchCandidate[pad] >= DEBOUNCE_SAMPLES) {
            touchedState[pad] = true;
            touchCandidate[pad] = 0;
            releaseCandidate[pad] = 0;

            lastActivityMs = millis();
            if (!screenOn) {
              digitalWrite(GFX_BL, LOW); // wake: backlight is active-low
              screenOn = true;
            }

            adjustDesiredWaterLevel(padLevelDelta[pad]);
            checkFilling();
            updateStatsDisplay();
          }
        } else {
          touchCandidate[pad] = 0;
        }
      } else {
        if (delta < RELEASE_DELTA) {
          if (++releaseCandidate[pad] >= DEBOUNCE_SAMPLES) {
            touchedState[pad] = false;
            releaseCandidate[pad] = 0;
          }
        } else {
          releaseCandidate[pad] = 0;
        }
      }
    }

    if (screenOn && millis() - lastActivityMs > SCREEN_TIMEOUT_MS) {
      digitalWrite(GFX_BL, HIGH); // active-low backlight: HIGH turns it off
      screenOn = false;
    }
  }
}
