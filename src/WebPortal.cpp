#include "WebPortal.h"

#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_random.h>
#include <time.h>

#include "WebPortalPages.h"
#include "secrets.h" // SETTINGS_PASSWORD - gitignored, copy secrets.h.example to fill in

// ===================== Shared state owned by main.cpp =====================
// Plain extern globals/functions, matching this codebase's existing flat
// style rather than introducing a bigger shared-state header.
extern bool wifiConnected;
extern float sensorVoltage;
extern float sensorPercentage;
extern float sensorWaterLevel;
extern bool completedFirstSensorReading;
extern unsigned long lastHeardFromSensorTime;
extern bool isFilling;
extern bool fillingPaused;
extern void adjustDesiredWaterLevel(float delta);
extern void toggleFreezeProtect();
extern void checkFilling();
extern float getPreferredWaterLevel();
extern bool getFreezeProtectState();

namespace WebPortal {

// ===================== Config, persisted in NVS under "wcv9" =====================
static Preferences prefs;
static String savedStaSsid;
static String savedStaPass;
static bool apConfirmed = false;
static String carbonHost;
static uint16_t carbonPort = CARBON_CACHE_PORT_DEFAULT;

static const char *AP_SSID = "WaterController-Setup";
static const char *NTP_SERVER = "pool.ntp.org";
// US Central time with automatic DST (CST6CDT, DST starts 2nd Sun in March, ends 1st Sun in November)
static const char *TZ_INFO = "CST6CDT,M3.2.0,M11.1.0";

static AsyncWebServer server(80);

// ===================== Session/auth =====================
// Single admin, single active session - proportionate to a LAN-only,
// one-household embedded device, not multi-user/enterprise auth.
static char sessionToken[33] = {0}; // 32 hex chars + NUL, 0-length = no session
static uint32_t sessionLastActivityMs = 0;
static const uint32_t SESSION_TTL_MS = 30UL * 60UL * 1000UL;

static uint8_t loginFailCount = 0;
static bool loginLockoutActive = false;
static uint32_t loginLockoutStartMs = 0;
static const uint8_t LOGIN_MAX_FAILS = 5;
static const uint32_t LOGIN_LOCKOUT_MS = 5UL * 60UL * 1000UL;

// ===================== WiFi test-then-confirm state machine =====================
enum WifiTestState { WIFI_TEST_IDLE, WIFI_TEST_TESTING, WIFI_TEST_SUCCESS, WIFI_TEST_FAILED };
static WifiTestState wifiTestState = WIFI_TEST_IDLE;
static uint32_t wifiTestStartMs = 0;
static uint32_t wifiTestResultStartMs = 0;
static String pendingTestSsid;
static String pendingTestPass;
static const uint32_t WIFI_TEST_TIMEOUT_MS = 20000;
static const uint32_t WIFI_TEST_RESULT_DISPLAY_MS = 8000;

static bool lastWifiConnectedState = false;

// ===================== STA reconnect =====================
// The ESP32 doesn't always recover from every disconnect on its own (e.g. an
// AP-side CCMP replay rejection can drop the driver to a bare "INIT" state
// that a soft WiFi.reconnect() won't reliably clear) - so once WiFi has been
// confirmed working, periodically re-issue a full WiFi.begin() while
// disconnected, rather than only reacting to the initial connect.
static uint32_t lastReconnectAttemptMs = 0;
static const uint32_t RECONNECT_INTERVAL_MS = 30000;

// ===================== Pending reboot =====================
static bool rebootPending = false;
static uint32_t rebootRequestedMs = 0;
static const uint32_t REBOOT_DELAY_MS = 500; // lets the HTTP response flush first

// ===================== Small helpers =====================

static String jsonEscape(const String &in) {
  String out;
  out.reserve(in.length() + 4);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    out += c;
  }
  return out;
}

static String getCookieValue(AsyncWebServerRequest *request, const String &name) {
  if (!request->hasHeader("Cookie")) {
    return String();
  }
  String cookie = request->header("Cookie");
  String key = name + "=";
  int idx = cookie.indexOf(key);
  if (idx < 0) {
    return String();
  }
  int start = idx + key.length();
  int end = cookie.indexOf(';', start);
  if (end < 0) {
    end = cookie.length();
  }
  return cookie.substring(start, end);
}

static bool isAuthed(AsyncWebServerRequest *request) {
  if (sessionToken[0] == '\0') {
    return false;
  }
  if (millis() - sessionLastActivityMs > SESSION_TTL_MS) {
    sessionToken[0] = '\0';
    return false;
  }
  String token = getCookieValue(request, "session");
  if (token.length() == 0 || token != String(sessionToken)) {
    return false;
  }
  sessionLastActivityMs = millis(); // sliding expiry
  return true;
}

static void startSession(AsyncWebServerResponse *response) {
  uint32_t r[4] = {esp_random(), esp_random(), esp_random(), esp_random()};
  snprintf(sessionToken, sizeof(sessionToken), "%08x%08x%08x%08x", r[0], r[1], r[2], r[3]);
  sessionLastActivityMs = millis();
  response->addHeader("Set-Cookie", String("session=") + sessionToken + "; Path=/; HttpOnly; Max-Age=1800");
}

static String renderPage(const char *pageProgmem) {
  String html(pageProgmem);
  html.replace("%CSS%", String(PORTAL_CSS));
  return html;
}

// ===================== WiFi state machine =====================

// Connects to previously-confirmed STA credentials, enables the core's own
// auto-reconnect as a first line of defense, and marks the retry clock so
// serviceStaReconnect() doesn't immediately pile on another attempt.
static void beginStaConnect(const String &ssid, const String &pass) {
  WiFi.setAutoReconnect(true);
  WiFi.begin(ssid.c_str(), pass.c_str());
  lastReconnectAttemptMs = millis();
}

static void beginWifiTest(const String &ssid, const String &pass) {
  pendingTestSsid = ssid;
  pendingTestPass = pass;
  wifiTestState = WIFI_TEST_TESTING;
  wifiTestStartMs = millis();
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID); // re-assert AP so the settings page stays reachable during the test
  WiFi.begin(pendingTestSsid.c_str(), pendingTestPass.c_str());
}

static void serviceWifiTestStateMachine() {
  if (wifiTestState == WIFI_TEST_TESTING) {
    if (WiFi.status() == WL_CONNECTED) {
      prefs.putString("sta_ssid", pendingTestSsid);
      prefs.putString("sta_pass", pendingTestPass);
      prefs.putBool("ap_confirmed", true);
      savedStaSsid = pendingTestSsid;
      savedStaPass = pendingTestPass;
      apConfirmed = true;
      wifiTestState = WIFI_TEST_SUCCESS;
      wifiTestResultStartMs = millis();
    } else if (millis() - wifiTestStartMs > WIFI_TEST_TIMEOUT_MS) {
      WiFi.disconnect();
      if (savedStaSsid.length() > 0) {
        beginStaConnect(savedStaSsid, savedStaPass); // restore the previously-working connection
      }
      wifiTestState = WIFI_TEST_FAILED;
      wifiTestResultStartMs = millis();
    }
  } else if (wifiTestState == WIFI_TEST_SUCCESS || wifiTestState == WIFI_TEST_FAILED) {
    if (millis() - wifiTestResultStartMs > WIFI_TEST_RESULT_DISPLAY_MS) {
      if (wifiTestState == WIFI_TEST_SUCCESS) {
        WiFi.mode(WIFI_STA); // confirmed - drop the AP for good until an explicit reset
      } else {
        WiFi.mode(WIFI_AP);
        WiFi.softAP(AP_SSID);
      }
      wifiTestState = WIFI_TEST_IDLE;
    }
  }
}

static void serviceWifiConnectedEdge() {
  bool nowConnected = WiFi.status() == WL_CONNECTED;
  if (nowConnected == lastWifiConnectedState) {
    return;
  }
  lastWifiConnectedState = nowConnected;
  wifiConnected = nowConnected;
  if (nowConnected) {
    Serial.printf("WiFi connected, IP: %s\n", WiFi.localIP().toString().c_str());
    configTzTime(TZ_INFO, NTP_SERVER);
  } else {
    Serial.println("WiFi disconnected");
  }
}

// Only relevant once WiFi has been confirmed and we're not mid-test - the
// AP-only (unconfigured) and WiFi-test states manage the radio themselves.
static void serviceStaReconnect() {
  if (!apConfirmed || wifiTestState != WIFI_TEST_IDLE) {
    return;
  }
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  if (millis() - lastReconnectAttemptMs < RECONNECT_INTERVAL_MS) {
    return;
  }
  Serial.println("WiFi still disconnected - retrying...");
  WiFi.disconnect();
  beginStaConnect(savedStaSsid, savedStaPass);
}

static const char *wifiTestStateName() {
  switch (wifiTestState) {
  case WIFI_TEST_TESTING:
    return "testing";
  case WIFI_TEST_SUCCESS:
    return "success";
  case WIFI_TEST_FAILED:
    return "failed";
  default:
    return "idle";
  }
}

// ===================== Route handlers =====================

static void handleRoot(AsyncWebServerRequest *request) {
  if (isAuthed(request)) {
    request->send(200, "text/html", renderPage(SETTINGS_PAGE_HTML));
  } else {
    request->send(200, "text/html", renderPage(LOGIN_PAGE_HTML));
  }
}

static void handleLogin(AsyncWebServerRequest *request) {
  if (loginLockoutActive) {
    if (millis() - loginLockoutStartMs > LOGIN_LOCKOUT_MS) {
      loginLockoutActive = false;
    } else {
      uint32_t retryAfterSec = (LOGIN_LOCKOUT_MS - (millis() - loginLockoutStartMs)) / 1000;
      char body[48];
      snprintf(body, sizeof(body), "{\"retryAfterSec\":%lu}", (unsigned long)retryAfterSec);
      request->send(429, "application/json", body);
      return;
    }
  }

  String password = request->hasArg("password") ? request->arg("password") : "";
  if (password.length() > 0 && password == SETTINGS_PASSWORD) {
    loginFailCount = 0;
    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "ok");
    startSession(response);
    request->send(response);
    return;
  }

  loginFailCount++;
  if (loginFailCount >= LOGIN_MAX_FAILS) {
    loginLockoutActive = true;
    loginLockoutStartMs = millis();
    loginFailCount = 0;
  }
  request->send(401, "text/plain", "unauthorized");
}

static void handleLogout(AsyncWebServerRequest *request) {
  sessionToken[0] = '\0';
  request->send(200, "text/plain", "ok");
}

static void handleStatus(AsyncWebServerRequest *request) {
  if (!isAuthed(request)) {
    request->send(401, "text/plain", "unauthorized");
    return;
  }

  bool apActive = WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA;
  String staSsid = wifiConnected ? WiFi.SSID() : savedStaSsid;
  String staIp = wifiConnected ? WiFi.localIP().toString() : "";
  long lastHeardSecsAgo = completedFirstSensorReading ? (long)((millis() - lastHeardFromSensorTime) / 1000) : -1;

  char body[512];
  snprintf(body, sizeof(body),
           "{\"wifiConnected\":%s,\"apActive\":%s,\"staSsid\":\"%s\",\"staIp\":\"%s\","
           "\"wifiTest\":\"%s\",\"sensorWaterLevel\":%.2f,\"sensorVoltage\":%.2f,"
           "\"sensorPercentage\":%.1f,\"preferredWaterLevel\":%.1f,\"inFreezeProtect\":%s,"
           "\"isFilling\":%s,\"fillingPaused\":%s,\"lastHeardSecsAgo\":%ld,"
           "\"carbonHost\":\"%s\",\"carbonPort\":%u,\"uptimeMs\":%lu}",
           wifiConnected ? "true" : "false", apActive ? "true" : "false", jsonEscape(staSsid).c_str(),
           staIp.c_str(), wifiTestStateName(), sensorWaterLevel, sensorVoltage, sensorPercentage,
           getPreferredWaterLevel(), getFreezeProtectState() ? "true" : "false", isFilling ? "true" : "false",
           fillingPaused ? "true" : "false", lastHeardSecsAgo, jsonEscape(carbonHost).c_str(), carbonPort,
           (unsigned long)millis());
  request->send(200, "application/json", body);
}

static void handleLevel(AsyncWebServerRequest *request) {
  if (!isAuthed(request)) {
    request->send(401, "text/plain", "unauthorized");
    return;
  }
  float delta = request->hasArg("delta") ? request->arg("delta").toFloat() : 0.0f;
  adjustDesiredWaterLevel(delta);
  checkFilling();
  request->send(200, "text/plain", "ok");
}

static void handleFreeze(AsyncWebServerRequest *request) {
  if (!isAuthed(request)) {
    request->send(401, "text/plain", "unauthorized");
    return;
  }
  toggleFreezeProtect();
  checkFilling();
  request->send(200, "text/plain", "ok");
}

static void handleWifiConfig(AsyncWebServerRequest *request) {
  if (!isAuthed(request)) {
    request->send(401, "text/plain", "unauthorized");
    return;
  }
  String ssid = request->hasArg("ssid") ? request->arg("ssid") : "";
  String pass = request->hasArg("password") ? request->arg("password") : "";
  if (ssid.length() == 0) {
    request->send(400, "text/plain", "ssid required");
    return;
  }
  beginWifiTest(ssid, pass);
  request->send(200, "text/plain", "testing");
}

static void handleCarbonConfig(AsyncWebServerRequest *request) {
  if (!isAuthed(request)) {
    request->send(401, "text/plain", "unauthorized");
    return;
  }
  String host = request->hasArg("host") ? request->arg("host") : "";
  int port = request->hasArg("port") ? request->arg("port").toInt() : 0;
  if (host.length() == 0 || port <= 0 || port > 65535) {
    request->send(400, "text/plain", "invalid host/port");
    return;
  }
  setCarbonConfig(host, (uint16_t)port);
  request->send(200, "text/plain", "ok");
}

static void handleReboot(AsyncWebServerRequest *request) {
  if (!isAuthed(request)) {
    request->send(401, "text/plain", "unauthorized");
    return;
  }
  rebootPending = true;
  rebootRequestedMs = millis();
  request->send(200, "text/plain", "rebooting");
}

static void registerRoutes() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/login", HTTP_POST, handleLogin);
  server.on("/logout", HTTP_POST, handleLogout);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/level", HTTP_POST, handleLevel);
  server.on("/api/freeze", HTTP_POST, handleFreeze);
  server.on("/api/wifi", HTTP_POST, handleWifiConfig);
  server.on("/api/carbon", HTTP_POST, handleCarbonConfig);
  server.on("/api/reboot", HTTP_POST, handleReboot);
}

// ===================== Public API =====================

void begin() {
  prefs.begin("wcv9", false);
  apConfirmed = prefs.getBool("ap_confirmed", false);
  savedStaSsid = prefs.getString("sta_ssid", "");
  savedStaPass = prefs.getString("sta_pass", "");
  carbonHost = prefs.getString("carbon_host", CARBON_CACHE_HOSTNAME_DEFAULT);
  carbonPort = prefs.getUShort("carbon_port", CARBON_CACHE_PORT_DEFAULT);

  registerRoutes();

  if (apConfirmed && savedStaSsid.length() > 0) {
    WiFi.mode(WIFI_STA);
    beginStaConnect(savedStaSsid, savedStaPass);
    Serial.printf("Connecting to WiFi \"%s\"...\n", savedStaSsid.c_str());
  } else {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID);
    Serial.printf("Access point \"%s\" started, IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  }

  server.begin();
}

void poll() {
  serviceWifiTestStateMachine();
  serviceWifiConnectedEdge();
  serviceStaReconnect();
  if (rebootPending && millis() - rebootRequestedMs > REBOOT_DELAY_MS) {
    ESP.restart();
  }
}

String getCarbonHost() { return carbonHost; }

uint16_t getCarbonPort() { return carbonPort; }

void setCarbonConfig(const String &host, uint16_t port) {
  carbonHost = host;
  carbonPort = port;
  prefs.putString("carbon_host", host);
  prefs.putUShort("carbon_port", port);
}

void resetWifiAndReboot() {
  prefs.remove("sta_ssid");
  prefs.remove("sta_pass");
  prefs.putBool("ap_confirmed", false);
  ESP.restart();
}

} // namespace WebPortal
