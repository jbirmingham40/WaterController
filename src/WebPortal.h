#pragma once

#include <Arduino.h>

// Fallback defaults for the Carbon Cache (Graphite line-protocol) receiver
// that sendMetric() (main.cpp) sends to. The actual host/port are
// configurable from the settings web page and persisted in NVS - these are
// only what a freshly-provisioned device starts with.
#define CARBON_CACHE_HOSTNAME_DEFAULT "grafana.jbirmingham.linkpc.net"
#define CARBON_CACHE_PORT_DEFAULT 2003

// Owns the device's WiFi AP/STA provisioning state machine and the settings
// web server (login, status polling, WiFi/Carbon Cache config, level/freeze
// controls, reboot). See src/WebPortal.cpp.
namespace WebPortal {

// Call once from setup(): loads persisted config, brings up AP or STA
// depending on whether WiFi has previously been confirmed working, and
// starts the web server.
void begin();

// Call once per loop() iteration: drives the WiFi-test-then-confirm state
// machine, detects WiFi connect/disconnect edges, and services pending
// reboot requests. Cheap when idle.
void poll();

String getCarbonHost();
uint16_t getCarbonPort();
void setCarbonConfig(const String &host, uint16_t port);

// Clears stored WiFi STA credentials + the "confirmed working" flag and
// restarts the device into AP mode. Called from the pad-3 double-press
// confirm flow in main.cpp. Does not return.
void resetWifiAndReboot();

} // namespace WebPortal
