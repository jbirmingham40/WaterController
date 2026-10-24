#pragma once

#include <Arduino.h>

// Why the firmware itself chose to restart. Stored in RTC memory that
// survives ESP.restart() (but not a power cycle), read back once in setup(),
// and reported on /api/status and as autofill.controller.restart_cause - the
// only way to tell the restarts apart in the field, since Serial output is
// dropped with no monitor attached.
enum RestartCause : uint32_t {
  RESTART_CAUSE_NONE = 0, // not a firmware-requested restart (power-on, watchdog, crash)
  RESTART_CAUSE_SCHEDULED = 1,
  RESTART_CAUSE_SENSOR_SILENCE = 2,
  RESTART_CAUSE_WEB_REQUEST = 3,
  RESTART_CAUSE_WIFI_RESET = 4,
  RESTART_CAUSE_RADIO_INIT_FAILED = 5,
};

// Records the cause, then ESP.restart(). Does not return.
[[noreturn]] void restartWithCause(RestartCause cause);
