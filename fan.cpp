#include "fan.h"

static const int     FAN_PIN        = 6;
static const uint32_t FAN_ON_MS    = 30000;  // 30 seconds on
static const uint32_t FAN_OFF_MS   = 10000;  // 10 seconds off

static bool     g_fan_on       = true;
static uint32_t g_phase_start  = 0;

void init_fan() {
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, HIGH);
  g_fan_on      = true;
  g_phase_start = millis();
}

void poll_fan() {
  const uint32_t now     = millis();
  const uint32_t elapsed = now - g_phase_start;

  if (g_fan_on && elapsed >= FAN_ON_MS) {
    digitalWrite(FAN_PIN, LOW);
    g_fan_on      = false;
    g_phase_start = now;
  } else if (!g_fan_on && elapsed >= FAN_OFF_MS) {
    digitalWrite(FAN_PIN, HIGH);
    g_fan_on      = true;
    g_phase_start = now;
  }
}
