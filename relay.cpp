#include "relay.h"

static const int RELAY_PIN = 17;

static bool g_state = false;  // start OFF (LOW)

void init_relay() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
}

void poll_relay() {
  // Nothing to poll — state is set on demand via RPC
}

bool relay_get_state() {
  return g_state;
}

void relay_set_state(bool on) {
  g_state = on;
  digitalWrite(RELAY_PIN, g_state ? HIGH : LOW);
}
