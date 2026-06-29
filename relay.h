#ifndef RELAY_H
#define RELAY_H

#include <Arduino.h>

void init_relay();
void poll_relay();
bool relay_get_state();        // true = HIGH, false = LOW
void relay_set_state(bool on);

#endif
