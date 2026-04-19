#ifndef SC05_READER_H
#define SC05_READER_H

#include <Arduino.h>

#include "sensor_status.h"

typedef void (*Sc05ReadingCallback)(SensorStatus status, uint16_t h2s_ppm);

static const uint16_t SC05_INVALID_PPM = 0xFFFF;

void init_sc05();
void set_sc05_callback(Sc05ReadingCallback callback);
void poll_sc05();

#endif
