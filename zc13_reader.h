#ifndef ZC13_READER_H
#define ZC13_READER_H

#include <Arduino.h>

#include "sensor_status.h"

typedef void (*Zc13ReadingCallback)(SensorStatus status, uint16_t ch4_ppm);

static const uint16_t ZC13_INVALID_PPM = 0xFFFF;

void init_zc13();
void set_zc13_callback(Zc13ReadingCallback callback);
void poll_zc13();

#endif
