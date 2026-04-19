#ifndef SCD4X_READER_H
#define SCD4X_READER_H

#include <Arduino.h>

#include "sensor_status.h"

typedef void (*Scd4xReadingCallback)(
    SensorStatus status,
    uint16_t co2_ppm,
    float temperature_c,
    float humidity_rh);

void init_scd4x();
void set_scd4x_callback(Scd4xReadingCallback callback);
void poll_scd4x();

#endif
