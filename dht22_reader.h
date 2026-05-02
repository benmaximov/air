#ifndef DHT22_READER_H
#define DHT22_READER_H

#include <Arduino.h>

#include "sensor_status.h"

typedef void (*Dht22ReadingCallback)(
    SensorStatus status,
    float temperature_c,
    float humidity_rh);

void init_dht22();
void set_dht22_callback(Dht22ReadingCallback callback);
void poll_dht22();

#endif
