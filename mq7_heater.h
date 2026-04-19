#ifndef MQ7_HEATER_H
#define MQ7_HEATER_H

#include <Arduino.h>

#include "sensor_status.h"

typedef void (*Mq7ReadingCallback)(SensorStatus status, float ppm);

void init_mq7_heater();
void set_mq7_callback(Mq7ReadingCallback callback);
void poll_mq7_heater();

#endif
