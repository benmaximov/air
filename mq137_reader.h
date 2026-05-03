#ifndef MQ137_READER_H
#define MQ137_READER_H

#include <Arduino.h>

#include "sensor_status.h"

typedef void (*Mq137ReadingCallback)(SensorStatus status, float ppm);

void init_mq137();
void set_mq137_callback(Mq137ReadingCallback callback);
void poll_mq137();

#endif
