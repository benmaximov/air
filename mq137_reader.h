#ifndef MQ137_READER_H
#define MQ137_READER_H

#include <Arduino.h>

#include "sensor_status.h"

typedef void (*Mq137ReadingCallback)(SensorStatus status, float ppm);

void  init_mq137();
void  set_mq137_callback(Mq137ReadingCallback callback);
void  poll_mq137();
float mq137_get_r0();
float mq137_get_suggested_r0();
void  mq137_set_r0(float r0);

#endif
