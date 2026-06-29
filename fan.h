#ifndef FAN_H
#define FAN_H

#include <Arduino.h>

void    init_fan();
void    poll_fan();
uint8_t fan_get_duty();       // 0-255
void    fan_set_duty(uint8_t duty);

#endif
