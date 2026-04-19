#ifndef VFD_H
#define VFD_H

#include <Arduino.h>

#include "display_message.h"

void init_vfd();
void cycle(const DisplayMessage *messages, size_t count, uint32_t interval_ms);

#endif
