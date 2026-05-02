#ifndef VFD_H
#define VFD_H

#include <Arduino.h>

#include "display_message.h"

void init_vfd();
// messages is non-const because cycle() temporarily mutates label for blink
void cycle(DisplayMessage *messages, size_t count);

#endif
