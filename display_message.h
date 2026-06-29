#ifndef DISPLAY_MESSAGE_H
#define DISPLAY_MESSAGE_H

#include <Arduino.h>

struct DisplayMessage {
  String label;
  String value;
  float   current_value; // numeric reading, for threshold comparison
  float   threshold;     // alarm level; 0 = no alarm for this entry
  uint32_t normal_ms;    // display dwell when not in alarm
  uint32_t alarm_ms;     // display dwell when this entry is in alarm
  bool    valid;         // true once sensor has reported a valid reading
};

#endif
