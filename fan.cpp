#include "fan.h"

static const int      FAN_PIN      = 6;
static const uint32_t FAN_FREQ_HZ  = 25000;  // 25 kHz PWM
static const uint8_t  FAN_RES_BITS = 8;      // 0-255 duty range

static uint8_t g_duty = 15;

void init_fan() {
  ledcAttach(FAN_PIN, FAN_FREQ_HZ, FAN_RES_BITS);
  ledcWrite(FAN_PIN, g_duty);
}

void poll_fan() {
  // Nothing to poll — duty is set on demand
}

uint8_t fan_get_duty() {
  return g_duty;
}

void fan_set_duty(uint8_t duty) {
  g_duty = duty;
  ledcWrite(FAN_PIN, g_duty);
}
