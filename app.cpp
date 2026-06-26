#include <Arduino.h>

#include "dht22_reader.h"
#include "fan.h"
#include "mq137_reader.h"
#include "mq7_heater.h"
#include "sc05_reader.h"
#include "scd4x_reader.h"
#include "vfd.h"
#include "wifi_server.h"
#include "zc13_reader.h"

// Set to 1 to enable ADC calibration mode — samples GPIO5 raw and prints mV, skips all other code
#define ADC_CAL_MODE 0

// {label, value, current_value, threshold, normal_ms, alarm_ms}
// threshold=0 means no alarm (T, RH)
DisplayMessage messages[] = {
  {"CH\x04",  "---- ppm",    0.0f,   100.0f, 3000,  15000}, // CH4  alarm >200 ppm (basement gas installation — early warning)
  {"H\x02S",  "---- ppm",    0.0f,     1.0f, 3000,  15000}, // H2S  alarm >1 ppm
  {"CO\x02",  "---- ppm",    0.0f,  1000.0f, 3000,  15000}, // CO2  alarm >1000 ppm
  {"T",       "---- \x01""C",0.0f,     0.0f, 3000,   3000}, // T    no alarm
  {"RH",      "---- %",      0.0f,     0.0f, 3000,   3000}, // RH   no alarm
  {"CO",      "---- ppm",    0.0f,    10.0f, 3000,  15000}, // CO   alarm >10 ppm
  {"NH\x03",  "---- ppm",   0.0f,    25.0f, 3000,  15000}, // NH3  alarm >25 ppm
};
extern const size_t messages_count = sizeof(messages) / sizeof(messages[0]);

static void on_zc13_reading(SensorStatus status, uint16_t ch4_ppm) {
  if (status == SensorStatus::OK) {
    messages[0].value         = String(ch4_ppm) + " ppm";
    messages[0].current_value = (float)ch4_ppm;
  } else {
    messages[0].value         = "---- ppm";
    messages[0].current_value = 0.0f;
  }
}

static void on_sc05_reading(SensorStatus status, float h2s_ppm) {
  if (status == SensorStatus::OK) {
    messages[1].value         = String(h2s_ppm, 2) + " ppm";
    messages[1].current_value = h2s_ppm;
  } else {
    messages[1].value         = "---- ppm";
    messages[1].current_value = 0.0f;
  }
}

static void on_scd4x_reading(
    SensorStatus status,
    uint16_t co2_ppm,
    float temperature_c,
    float humidity_rh) {
  (void)temperature_c;
  (void)humidity_rh;
  if (status == SensorStatus::OK) {
    messages[2].value         = String(co2_ppm) + " ppm";
    messages[2].current_value = (float)co2_ppm;
  } else {
    messages[2].value         = "---- ppm";
    messages[2].current_value = 0.0f;
  }
}

static void on_dht22_reading(SensorStatus status, float temperature_c, float humidity_rh) {
  if (status == SensorStatus::OK) {
    messages[3].value         = String(temperature_c, 1) + " \x01""C";
    messages[3].current_value = temperature_c;
    messages[4].value         = String(humidity_rh, 1) + " %";
    messages[4].current_value = humidity_rh;
  } else {
    messages[3].value         = "---- \x01""C";
    messages[3].current_value = 0.0f;
    messages[4].value         = "---- %";
    messages[4].current_value = 0.0f;
  }
}

static void on_mq7_reading(SensorStatus status, float ppm) {
  if (status == SensorStatus::OK) {
    messages[5].value         = String(ppm, 1) + " ppm";
    messages[5].current_value = ppm;
  } else {
    messages[5].value         = "---- ppm";
    messages[5].current_value = 0.0f;
  }
}

static void on_mq137_reading(SensorStatus status, float ppm) {
  if (status == SensorStatus::OK) {
    messages[6].value         = String(ppm, 1) + " ppm";
    messages[6].current_value = ppm;
  } else {
    messages[6].value         = "---- ppm";
    messages[6].current_value = 0.0f;
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

#if ADC_CAL_MODE
  analogSetPinAttenuation(5, ADC_11db);
  Serial.println("ADC_CAL_MODE: sampling GPIO5 every 500ms");
  return;
#endif

  init_vfd();

  set_mq7_callback(on_mq7_reading);
  init_mq7_heater();

  set_scd4x_callback(on_scd4x_reading);
  init_scd4x();

  set_dht22_callback(on_dht22_reading);
  init_dht22();

  set_zc13_callback(on_zc13_reading);
  init_zc13();

  set_sc05_callback(on_sc05_reading);
  init_sc05();

  set_mq137_callback(on_mq137_reading);
  init_mq137();

  init_wifi_server();

  init_fan();
}

void loop() {
#if ADC_CAL_MODE
  const float mv = analogReadMilliVolts(5);
  Serial.printf("GPIO5 ADC = %.2f mV\r\n", mv);
  delay(500);
  return;
#endif

  const size_t count = messages_count;

  poll_wifi_server();
  poll_scd4x();
  poll_zc13();
  poll_sc05();
  poll_mq7_heater();
  poll_mq137();
  poll_dht22();
  poll_fan();
  cycle(messages, count);
}
