#include <Arduino.h>

#include "dht22_reader.h"
#include "mq7_heater.h"
#include "sc05_reader.h"
#include "scd4x_reader.h"
#include "vfd.h"
#include "zc13_reader.h"

// {label, value, current_value, threshold, normal_ms, alarm_ms}
// threshold=0 means no alarm (T, RH)
DisplayMessage messages[] = {
  {"CH\x03",  "---- ppm",    0.0f,   100.0f, 3000,  12000}, // CH4  alarm >100 ppm
  {"H\x01S",  "---- ppm",    0.0f,     5.0f, 3000,  12000}, // H2S  alarm >5 ppm
  {"CO\x01",  "---- ppm",    0.0f,  1000.0f, 3000,  12000}, // CO2  alarm >1000 ppm
  {"T",       "---- \x02""C",0.0f,     0.0f, 3000,   3000}, // T    no alarm
  {"RH",      "---- %",      0.0f,     0.0f, 3000,   3000}, // RH   no alarm
  {"CO",      "---- ppm",    0.0f,    10.0f, 3000,  12000}, // CO   alarm >10 ppm
};

static void on_zc13_reading(SensorStatus status, uint16_t ch4_ppm) {
  if (status == SensorStatus::OK) {
    messages[0].value         = String(ch4_ppm) + " ppm";
    messages[0].current_value = (float)ch4_ppm;
  } else {
    messages[0].value         = "---- ppm";
    messages[0].current_value = 0.0f;
  }
}

static void on_sc05_reading(SensorStatus status, uint16_t h2s_ppm) {
  if (status == SensorStatus::OK) {
    messages[1].value         = String(h2s_ppm) + " ppm";
    messages[1].current_value = (float)h2s_ppm;
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
    messages[3].value         = String(temperature_c, 1) + " \x02""C";
    messages[3].current_value = temperature_c;
    messages[4].value         = String(humidity_rh, 1) + " %";
    messages[4].current_value = humidity_rh;
  } else {
    messages[3].value         = "---- \x02""C";
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

void setup() {
  Serial.begin(115200);
  delay(100);

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
}

void loop() {
  const size_t count = sizeof(messages) / sizeof(messages[0]);

  poll_scd4x();
  poll_zc13();
  poll_sc05();
  poll_mq7_heater();
  poll_dht22();
  cycle(messages, count);
}
