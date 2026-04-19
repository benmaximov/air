#include <Arduino.h>

#include "mq7_heater.h"
#include "sc05_reader.h"
#include "scd4x_reader.h"
#include "vfd.h"
#include "zc13_reader.h"

DisplayMessage messages[] = {
  {"CH\x03", "---- ppm"},
  {"H\x01S", "---- ppm"},
  {"CO\x01", "---- ppm"},
  {"T", "---- \x02" "C"},
  {"RH", "---- %"},
  {"CO", "---- ppm"},
};

static void on_zc13_reading(SensorStatus status, uint16_t ch4_ppm) {
  if (status == SensorStatus::OK) {
    messages[0].value = String(ch4_ppm) + " ppm";
  } else {
    messages[0].value = "---- ppm";
  }
}

static void on_sc05_reading(SensorStatus status, uint16_t h2s_ppm) {
  if (status == SensorStatus::OK) {
    messages[1].value = String(h2s_ppm) + " ppm";
  } else {
    messages[1].value = "---- ppm";
  }
}

static void on_scd4x_reading(
    SensorStatus status,
    uint16_t co2_ppm,
    float temperature_c,
    float humidity_rh) {
  if (status == SensorStatus::OK) {
    messages[2].value = String(co2_ppm) + " ppm";
    messages[3].value = String(temperature_c, 1) + " \x02" "C";
    messages[4].value = String(humidity_rh, 1) + " %";
  } else {
    messages[2].value = "---- ppm";
    messages[3].value = "---- \x02" "C";
    messages[4].value = "---- %";
  }
}

static void on_mq7_reading(SensorStatus status, float ppm, int raw, int mv) {
  (void)raw;
  (void)mv;

  if (status == SensorStatus::OK) {
    messages[5].value = String(ppm, 1) + " ppm";
  } else {
    messages[5].value = "---- ppm";
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
  cycle(messages, count, 5000);
}
