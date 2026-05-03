#include "dht22_reader.h"

#include <DHT.h>

#define DEBUG_SENSOR 0

#if DEBUG_SENSOR
#define DBG_PRINT(...)  Serial.print(__VA_ARGS__)
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#define DBG_PRINTF(...)
#endif

static const int DHT22_PIN = 14;
static const uint32_t POLL_INTERVAL_MS = 2500; // DHT22 max sample rate ~0.5 Hz — 2500ms avoids MQ7 1000ms ADC alignment

static DHT g_dht(DHT22_PIN, DHT22);
static Dht22ReadingCallback g_callback = nullptr;
static uint32_t g_last_poll_ms = 0;

static void emit_status(SensorStatus status, float t, float rh)
{
  if (g_callback != nullptr)
  {
    g_callback(status, t, rh);
  }
}

void set_dht22_callback(Dht22ReadingCallback callback)
{
  g_callback = callback;
}

void init_dht22()
{
  g_dht.begin();
  g_last_poll_ms = 0;
  DBG_PRINTF("DHT22 init: pin=%d\r\n", DHT22_PIN);
}

void poll_dht22()
{
  const uint32_t now = millis();
  if (g_last_poll_ms != 0 && now - g_last_poll_ms < POLL_INTERVAL_MS)
  {
    return;
  }
  g_last_poll_ms = now;

  float t = g_dht.readTemperature();
  float rh = g_dht.readHumidity();

  if (isnan(t) || isnan(rh))
  {
    DBG_PRINT("DHT22 read failed\r\n");
    emit_status(SensorStatus::ERROR, 0.0f, 0.0f);
    return;
  }

  DBG_PRINTF("DHT22: T=%.1f C, RH=%.1f %%\r\n", t, rh);
  emit_status(SensorStatus::OK, t, rh);
}
