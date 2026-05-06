#include "scd4x_reader.h"

#include <DFRobot_SCD4X.h>
#include <Wire.h>

#define DEBUG_SENSOR 0

// Set to 1 to perform a one-shot forced recalibration after CALIBRATION_DELAY_MS
// of stable readings. Flash with this enabled while the sensor is OUTSIDE in
// fresh air, wait for the Serial message, then reflash with it set back to 0.
#define FORCED_CALIBRATION 0
#define FORCED_CALIBRATION_PPM 420
#define FORCED_CALIBRATION_DELAY_MS (5UL * 60UL * 1000UL) // 5 minutes

#if DEBUG_SENSOR
#define DBG_PRINT(...) Serial.print(__VA_ARGS__)
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#define DBG_PRINTF(...)
#endif

// ESP32-S3 I2C pins
static const uint8_t I2C_SDA_PIN = 8;
static const uint8_t I2C_SCL_PIN = 18; // SDA=8, SCL=18
static const uint8_t SCD4X_ADDR = SCD4X_I2C_ADDR;

static DFRobot_SCD4X g_scd4x(&Wire, SCD4X_ADDR);
static bool g_scd4x_ok = false;
static Scd4xReadingCallback g_reading_callback = nullptr;
#if FORCED_CALIBRATION
static bool g_calibration_done = false;
#endif

static void emit_status(SensorStatus status, uint16_t co2_ppm, float t, float rh) {
  if (g_reading_callback != nullptr) {
    g_reading_callback(status, co2_ppm, t, rh);
  }
}

// Reconnect behavior
static uint32_t g_last_init_try_ms = 0;
static uint32_t g_last_ready_ms = 0;
static const uint32_t INIT_RETRY_MS = 2000;
static const uint32_t READY_TIMEOUT_MS = 20000;

static bool scd4x_ping() {
  Wire.beginTransmission(SCD4X_ADDR);
  return (Wire.endTransmission() == 0);
}

static bool scd4x_try_init() {
  const uint32_t now = millis();
  if (now - g_last_init_try_ms < INIT_RETRY_MS) {
    return false;
  }
  g_last_init_try_ms = now;

  if (!scd4x_ping()) {
    DBG_PRINT("SCD4x not found on I2C\r\n");
    g_scd4x_ok = false;
    emit_status(SensorStatus::TIMEOUT, 0, 0.0f, 0.0f);
    return false;
  }

  if (!g_scd4x.begin()) {
    DBG_PRINT("SCD4x begin failed\r\n");
    g_scd4x_ok = false;
    emit_status(SensorStatus::ERROR, 0, 0.0f, 0.0f);
    return false;
  }

  // Set ambient pressure compensation (944 mbar = 94400 Pa).
  // The library divides by 100 internally before sending to the sensor register,
  // so pass Pa here. This corrects CO2 upward for our below-sea-level pressure.
  //g_scd4x.setAmbientPressure(94400);

  // Start periodic measurement mode.
  g_scd4x.enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);
  g_scd4x_ok = true;
  g_last_ready_ms = now;
  DBG_PRINT("SCD4x initialized and periodic mode started\r\n");
  return true;
}

void init_scd4x() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  DBG_PRINTF("SCD4x I2C init: SDA=%u SCL=%u\r\n", I2C_SDA_PIN, I2C_SCL_PIN);

  // Force immediate first init attempt.
  g_last_init_try_ms = 0;
  scd4x_try_init();
}

void set_scd4x_callback(Scd4xReadingCallback callback) {
  g_reading_callback = callback;
}

void poll_scd4x() {

  // If sensor is currently offline, keep trying to recover.
  if (!g_scd4x_ok) {
    scd4x_try_init();
    return;
  }

  // If sensor disappears from bus, mark offline and retry init later.
  if (!scd4x_ping()) {
    DBG_PRINT("SCD4x lost on I2C, reinitializing...\r\n");
    g_scd4x_ok = false;
    emit_status(SensorStatus::TIMEOUT, 0, 0.0f, 0.0f);
    return;
  }

  // Data ready status should eventually become true in periodic mode.
  if (!g_scd4x.getDataReadyStatus()) {
    const uint32_t now = millis();
    if (now - g_last_ready_ms > READY_TIMEOUT_MS) {
      DBG_PRINT("SCD4x timeout waiting for data, restarting sensor...\r\n");
      g_scd4x_ok = false;
      emit_status(SensorStatus::TIMEOUT, 0, 0.0f, 0.0f);
    }
    return;
  }

  DFRobot_SCD4X::sSensorMeasurement_t data;
  g_scd4x.readMeasurement(&data);
  g_last_ready_ms = millis();

  // Basic sanity check to detect invalid post-reset reads.
  if (data.CO2ppm == 0 || !isfinite(data.temp) || !isfinite(data.humidity)) {
    DBG_PRINT("SCD4x invalid read\r\n");
    emit_status(SensorStatus::ERROR, 0, 0.0f, 0.0f);
    return;
  }

  emit_status(SensorStatus::OK, data.CO2ppm, data.temp, data.humidity);

  DBG_PRINTF(
      "SCD4x: CO2=%u ppm, T=%.1f C, RH=%.1f %%\r\n",
      data.CO2ppm,
      data.temp,
      data.humidity);

#if FORCED_CALIBRATION
  if (!g_calibration_done && millis() >= FORCED_CALIBRATION_DELAY_MS) {
    Serial.printf(
        "SCD4x: starting forced recalibration to %u ppm (current: %u ppm)\r\n",
        (unsigned)FORCED_CALIBRATION_PPM, (unsigned)data.CO2ppm);
    g_scd4x.enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);
    int16_t correction = g_scd4x.performForcedRecalibration(FORCED_CALIBRATION_PPM);
    if (correction == (int16_t)0x7fff) {
      Serial.print("SCD4x: forced recalibration FAILED\r\n");
    } else {
      Serial.printf("SCD4x: forced recalibration done, correction=%d ppm\r\n", (int)correction - 0x8000);
    }
    g_scd4x.enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);
    g_calibration_done = true;
  }
#endif
}
