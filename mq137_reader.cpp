#include "mq137_reader.h"

#define DEBUG_SENSOR 1

// Set to 1 to print suggested R0 at end of each averaging window for calibration
#define MQ137_CAL_MODE 1

#if DEBUG_SENSOR
#define DBG_PRINT(...)  Serial.print(__VA_ARGS__)
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#define DBG_PRINTF(...)
#endif

// MQ-137 analog sense input on GPIO7 (ADC1_CH6)
static const int MQ137_ADC_PIN = 7;

// Divider: 33kΩ + 10kΩ — same as MQ-7
static const float R_TOP        = 33000.0f; // top resistor (MQ-B to ADC)
static const float R_BOTTOM     = 10000.0f; // bottom resistor (ADC to GND)
static const float RL           = R_TOP + R_BOTTOM;
static const float DIVIDER_MULT = RL / R_BOTTOM;
static const float VIN_MV       = 4700.0f;  // measured at sensor heater / VCC pin

// Calibration — R0 in clean air, update after burn-in
static const float R0 = 12000.0f; // placeholder — run CAL_MODE to calibrate

// Sampling: average over a window, emit once per window
static const uint32_t SAMPLE_INTERVAL_MS = 1000;  // 1 sample/sec
static const uint32_t WINDOW_MS          = 30000; // 30s averaging window

static uint32_t g_last_sample_ms  = 0;
static uint32_t g_window_start_ms = 0;
static float    g_meas_sum        = 0.0f;
static uint32_t g_meas_count      = 0;

static Mq137ReadingCallback g_callback = nullptr;

static void emit_status(SensorStatus status, float ppm)
{
  if (g_callback != nullptr)
  {
    g_callback(status, ppm);
  }
}

void set_mq137_callback(Mq137ReadingCallback callback)
{
  g_callback = callback;
}

void init_mq137()
{
  analogSetPinAttenuation(MQ137_ADC_PIN, ADC_11db);

  g_last_sample_ms  = 0;
  g_window_start_ms = millis();
  g_meas_sum        = 0.0f;
  g_meas_count      = 0;

  DBG_PRINTF("MQ137 init: ADC=GPIO%d  window=%lus\r\n", MQ137_ADC_PIN, WINDOW_MS / 1000);
}

void poll_mq137()
{
  const uint32_t now = millis();

  // Accumulate one sample per second
  if (g_last_sample_ms == 0 || now - g_last_sample_ms >= SAMPLE_INTERVAL_MS)
  {
    g_last_sample_ms = now;

    const float pin_mv = analogReadMilliVolts(MQ137_ADC_PIN);
    const float v_out  = pin_mv * DIVIDER_MULT;
    const float v_rs   = VIN_MV - v_out;
    DBG_PRINTF("--- MQ137 sample v_out=%.1f mV  V_RS=%.1f mV\r\n", v_out, v_rs);

    if (pin_mv > 0)
    {
      g_meas_sum += pin_mv;
      g_meas_count++;
    }
  }

  // End of averaging window — compute and emit
  if (now - g_window_start_ms >= WINDOW_MS)
  {
    g_window_start_ms = now;

    if (g_meas_count > 0)
    {
      // 1. Average raw ADC reading
      const float avg_pin_mv = g_meas_sum / g_meas_count;

      // 2. Reconstruct voltage at MQ137-B
      float v_out = avg_pin_mv * DIVIDER_MULT;
      if (v_out <= 0) v_out = 1.0f;

      // 3. Sensor resistance
      const float rs = ((VIN_MV - v_out) / v_out) * RL;

      // 4. PPM via MQ-137 power law curve (NH3)
      // Datasheet curve: Rs/R0 vs ppm, fitted: ppm = A * (Rs/R0)^B
      // MQ-137 NH3 curve coefficients (Winsen datasheet)
      float ratio = rs / R0;
      if (ratio > 1.0f) ratio = 1.0f;
      float ppm = 26.572f * pow(ratio, -1.2481f) - 30.0f;
      if (ppm < 0) ppm = 0;

      DBG_PRINTF("--- MQ137 avg v_out=%.1f mV  Rs=%.0f  ppm=%.1f  (%lu samples)\r\n",
                 v_out, rs, ppm, g_meas_count);

#if MQ137_CAL_MODE
      DBG_PRINTF("\r\n*** MQ137 CAL: suggested R0 = %.0f (avg over %lu samples)\r\n", rs, g_meas_count);
      DBG_PRINTF("*** Update: static const float R0 = %.0ff;\r\n\r\n", rs);
#endif

      emit_status(SensorStatus::OK, ppm);
    }

    g_meas_sum   = 0.0f;
    g_meas_count = 0;
  }
}
