#include "mq7_heater.h"

#define DEBUG_SENSOR 1

// Set to 1 to average entire MEASURE phase and print suggested R0 for calibration
#define MQ7_CAL_MODE 1

#if DEBUG_SENSOR
#define DBG_PRINT(...)   Serial.print(__VA_ARGS__)
#define DBG_PRINTF(...)  Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#define DBG_PRINTF(...)
#endif

// MQ-7 heater drive
static const int MQ7_PWM_PIN      = 13;
static const int MQ7_PWM_FREQ_HZ  = 5000;
static const int MQ7_PWM_RES_BITS = 8;

// Supply and divider (must come before duty calculation)
static const float VIN_MV       = 4700.0f;  // measured at sensor VCC pin
static const float R_TOP        = 33000.0f; // top resistor (MQ-B to ADC)
static const float R_BOTTOM     = 10000.0f; // bottom resistor (ADC to GND)
static const float RL           = R_TOP + R_BOTTOM;
static const float DIVIDER_MULT = RL / R_BOTTOM;
static float       R0           = 20000.0f; // baseline in clean air (runtime-settable)

// Duty targets
static const float MQ7_HEATER_LOW_MV = 1500.0f; // target low temp heater voltage (datasheet: 1.5V)
static const int   MQ7_DUTY_HEAT     = 255;      // 100%
static const int   MQ7_DUTY_LOW      = (int)(MQ7_HEATER_LOW_MV / VIN_MV * 255.0f + 0.5f);

// Phase timing
static const uint32_t HEAT_PHASE_MS    = 60000; // 60s at 5V   (cleaning)
static const uint32_t WAIT_PHASE_MS    = 70000; // 70s at 1.5V (cooling, no read)
static const uint32_t MEASURE_PHASE_MS = 20000; // 20s at 1.5V (ceramic cooled, all readings valid)

// MQ-7 analog sense input on GPIO9 (ADC1_CH8)
static const int MQ7_ADC_PIN = 9;
static uint32_t g_last_adc_ms = 0;
static const uint32_t ADC_PRINT_INTERVAL_MS = 1000;

enum class Mq7Phase
{
  HEAT,
  WAIT,
  MEASURE,
};

static Mq7Phase  g_phase         = Mq7Phase::HEAT;
static uint32_t  g_phase_start_ms = 0;
static Mq7ReadingCallback g_callback = nullptr;

// Accumulator for MEASURE phase averaging (shared by normal + CAL mode)
static float    g_meas_sum        = 0.0f;
static uint32_t g_meas_count      = 0;
static float    g_suggested_r0    = 0.0f; // last CAL-calculated R0

static void emit_status(SensorStatus status, float ppm)
{
  if (g_callback != nullptr)
  {
    g_callback(status, ppm);
  }
}

static void set_heater_duty(int duty)
{
  ledcWrite(MQ7_PWM_PIN, duty);
}

void set_mq7_callback(Mq7ReadingCallback callback)
{
  g_callback = callback;
}

void init_mq7_heater()
{
  ledcAttach(MQ7_PWM_PIN, MQ7_PWM_FREQ_HZ, MQ7_PWM_RES_BITS);

  analogSetPinAttenuation(MQ7_ADC_PIN, ADC_11db);

  g_phase = Mq7Phase::HEAT;
  g_phase_start_ms = millis();
  g_last_adc_ms = 0;
  set_heater_duty(MQ7_DUTY_HEAT);

    DBG_PRINTF("MQ7 phase: HEAT (%lus, 100%%)\r\n", HEAT_PHASE_MS / 1000);
    // emit_status(SensorStatus::TIMEOUT, 0.0f);
}

void poll_mq7_heater()
{
  const uint32_t now = millis();

  if (g_phase == Mq7Phase::HEAT)
  {
    if (now - g_phase_start_ms >= HEAT_PHASE_MS)
    {
      g_phase = Mq7Phase::WAIT;
      g_phase_start_ms = now;
      g_last_adc_ms = 0;
      set_heater_duty(MQ7_DUTY_LOW);
      DBG_PRINTF("MQ7 phase: WAIT (%lus, %d%%)\r\n", WAIT_PHASE_MS / 1000, (MQ7_DUTY_LOW * 100) / 255);
      // emit_status(SensorStatus::TIMEOUT, 0.0f);
    }
    return;
  }

  if (g_phase == Mq7Phase::WAIT)
  {
    if (now - g_phase_start_ms >= WAIT_PHASE_MS)
    {
      g_phase = Mq7Phase::MEASURE;
      g_phase_start_ms = now;
      g_last_adc_ms = 0;
      set_heater_duty(MQ7_DUTY_LOW);
      DBG_PRINTF("MQ7 phase: MEASURE (%lus, %d%%)\r\n", MEASURE_PHASE_MS / 1000, (MQ7_DUTY_LOW * 100) / 255);
      g_meas_sum   = 0.0f;
      g_meas_count = 0;
    }
    return;
  }

  if (g_phase == Mq7Phase::MEASURE)
  {
    // Accumulate one raw ADC sample per second
    if (g_last_adc_ms == 0 || now - g_last_adc_ms >= ADC_PRINT_INTERVAL_MS)
    {
      g_last_adc_ms = now;

      const float pin_mv = analogReadMilliVolts(MQ7_ADC_PIN);
      const float v_out  = pin_mv * DIVIDER_MULT;
      const float v_rs   = VIN_MV - v_out;
      DBG_PRINTF("--- MQ7 sample v_out=%.1f mV  V_RS=%.1f mV\r\n", v_out, v_rs);

      if (pin_mv > 0)
      {
        g_meas_sum += pin_mv;
        g_meas_count++;
      }
    }
  }

  if (now - g_phase_start_ms >= MEASURE_PHASE_MS)
  {
    if (g_meas_count > 0)
    {
      // 1. Average raw ADC reading
      const float avg_pin_mv = g_meas_sum / g_meas_count;

      // 2. Reconstruct voltage at MQ7-B
      float v_out = avg_pin_mv * DIVIDER_MULT;
      if (v_out <= 0) v_out = 1.0f;

      // 3. Sensor resistance
      const float rs = ((VIN_MV - v_out) / v_out) * RL;

      // 4. PPM via power law
      float ratio = rs / R0;
      if (ratio > 1.0f) ratio = 1.0f;
      float ppm = 9.838f * pow(ratio, -1.45f) - 10.0f;
      if (ppm < 0) ppm = 0;

      DBG_PRINTF("--- MQ7 avg v_out=%.1f mV  Rs=%.0f  ppm=%.1f  (%lu samples)\r\n",
                 v_out, rs, ppm, g_meas_count);

#if MQ7_CAL_MODE
      g_suggested_r0 = rs;
      DBG_PRINTF("\r\n*** MQ7 CAL: suggested R0 = %.0f (avg over %lu samples)\r\n", rs, g_meas_count);
      DBG_PRINTF("*** Update: static const float R0 = %.0ff;\r\n\r\n", rs);
#endif

      emit_status(SensorStatus::OK, ppm);
    }

    g_phase = Mq7Phase::HEAT;
    g_phase_start_ms = now;
    g_last_adc_ms = 0;
    set_heater_duty(MQ7_DUTY_HEAT);
  DBG_PRINTF("MQ7 phase: HEAT (%lus, 100%%)\r\n", HEAT_PHASE_MS / 1000);
  }
}

float mq7_get_r0()           { return R0; }
float mq7_get_suggested_r0() { return g_suggested_r0; }
void  mq7_set_r0(float r0)   { R0 = r0; DBG_PRINTF("MQ7 R0 set to %.0f\r\n", R0); }
