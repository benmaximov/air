#include "mq7_heater.h"

// MQ-7 heater drive on GPIO4
static const int MQ7_PWM_PIN = 4;
static const int MQ7_PWM_CHANNEL = 3;
static const int MQ7_PWM_FREQ_HZ = 5000;
static const int MQ7_PWM_RES_BITS = 8;

// Duty targets
static const int MQ7_DUTY_HEAT = 255; // 100%
static const int MQ7_DUTY_LOW = 10;    // ~2.7%

// Phase timing
static const uint32_t HEAT_PHASE_MS = 60000;    // 60s
static const uint32_t WAIT_PHASE_MS = 30000;    // 30s
static const uint32_t MEASURE_PHASE_MS = 60000; // 60s

// MQ-7 analog sense input on GPIO5 (ADC1_4)
static const int MQ7_ADC_PIN = 5;
static uint32_t g_last_adc_ms = 0;
static const uint32_t ADC_PRINT_INTERVAL_MS = 1000;

// Calibration constants
static const float RL = 43000.0f;    // 33k + 10k
static const float R0 = 110810.0f;   // baseline in clean air
static const float VIN_MV = 4400.0f; // 5V actual
static const float DIVIDER_MULT = 4.3f;

enum class Mq7Phase
{
  HEAT,
  WAIT,
  MEASURE,
};

static Mq7Phase g_phase = Mq7Phase::HEAT;
static uint32_t g_phase_start_ms = 0;
static Mq7ReadingCallback g_callback = nullptr;

static void emit_status(SensorStatus status, float ppm)
{
  if (g_callback != nullptr)
  {
    g_callback(status, ppm);
  }
}

static void set_heater_duty(int duty)
{
  ledcWrite(MQ7_PWM_CHANNEL, duty);
}

void set_mq7_callback(Mq7ReadingCallback callback)
{
  g_callback = callback;
}

void init_mq7_heater()
{
  ledcSetup(MQ7_PWM_CHANNEL, MQ7_PWM_FREQ_HZ, MQ7_PWM_RES_BITS);
  ledcAttachPin(MQ7_PWM_PIN, MQ7_PWM_CHANNEL);

  analogSetPinAttenuation(MQ7_ADC_PIN, ADC_11db);

  g_phase = Mq7Phase::HEAT;
  g_phase_start_ms = millis();
  g_last_adc_ms = 0;
  set_heater_duty(MQ7_DUTY_HEAT);

  Serial.print("MQ7 phase: HEAT (60s, 100%)\r\n");
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
      Serial.print("MQ7 phase: WAIT (30s, 7%)\r\n");
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
      Serial.print("MQ7 phase: MEASURE (60s, 7%)\r\n");
    }
    return;
  }

  if (g_phase == Mq7Phase::MEASURE)
  {
    if (g_last_adc_ms == 0 || now - g_last_adc_ms >= ADC_PRINT_INTERVAL_MS)
    {
      g_last_adc_ms = now;

      // 1. Get the voltage at the ESP32 pin in mV
      float pin_mv = analogReadMilliVolts(MQ7_ADC_PIN);

      // 2. Calculate actual voltage (before the divider)
      float v_out = pin_mv * DIVIDER_MULT;
      Serial.printf("--- MQ7 sensor_v_out = %.2f mV\r\n", v_out);

      // 3. Prevent division by zero if sensor is disconnected
      if (v_out <= 0)
        v_out = 1.0;

      // 4. Calculate Sensor Resistance (Rs)
      // Formula: Rs = ((Vin - Vout) / Vout) * RL
      float rs = ((VIN_MV - v_out) / v_out) * RL;

      // 5. Calculate Ratio (Rs/R_clean)
      // Ensure R0 is set to your stable "Clean Air" resistance (e.g., 110810.0)
      float ratio = rs / R0;
      if (ratio > 1.0)
        ratio = 1.0; // Clamping: If Rs > R0, we are definitely in clean air

      // 6. Calculate PPM
      // We use a modified power law so that Clean Air (Ratio 1.0) starts near 0.
      // Standard MQ-7 Curve: PPM = 9.838 * pow(ratio, -1.45) - 10.0;
      float ppm = 9.838 * pow(ratio, -1.45) - 10.0;

      // Safety: Don't show negative numbers
      if (ppm < 0)
        ppm = 0;

      Serial.printf("--- MQ7 actual ppm=%.1f\r\n", ppm);
      emit_status(SensorStatus::OK, ppm);
    }
  }

  if (now - g_phase_start_ms >= MEASURE_PHASE_MS)
  {
    g_phase = Mq7Phase::HEAT;
    g_phase_start_ms = now;
    g_last_adc_ms = 0;
    set_heater_duty(MQ7_DUTY_HEAT);
    Serial.print("MQ7 phase: HEAT (60s, 100%)\r\n");
    // emit_status(SensorStatus::TIMEOUT, 0.0f);
  }
}
