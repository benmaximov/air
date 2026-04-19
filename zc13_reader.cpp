#include "zc13_reader.h"

#include <HardwareSerial.h>

#define DEBUG_SENSOR 0

#if DEBUG_SENSOR
#define DBG_PRINT(...) Serial.print(__VA_ARGS__)
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#define DBG_PRINTF(...)
#endif

// ZC13 UART: 9600 8N1
// User wiring: module RX/TX to ESP32 GPIO1/GPIO2.
// ESP32 side: RX pin gets module TX, TX pin goes to module RX.
static const int ZC13_RX_PIN = 1;
static const int ZC13_TX_PIN = 2;
static const uint32_t ZC13_BAUD = 9600;

static HardwareSerial zc13_uart(1);
static Zc13ReadingCallback g_callback = nullptr;

static void emit_status(SensorStatus status, uint16_t ch4_ppm) {
  if (g_callback != nullptr) {
    g_callback(status, ch4_ppm);
  }
}

static uint32_t g_last_query_ms = 0;
static const uint32_t QUERY_INTERVAL_MS = 1200;

static const uint8_t QUERY_FRAME[9] = {
    0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

static uint32_t g_last_frame_ms = 0;

static uint8_t calc_checksum(const uint8_t *data, size_t len) {
  // Datasheet: checksum = (~sum(bytes 1..len-2)) + 1
  uint16_t sum = 0;
  for (size_t i = 1; i + 1 < len; i++) {
    sum += data[i];
  }
  return static_cast<uint8_t>((~sum + 1) & 0xFF);
}

static bool read_frame(uint8_t out[9]) {
  while (zc13_uart.available() >= 9) {
    uint8_t b0 = static_cast<uint8_t>(zc13_uart.read());
    if (b0 != 0xFF) {
      continue;
    }

    out[0] = b0;
    size_t got = zc13_uart.readBytes(reinterpret_cast<char *>(&out[1]), 8);
    if (got != 8) {
      return false;
    }

    const uint8_t expected = calc_checksum(out, 9);
    if (expected != out[8]) {
      continue;
    }

    // Accepted frames:
    // - Query response mode: FF 86 ...
    // - Active upload mode:   FF 01 03 ...
    if (out[1] == 0x86 || (out[1] == 0x01 && out[2] == 0x03)) {
      return true;
    }
  }
  return false;
}

static uint16_t parse_ch4_ppm(const uint8_t frame[9]) {
  // Query response mode: concentration in byte2/3 (13-bit, high 5 bits + low byte)
  if (frame[1] == 0x86) {
    return static_cast<uint16_t>((frame[2] & 0x1F) * 256U + frame[3]);
  }

  // Active upload mode: concentration in byte4/5
  if (frame[1] == 0x01 && frame[2] == 0x03) {
    return static_cast<uint16_t>((frame[4] & 0x1F) * 256U + frame[5]);
  }

  return 0;
}

void init_zc13() {
  zc13_uart.begin(ZC13_BAUD, SERIAL_8N1, ZC13_RX_PIN, ZC13_TX_PIN);
  DBG_PRINTF("ZC13 UART init: RX=%d TX=%d baud=%lu\r\n", ZC13_RX_PIN, ZC13_TX_PIN, ZC13_BAUD);

  while (zc13_uart.available() > 0) {
    zc13_uart.read();
  }

  g_last_query_ms = 0;
  g_last_frame_ms = millis();
}

void set_zc13_callback(Zc13ReadingCallback callback) {
  g_callback = callback;
}

void poll_zc13() {
  const uint32_t now = millis();

  // Sensor may be in active upload mode; don't spam query frames.
  if (g_last_query_ms == 0 || now - g_last_query_ms >= QUERY_INTERVAL_MS) {
    zc13_uart.write(QUERY_FRAME, sizeof(QUERY_FRAME));
    g_last_query_ms = now;
  }

  uint8_t frame[9];
  if (!read_frame(frame)) {
    if (now - g_last_frame_ms > 5000) {
      DBG_PRINT("ZC13 timeout: no valid frame for >5s\r\n");
      emit_status(SensorStatus::TIMEOUT, ZC13_INVALID_PPM);
      g_last_frame_ms = now;
    }
    return;
  }

  g_last_frame_ms = now;

  bool sensor_fault = false;
  if (frame[1] == 0x86) {
    sensor_fault = (frame[2] & 0x80U) != 0;
  } else if (frame[1] == 0x01 && frame[2] == 0x03) {
    sensor_fault = (frame[4] & 0x80U) != 0;
  }

  const uint16_t ch4_ppm = parse_ch4_ppm(frame);

  if (sensor_fault) {
    DBG_PRINT("ZC13 error: sensor fault bit set\r\n");
    emit_status(SensorStatus::ERROR, ZC13_INVALID_PPM);
    return;
  }

  // zero is valid for CH4; pass through as OK
  DBG_PRINTF("ZC13 CH4=%u ppm\r\n", ch4_ppm);
  emit_status(SensorStatus::OK, ch4_ppm);
}
