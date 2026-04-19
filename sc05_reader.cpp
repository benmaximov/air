#include "sc05_reader.h"

#include <HardwareSerial.h>

#define DEBUG_SENSOR 0

#if DEBUG_SENSOR
#define DBG_PRINT(...) Serial.print(__VA_ARGS__)
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#define DBG_PRINTF(...)
#endif

// SC05-H2S UART on dedicated Serial2
// ESP32 side: RX=GPIO47, TX=GPIO48
static const int SC05_RX_PIN = 47;
static const int SC05_TX_PIN = 48;
static const uint32_t SC05_BAUD = 9600;

static HardwareSerial sc05_uart(2);
static Sc05ReadingCallback g_callback = nullptr;

static uint32_t g_last_valid_ms = 0;
static const uint32_t SC05_TIMEOUT_MS = 7000;

static void emit_status(SensorStatus status, uint16_t ppm) {
  if (g_callback != nullptr) {
    g_callback(status, ppm);
  }
}

static uint8_t calc_checksum(const uint8_t *data, size_t len) {
  // checksum = (~sum(bytes 1..len-2)) + 1
  uint16_t sum = 0;
  for (size_t i = 1; i + 1 < len; i++) {
    sum += data[i];
  }
  return static_cast<uint8_t>((~sum + 1) & 0xFF);
}

static bool read_binary_frame(uint8_t out[9]) {
  static uint8_t frame[9];
  static uint8_t idx = 0;

  while (sc05_uart.available() > 0) {
    const uint8_t b = static_cast<uint8_t>(sc05_uart.read());

    if (idx == 0) {
      if (b != 0xFF) {
        continue;
      }
      frame[idx++] = b;
      continue;
    }

    frame[idx++] = b;

    if (idx < 9) {
      continue;
    }

    idx = 0;

    const uint8_t cs = calc_checksum(frame, 9);
    if (cs != frame[8]) {
      DBG_PRINTF("SC05 RX bad checksum: calc=%02X got=%02X\r\n", cs, frame[8]);
      continue;
    }

    if (frame[1] != 0x86 && frame[1] != 0x17) {
      continue;
    }

    memcpy(out, frame, 9);
    DBG_PRINTF(
        "SC05 RX frame: %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
        out[0], out[1], out[2], out[3], out[4], out[5], out[6], out[7], out[8]);
    return true;
  }

  return false;
}

static uint16_t parse_binary_ppm(const uint8_t frame[9]) {
  if (frame[1] == 0x86) {
    return static_cast<uint16_t>((frame[2] << 8) | frame[3]);
  }

  if (frame[1] == 0x17) {
    return static_cast<uint16_t>((frame[4] << 8) | frame[5]);
  }

  return SC05_INVALID_PPM;
}

static bool parse_ascii_ppm(uint16_t &ppm_out) {
  static char line[32];
  static uint8_t idx = 0;

  while (sc05_uart.available() > 0) {
    const char c = static_cast<char>(sc05_uart.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      line[idx] = '\0';
      idx = 0;

      uint16_t value = 0;
      bool seen_digit = false;
      for (uint8_t i = 0; line[i] != '\0'; i++) {
        if (line[i] >= '0' && line[i] <= '9') {
          seen_digit = true;
          value = static_cast<uint16_t>(value * 10 + (line[i] - '0'));
        } else if (seen_digit) {
          break;
        }
      }

      if (seen_digit) {
        ppm_out = value;
        return true;
      }

      return false;
    }

    if (idx < sizeof(line) - 1) {
      line[idx++] = c;
    } else {
      idx = 0;
      return false;
    }
  }

  return false;
}

void init_sc05() {
  sc05_uart.begin(SC05_BAUD, SERIAL_8N1, SC05_RX_PIN, SC05_TX_PIN);
  DBG_PRINTF("SC05 UART init: RX=%d TX=%d baud=%lu\r\n", SC05_RX_PIN, SC05_TX_PIN, SC05_BAUD);

  while (sc05_uart.available() > 0) {
    sc05_uart.read();
  }

  g_last_valid_ms = millis();
}

void set_sc05_callback(Sc05ReadingCallback callback) {
  g_callback = callback;
}

void poll_sc05() {
  const uint32_t now = millis();

  uint8_t frame[9];
  if (read_binary_frame(frame)) {
    const uint16_t ppm = parse_binary_ppm(frame);
    if (ppm != SC05_INVALID_PPM) {
      g_last_valid_ms = now;
      DBG_PRINTF("SC05 H2S=%u ppm (binary)\r\n", ppm);
      emit_status(SensorStatus::OK, ppm);
      return;
    }
  }

  uint16_t ppm_ascii = 0;
  if (parse_ascii_ppm(ppm_ascii)) {
    g_last_valid_ms = now;
    DBG_PRINTF("SC05 H2S=%u ppm (ascii)\r\n", ppm_ascii);
    emit_status(SensorStatus::OK, ppm_ascii);
    return;
  }

  if (now - g_last_valid_ms > SC05_TIMEOUT_MS) {
    DBG_PRINT("SC05 timeout: no valid UART data\r\n");
    emit_status(SensorStatus::TIMEOUT, SC05_INVALID_PPM);
    g_last_valid_ms = now;
  }
}
