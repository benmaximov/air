#include "vfd.h"

static const uint8_t VFD_CS = 10;
static const uint8_t VFD_SDI = 11;
static const uint8_t VFD_CLK = 12;
static const uint8_t VFD_WIDTH = 16;

static void spi_write_byte(uint8_t value) {
  for (uint8_t i = 0; i < 8; i++) {
    digitalWrite(VFD_CLK, LOW);
    digitalWrite(VFD_SDI, (value & 0x01) ? HIGH : LOW);
    delayMicroseconds(2);
    digitalWrite(VFD_CLK, HIGH);
    delayMicroseconds(2);
    value >>= 1;
  }
}

static void vfd_cmd(uint8_t cmd) {
  digitalWrite(VFD_CS, LOW);
  spi_write_byte(cmd);
  digitalWrite(VFD_CS, HIGH);
  delayMicroseconds(5);
}

static void vfd_show() {
  vfd_cmd(0xE8);
}

static uint8_t reverse7(uint8_t v) {
  uint8_t out = 0;
  for (uint8_t i = 0; i < 7; i++) {
    if (v & (1U << i)) {
      out |= (1U << (6 - i));
    }
  }
  return out;
}

static void vfd_write_user_glyph(uint8_t slot, const uint8_t glyph[7]) {
  digitalWrite(VFD_CS, LOW);
  spi_write_byte(0x40 + slot); // CGRAM slot 0..7
  for (uint8_t i = 0; i < 5; i++) {
    spi_write_byte(reverse7(glyph[i]));
  }
  digitalWrite(VFD_CS, HIGH);
  vfd_show();
}

static void vfd_init_16_digit() {
  digitalWrite(VFD_CS, LOW);
  spi_write_byte(0xE0);
  delayMicroseconds(5);
  spi_write_byte(0x0F);
  digitalWrite(VFD_CS, HIGH);
  delayMicroseconds(5);

  digitalWrite(VFD_CS, LOW);
  spi_write_byte(0xE4);
  delayMicroseconds(5);
  spi_write_byte(0x80); // ~50% brightness
  digitalWrite(VFD_CS, HIGH);
  delayMicroseconds(5);

  vfd_show();
}

static String format_line(const DisplayMessage &message) {
  String left = message.label;
  String right = message.value;

  if (right.length() >= VFD_WIDTH) {
    return right.substring(right.length() - VFD_WIDTH);
  }

  if (left.length() >= VFD_WIDTH) {
    return left.substring(0, VFD_WIDTH);
  }

  size_t spaces = 1;
  if (left.length() + right.length() < VFD_WIDTH) {
    spaces = VFD_WIDTH - left.length() - right.length();
  }

  String line = left;
  for (size_t i = 0; i < spaces; i++) {
    line += ' ';
  }
  line += right;

  if (line.length() > VFD_WIDTH) {
    line = line.substring(0, VFD_WIDTH);
  }

  return line;
}

static void vfd_write_padded(const String &text) {
  digitalWrite(VFD_CS, LOW);
  spi_write_byte(0x20);

  const size_t n = text.length();
  const size_t visible = (n > VFD_WIDTH) ? VFD_WIDTH : n;
  const size_t left_pad = VFD_WIDTH - visible;
  const size_t start = n - visible;

  for (uint8_t i = 0; i < VFD_WIDTH; i++) {
    char ch = ' ';
    if (i >= left_pad) {
      ch = text[start + (i - left_pad)];
    }
    spi_write_byte(static_cast<uint8_t>(ch));
  }

  digitalWrite(VFD_CS, HIGH);
  vfd_show();
}

void init_vfd() {
  pinMode(VFD_CS, OUTPUT);
  pinMode(VFD_SDI, OUTPUT);
  pinMode(VFD_CLK, OUTPUT);
  digitalWrite(VFD_CS, HIGH);
  digitalWrite(VFD_CLK, HIGH);
  digitalWrite(VFD_SDI, LOW);

  delay(100);
  vfd_init_16_digit();

  // CGRAM expects column-wise bytes (bottom->top bits), not row-wise.
  // slot 1: superscript-like "2" (working)
  //static const uint8_t glyph_sup2[5] = {0x00, 0x4C, 0x54, 0x54, 0x24};
  static const uint8_t glyph_sup2[5] = {0x00, 0x13, 0x15, 0x15, 0x09};
  // slot 2: degree symbol (working)
  static const uint8_t glyph_deg[5] = {0x30, 0x48, 0x48, 0x30, 0x00};
  // slot 3: superscript-like "4" tuned to match sample
  static const uint8_t glyph_sup4[5] = {0x00, 0x06, 0x0A, 0x12, 0x07};
  vfd_write_user_glyph(1, glyph_sup2);
  vfd_write_user_glyph(2, glyph_deg);
  vfd_write_user_glyph(3, glyph_sup4);

  vfd_write_padded(" ");
}

void cycle(const DisplayMessage *messages, size_t count, uint32_t interval_ms) {
  static size_t index = 0;
  static uint32_t last_ms = 0;

  if (count == 0 || messages == nullptr) {
    return;
  }

  const uint32_t now = millis();
  if (last_ms == 0 || now - last_ms >= interval_ms) {
    const String line = format_line(messages[index]);
    vfd_write_padded(line);
    //Serial.print("VFD: ");
    //Serial.print(line);
    //Serial.print("\r\n");

    index++;
    if (index >= count) {
      index = 0;
    }
    last_ms = now;
  }
}
