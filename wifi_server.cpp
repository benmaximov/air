#include "wifi_server.h"

#include <WiFi.h>
#include <WebServer.h>

#include "display_message.h"
#include "fan.h"
#include "relay.h"
#include "mq7_heater.h"
#include "mq137_reader.h"
#include "scd4x_reader.h"

#define DEBUG_WIFI 1

#if DEBUG_WIFI
#define DBG_PRINT(...)  Serial.print(__VA_ARGS__)
#define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#define DBG_PRINTF(...)
#endif

static const char* WIFI_SSID = "Socket3";
static const char* WIFI_PASS = "mercedes*";
static const int   HTTP_PORT = 8080;

static WebServer g_server(HTTP_PORT);

// Reference to messages array defined in app.cpp
extern DisplayMessage messages[];
extern const size_t    messages_count;

// VFD custom chars → plain ASCII
// \x01 = degree C, \x02 = 2, \x03 = 3, \x04 = 4
static String sanitize_vfd(const String& s)
{
  static const char* map[] = {
    "",   // \x01 → degree symbol, already followed by C → drop it, gives "25.5 C"
    "2",  // \x02 → subscript 2
    "3",  // \x03 → subscript 3
    "4",  // \x04 → subscript 4
  };
  String out;
  for (size_t i = 0; i < s.length(); i++)
  {
    const char c = s[i];
    if (c >= 0x01 && c <= 0x04)
      out += map[(uint8_t)c - 1];
    else
      out += c;
  }
  return out;
}

static void handle_calibrate()
{
  // Apply any R0 updates from query params
  // Usage: /calibrate?mq7=31855  or  /calibrate?mq137=85000  or both
  if (g_server.hasArg("mq7"))
  {
    const float val = g_server.arg("mq7").toFloat();
    if (val > 0) mq7_set_r0(val);
  }
  if (g_server.hasArg("mq137"))
  {
    const float val = g_server.arg("mq137").toFloat();
    if (val > 0) mq137_set_r0(val);
  }

  // Always return current state
  String json = "{";

  json += "\"MQ7\":{";
  json += "\"r0_active\":"    + String(mq7_get_r0(),           0) + ",";
  json += "\"r0_suggested\":" + String(mq7_get_suggested_r0(), 0);
  json += "},";

  json += "\"MQ137\":{";
  json += "\"r0_active\":"    + String(mq137_get_r0(),           0) + ",";
  json += "\"r0_suggested\":" + String(mq137_get_suggested_r0(), 0);
  json += "}";

  json += "}";
  g_server.send(200, "application/json", json);
}

static void handle_calibrate_co2()
{
  // Usage: /calibrate_co2?ppm=420
  // Performs forced recalibration to the given reference CO2 ppm.
  // The sensor must have been running for at least 3 minutes in stable conditions.
  if (!g_server.hasArg("ppm"))
  {
    g_server.send(400, "application/json", "{\"error\":\"missing ppm parameter\"}");
    return;
  }

  const int ppm = g_server.arg("ppm").toInt();
  if (ppm < 400 || ppm > 2000)
  {
    g_server.send(400, "application/json", "{\"error\":\"ppm must be 400-2000\"}");
    return;
  }

  int16_t result = scd4x_force_recalibration((uint16_t)ppm);

  String json = "{\"reference_ppm\":" + String(ppm);
  if (result == (int16_t)0x7fff) {
    json += ",\"success\":false,\"error\":\"recalibration failed\"";
  } else {
    json += ",\"success\":true,\"correction\":" + String((int)result - 0x8000);
  }
  json += "}";
  g_server.send(200, "application/json", json);
}

static void handle_fan()
{
  // SET: /fan?duty=128  (0-255)
  if (g_server.hasArg("duty"))
  {
    const int val = g_server.arg("duty").toInt();
    fan_set_duty((uint8_t)constrain(val, 0, 255));
  }

  // GET: always return current duty
  String json = "{\"duty\":" + String(fan_get_duty()) + "}";
  g_server.send(200, "application/json", json);
}

static void handle_relay()
{
  // SET: /relay?state=1 or /relay?state=0
  if (g_server.hasArg("state"))
  {
    const int val = g_server.arg("state").toInt();
    relay_set_state(val != 0);
  }

  // GET: always return current state
  String json = "{\"state\":" + String(relay_get_state() ? 1 : 0) + "}";
  g_server.send(200, "application/json", json);
}

static void handle_root()
{
  String json = "{";
  for (size_t i = 0; i < messages_count; i++)
  {
    if (i > 0) json += ",";
    json += "\"";
    json += sanitize_vfd(messages[i].label);
    json += "\":{\"value\":\"";
    json += sanitize_vfd(messages[i].value);
    json += "\",\"raw\":";
    json += messages[i].valid ? String(messages[i].current_value, 2) : "null";
    json += ",\"threshold\":";
    json += String(messages[i].threshold, 2);
    json += ",\"alarm\":";
    json += (messages[i].valid && messages[i].threshold > 0 && messages[i].current_value >= messages[i].threshold) ? "true" : "false";
    json += "}";
  }
  json += "}";

  g_server.send(200, "application/json", json);
}

void init_wifi_server()
{
  DBG_PRINTF("WiFi connecting to %s\r\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  const uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000)
  {
    delay(500);
    DBG_PRINT(".");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    DBG_PRINTF("\r\nWiFi connected: %s  port %d\r\n", WiFi.localIP().toString().c_str(), HTTP_PORT);
  }
  else
  {
    DBG_PRINT("\r\nWiFi connect failed — server disabled\r\n");
    return;
  }

  g_server.on("/",              HTTP_GET, handle_root);
  g_server.on("/fan",          HTTP_GET, handle_fan);
  g_server.on("/relay",        HTTP_GET, handle_relay);
  g_server.on("/calibrate",    HTTP_GET, handle_calibrate);
  g_server.on("/calibrate_co2", HTTP_GET, handle_calibrate_co2);
  g_server.begin();
  DBG_PRINTF("HTTP server started on port %d\r\n", HTTP_PORT);
}

void poll_wifi_server()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    g_server.handleClient();
  }
}
