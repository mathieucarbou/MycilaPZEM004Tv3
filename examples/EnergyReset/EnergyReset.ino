/*
 * This example shows how to reset the energy counter of the PZEM004Tv3
 * when the energy consumed is greater than 0.002 kWh.
 *
 * The circuit:
 * - PZEM004Tv3 connected to Serial1 (RX=14, TX=27)
 *
 * Compile with -D MYCILA_JSON_SUPPORT to enable JSON support
 * Add ArduinoJson library to your project
 */
#include <Arduino.h>
#include <ArduinoJson.h>
#include <MycilaPZEM004Tv3.h>

Mycila::PZEM pzem;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  JsonDocument doc;

  pzem.begin(Serial1, 14, 27, 0x01);

  pzem.read();
  pzem.toJson(doc.to<JsonObject>());
  serializeJson(doc, Serial);
  Serial.println();

  pzem.resetEnergy();

  pzem.read();
  pzem.toJson(doc.to<JsonObject>());
  serializeJson(doc, Serial);
  Serial.println();
}

void loop() {
  vTaskDelete(NULL);
}
