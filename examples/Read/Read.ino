/*
  * This example shows how to read the energy data of a PZEM004Tv3
  *
  * The circuit:
  * - PZEM004Tv3 connected to Serial1 (RX=14, TX=27)
  *
  * Compile with -D MYCILA_PZEM_JSON_SUPPORT to enable JSON support
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

  pzem.begin(&Serial1, 14, 27);
}

void loop() {
  if (pzem.read()) {
    JsonDocument doc;
    pzem.toJson(doc.to<JsonObject>());
    serializeJson(doc, Serial);
    Serial.println();
  }
  delay(1000);
}
