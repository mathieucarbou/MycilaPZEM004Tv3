/*
  * This example shows how to read the energy data of a PZEM004Tv3
  *
  * The circuit:
  * - PZEM004Tv3 connected to Serial1 (RX=27, TX=14)
  *
  * Compile with -D MYCILA_PZEM_JSON_SUPPORT to enable JSON support
  * Add ArduinoJson library to your project
*/
#include <Arduino.h>
#include <ArduinoJson.h>
#include <MycilaPZEM004Tv3.h>

#define PZEM_ADDRESS 0x01

#ifndef PZEM_ADDRESS
#define PZEM_ADDRESS MYCILA_PZEM_DEFAULT_ADDRESS
#endif

Mycila::PZEM pzem;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pzem.begin(&Serial1, 27, 14, PZEM_ADDRESS, true);
}

void loop() {
  delay(2000);
  if (pzem.isEnabled()) {
    JsonDocument doc;
    pzem.toJson(doc.to<JsonObject>());
    serializeJson(doc, Serial);
    Serial.println();
  }
}
