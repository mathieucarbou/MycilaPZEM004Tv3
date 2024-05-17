/*
  * This example shows how to reset the energy counter of the PZEM004Tv3
  * when the energy consumed is greater than 0.002 kWh.
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

  pzem.begin(Serial1, 14, 27);
}

void loop() {
  delay(1000);

  if (pzem.read()) {

    JsonDocument doc;
    pzem.toJson(doc.to<JsonObject>());
    serializeJson(doc, Serial);
    Serial.println();
  }

  if (pzem.getEnergy() > 0.002)
    pzem.resetEnergy();
}
