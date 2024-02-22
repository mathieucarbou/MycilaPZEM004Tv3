#include <Arduino.h>
#include <ArduinoJson.h>
#include <MycilaPZEM004Tv3.h>

Mycila::PZEM pzem1; // 0x01
Mycila::PZEM pzem2; // 0x02

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pzem1.begin(&Serial1, 27, 14, 0x01, true);
  pzem2.begin(&Serial1, 27, 14, 0x02, true);
}

void loop() {
  delay(1000);
  if (pzem1.isEnabled()) {
    JsonDocument doc;
    pzem1.toJson(doc.to<JsonObject>());
    Serial.print("0x01: ");
    serializeJson(doc, Serial);
    Serial.println();
  }

  if (pzem2.isEnabled()) {
    JsonDocument doc;
    pzem2.toJson(doc.to<JsonObject>());
    Serial.print("0x02: ");
    serializeJson(doc, Serial);
    Serial.println();
  }
}
