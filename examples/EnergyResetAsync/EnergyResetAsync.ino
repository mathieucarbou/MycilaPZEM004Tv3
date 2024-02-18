#include <Arduino.h>
#include <ArduinoJson.h>
#include <MycilaPZEM004Tv3.h>

Mycila::PZEM pzem;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pzem.begin(&Serial1, 27, 14, MYCILA_PZEM_DEFAULT_ADDRESS, true);
}

void loop() {
  delay(1000);

  if (pzem.isEnabled()) {
    JsonDocument doc;
    pzem.toJson(doc.to<JsonObject>());
    serializeJson(doc, Serial);
    Serial.println();

    if (pzem.getEnergy() > 0.002)
      pzem.resetEnergy();
  }
}
