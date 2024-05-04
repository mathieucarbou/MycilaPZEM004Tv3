#include <Arduino.h>
#include <ArduinoJson.h>
#include <MycilaPZEM004Tv3.h>

// Pin: Relay  (ESP32)
#define RELAY_PIN 26
// #define RELAY_PIN 32

#define PZEM_ADDRESS 0x02

Mycila::PZEM pzem;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pzem.setCallback([](const Mycila::PZEMEventType eventType) {
    if (eventType == Mycila::PZEMEventType::EVT_CHANGE) {
      Serial.printf(" - %" PRIu32 " EVT_CHANGE: %f V, %f A, %f W\n", millis(), pzem.getVoltage(), pzem.getCurrent(), pzem.getPower());
    }
  });

  pzem.begin(&Serial1, 14, 27, PZEM_ADDRESS, true);

  pinMode(RELAY_PIN, OUTPUT);
}

bool state = LOW;

void loop() {
  state = !state;
  digitalWrite(RELAY_PIN, state);
  delay(5000);
}
