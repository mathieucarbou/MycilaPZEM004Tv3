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

  pzem.setCallback([](const Mycila::PZEM::EventType eventType) {
    if (eventType == Mycila::PZEM::EventType::EVT_CHANGE) {
      Serial.printf(" - %" PRIu32 " EVT_CHANGE: %f V, %f A, %f W\n", millis(), pzem.data.voltage, pzem.data.current, pzem.data.activePower);
    }
  });

  pzem.begin(Serial1, 14, 27, PZEM_ADDRESS, true);

  pinMode(RELAY_PIN, OUTPUT);
}

bool state = LOW;

void loop() {
  state = !state;
  digitalWrite(RELAY_PIN, state);
  delay(5000);
}
