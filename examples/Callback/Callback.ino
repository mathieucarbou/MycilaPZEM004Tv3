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
    int64_t now = esp_timer_get_time();
    switch (eventType) {
      case Mycila::PZEMEventType::EVT_READ:
        Serial.printf(" - %" PRId64 " EVT_READ\n", now);
        break;
      case Mycila::PZEMEventType::EVT_CHANGE:
        Serial.printf(" - %" PRIu32 " EVT_CHANGE: %f V, %f A, %f W\n", millis(), pzem.getVoltage(), pzem.getCurrent(), pzem.getPower());
        break;
      default:
        Serial.printf(" - %" PRId64 " ERR\n", now);
        break;
    }
  });

  pzem.begin(&Serial1, 14, 27, PZEM_ADDRESS, true);

  pinMode(RELAY_PIN, OUTPUT);

  digitalWrite(RELAY_PIN, HIGH);

  int64_t start = esp_timer_get_time();
  while (esp_timer_get_time() - start < (int64_t)2000000) {
    pzem.read();
  }

  digitalWrite(RELAY_PIN, LOW);

  start = esp_timer_get_time();
  while (esp_timer_get_time() - start < (int64_t)2000000) {
    pzem.read();
  }

  pzem.end();
}

void loop() {
  vTaskDelete(NULL);
}
