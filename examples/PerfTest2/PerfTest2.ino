#include <ArduinoJson.h>
#include <MycilaPZEM.h>

// Pin: Relay  (ESP32)
#define RELAY_PIN 26
// #define RELAY_PIN 32

#define PZEM_ADDRESS 0x02

Mycila::PZEM pzem;
Mycila::PZEM::Data pzemData;
JsonDocument doc;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pzem.setCallback([](Mycila::PZEM::EventType eventType, const Mycila::PZEM::Data& data) {
    pzemData = data;
  });

  pzem.begin(Serial1, 14, 27, PZEM_ADDRESS, true);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);

  delay(1000);
  pzem.read();

  delay(1000);
  pzem.read();

  delay(1000);
  pzem.read();

  pzem.toJson(doc.to<JsonObject>());
  serializeJsonPretty(doc, Serial);
  Serial.println();

  constexpr float minimumLoadPower = 600;
  constexpr float toleratedDiff = minimumLoadPower / 100; // 1%

  // speed test

  Serial.printf("\npzem.read():\n");

  for (size_t rounds = 1; rounds <= 4; rounds++) {
    Serial.printf(" - ROUND: %d\n", rounds);

    digitalWrite(RELAY_PIN, LOW);
    while (pzemData.activePower > 0) {
      pzem.read();
    }

    float last = 0;
    float now = 0;

    int64_t reactivityTime = 0;
    size_t reactivityCount = 1;

    int64_t rampUpTime = 0;
    size_t rampUpCount = 1;

    int64_t rampDownTime = 0;
    size_t rampDownCount = 1;

    digitalWrite(RELAY_PIN, HIGH);
    int64_t start = esp_timer_get_time();
    while (true) {
      pzem.read();
      now = pzemData.activePower;

      if (reactivityTime == 0) {
        if (now > 0) {
          reactivityTime = esp_timer_get_time() - start;
        } else {
          reactivityCount++;
        }
      }

      if (now > minimumLoadPower && abs(now - last) <= toleratedDiff) {
        break;
      } else {
        last = now;
        rampUpCount++;
      }
    }
    rampUpTime = esp_timer_get_time() - start;

    Serial.printf("   * Load Detection time: %" PRId64 " us (%d reads)\n", reactivityTime, reactivityCount);
    Serial.printf("   * Ramp up time: %" PRId64 " us (%d reads)\n", rampUpTime, rampUpCount);

    digitalWrite(RELAY_PIN, LOW);
    start = esp_timer_get_time();
    while (true) {
      pzem.read();
      now = pzemData.activePower;

      if (now == 0) {
        break;
      } else {
        rampDownCount++;
      }
    }
    rampDownTime = esp_timer_get_time() - start;

    Serial.printf("   * Ramp down time: %" PRId64 " us (%d reads)\n", rampDownTime, rampDownCount);
  }
}

void loop() {
  vTaskDelete(NULL);
}
