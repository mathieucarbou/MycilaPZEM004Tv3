#include <ArduinoJson.h>
#include <MycilaPZEM004Tv3.h>

// Pin: Relay  (ESP32)
#define RELAY_PIN 26
// #define RELAY_PIN 32

#define PZEM_ADDRESS 0x02

Mycila::PZEM pzem;
JsonDocument doc;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

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

  // speed test

  Serial.printf("\npzem.read():\n");

  // Serial.printf(" -");
  int64_t times[50];
  for (size_t j = 0; j < 50; j++) {
    times[j] = esp_timer_get_time();
    pzem.read();
    times[j] = esp_timer_get_time() - times[j];
    // Serial.printf(" %" PRId64, times[j]);
  }
  // Serial.println();

  digitalWrite(RELAY_PIN, LOW);

  // compute
  double sum = 0;
  size_t n = 0;
  size_t err = 0;
  int64_t min = INT64_MAX;
  int64_t max = 0;
  for (size_t j = 0; j < 50; j++) {
    if (times[j] > 0) {
      sum += times[j];
      n++;
      min = min < times[j] ? min : times[j];
      max = max > times[j] ? max : times[j];
    } else {
      err++;
    }
  }

  // show results
  Serial.printf(" - Errors: %d\n", err);
  Serial.printf(" - Average read time: %" PRId64 " us\n", static_cast<int64_t>(sum / n));
  Serial.printf(" - Min read time: %" PRId64 " us\n", (min == INT64_MAX) ? 0 : min);
  Serial.printf(" - Max read time: %" PRId64 " us\n", max);
}

void loop() {
  vTaskDelete(NULL);
}
