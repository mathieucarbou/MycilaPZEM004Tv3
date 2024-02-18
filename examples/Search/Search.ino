#include <Arduino.h>
#include <MycilaPZEM004Tv3.h>

Mycila::PZEM pzem;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pzem.begin(&Serial1, 27, 14);

  uint8_t addresses[255];
  size_t count = pzem.search(addresses, 255);

  Serial.printf("Found %d devices\n", count);
  for (size_t i = 0; i < count; i++) {
    Serial.printf("Address: 0x%02X\n", addresses[i]);
  }

  pzem.end();
}

void loop() {
  vTaskDelete(NULL);
}
