#include <Arduino.h>
#include <MycilaPZEM004Tv3.h>

#define TARGET_ADDRESS 0x42

Mycila::PZEM pzem;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pzem.begin(&Serial1, 27, 14);

  uint8_t addresses[MYCILA_PZEM_MAX_ADDRESS_COUNT];
  size_t count = pzem.search(addresses, MYCILA_PZEM_MAX_ADDRESS_COUNT);

  Serial.printf("Found %d devices\n", count);

  for (size_t i = 0; i < count; i++) {
    Serial.printf("Address: 0x%02X\n", addresses[i]);
  }

  for (size_t i = 0; i < count; i++) {
    if (addresses[i] == TARGET_ADDRESS) {
      continue;
    }
    Serial.printf("Resetting PZEM @ 0x%02X\n", addresses[i]);
    pzem.end();
    pzem.begin(&Serial1, 27, 14, addresses[i]);
    pzem.setAddress(TARGET_ADDRESS);
    pzem.end();
  }
}

void loop() {
  vTaskDelete(NULL);
}
