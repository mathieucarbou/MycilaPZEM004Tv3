#include <Arduino.h>
#include <MycilaPZEM004Tv3.h>

#define TARGET_ADDRESS 0x02

Mycila::PZEM pzem1;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pzem1.begin(&Serial1, 27, 14);

  while (true) {
    uint8_t address = pzem1.readAddress();
    Serial.printf("Address: 0x%02X\n", address);
    if (address == TARGET_ADDRESS) {
      break;
    }
    if (address != MYCILA_PZEM_INVALID_ADDRESS)
      pzem1.setAddress(TARGET_ADDRESS);
    delay(1000);
  }

  pzem1.end();

  pzem1.begin(&Serial1, 27, 14, TARGET_ADDRESS);
}

void loop() {
  delay(1000);
  if (pzem1.read()) {
    uint8_t address = pzem1.readAddress();
    Serial.printf("Address: 0x%02X\n", address);

    Serial.print("Power: ");
    Serial.print(pzem1.getPower());
    Serial.println("W");

    Serial.print("Voltage: ");
    Serial.print(pzem1.getVoltage());
    Serial.println("V");
  }
}
