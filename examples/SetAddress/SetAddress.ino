/*
  * This example shows how to set an address to a PZEM004Tv3 device.
  *
  * The circuit:
  * - PZEM004Tv3 connected to Serial1 (RX=27, TX=14)
*/
#include <Arduino.h>
#include <MycilaPZEM004Tv3.h>

#define TARGET_ADDRESS 0x42

Mycila::PZEM pzem;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  pzem.begin(&Serial1, 27, 14);

  while (true) {
    uint8_t address = pzem.readAddress();
    Serial.printf("Address: 0x%02X\n", address);
    if (address == TARGET_ADDRESS) {
      break;
    }
    if (address != MYCILA_PZEM_INVALID_ADDRESS)
      pzem.setAddress(TARGET_ADDRESS);
    delay(1000);
  }

  pzem.end();

  pzem.begin(&Serial1, 27, 14, TARGET_ADDRESS);
}

void loop() {
  delay(1000);
  if (pzem.read()) {
    uint8_t address = pzem.readAddress();
    Serial.printf("Address: 0x%02X\n", address);

    Serial.print("Power: ");
    Serial.print(pzem.getPower());
    Serial.println("W");

    Serial.print("Voltage: ");
    Serial.print(pzem.getVoltage());
    Serial.println("V");
  }
}
