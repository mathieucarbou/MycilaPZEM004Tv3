/*
 * This example shows how to set an address to a PZEM004Tv3 device.
 *
 * The circuit:
 * - PZEM004Tv3 connected to Serial1 (RX=14, TX=27)
 */
#include <Arduino.h>
#include <ArduinoJson.h>
#include <MycilaPZEM.h>

Mycila::PZEM pzem;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    continue;

  JsonDocument doc;

  pzem.begin(Serial1, 14, 27);
  uint8_t address = pzem.getDeviceAddress();

  pzem.read();
  pzem.toJson(doc.to<JsonObject>());
  Serial.printf("0x%02X ", pzem.getDeviceAddress());
  serializeJson(doc, Serial);
  Serial.println();

  address += 100;
  if (pzem.setDeviceAddress(address)) {
    Serial.printf("Address set to 0x%02X\n", pzem.getDeviceAddress());
  }

  pzem.read();
  pzem.toJson(doc.to<JsonObject>());
  Serial.printf("0x%02X ", pzem.getDeviceAddress());
  serializeJson(doc, Serial);
  Serial.println();

  // uint8_t finalAddress = 0x01;
  // uint8_t finalAddress = 0x02;
  uint8_t finalAddress = 0x03;
  if (pzem.setDeviceAddress(finalAddress)) {
    Serial.printf("Address set to 0x%02X\n", pzem.getDeviceAddress());
  }

  pzem.end();
}

void loop() {
  vTaskDelete(NULL);
}
