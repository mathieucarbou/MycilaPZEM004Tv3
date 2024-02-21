// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou and others
 */
#include "MycilaPZEM004Tv3.h"

#define TAG "PZEM004T"

#define PZEM_BAUD_RATE 9600

#define PZEM_CMD_RHR  0x03
#define PZEM_CMD_RIR  0X04
#define PZEM_CMD_WSR  0x06
#define PZEM_CMD_REST 0x42

#define PZEM_WREG_ADDR 0x0002

#define PZEM_READ_RESPONSE_SIZE  25
#define PZEM_RESET_RESPONSE_SIZE 5
#define PZEM_ADDR_RESPONSE_SIZE  7

// Pre computed CRC table
static const uint16_t crcTable[] PROGMEM = {
  0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
  0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
  0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
  0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
  0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
  0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
  0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
  0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
  0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
  0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
  0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
  0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
  0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
  0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
  0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
  0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
  0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
  0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
  0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
  0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
  0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
  0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
  0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
  0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
  0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
  0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
  0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
  0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
  0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
  0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
  0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
  0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040};

void Mycila::PZEM::begin(HardwareSerial* serial,
                         const uint8_t pzemRXPin,
                         const uint8_t pzemTXPin,
                         const uint8_t address,
                         const bool async,
                         const uint8_t core,
                         const uint32_t stackSize) {
  if (_enabled)
    return;

  if (serial == nullptr) {
    ESP_LOGE(TAG, "Disable PZEM: Invalid serial");
    return;
  }

  if (GPIO_IS_VALID_OUTPUT_GPIO(pzemRXPin)) {
    _pinRX = (gpio_num_t)pzemRXPin;
  } else {
    ESP_LOGE(TAG, "Disable PZEM: Invalid PZEM RX pin: %u", _pinRX);
    _pinRX = GPIO_NUM_NC;
    return;
  }

  if (GPIO_IS_VALID_GPIO(pzemTXPin)) {
    _pinTX = (gpio_num_t)pzemTXPin;
  } else {
    ESP_LOGE(TAG, "Disable PZEM: Invalid PZEM TX pin: %u", _pinTX);
    _pinTX = GPIO_NUM_NC;
    return;
  }

  if (address < 0x01 || address > MYCILA_PZEM_DEFAULT_ADDRESS) {
    ESP_LOGE(TAG, "Disable PZEM: Invalid address: 0x%02X", address);
    return;
  }

  ESP_LOGI(TAG, "Enable PZEM @ 0x%02X...", address);
  ESP_LOGD(TAG, "- PZEM RX Pin (Serial TX): %u", _pinRX);
  ESP_LOGD(TAG, "- PZEM TX Pin (Serial RX): %u", _pinTX);
  ESP_LOGD(TAG, "- Async: %s", async ? "true" : "false");

  _serial = serial;
  _address = address;

  _openSerial();

  if (!_canRead()) {
    ESP_LOGW(TAG, "Unable to read PZEM at address 0x%02X. Please verify that the device is powered and that its address is correctly set.", address);
  }

  assert(!async || xTaskCreateUniversal(_pzemTask, "pzemTask", stackSize, this, MYCILA_PZEM__ASYNC_PRIORITY, &_taskHandle, core) == pdPASS);

  _enabled = true;
}

void Mycila::PZEM::end() {
  if (_enabled) {
    ESP_LOGI(TAG, "Disable PZEM at address 0x%02X...", _address);
    _enabled = false;
    _address = MYCILA_PZEM_DEFAULT_ADDRESS;
    while (_taskHandle != NULL) {
      // PZEM takes a few ms to finish a read
      delay(50);
    }
    _current = 0;
    _frequency = 0;
    _power = 0;
    _powerFactor = 0;
    _voltage = 0;
    _serial->end();
  }
}

bool Mycila::PZEM::read() {
  if (!_enabled)
    return false;

  if (!_mutex.try_lock_for(std::chrono::milliseconds(1000))) {
    ESP_LOGW(TAG, "Cannot read: Serial is busy!");
    return false;
  }

  _sendCmd8(PZEM_CMD_RIR, 0x00, 0x0A, false);

  uint8_t buffer[PZEM_READ_RESPONSE_SIZE];
  const size_t count = _timedRead(buffer, PZEM_READ_RESPONSE_SIZE);

  _mutex.unlock();

  if (!count) {
    _current = 0;
    _frequency = 0;
    _power = 0;
    _powerFactor = 0;
    _voltage = 0;
    // timeout or no electricity
    return false;
  }

  if (count != PZEM_READ_RESPONSE_SIZE) {
    _current = 0;
    _frequency = 0;
    _power = 0;
    _powerFactor = 0;
    _voltage = 0;
    ESP_LOGD(TAG, "Read failed: %d", count);
    return false;
  }

  _voltage = ((uint32_t)buffer[3] << 8 | (uint32_t)buffer[4]) / 10.0;                                                              // Raw voltage in 0.1V
  _current = ((uint32_t)buffer[5] << 8 | (uint32_t)buffer[6] | (uint32_t)buffer[7] << 24 | (uint32_t)buffer[8] << 16) / 1000.0;    // Raw current in 0.001A
  _power = ((uint32_t)buffer[9] << 8 | (uint32_t)buffer[10] | (uint32_t)buffer[11] << 24 | (uint32_t)buffer[12] << 16) / 10.0;     // Raw power in 0.1W
  _energy = ((uint32_t)buffer[13] << 8 | (uint32_t)buffer[14] | (uint32_t)buffer[15] << 24 | (uint32_t)buffer[16] << 16) / 1000.0; // Raw Energy in 1Wh
  _frequency = ((uint32_t)buffer[17] << 8 | (uint32_t)buffer[18]) / 10.0;                                                          // Raw Frequency in 0.1Hz
  _powerFactor = ((uint32_t)buffer[19] << 8 | (uint32_t)buffer[20]) / 100.0;                                                       // Raw pf in 0.01

  _lastReadSuccess = millis();
  return true;
}

bool Mycila::PZEM::resetEnergy() {
  if (!_enabled)
    return false;

  if (!_mutex.try_lock_for(std::chrono::milliseconds(1000))) {
    ESP_LOGW(TAG, "Cannot reset: Serial is busy!");
    return false;
  }

  ESP_LOGI(TAG, "Reset PZEM Energy data...");

  uint8_t request[] = {0x00, PZEM_CMD_REST, 0x00, 0x00};
  request[0] = _address;
  _crcSet(request, 4);

  _serial->write(request, 4);

  uint8_t response[PZEM_RESET_RESPONSE_SIZE];
  const size_t count = _timedRead(response, PZEM_RESET_RESPONSE_SIZE);

  _mutex.unlock();

  if (!count) {
    ESP_LOGD(TAG, "Reset timeout");
    return false;
  }

  if (count == PZEM_RESET_RESPONSE_SIZE) {
    ESP_LOGD(TAG, "Reset failed!");
    return false;
  }

  return true;
}

bool Mycila::PZEM::setAddress(const uint8_t address) {
  if (!_enabled)
    return false;

  if (address < 0x01 || address > 0xF7)
    return false;

  if (_address == address)
    return true;

  if (!_mutex.try_lock_for(std::chrono::milliseconds(1000))) {
    ESP_LOGW(TAG, "Cannot set address: Serial is busy!");
    return false;
  }

  ESP_LOGI(TAG, "Set address to 0x%02X...", address);

  const bool success = _sendCmd8(PZEM_CMD_WSR, PZEM_WREG_ADDR, address, true);

  if (success) {
    _address = address;
  }

  _mutex.unlock();

  if (!success) {
    ESP_LOGD(TAG, "Set address failed!");
  }

  return success;
}

uint8_t Mycila::PZEM::readAddress(bool update) {
  if (!_enabled)
    return false;

  if (!_mutex.try_lock_for(std::chrono::milliseconds(1000))) {
    ESP_LOGW(TAG, "Cannot read address: Serial is busy!");
    return false;
  }

  uint8_t address = MYCILA_PZEM_INVALID_ADDRESS;

  _sendCmd8(PZEM_CMD_RHR, PZEM_WREG_ADDR, 0x01, false);

  uint8_t buffer[PZEM_ADDR_RESPONSE_SIZE];
  const size_t count = _timedRead(buffer, PZEM_ADDR_RESPONSE_SIZE);

  if (count == PZEM_ADDR_RESPONSE_SIZE) {
    address = ((uint32_t)buffer[3] << 8 | (uint32_t)buffer[4]);
    if (update) {
      _address = address;
    }
  }

  _mutex.unlock();

  if (!count) {
    ESP_LOGD(TAG, "Read address timeout");
    return MYCILA_PZEM_INVALID_ADDRESS;
  }

  if (count != PZEM_ADDR_RESPONSE_SIZE) {
    ESP_LOGD(TAG, "Read address failed!");
    return MYCILA_PZEM_INVALID_ADDRESS;
  }

  ESP_LOGD(TAG, "Read address: 0x%02X...", address);

  return address;
}

size_t Mycila::PZEM::search(uint8_t* addresses, const size_t maxCount) {
  if (!_enabled)
    return 0;

  if (maxCount == 0)
    return 0;

  uint8_t buffer[PZEM_ADDR_RESPONSE_SIZE];
  size_t count = 0;

  for (uint16_t address = 0x01; address <= 0xF8 && count < maxCount; address++) {
    if (!_mutex.try_lock_for(std::chrono::milliseconds(1000))) {
      ESP_LOGW(TAG, "Cannot search address 0x%02X: Serial is busy!", address);
      break;
    }

    _sendCmd8(PZEM_CMD_RIR, 0x00, 0x01, false, address);
    const size_t nRead = _timedRead(buffer, PZEM_ADDR_RESPONSE_SIZE);

    _mutex.unlock();

    if (nRead == 0) {
      // ESP_LOGD(TAG, "Search timeout on address 0x%02X", address);
      yield();
      continue;
    }

    if (nRead != PZEM_ADDR_RESPONSE_SIZE) {
      ESP_LOGD(TAG, "Search failed on address 0x%02X", address);
      yield();
      continue;
    }

    ESP_LOGD(TAG, "Found device at address: 0x%02X", address);
    addresses[count++] = address;
    yield();
  }

  return count;
}

#ifdef MYCILA_PZEM_JSON_SUPPORT
void Mycila::PZEM::toJson(const JsonObject& root) const {
  root["address"] = _address;
  root["current"] = _current;
  root["enabled"] = _enabled;
  root["energy"] = _energy;
  root["frequency"] = _frequency;
  root["power_factor"] = _powerFactor;
  root["power"] = _power;
  root["voltage"] = _voltage;
  root["time"] = _lastReadSuccess;
}
#endif

void Mycila::PZEM::_openSerial() {
  _serial->begin(PZEM_BAUD_RATE, SERIAL_8N1, _pinTX, _pinRX);
  _serial->setTimeout(MYCILA_PZEM_READ_TIMEOUT_MS);
  while (!_serial)
    yield();
  _serial->flush();
  _drop();
  while (!_serial->availableForWrite())
    yield();
}

size_t Mycila::PZEM::_timedRead(uint8_t* buffer, size_t length) {
  // const uint32_t now = millis();
  const size_t count = _serial->readBytes(buffer, length) + _drop();
  if (count == 0) {
    // ESP_LOGD(TAG, "Read timeout");
    return count;
  }
  // ESP_LOGD(TAG, "Read available after %d ms", millis() - now);
  if (!_crcCheck(buffer, count)) {
    ESP_LOGD(TAG, "CRC check error");
    return 0;
  }
  return count;
}

size_t Mycila::PZEM::_drop() {
  size_t count = 0;
  // Serial.printf("Drop: ");
  while (_serial->available()) {
    // Serial.printf("0x%02X", _serial->read());
    _serial->read();
    count++;
  }
  // if (count > 0)
  //   Serial.printf("Drop: %d\n", count);
  return count;
}

bool Mycila::PZEM::_canRead() {
  _sendCmd8(PZEM_CMD_RIR, 0x00, 0x0A, false);
  uint8_t buffer[PZEM_READ_RESPONSE_SIZE];
  return _timedRead(buffer, PZEM_READ_RESPONSE_SIZE) == PZEM_READ_RESPONSE_SIZE;
}

bool Mycila::PZEM::_sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check, uint16_t slave_addr) {
  uint8_t sendBuffer[8]; // Send buffer

  if ((slave_addr == 0xFFFF) ||
      (slave_addr < 0x01) ||
      (slave_addr > 0xF7)) {
    slave_addr = _address;
  }

  sendBuffer[0] = slave_addr; // Set slave address
  sendBuffer[1] = cmd;        // Set command

  sendBuffer[2] = (rAddr >> 8) & 0xFF; // Set high byte of register address
  sendBuffer[3] = (rAddr) & 0xFF;      // Set low byte =//=

  sendBuffer[4] = (val >> 8) & 0xFF; // Set high byte of register value
  sendBuffer[5] = (val) & 0xFF;      // Set low byte =//=

  _crcSet(sendBuffer, 8); // Set CRC of frame

  _serial->write(sendBuffer, 8); // send frame
  _serial->flush();

  if (check) {
    uint8_t respBuffer[8];            // Response buffer (only used when check is true)
    if (!_timedRead(respBuffer, 8)) { // if check enabled, read the response
      return false;                   // timeout
    }
    // Check if response is same as send
    for (uint8_t i = 0; i < 8; i++) {
      if (sendBuffer[i] != respBuffer[i])
        ESP_LOGD(TAG, "Response check error");
      return false;
    }
  }

  return true;
}

void Mycila::PZEM::_pzemTask(void* params) {
  PZEM* pzem = reinterpret_cast<PZEM*>(params);
  while (pzem->_enabled) {
    pzem->read();
    yield();
  }
  pzem->_taskHandle = NULL;
  vTaskDelete(NULL);
}

void Mycila::PZEM::_crcSet(uint8_t* buf, uint16_t len) {
  if (len <= 2)
    return;
  uint16_t crc = _crc16(buf, len - 2);
  buf[len - 2] = crc & 0xFF;        // Low byte first
  buf[len - 1] = (crc >> 8) & 0xFF; // High byte second
}

bool Mycila::PZEM::_crcCheck(const uint8_t* buf, uint16_t len) {
  if (len <= 2)
    return false;
  uint16_t crc = _crc16(buf, len - 2); // Compute CRC of data
  return ((uint16_t)buf[len - 2] | (uint16_t)buf[len - 1] << 8) == crc;
}

uint16_t Mycila::PZEM::_crc16(const uint8_t* data, uint16_t len) {
  uint8_t nTemp;         // CRC table index
  uint16_t crc = 0xFFFF; // Default value
  while (len--) {
    nTemp = *data++ ^ crc;
    crc >>= 8;
    crc ^= (uint16_t)pgm_read_word(&crcTable[nTemp]);
  }
  return crc;
}
