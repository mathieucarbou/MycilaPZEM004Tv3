// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 */
#include "MycilaPZEM004Tv3.h"

#ifdef MYCILA_LOGGER_SUPPORT
  #include <MycilaLogger.h>
extern Mycila::Logger logger;
  #define LOGD(tag, format, ...) logger.debug(tag, format, ##__VA_ARGS__)
  #define LOGI(tag, format, ...) logger.info(tag, format, ##__VA_ARGS__)
  #define LOGW(tag, format, ...) logger.warn(tag, format, ##__VA_ARGS__)
  #define LOGE(tag, format, ...) logger.error(tag, format, ##__VA_ARGS__)
#else
  #define LOGD(tag, format, ...) ESP_LOGD(tag, format, ##__VA_ARGS__)
  #define LOGI(tag, format, ...) ESP_LOGI(tag, format, ##__VA_ARGS__)
  #define LOGW(tag, format, ...) ESP_LOGW(tag, format, ##__VA_ARGS__)
  #define LOGE(tag, format, ...) ESP_LOGE(tag, format, ##__VA_ARGS__)
#endif

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

#ifndef GPIO_IS_VALID_OUTPUT_GPIO
  #define GPIO_IS_VALID_OUTPUT_GPIO(gpio_num) ((gpio_num >= 0) && \
                                               (((1ULL << (gpio_num)) & SOC_GPIO_VALID_OUTPUT_GPIO_MASK) != 0))
#endif

#ifndef GPIO_IS_VALID_GPIO
  #define GPIO_IS_VALID_GPIO(gpio_num) ((gpio_num >= 0) && \
                                        (((1ULL << (gpio_num)) & SOC_GPIO_VALID_GPIO_MASK) != 0))
#endif

// Pre computed CRC table
static const uint16_t crcTable[] PROGMEM = {
  0X0000,
  0XC0C1,
  0XC181,
  0X0140,
  0XC301,
  0X03C0,
  0X0280,
  0XC241,
  0XC601,
  0X06C0,
  0X0780,
  0XC741,
  0X0500,
  0XC5C1,
  0XC481,
  0X0440,
  0XCC01,
  0X0CC0,
  0X0D80,
  0XCD41,
  0X0F00,
  0XCFC1,
  0XCE81,
  0X0E40,
  0X0A00,
  0XCAC1,
  0XCB81,
  0X0B40,
  0XC901,
  0X09C0,
  0X0880,
  0XC841,
  0XD801,
  0X18C0,
  0X1980,
  0XD941,
  0X1B00,
  0XDBC1,
  0XDA81,
  0X1A40,
  0X1E00,
  0XDEC1,
  0XDF81,
  0X1F40,
  0XDD01,
  0X1DC0,
  0X1C80,
  0XDC41,
  0X1400,
  0XD4C1,
  0XD581,
  0X1540,
  0XD701,
  0X17C0,
  0X1680,
  0XD641,
  0XD201,
  0X12C0,
  0X1380,
  0XD341,
  0X1100,
  0XD1C1,
  0XD081,
  0X1040,
  0XF001,
  0X30C0,
  0X3180,
  0XF141,
  0X3300,
  0XF3C1,
  0XF281,
  0X3240,
  0X3600,
  0XF6C1,
  0XF781,
  0X3740,
  0XF501,
  0X35C0,
  0X3480,
  0XF441,
  0X3C00,
  0XFCC1,
  0XFD81,
  0X3D40,
  0XFF01,
  0X3FC0,
  0X3E80,
  0XFE41,
  0XFA01,
  0X3AC0,
  0X3B80,
  0XFB41,
  0X3900,
  0XF9C1,
  0XF881,
  0X3840,
  0X2800,
  0XE8C1,
  0XE981,
  0X2940,
  0XEB01,
  0X2BC0,
  0X2A80,
  0XEA41,
  0XEE01,
  0X2EC0,
  0X2F80,
  0XEF41,
  0X2D00,
  0XEDC1,
  0XEC81,
  0X2C40,
  0XE401,
  0X24C0,
  0X2580,
  0XE541,
  0X2700,
  0XE7C1,
  0XE681,
  0X2640,
  0X2200,
  0XE2C1,
  0XE381,
  0X2340,
  0XE101,
  0X21C0,
  0X2080,
  0XE041,
  0XA001,
  0X60C0,
  0X6180,
  0XA141,
  0X6300,
  0XA3C1,
  0XA281,
  0X6240,
  0X6600,
  0XA6C1,
  0XA781,
  0X6740,
  0XA501,
  0X65C0,
  0X6480,
  0XA441,
  0X6C00,
  0XACC1,
  0XAD81,
  0X6D40,
  0XAF01,
  0X6FC0,
  0X6E80,
  0XAE41,
  0XAA01,
  0X6AC0,
  0X6B80,
  0XAB41,
  0X6900,
  0XA9C1,
  0XA881,
  0X6840,
  0X7800,
  0XB8C1,
  0XB981,
  0X7940,
  0XBB01,
  0X7BC0,
  0X7A80,
  0XBA41,
  0XBE01,
  0X7EC0,
  0X7F80,
  0XBF41,
  0X7D00,
  0XBDC1,
  0XBC81,
  0X7C40,
  0XB401,
  0X74C0,
  0X7580,
  0XB541,
  0X7700,
  0XB7C1,
  0XB681,
  0X7640,
  0X7200,
  0XB2C1,
  0XB381,
  0X7340,
  0XB101,
  0X71C0,
  0X7080,
  0XB041,
  0X5000,
  0X90C1,
  0X9181,
  0X5140,
  0X9301,
  0X53C0,
  0X5280,
  0X9241,
  0X9601,
  0X56C0,
  0X5780,
  0X9741,
  0X5500,
  0X95C1,
  0X9481,
  0X5440,
  0X9C01,
  0X5CC0,
  0X5D80,
  0X9D41,
  0X5F00,
  0X9FC1,
  0X9E81,
  0X5E40,
  0X5A00,
  0X9AC1,
  0X9B81,
  0X5B40,
  0X9901,
  0X59C0,
  0X5880,
  0X9841,
  0X8801,
  0X48C0,
  0X4980,
  0X8941,
  0X4B00,
  0X8BC1,
  0X8A81,
  0X4A40,
  0X4E00,
  0X8EC1,
  0X8F81,
  0X4F40,
  0X8D01,
  0X4DC0,
  0X4C80,
  0X8C41,
  0X4400,
  0X84C1,
  0X8581,
  0X4540,
  0X8701,
  0X47C0,
  0X4680,
  0X8641,
  0X8201,
  0X42C0,
  0X4380,
  0X8341,
  0X4100,
  0X81C1,
  0X8081,
  0X4040};

TaskHandle_t Mycila::PZEM::_taskHandle = NULL;
Mycila::PZEM* Mycila::PZEM::_instances[MYCILA_PZEM_ASYNC_MAX_INSTANCES];
std::timed_mutex Mycila::PZEM::_mutex;

void Mycila::PZEM::begin(HardwareSerial& serial,
                         const int8_t rxPin,
                         const int8_t txPin,
                         const uint8_t address,
                         const bool async) {
  if (_enabled)
    return;

  if (GPIO_IS_VALID_GPIO(rxPin)) {
    _pinRX = (gpio_num_t)rxPin;
  } else {
    LOGE(TAG, "Disable PZEM: Invalid RX pin: %" PRId8, rxPin);
    _pinRX = GPIO_NUM_NC;
    return;
  }

  if (GPIO_IS_VALID_OUTPUT_GPIO(txPin)) {
    _pinTX = (gpio_num_t)txPin;
  } else {
    LOGE(TAG, "Disable PZEM: Invalid TX pin: %" PRId8, txPin);
    _pinTX = GPIO_NUM_NC;
    return;
  }

  if (address < 0x01 || address > MYCILA_PZEM_DEFAULT_ADDRESS) {
    LOGE(TAG, "Disable PZEM: Invalid address: 0x%02X", address);
    return;
  }

  LOGI(TAG, "[0x%02X] Enable PZEM: Serial RX (PZEM TX Pin): %" PRId8 ", Serial TX (PZEM RX Pin): %" PRId8, address, rxPin, txPin);

  _serial = &serial;
  _address = address;

  if (async) {
    if (!_add(this))
      return;

  } else {
    _openSerial(&serial, _pinRX, _pinTX);
    if (!_canRead(&serial, address)) {
      LOGW(TAG, "[0x%02X] Unable to read PZEM. Please verify that the device is powered and that its address is correctly set.", address);
    }
  }

  _enabled = true;
}

void Mycila::PZEM::end() {
  if (_enabled) {
    LOGI(TAG, "[0x%02X] Disable PZEM...", _address);
    _enabled = false;
    _remove(this);
    delay(MYCILA_PZEM_READ_TIMEOUT_MS);
    _address = MYCILA_PZEM_INVALID_ADDRESS;
    _current = 0;
    _frequency = 0;
    _power = 0;
    _powerFactor = 0;
    _voltage = 0;
  }
}

bool Mycila::PZEM::read() {
  if (!_enabled)
    return false;

  if (!_mutex.try_lock_for(std::chrono::milliseconds(1000))) {
    LOGW(TAG, "[0x%02X] Cannot read PZEM: Serial is busy!", _address);
    return false;
  }

  _sendCmd8(_serial, PZEM_CMD_RIR, 0x00, 0x0A, false, _address);

  uint8_t buffer[PZEM_READ_RESPONSE_SIZE];
  const size_t count = _timedRead(_serial, buffer, PZEM_READ_RESPONSE_SIZE);

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
    LOGD(TAG, "[0x%02X] Read failed: %d", _address, count);
    return false;
  }

  float voltage = ((uint32_t)buffer[3] << 8 | (uint32_t)buffer[4]) / 10.0;                                                              // Raw voltage in 0.1V
  float current = ((uint32_t)buffer[5] << 8 | (uint32_t)buffer[6] | (uint32_t)buffer[7] << 24 | (uint32_t)buffer[8] << 16) / 1000.0;    // Raw current in 0.001A
  float power = ((uint32_t)buffer[9] << 8 | (uint32_t)buffer[10] | (uint32_t)buffer[11] << 24 | (uint32_t)buffer[12] << 16) / 10.0;     // Raw power in 0.1W
  float energy = ((uint32_t)buffer[13] << 8 | (uint32_t)buffer[14] | (uint32_t)buffer[15] << 24 | (uint32_t)buffer[16] << 16) / 1000.0; // Raw Energy in 1Wh
  float frequency = ((uint32_t)buffer[17] << 8 | (uint32_t)buffer[18]) / 10.0;                                                          // Raw Frequency in 0.1Hz
  float powerFactor = ((uint32_t)buffer[19] << 8 | (uint32_t)buffer[20]) / 100.0;                                                       // Raw pf in 0.01

  bool changed = voltage != _voltage ||
                 current != _current ||
                 power != _power ||
                 energy != _energy ||
                 frequency != _frequency ||
                 powerFactor != _powerFactor;

  if (changed) {
    _voltage = voltage;
    _current = current;
    _power = power;
    _energy = energy;
    _frequency = frequency;
    _powerFactor = powerFactor;
  }

  _lastReadSuccess = millis();

  if (_callback) {
    _callback(PZEMEventType::EVT_READ);
    if (changed) {
      _callback(PZEMEventType::EVT_CHANGE);
    }
  }

  return true;
}

bool Mycila::PZEM::resetEnergy() {
  if (!_enabled)
    return false;

  if (!_mutex.try_lock_for(std::chrono::milliseconds(1000))) {
    LOGW(TAG, "[0x%02X] Cannot reset PZEM: Serial is busy!", _address);
    return false;
  }

  LOGI(TAG, "[0x%02X] Reset Energy data...", _address);

  uint8_t request[] = {0x00, PZEM_CMD_REST, 0x00, 0x00};
  request[0] = _address;
  _crcSet(request, 4);

  _serial->write(request, 4);

  uint8_t response[PZEM_RESET_RESPONSE_SIZE];
  const size_t count = _timedRead(_serial, response, PZEM_RESET_RESPONSE_SIZE);

  _mutex.unlock();

  if (!count) {
    LOGD(TAG, "[0x%02X] Reset timeout", _address);
    return false;
  }

  if (count == PZEM_RESET_RESPONSE_SIZE) {
    LOGD(TAG, "[0x%02X] Reset failed!", _address);
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
    LOGW(TAG, "[0x%02X] Cannot set address: Serial is busy!", _address);
    return false;
  }

  LOGI(TAG, "[0x%02X] Set address to 0x%02X...", _address, address);

  const bool success = _sendCmd8(_serial, PZEM_CMD_WSR, PZEM_WREG_ADDR, address, true, _address);

  if (success) {
    _address = address;
  }

  _mutex.unlock();

  if (!success) {
    LOGD(TAG, "[0x%02X] Set address failed!", _address);
  }

  return success;
}

uint8_t Mycila::PZEM::readAddress(bool update) {
  if (!_enabled)
    return false;

  if (!_mutex.try_lock_for(std::chrono::milliseconds(1000))) {
    LOGW(TAG, "[0x%02X] Cannot read address: Serial is busy!", _address);
    return false;
  }

  uint8_t address = MYCILA_PZEM_INVALID_ADDRESS;

  _sendCmd8(_serial, PZEM_CMD_RHR, PZEM_WREG_ADDR, 0x01, false, _address);

  uint8_t buffer[PZEM_ADDR_RESPONSE_SIZE];
  const size_t count = _timedRead(_serial, buffer, PZEM_ADDR_RESPONSE_SIZE);

  if (count == PZEM_ADDR_RESPONSE_SIZE) {
    address = ((uint32_t)buffer[3] << 8 | (uint32_t)buffer[4]);
    if (update) {
      _address = address;
    }
  }

  _mutex.unlock();

  if (!count) {
    LOGD(TAG, "[0x%02X] Read address timeout", _address);
    return MYCILA_PZEM_INVALID_ADDRESS;
  }

  if (count != PZEM_ADDR_RESPONSE_SIZE) {
    LOGD(TAG, "[0x%02X] Read address failed!", _address);
    return MYCILA_PZEM_INVALID_ADDRESS;
  }

  return address;
}

size_t Mycila::PZEM::search(uint8_t* addresses, const size_t maxCount) {
  if (!_enabled)
    return 0;

  if (maxCount == 0)
    return 0;

  uint8_t buffer[PZEM_ADDR_RESPONSE_SIZE];
  size_t count = 0;

  for (uint16_t address = 0x01; address <= MYCILA_PZEM_DEFAULT_ADDRESS && count < maxCount; address++) {
    if (!_mutex.try_lock_for(std::chrono::milliseconds(1000))) {
      LOGW(TAG, "[0x%02X] Cannot search: Serial is busy!", _address);
      break;
    }

    _sendCmd8(_serial, PZEM_CMD_RIR, 0x00, 0x01, false, address);
    const size_t nRead = _timedRead(_serial, buffer, PZEM_ADDR_RESPONSE_SIZE);

    _mutex.unlock();

    if (nRead == 0) {
      // LOGD(TAG, "Search timeout on address 0x%02X", address);
      yield();
      continue;
    }

    if (nRead != PZEM_ADDR_RESPONSE_SIZE) {
      LOGD(TAG, "[0x%02X] Search response error", _address);
      yield();
      continue;
    }

    LOGD(TAG, "[0x%02X] Found device at address: 0x%02X", _address, address);
    addresses[count++] = address;
    yield();
  }

  return count;
}

#ifdef MYCILA_JSON_SUPPORT
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

size_t Mycila::PZEM::_timedRead(HardwareSerial* serial, uint8_t* buffer, size_t length) {
  // const uint32_t now = millis();
  const size_t count = serial->readBytes(buffer, length) + _drop(serial);
  if (count == 0) {
    // LOGD(TAG, "Read timeout");
    return count;
  }
  // LOGD(TAG, "Read available after %d ms", millis() - now);
  if (!_crcCheck(buffer, count)) {
    LOGD(TAG, "CRC check error");
    return 0;
  }
  return count;
}

size_t Mycila::PZEM::_drop(HardwareSerial* serial) {
  size_t count = 0;
  // Serial.printf("Drop: ");
  while (serial->available()) {
    // Serial.printf("0x%02X", _serial->read());
    serial->read();
    count++;
  }
  // if (count > 0)
  //   Serial.printf("Drop: %d\n", count);
  return count;
}

bool Mycila::PZEM::_canRead(HardwareSerial* serial, uint16_t slaveAddr) {
  _sendCmd8(serial, PZEM_CMD_RIR, 0x00, 0x0A, false, slaveAddr);
  uint8_t buffer[PZEM_READ_RESPONSE_SIZE];
  return _timedRead(serial, buffer, PZEM_READ_RESPONSE_SIZE) == PZEM_READ_RESPONSE_SIZE;
}

bool Mycila::PZEM::_sendCmd8(HardwareSerial* serial, uint8_t cmd, uint16_t rAddr, uint16_t val, bool check, uint16_t slaveAddr) {
  uint8_t sendBuffer[8]; // Send buffer

  sendBuffer[0] = slaveAddr; // Set slave address
  sendBuffer[1] = cmd;       // Set command

  sendBuffer[2] = (rAddr >> 8) & 0xFF; // Set high byte of register address
  sendBuffer[3] = (rAddr) & 0xFF;      // Set low byte =//=

  sendBuffer[4] = (val >> 8) & 0xFF; // Set high byte of register value
  sendBuffer[5] = (val) & 0xFF;      // Set low byte =//=

  _crcSet(sendBuffer, 8); // Set CRC of frame

  serial->write(sendBuffer, 8); // send frame
  serial->flush();

  if (check) {
    uint8_t respBuffer[8];                    // Response buffer (only used when check is true)
    if (!_timedRead(serial, respBuffer, 8)) { // if check enabled, read the response
      return false;                           // timeout
    }
    // Check if response is same as send
    for (uint8_t i = 0; i < 8; i++) {
      if (sendBuffer[i] != respBuffer[i])
        LOGD(TAG, "Response check error");
      return false;
    }
  }

  return true;
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

bool Mycila::PZEM::_add(PZEM* pzem) {
  for (size_t i = 0; i < MYCILA_PZEM_ASYNC_MAX_INSTANCES; i++) {
    if (_instances[i] == nullptr) {
      LOGD(TAG, "Adding instance at address 0x%02X to async task...", pzem->_address);

      if (_taskHandle == NULL) {
        _openSerial(pzem->_serial, pzem->_pinRX, pzem->_pinTX);

        if (!_canRead(pzem->_serial, pzem->_address)) {
          LOGW(TAG, "Unable to read at address 0x%02X. Please verify that the device is powered and that its address is correctly set.", pzem->_address);
        }

        _instances[i] = pzem;

        LOGD(TAG, "Creating async task 'pzemTask'...");
        if (xTaskCreateUniversal(_pzemTask, "pzemTask", MYCILA_PZEM_ASYNC_STACK_SIZE, nullptr, MYCILA_PZEM_ASYNC_PRIORITY, &_taskHandle, MYCILA_PZEM_ASYNC_CORE) == pdPASS) {
          return true;
        } else {
          _instances[i] = nullptr;
          LOGE(TAG, "Failed to create task 'pzemTask'");
          return false;
        }

      } else {
        if (!_canRead(pzem->_serial, pzem->_address)) {
          LOGW(TAG, "Unable to read at address 0x%02X. Please verify that the device is powered and that its address is correctly set.", pzem->_address);
        }
        _instances[i] = pzem;
      }

      return true;
    }
  }
  return false;
}

void Mycila::PZEM::_remove(PZEM* pzem) {
  // check for remaining tasks
  bool remaining = false;
  for (size_t i = 0; i < MYCILA_PZEM_ASYNC_MAX_INSTANCES; i++) {
    if (_instances[i] != nullptr && _instances[i] != pzem) {
      remaining = true;
      break;
    }
  }
  // first delete  the task if no remaining instances
  if (!remaining && _taskHandle != NULL) {
    LOGD(TAG, "Deleting async task 'pzemTask'...");
    vTaskDelete(_taskHandle);
    _taskHandle = NULL;
  }
  // in any case, remove the instance from the list
  for (size_t i = 0; i < MYCILA_PZEM_ASYNC_MAX_INSTANCES; i++) {
    if (_instances[i] == pzem) {
      _instances[i] = nullptr;
      break;
    }
  }
}

void Mycila::PZEM::_pzemTask(void* params) {
  while (true) {
    bool read = false;
    for (size_t i = 0; i < MYCILA_PZEM_ASYNC_MAX_INSTANCES; i++) {
      if (_instances[i] != nullptr) {
        read |= _instances[i]->read();
        yield();
      }
    }
    if (!read)
      delay(MYCILA_PZEM_READ_TIMEOUT_MS);
  }
  _taskHandle = NULL;
  vTaskDelete(NULL);
}

void Mycila::PZEM::_openSerial(HardwareSerial* serial, const uint8_t rxPin, const uint8_t txPin) {
  LOGD(TAG, "Open serial...");
  serial->begin(PZEM_BAUD_RATE, SERIAL_8N1, rxPin, txPin);
  serial->setTimeout(MYCILA_PZEM_READ_TIMEOUT_MS);
  while (!serial)
    yield();
  serial->flush();
  _drop(serial);
  while (!serial->availableForWrite())
    yield();
}
