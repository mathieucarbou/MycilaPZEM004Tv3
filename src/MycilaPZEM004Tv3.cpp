// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2025 Mathieu Carbou
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
#define PZEM_TIMEOUT   200

#define PZEM_CMD_RHR  0x03 // Read holding register
#define PZEM_CMD_RIR  0X04 // Read input register
#define PZEM_CMD_WSR  0x06 // Write single register
#define PZEM_CMD_REST 0x42 // Reset Energy

#define PZEM_REGISTER_ADDRESS 0x0002

#define PZEM_REGISTER_VOLTAGE   0x0000
#define PZEM_REGISTER_CURRENT   0x0001 // 0x0001 (low) 0x0002 (high)
#define PZEM_REGISTER_POWER     0x0003 // 0x0003 (low) 0x0004 (high)
#define PZEM_REGISTER_ENERGY    0x0005 // 0x0005 (low) 0x0006 (high)
#define PZEM_REGISTER_FREQUENCY 0x0007
#define PZEM_REGISTER_PF        0x0008

#define PZEM_REGISTER_LEN   2 // 2 bytes per register
#define PZEM_REGISTER_COUNT 9 // 9 registers

#define PZEM_RESPONSE_SIZE_READ_METRICS 23 // address(1), cmd(1), len(1), data(9*2), crc(2)
#define PZEM_RESPONSE_SIZE_READ_ADDR    7  // address(1), cmd(1), len(1), address(2), crc(2)
#define PZEM_RESPONSE_SIZE_RESET        4  // address(1), cmd(1), crc(2)
#define PZEM_RESPONSE_SIZE_SET_ADDR     8  // address(1), cmd(1), len(1), register(1), data(2), crc(2)

#define PZEM_RESPONSE_READ_DATA 3
#define PZEM_RESPONSE_ADDRESS   0

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
std::mutex Mycila::PZEM::_mutex;

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

  if (address < MYCILA_PZEM_ADDRESS_MIN || address > MYCILA_PZEM_ADDRESS_GENERAL) {
    LOGE(TAG, "Disable PZEM: Invalid address: 0x%02X", address);
    return;
  }

  _serial = &serial;
  _address = address;

  if (async) {
    LOGI(TAG, "Enable PZEM @ 0x%02X on Serial RX (PZEM TX Pin): %" PRId8 " and Serial TX (PZEM RX Pin): %" PRId8, address, rxPin, txPin);
    _enabled = _add(this);

  } else {
    _enabled = true;
    _openSerial(_pinRX, _pinTX);
    if (_address == MYCILA_PZEM_ADDRESS_GENERAL && readDeviceAddress(true) == MYCILA_PZEM_ADDRESS_UNKNOWN) {
      LOGE(TAG, "Unable to read any PZEM @ 0x%02X: Please verify that the device is powered on and that its address is correctly set if not using the default address", address);
      serial.flush();
      _drop();
    } else {
      LOGI(TAG, "Enable PZEM @ 0x%02X on Serial RX (PZEM TX Pin): %" PRId8 " and Serial TX (PZEM RX Pin): %" PRId8, _address, _pinRX, _pinTX);
    }
  }
}

void Mycila::PZEM::end() {
  if (_enabled) {
    LOGI(TAG, "Disable PZEM @ 0x%02X", _address);
    _enabled = false;
    _remove(this);
    std::lock_guard<std::mutex> lock(_mutex);
    _serial->end();
    _serial = nullptr;
    _lastAddress = MYCILA_PZEM_ADDRESS_UNKNOWN;
    _address = MYCILA_PZEM_ADDRESS_GENERAL;
    _data.clear();
  }
}

///////////////////////////////////////////////////////////////////////////////
// read
///////////////////////////////////////////////////////////////////////////////

bool Mycila::PZEM::read(uint8_t address) {
  if (!_enabled)
    return false;

  std::lock_guard<std::mutex> lock(_mutex);

  _send(address, PZEM_CMD_RIR, PZEM_REGISTER_VOLTAGE, PZEM_REGISTER_COUNT);
  ReadResult result = _timedRead(address, PZEM_RESPONSE_SIZE_READ_METRICS);

  if (result == ReadResult::READ_TIMEOUT) {
    // reset live values in case of read timeout
    _data.clear();
    if (_callback) {
      _callback(EventType::EVT_READ_TIMEOUT, _data);
    }
    return false;
  }

  if (result == ReadResult::READ_ERROR_COUNT || result == ReadResult::READ_ERROR_CRC) {
    // reset live values in case of read failure
    _data.clear();
    if (_callback) {
      _callback(EventType::EVT_READ_ERROR, _data);
    }
    return false;
  }

  if (result == ReadResult::READ_ERROR_ADDRESS) {
    // we have set a destination address, but we read another device
    if (_callback) {
      _callback(EventType::EVT_READ_ERROR, _data);
    }
    return false;
  }

  assert(result == ReadResult::READ_SUCCESS);

  _data.voltage = (static_cast<uint32_t>(_buffer[3]) << 8 | static_cast<uint32_t>(_buffer[4])) * 0.1f;                                                                                            // Raw voltage in 0.1V
  _data.current = (static_cast<uint32_t>(_buffer[5]) << 8 | static_cast<uint32_t>(_buffer[6] | static_cast<uint32_t>(_buffer[7]) << 24 | static_cast<uint32_t>(_buffer[8]) << 16)) * 0.001f;      // Raw current in 0.001A
  _data.activePower = (static_cast<uint32_t>(_buffer[9]) << 8 | static_cast<uint32_t>(_buffer[10] | static_cast<uint32_t>(_buffer[11]) << 24 | static_cast<uint32_t>(_buffer[12]) << 16)) * 0.1f; // Raw power in 0.1W
  _data.activeEnergy = (static_cast<uint32_t>(_buffer[13]) << 8 | static_cast<uint32_t>(_buffer[14] | static_cast<uint32_t>(_buffer[15]) << 24 | static_cast<uint32_t>(_buffer[16]) << 16));      // Raw Energy in 1Wh
  _data.frequency = (static_cast<uint32_t>(_buffer[17]) << 8 | static_cast<uint32_t>(_buffer[18])) * 0.1f;                                                                                        // Raw Frequency in 0.1Hz
  _data.powerFactor = (static_cast<uint32_t>(_buffer[19]) << 8 | static_cast<uint32_t>(_buffer[20])) * 0.01f;                                                                                     // Raw pf in 0.01

  // calculate remaining metrics

  // S = P / PF
  _data.apparentPower = _data.powerFactor == 0 ? 0 : std::abs(_data.activePower / _data.powerFactor);
  // Q = std::sqrt(S^2 - P^2)
  _data.reactivePower = std::sqrt(_data.apparentPower * _data.apparentPower - _data.activePower * _data.activePower);

  _time = millis();

  if (_callback) {
    _callback(EventType::EVT_READ, _data);
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////
// resetEnergy
///////////////////////////////////////////////////////////////////////////////

bool Mycila::PZEM::resetEnergy(uint8_t address) {
  if (!_enabled)
    return false;

  LOGD(TAG, "resetEnergy(0x%02X)", address);

  std::lock_guard<std::mutex> lock(_mutex);

  _buffer[0] = address;
  _buffer[1] = PZEM_CMD_REST;
  _crcSet(_buffer, 4);

  _serial->write(_buffer, 4);
  _serial->flush();

  ReadResult result = _timedRead(address, PZEM_RESPONSE_SIZE_RESET);

  return result == ReadResult::READ_SUCCESS;
}

///////////////////////////////////////////////////////////////////////////////
// settings
///////////////////////////////////////////////////////////////////////////////

bool Mycila::PZEM::setDeviceAddress(const uint8_t address, const uint8_t newAddress) {
  if (!_enabled)
    return false;

  if (newAddress < 0x01 || newAddress > 0xF7) {
    LOGE(TAG, "Invalid address: 0x%02X", newAddress);
    return false;
  }

  std::lock_guard<std::mutex> lock(_mutex);

  LOGD(TAG, "setDeviceAddress(0x%02X, 0x%02X)", address, newAddress);

  _send(address, PZEM_CMD_WSR, PZEM_REGISTER_ADDRESS, newAddress);
  ReadResult result = _timedRead(address, PZEM_RESPONSE_SIZE_SET_ADDR);

  if (result != ReadResult::READ_SUCCESS && result != ReadResult::READ_ERROR_ADDRESS) {
    LOGD(TAG, "Unable to set address @ 0x%02X", address);
    return false;
  }

  if (!_canRead(newAddress)) {
    LOGE(TAG, "Unable to read PZEM @ 0x%02X", newAddress);
    return false;
  }

  // update destination address if needed
  if (_address != MYCILA_PZEM_ADDRESS_GENERAL && _address == address) {
    _address = newAddress;
  }

  return true;
}

uint8_t Mycila::PZEM::readDeviceAddress(bool update) {
  if (!_enabled)
    return false;

  std::lock_guard<std::mutex> lock(_mutex);

  uint8_t address = MYCILA_PZEM_ADDRESS_UNKNOWN;

  _send(MYCILA_PZEM_ADDRESS_GENERAL, PZEM_CMD_RHR, PZEM_REGISTER_ADDRESS, 1);
  ReadResult result = _timedRead(MYCILA_PZEM_ADDRESS_GENERAL, PZEM_RESPONSE_SIZE_READ_ADDR);

  if (result == ReadResult::READ_SUCCESS) {
    address = _buffer[4];
    if (update) {
      _address = address;
    }
  } else {
    LOGD(TAG, "Unable to read address @ 0x%02X", MYCILA_PZEM_ADDRESS_GENERAL);
  }

  return address;
}

size_t Mycila::PZEM::search(uint8_t* addresses, const size_t maxCount) {
  if (!_enabled)
    return 0;

  if (maxCount == 0)
    return 0;

  std::lock_guard<std::mutex> lock(_mutex);

  size_t count = 0;

  for (uint8_t address = MYCILA_PZEM_ADDRESS_MIN; address <= MYCILA_PZEM_ADDRESS_MAX && count < maxCount; address++) {
    _send(address, PZEM_CMD_RHR, PZEM_REGISTER_ADDRESS, 1);
    if (_timedRead(address, PZEM_RESPONSE_SIZE_READ_ADDR) == ReadResult::READ_SUCCESS) {
      addresses[count++] = address;
      LOGD(TAG, "Found PZEM @ 0x%02X", address);
    }
    yield();
  }

  return count;
}

///////////////////////////////////////////////////////////////////////////////
// toJson
///////////////////////////////////////////////////////////////////////////////

#ifdef MYCILA_JSON_SUPPORT
void Mycila::PZEM::toJson(const JsonObject& root) const {
  root["enabled"] = _enabled;
  root["address"] = _address;
  root["time"] = _time;
  _data.toJson(root);
}
#endif

///////////////////////////////////////////////////////////////////////////////
// I/O
///////////////////////////////////////////////////////////////////////////////

Mycila::PZEM::ReadResult Mycila::PZEM::_timedRead(uint8_t expectedAddress, size_t expectedLen) {
  size_t count = 0;
  while (count < expectedLen) {
    size_t read = _serial->readBytes(_buffer + count, expectedLen - count);
    if (read) {
      count += read;
    } else {
      break;
    }
  }

#ifdef MYCILA_PZEM_DEBUG
  Serial.printf("[PZEM] timedRead() %d < ", count);
  for (size_t i = 0; i < count; i++) {
    Serial.printf("0x%02X ", _buffer[i]);
  }
  Serial.println();
#endif

  _drop();

  // timeout ?
  if (count == 0) {
    // LOGD(TAG, "timedRead() timeout");
    return ReadResult::READ_TIMEOUT;
  }

  // check length
  if (count != expectedLen) {
    LOGD(TAG, "timedRead() error: len %d != %d", count, expectedLen);
    return ReadResult::READ_ERROR_COUNT;
  }

  // CRC check
  if (!_crcCheck(_buffer, count)) {
    LOGD(TAG, "timedRead() error: bad CRC");
    return ReadResult::READ_ERROR_CRC;
  }

  _lastAddress = _buffer[PZEM_RESPONSE_ADDRESS];

  // address check
  if (expectedAddress != MYCILA_PZEM_ADDRESS_GENERAL && expectedAddress != _lastAddress) {
    LOGD(TAG, "timedRead(0x%02X) error: wrong device address 0x%02X", expectedAddress, _lastAddress);
    return ReadResult::READ_ERROR_ADDRESS;
  }

  return ReadResult::READ_SUCCESS;
}

bool Mycila::PZEM::_canRead(uint8_t address) {
#ifdef MYCILA_PZEM_DEBUG
  Serial.printf("[PZEM] _canRead(0x%02X)\n", address);
#endif
  _send(address, PZEM_CMD_RHR, PZEM_REGISTER_ADDRESS, 1);
  return _timedRead(address, PZEM_RESPONSE_SIZE_READ_ADDR) == ReadResult::READ_SUCCESS;
}

void Mycila::PZEM::_send(uint8_t address, uint8_t cmd, uint16_t rAddr, uint16_t val) {
  _buffer[0] = address; // Set slave address
  _buffer[1] = cmd;     // Set command

  _buffer[2] = (rAddr >> 8) & 0xFF; // Set high byte of register address
  _buffer[3] = (rAddr) & 0xFF;      // Set low byte =//=

  _buffer[4] = (val >> 8) & 0xFF; // Set high byte of register value
  _buffer[5] = (val) & 0xFF;      // Set low byte =//=

  _crcSet(_buffer, 8); // Set CRC of frame

#ifdef MYCILA_PZEM_DEBUG
  Serial.printf("[PZEM] send(0x%02X) %d > ", address, 8);
  for (size_t i = 0; i < 8; i++) {
    Serial.printf("0x%02X ", _buffer[i]);
  }
  Serial.println();
#endif

  _serial->write(_buffer, 8); // send frame
  _serial->flush();
}

size_t Mycila::PZEM::_drop() {
  size_t count = 0;
  if (_serial->available()) {
#ifdef MYCILA_PZEM_DEBUG
    Serial.printf("[PZEM] drop < ");
#endif
    while (_serial->available()) {
#ifdef MYCILA_PZEM_DEBUG
      Serial.printf("0x%02X ", _serial->read());
#else
      _serial->read();
#endif
      count++;
    }
#ifdef MYCILA_PZEM_DEBUG
    Serial.println();
#endif
  }
  return count;
}

void Mycila::PZEM::_openSerial(const uint8_t rxPin, const uint8_t txPin) {
  LOGD(TAG, "openSerial()");
  _serial->begin(PZEM_BAUD_RATE, SERIAL_8N1, rxPin, txPin);
  _serial->setTimeout(PZEM_TIMEOUT);
  while (!_serial)
    yield();
  while (!_serial->availableForWrite())
    yield();
  _drop();
  _serial->flush(false);
}

///////////////////////////////////////////////////////////////////////////////
// CRC
///////////////////////////////////////////////////////////////////////////////

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
  return (static_cast<uint16_t>(buf[len - 2]) | static_cast<uint16_t>(buf[len - 1]) << 8) == crc;
}

uint16_t Mycila::PZEM::_crc16(const uint8_t* data, uint16_t len) {
  uint8_t nTemp;         // CRC table index
  uint16_t crc = 0xFFFF; // Default value
  while (len--) {
    nTemp = *data++ ^ crc;
    crc >>= 8;
    crc ^= static_cast<uint16_t>(pgm_read_word(&crcTable[nTemp]));
  }
  return crc;
}

///////////////////////////////////////////////////////////////////////////////
// static
///////////////////////////////////////////////////////////////////////////////

bool Mycila::PZEM::_add(PZEM* pzem) {
  for (size_t i = 0; i < MYCILA_PZEM_ASYNC_MAX_INSTANCES; i++) {
    if (_instances[i] == nullptr) {
      LOGD(TAG, "Adding instance at address 0x%02X to async task...", pzem->_address);

      if (_taskHandle == NULL) {
        pzem->_openSerial(pzem->_pinRX, pzem->_pinTX);

        if (!pzem->_canRead(pzem->_address)) {
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
        if (!pzem->_canRead(pzem->_address)) {
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
      delay(10);
  }
  _taskHandle = NULL;
  vTaskDelete(NULL);
}
