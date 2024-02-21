// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou and others
 */
#pragma once

#include <HardwareSerial.h>

#include <mutex>

#ifdef MYCILA_PZEM_JSON_SUPPORT
#include <ArduinoJson.h>
#endif

#define MYCILA_PZEM_VERSION          "1.0.0"
#define MYCILA_PZEM_VERSION_MAJOR    1
#define MYCILA_PZEM_VERSION_MINOR    0
#define MYCILA_PZEM_VERSION_REVISION 0

#ifndef MYCILA_PZEM_ASYNC_CORE
#define MYCILA_PZEM_ASYNC_CORE 0
#endif

#ifndef MYCILA_PZEM__ASYNC_PRIORITY
#define MYCILA_PZEM__ASYNC_PRIORITY 1
#endif

#ifndef MYCILA_PZEM_ASYNC_STACK_SIZE
#define MYCILA_PZEM_ASYNC_STACK_SIZE 2048
#endif

#ifndef MYCILA_PZEM_READ_TIMEOUT_MS
#define MYCILA_PZEM_READ_TIMEOUT_MS 100
#endif

#ifndef MYCILA_PZEM_DEFAULT_ADDRESS
#define MYCILA_PZEM_DEFAULT_ADDRESS 0xF8
#endif

#define MYCILA_PZEM_INVALID_ADDRESS 0x00

namespace Mycila {
  class PZEM {
    public:
      ~PZEM() { end(); }

      // - pzemRXPin: pin connected to the RX of the PZEM,
      // - pzemTXPin: pin connected to the TX of the PZEM
      // - address: the address of the PZEM. Default to MYCILA_PZEM_DEFAULT_ADDRESS. Set to a value between 0x01 and 0xF7 included, or MYCILA_PZEM_DEFAULT_ADDRESS (default)
      void begin(HardwareSerial* serial,
                 const uint8_t pzemRXPin,
                 const uint8_t pzemTXPin,
                 const uint8_t address = MYCILA_PZEM_DEFAULT_ADDRESS,
                 const bool async = false,
                 const uint8_t core = MYCILA_PZEM_ASYNC_CORE,
                 const uint32_t stackSize = MYCILA_PZEM_ASYNC_STACK_SIZE);

      void end();

      // No need to call read in async mode
      bool read();

      // Resets energy counters. Returns true if the reset was successful
      bool resetEnergy();

      // Try to change the address. Returns true if changed
      bool setAddress(const uint8_t address);

      // read address from device and update the address variable if true. Returns 0 in case of error.
      uint8_t readAddress(bool update = false);

      // will start searching for PZEM devices on custom addresses from 0x01 to 0xF8 (MYCILA_PZEM_DEFAULT_ADDRESS)
      // A full scan can take up to 30 seconds
      // Returns the number of devices found
      size_t search(uint8_t* addresses, const size_t maxCount);

#ifdef MYCILA_PZEM_JSON_SUPPORT
      void toJson(const JsonObject& root) const;
#endif

      gpio_num_t getRXPin() const { return _pinRX; }
      gpio_num_t getTXPin() const { return _pinTX; }
      uint8_t getAddress() const { return _address; }
      bool isEnabled() const { return _enabled; }

      float getCurrent() const { return _current; }
      float getEnergy() const { return _energy; }
      float getFrequency() const { return _frequency; }
      // note :this is the active power, not the apparent power
      float getPower() const { return _power; }
      float getPowerFactor() const { return _powerFactor; }
      float getApparentPower() const { return _power / _powerFactor; }
      float getVoltage() const { return _voltage; }

      // get the uptime in milliseconds of the last successful read
      uint32_t getTime() const { return _lastReadSuccess; }

    private:
      volatile float _current = 0;   // A
      volatile float _energy = 0;    // kWh
      volatile float _frequency = 0; // Hz
      volatile float _power = 0;     // W
      volatile float _powerFactor = 0;
      volatile float _voltage = 0; // V

    private:
      gpio_num_t _pinRX = GPIO_NUM_NC;
      gpio_num_t _pinTX = GPIO_NUM_NC;
      HardwareSerial* _serial = nullptr;
      uint32_t _lastReadSuccess = 0;
      TaskHandle_t _taskHandle;
      volatile bool _enabled = false;
      volatile uint8_t _address = MYCILA_PZEM_DEFAULT_ADDRESS;
      std::timed_mutex _mutex;

    private:
      void _openSerial();
      size_t _drop();
      bool _canRead();
      size_t _timedRead(uint8_t* buffer, size_t length);
      bool _sendCmd8(uint8_t cmd, uint16_t rAddr, uint16_t val, bool check = false, uint16_t slave_addr = 0xFFFF);

    private:
      static void _pzemTask(void* pvParameters);
      static void _crcSet(uint8_t* buf, uint16_t len);
      static bool _crcCheck(const uint8_t* buf, uint16_t len);
      static uint16_t _crc16(const uint8_t* data, uint16_t len);
  };
} // namespace Mycila
