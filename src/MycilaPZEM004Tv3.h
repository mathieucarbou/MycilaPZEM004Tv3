// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 */
#pragma once

#include <HardwareSerial.h>

#include <mutex>
#include <shared_mutex>

#ifdef MYCILA_JSON_SUPPORT
  #include <ArduinoJson.h>
#endif

#define MYCILA_PZEM_VERSION          "5.0.1"
#define MYCILA_PZEM_VERSION_MAJOR    5
#define MYCILA_PZEM_VERSION_MINOR    0
#define MYCILA_PZEM_VERSION_REVISION 1

#ifndef MYCILA_PZEM_ASYNC_CORE
  #define MYCILA_PZEM_ASYNC_CORE 0
#endif

#ifndef MYCILA_PZEM_ASYNC_PRIORITY
  #define MYCILA_PZEM_ASYNC_PRIORITY 1
#endif

#ifndef MYCILA_PZEM_ASYNC_STACK_SIZE
  #define MYCILA_PZEM_ASYNC_STACK_SIZE 3072 // 512 * 6
#endif

#ifndef MYCILA_PZEM_ASYNC_MAX_INSTANCES
  #define MYCILA_PZEM_ASYNC_MAX_INSTANCES 4
#endif

#define MYCILA_PZEM_ADDRESS_UNKNOWN 0x00
#define MYCILA_PZEM_ADDRESS_MIN     0x01
#define MYCILA_PZEM_ADDRESS_MAX     0xF7
// default general address in a single device configuration which is able to reach any device
// in a mutli-device configuration, the address should be set to a value between 0x01 and 0xF7
#define MYCILA_PZEM_ADDRESS_GENERAL 0xF8

namespace Mycila {
  class PZEM {
    public:
      enum class EventType {
        // PZEM has successfully read the data
        EVT_READ = 0,
        // PZEM has successfully read the data and the values have changed
        EVT_CHANGE,
        // wrong data received when reading values
        EVT_READ_ERROR,
        // timeout reached when reading values
        EVT_READ_TIMEOUT
      };

      class Data {
        public:
          float frequency = NAN; // Hz

          /**
           * @brief Voltage in volts (V).
           */
          float voltage = NAN;

          /**
           * @brief Current in amperes (A).
           */
          float current = NAN;

          /**
           * @brief Active power in watts (W).
           */
          float activePower = NAN;

          /**
           * @brief Power factor
           */
          float powerFactor = NAN;

          /**
           * @brief Apparent power in volt-amperes (VA).
           */
          float apparentPower = NAN;

          /**
           * @brief Reactive power in volt-amperes reactive (VAr).
           */
          float reactivePower = NAN;

          /**
           * @brief Active energy in kilowatt-hours (kWh).
           */
          float activeEnergy = NAN;

          /**
           * @brief Compute the total harmonic distortion of current (THDi).
           * See: https://fr.electrical-installation.org/frwiki/Indicateur_de_distorsion_harmonique_:_facteur_de_puissance
           * @param phi The phase shift angle in radians (0 for resistive load)
           * @return The total harmonic distortion of current (THDi)
           */
          float thdi(float phi = 0) const;

          /**
           * @brief Compute the resistance of the load in ohms (R = P / I^2).
           */
          float resistance() const;

          /**
           * @brief Compute the dimmed voltage (V = P / I).
           * @note The dimmed voltage is the voltage that would be measured at the output of a TRIAC, SSR or voltage regulator device.
           */
          float dimmedVoltage() const;

          /**
           * @brief Compute the nominal power of the load in watts (P = V^2 / R).
           * @note The voltage is the nominal voltage measured by the device and R is the measured resistance of the load, which can be regulated by a TRIAC, SSR or voltage regulator.
           */
          float nominalPower() const;

          // clear all values
          void clear();

          // compare two data
          bool operator==(const Data& other) const;
          // compare two data
          bool operator!=(const Data& other) const { return !(*this == other); }
          // copy a data
          void operator=(const Data& other);

#ifdef MYCILA_JSON_SUPPORT
          void toJson(const JsonObject& root) const;
#endif

        private:
          friend class PZEM;
          mutable std::shared_mutex _mutexData;
      };

      typedef std::function<void(EventType eventType)> Callback;

      ~PZEM() { end(); }

      // - rxPin: RX pin of the board, connected to the TX of the PZEM,
      // - txPin: TX pin of the board, connected to the RX of the PZEM
      // - address: the address of the PZEM. Default to MYCILA_PZEM_DEFAULT_ADDRESS. Set to a value between 0x01 and 0xF7 included, or MYCILA_PZEM_DEFAULT_ADDRESS (default)
      void begin(HardwareSerial& serial, // NOLINT
                 int8_t rxPin,
                 int8_t txPin,
                 uint8_t address = MYCILA_PZEM_ADDRESS_GENERAL,
                 bool async = false);

      void end();

      // No need to call read in async mode
      bool read() { return read(_address); }
      bool read(uint8_t address);

      // Resets energy counters. Returns true if the reset was successful
      bool resetEnergy() { return resetEnergy(_address); }
      bool resetEnergy(uint8_t address);

      /**
       * @brief Get the address of the last device's response.
       */
      uint8_t getLastAddress() const { return _lastAddress; }

      /**
       * @brief Get the address used to send requests.
       * @return The address used to send requests (1-255) or MYCILA_PZEM_ADDRESS_GENERAL (0) for all devices.
       */
      uint8_t getDeviceAddress() const { return _address; }

      // Try to change the address. Returns true if changed
      bool setDeviceAddress(uint8_t newAddress) { return setDeviceAddress(_address, newAddress); }
      bool setDeviceAddress(uint8_t address, uint8_t newAddress);

      // read address from device and update the destination address variable if true.
      // Returns MYCILA_PZEM_ADDRESS_UNKNOWN in case of error or the device address in case of success.
      uint8_t readDeviceAddress(bool update = false);

      // Will start searching for PZEM devices on custom addresses from 0x01 to 0xF7,
      // And then also MYCILA_PZEM_DEFAULT_ADDRESS (0xF8).
      // Will stop when reaching maxCount or when no more devices are found.
      // A full scan can take up to 30 seconds
      // Returns the number of devices found
      // MYCILA_PZEM_DEFAULT_ADDRESS (0xF8) is also searched for to be able to find PZEM devices that have not been assigned an address.
      size_t search(uint8_t* addresses, const size_t maxCount);

#ifdef MYCILA_JSON_SUPPORT
      void toJson(const JsonObject& root) const;
#endif

      gpio_num_t getRXPin() const { return _pinRX; }
      gpio_num_t getTXPin() const { return _pinTX; }
      bool isEnabled() const { return _enabled; }

      // get the uptime in milliseconds of the last successful read
      uint32_t getTime() const { return _time; }

      // check if the device is connected to the , meaning if last read was successful
      bool isConnected() const { return data.frequency > 0; }

      void setCallback(Callback callback) { _callback = callback; }

    public:
      /**
       * @brief Access the runtime PZEM data.
       */
      Data data;

    private:
      bool _enabled = false;
      Callback _callback = nullptr;
      gpio_num_t _pinRX = GPIO_NUM_NC;
      gpio_num_t _pinTX = GPIO_NUM_NC;
      HardwareSerial* _serial = nullptr;
      uint32_t _time = 0;
      uint8_t _buffer[32];
      uint8_t _address = MYCILA_PZEM_ADDRESS_GENERAL;
      uint8_t _lastAddress = MYCILA_PZEM_ADDRESS_UNKNOWN;

    private:
      enum class ReadResult {
        READ_SUCCESS = 0,
        READ_TIMEOUT,
        READ_ERROR_COUNT,
        READ_ERROR_CRC,
        READ_ERROR_ADDRESS,
      };

    private:
      bool _canRead(uint8_t address);
      ReadResult _timedRead(uint8_t expectedAddress, size_t expectedLen);
      void _send(uint8_t address, uint8_t cmd, uint16_t rAddr, uint16_t val);
      void _openSerial(const uint8_t rxPin, const uint8_t txPin);
      size_t _drop();

    private:
      static void _crcSet(uint8_t* buf, uint16_t len);
      static bool _crcCheck(const uint8_t* buf, uint16_t len);
      static uint16_t _crc16(const uint8_t* data, uint16_t len);

    private:
      static std::timed_mutex _mutex;
      static TaskHandle_t _taskHandle;
      static PZEM* _instances[MYCILA_PZEM_ASYNC_MAX_INSTANCES];
      static bool _add(PZEM* pzem);
      static void _remove(PZEM* pzem);
      static void _pzemTask(void* pvParameters);
  };
} // namespace Mycila
