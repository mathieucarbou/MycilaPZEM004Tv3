// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 */
#include "MycilaPZEM004Tv3.h"

float Mycila::PZEM::Data::thdi(float phi) const {
  if (powerFactor == 0)
    return NAN;
  const float cosPhi = phi == 0 ? 1 : std::cos(phi);
  return std::sqrt((cosPhi * cosPhi) / (powerFactor * powerFactor) - 1);
}

float Mycila::PZEM::Data::resistance() const { return current == 0 ? NAN : std::abs(activePower / (current * current)); }

float Mycila::PZEM::Data::dimmedVoltage() const { return current == 0 ? NAN : std::abs(activePower / current); }

float Mycila::PZEM::Data::nominalPower() const { return activePower == 0 ? NAN : std::abs(voltage * voltage * current * current / activePower); }

void Mycila::PZEM::Data::clear() {
  std::unique_lock lock(_mutexData);
  frequency = NAN;
  voltage = NAN;
  current = NAN;
  activePower = NAN;
  reactivePower = NAN;
  apparentPower = NAN;
  powerFactor = NAN;
  activeEnergy = NAN;
}

bool Mycila::PZEM::Data::operator==(const Mycila::PZEM::Data& other) const {
  std::shared_lock lock(_mutexData);
  std::shared_lock lockOther(other._mutexData);
  return (std::isnan(frequency) ? std::isnan(other.frequency) : frequency == other.frequency) &&
         (std::isnan(voltage) ? std::isnan(other.voltage) : voltage == other.voltage) &&
         (std::isnan(current) ? std::isnan(other.current) : current == other.current) &&
         (std::isnan(activePower) ? std::isnan(other.activePower) : activePower == other.activePower) &&
         (std::isnan(reactivePower) ? std::isnan(other.reactivePower) : reactivePower == other.reactivePower) &&
         (std::isnan(apparentPower) ? std::isnan(other.apparentPower) : apparentPower == other.apparentPower) &&
         (std::isnan(powerFactor) ? std::isnan(other.powerFactor) : powerFactor == other.powerFactor) &&
         (std::isnan(activeEnergy) ? std::isnan(other.activeEnergy) : activeEnergy == other.activeEnergy);
}

void Mycila::PZEM::Data::operator=(const Mycila::PZEM::Data& other) {
  std::unique_lock lock(_mutexData);
  std::shared_lock lockOther(other._mutexData);
  frequency = other.frequency;
  voltage = other.voltage;
  current = other.current;
  activePower = other.activePower;
  reactivePower = other.reactivePower;
  apparentPower = other.apparentPower;
  powerFactor = other.powerFactor;
  activeEnergy = other.activeEnergy;
}

#ifdef MYCILA_JSON_SUPPORT
void Mycila::PZEM::Data::toJson(const JsonObject& root) const {
  std::shared_lock lock(_mutexData);
  if (!std::isnan(frequency))
    root["frequency"] = frequency;
  if (!std::isnan(voltage))
    root["voltage"] = voltage;
  if (!std::isnan(current))
    root["current"] = current;
  if (!std::isnan(activePower))
    root["active_power"] = activePower;
  if (!std::isnan(reactivePower))
    root["reactive_power"] = reactivePower;
  if (!std::isnan(apparentPower))
    root["apparent_power"] = apparentPower;
  if (!std::isnan(powerFactor))
    root["power_factor"] = powerFactor;
  if (!std::isnan(activeEnergy))
    root["active_energy"] = activeEnergy;
  float r = resistance();
  float d = dimmedVoltage();
  float n = nominalPower();
  float t = thdi();
  if (!std::isnan(r))
    root["resistance"] = r;
  if (!std::isnan(d))
    root["dimmed_voltage"] = d;
  if (!std::isnan(n))
    root["nominal_power"] = n;
  if (!std::isnan(t))
    root["thdi"] = t;
}
#endif
