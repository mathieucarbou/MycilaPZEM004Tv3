// SPDX-License-Identifier: MIT
/*
 * Copyright (C) 2023-2024 Mathieu Carbou
 */
#include "MycilaPZEM004Tv3.h"

float Mycila::PZEM::Data::thdi(float phi) const {
  if (powerFactor == 0)
    return NAN;
  const float cosPhi = phi == 1 ? 1 : cos(phi);
  return sqrt((cosPhi * cosPhi) / (powerFactor * powerFactor) - 1);
}

float Mycila::PZEM::Data::resistance() const { return current == 0 ? NAN : abs(activePower / (current * current)); }

float Mycila::PZEM::Data::dimmedVoltage() const { return current == 0 ? NAN : abs(activePower / current); }

float Mycila::PZEM::Data::nominalPower() const { return activePower == 0 ? NAN : abs(voltage * voltage * current * current / activePower); }

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
  return (isnanf(frequency) ? isnanf(other.frequency) : frequency == other.frequency) &&
         (isnanf(voltage) ? isnanf(other.voltage) : voltage == other.voltage) &&
         (isnanf(current) ? isnanf(other.current) : current == other.current) &&
         (isnanf(activePower) ? isnanf(other.activePower) : activePower == other.activePower) &&
         (isnanf(reactivePower) ? isnanf(other.reactivePower) : reactivePower == other.reactivePower) &&
         (isnanf(apparentPower) ? isnanf(other.apparentPower) : apparentPower == other.apparentPower) &&
         (isnanf(powerFactor) ? isnanf(other.powerFactor) : powerFactor == other.powerFactor) &&
         (isnanf(activeEnergy) ? isnanf(other.activeEnergy) : activeEnergy == other.activeEnergy);
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
  if (!isnan(frequency))
    root["frequency"] = frequency;
  if (!isnan(voltage))
    root["voltage"] = voltage;
  if (!isnan(current))
    root["current"] = current;
  if (!isnan(activePower))
    root["active_power"] = activePower;
  if (!isnan(reactivePower))
    root["reactive_power"] = reactivePower;
  if (!isnan(apparentPower))
    root["apparent_power"] = apparentPower;
  if (!isnan(powerFactor))
    root["power_factor"] = powerFactor;
  if (!isnan(activeEnergy))
    root["active_energy"] = activeEnergy;
  float r = resistance();
  float d = dimmedVoltage();
  float n = nominalPower();
  float t = thdi();
  if (!isnan(r))
    root["resistance"] = r;
  if (!isnan(d))
    root["dimmed_voltage"] = d;
  if (!isnan(n))
    root["nominal_power"] = n;
  if (!isnan(t))
    root["thdi"] = t;
}
#endif
