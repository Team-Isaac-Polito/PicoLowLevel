#include "Battery.h"

float Battery::readVoltage() {
  float vpart = analogRead(pin) * (3.3f / 4095.0f);
  return vpart * ((r1 + r2)/r2);
}

bool Battery::charged() {
  return readVoltage() > BAT_LOW;
}