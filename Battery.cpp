#include "Battery.h"

/**
 * Reads the current battery voltage.
 * @return Battery voltage in volts.
 */
float Battery::readVoltage() {
  float vpart = analogRead(pin) * (3.3f / 1023.0f);
  return vpart * ((r1 + r2)/r2);
}

/**
 * Check the battery charge status.
 * @return true if the battery is still charged.
 */
bool Battery::charged() {
  return readVoltage() > BAT_LOW;
}