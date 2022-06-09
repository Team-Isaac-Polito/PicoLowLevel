#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>
#include "Debug.h"
#include "definitions.h"

class Battery { 
  public:
    Battery(byte pin = BAT_PIN, int r1 = BAT_R1, int r2 = BAT_R2) : pin(pin), r1(r1), r2(r2) {}
    float readVoltage();
    bool charged();

  private:
    byte pin;
    int r1, r2;
};

#endif