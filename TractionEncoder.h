#ifndef TRACTION_ENCODER_H
#define TRACTION_ENCODER_H

#include <Arduino.h>
#include <functional>
#include "Debug.h"
#include "definitions.h"

/**
 * Class used to read data from traction rotary encoders.
 */
class TractionEncoder { 
  public:
    TractionEncoder(byte pin_a, byte pin_b);
    void begin();
    int getSpeed();

  private:
    byte pin_a, pin_b;
    long countSteps;
    unsigned long time;

    void ISR();
    static void ISR_wrapper(TractionEncoder* te);
};

#endif
