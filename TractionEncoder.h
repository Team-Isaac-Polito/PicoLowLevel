#ifndef TRACTION_ENCODER_H
#define TRACTION_ENCODER_H

#include <Arduino.h>
#include <functional>
#include "Debug.h"
#include "definitions.h"
#include "MovingAvgFilter.h"

/**
 * Class used to read data from traction rotary encoders.
 */
class TractionEncoder { 
  public:
    TractionEncoder(byte pin_a, byte pin_b, Filter<int> *filter = NULL);
    void begin();
    void update();
    int getSpeed();

  private:
    byte pin_a, pin_b;
    bool old_a;
    long countSteps;
    unsigned long time;
    int speed;

    Filter<int> *filter;

    void ISR();
    static void ISR_wrapper(TractionEncoder* te);
};

#endif
