#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "Debug.h"
#include "definitions.h"

/**
 * Motor class used to control simple DC motors.
 */
class Motor {
  public:
    Motor(byte pwm, byte dir);
    void begin();
    void write(float value);
    
  private:
    byte pwm, dir;
};

#endif
