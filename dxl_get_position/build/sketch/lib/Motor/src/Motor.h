#line 1 "C:\\Users\\franc\\Desktop\\isaac\\RCR\\dxl_get_position\\lib\\Motor\\src\\Motor.h"
#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "../../../include/definitions.h"

/**
 * Motor class used to control simple DC motors.
 */
class Motor {
  public:
    Motor(byte pwm, byte dir, bool invert = false);
    void begin();
    void write(int value);

  private:
    byte pwm, dir;
    bool invert;
};

#endif
