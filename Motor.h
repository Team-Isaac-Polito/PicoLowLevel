#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
  public:
    Motor(byte pwm, byte dir);
    void begin();
    void write(float value);
    
  private:
    byte pwm, dir;
};

#endif
