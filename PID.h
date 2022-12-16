#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "Debug.h"

class PID {
  public:
    PID(float kp, float ki, float kd, float max_output, float alpha);
    void updateReferenceValue(float ref);
    float getOutput();
    float getReferenceValue();
    void calculate();
    void updateFeedback(float fb);

  private:
    float kp, ki, kd; // gains
    float max_output, alpha; // constants
    float referenceValue, output, feedback; // variables
    float old_fe, old_integral; // state
    unsigned long tempo;
};


#endif
