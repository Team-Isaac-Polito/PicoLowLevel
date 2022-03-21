#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include "config.h"


class PID {
  public:
    PID(float kp, float ki, float kd);
    void updateReferenceValue(float ref);
    float calculate();
    void updateFeedback(float fb);

  private:
    float KP,KI,KD;
    float referenceValue;
    float feedback,oldFeedback;
    float errorI,output,oldError;
    int tempo;
};


#endif
