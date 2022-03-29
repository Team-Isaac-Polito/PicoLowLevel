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
    void mediaTuning(float, int);
    float KP,KI,KD;
    float referenceValue;
    float feedback,oldFeedback;
    float errorI,output,oldError;
    float oldOutput = 0;
    float tOscillazione, campionamenti[N_CAMPIONAMENTI] = {0};
    int tempo, media=0, tuningIndex = 0, flagTuninInterno;
};


#endif
