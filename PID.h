#ifndef PID_H
#define PID_H

#include <Arduino.h>

class PID {
  public:
    PID(float kp, float ki, float kd, float max_output, float alpha);
    void updateReferenceValue(float ref);
    float getOutput();
    void calculate();
    void updateFeedback(float fb);

  private:
    float kp,ki,kd, referenceValue, output, max_output, feedback, old_ef, integral,alpha;
    int tempo;
};


#endif
