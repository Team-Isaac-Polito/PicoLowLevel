#include "PID.h"

PID::PID(float kp, float ki, float kd) {
  KP = kp;
  KI = ki;
  KP = kd;

  //ToDo inizializzare altre variabili: error_i, old_error
}

void PID::updateReferenceValue(float ref) {
  referenceValue = ref;
}

void PID::updateFeedback(float fb) {
  oldFeedback = feedback;
  feedback = fb;
}

float PID::calculate() {
  float output;
  float error=0, error_d=0;
  //calcolo errori dei 3 contributi
  error = referenceValue - feedback;    //contributo proporzionale
  error_i += error*DT /1000;            //contributo integrativo 
  error_d = 1000*(error - old_error) / DT;   //contributo derivativo
 
  //velocità di output Encoder Relativo
  output = KP*error + KI*error_i + KD*error_d;

  old_error = error;
  return output;

}