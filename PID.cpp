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
  float errorP=0, errorD=0;

  //calcolo errori dei 3 contributi
  errorP = referenceValue - feedback;
  //errorI += error * DT / 1000;            //contributo integrativo 
  errorD = 1000 * (errorP - oldError) / DT;   //contributo derivativo

  //anti-windup: si attiva se 1)l’output è in saturazione e 2)l’errore ha segno concorde all’output
  if ((output > MAX_OUTPUT || output < MIN_OUTPUT) && (errorP * output > 0)){
    //non incremento l’errore integrale

    if (output > MAX_OUTPUT)
      output = MAX_OUTPUT;    //riporta il valore troppo alto al valore massimo, così da mantenerlo reattivo in caso di errore negativo
    else
      output = MIN_OUTPUT;
  }
  else {
    errorI += errorP * DT / 1000;    //l’errore integrale con anti-windup non attivo [contributo integrativo]
  }
 
  // somma dei contributi
  output = KP * errorP + KI * errorI + KD * errorD;

  oldError = errorP;

  return output;

}
