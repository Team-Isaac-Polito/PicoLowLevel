#include "PID.h"

PID::PID(float kp, float ki, float kd, float max_output, float alpha) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->max_output = max_output;
  this->alpha = alpha;

  integral = 0;
  old_ef = 0;
  tempo = millis();
}

void PID::updateReferenceValue(float ref) {
  Debug.print("PID - Updated reference value  ", Levels::DEBUG);
  Debug.println(ref, Levels::DEBUG);

  referenceValue = ref;
}

void PID::updateFeedback(float fb) {  
  Debug.print("PID - Updated feedback  ", Levels::DEBUG);
  Debug.println(fb, Levels::DEBUG);
  
  feedback = fb;
}


float PID::getOutput() {
  return output;
}


float PID::getReferenceValue() {
  return referenceValue;
}

void PID::calculate() {
  int dt;
  float error,ef,derivative,new_integral;
  
  dt = millis() - tempo; // sarà fatto a frequenza fissata, andrà eliminato e magari aggiunto come parametro al costruttore

  // e[k] = r[k] - y[k], error between setpoint and true position
  error = referenceValue - feedback;

  // e_f[k] = α e[k] + (1-α) e_f[k-1], filtered error
  ef = alpha * error + (1 - alpha) * old_ef;
  
  // e_d[k] = (e_f[k] - e_f[k-1]) / Tₛ, filtered derivative
  derivative = (ef - old_ef) / dt;
  
  // e_i[k+1] = e_i[k] + Tₛ e[k], integral
  new_integral = integral + error * dt;

  // PID formula:
  // u[k] = Kp e[k] + Ki e_i[k] + Kd e_d[k], control signal
  output = kp * error + ki * integral + kd * derivative;

  // Clamp the output
  if (output > max_output)
      output = max_output;
  else if (output < -max_output)
      output = -max_output;
  else // Anti-windup
      integral = new_integral;

  Debug.print("PID - Calculated output  ", Levels::DEBUG);
  Debug.println(output, Levels::DEBUG);

  
  // store the state for the next iteration
  old_ef = ef;
  tempo = millis();
}

//old version
/*
float PID::calculate() {
  float errorP, errorD,output;


  
  //calcolo errori dei 3 contributi
  errorP = referenceValue - feedback;
  //errorI += error * DT / 1000;            //contributo integrativo 
  errorD = 1000 * (errorP - oldError) / dt;   //contributo derivativo

  output
  //anti-windup: si attiva se 1)l’output è in saturazione e 2)l’errore ha segno concorde all’output
  if ((output > MAX_OUTPUT || output < MIN_OUTPUT) && (errorP * output > 0)){
    //non incremento l’errore integrale
    DEBUG("ANTIWINDUP ATTIVO ----------------------------------------");

    if (output > MAX_OUTPUT)
      output = MAX_OUTPUT;    //riporta il valore troppo alto al valore massimo, così da mantenerlo reattivo in caso di errore negativo
    else
      output = MIN_OUTPUT;
  }
  else {
    errorI += errorP * dt / 1000;    //l’errore integrale con anti-windup non attivo [contributo integrativo]
  }
 
  // somma dei contributi
  output = KP * errorP + KI * errorI + KD * errorD;

  oldError = errorP;
  tempo = millis();
}


void PID::mediaTuning(float output, int tempo){      //codice per il calcolo del periodo di oscillazione della sinusoide ottenuta durante il Tuning [Metodo ziegler-Nichols]
  if (tuningIndex != N_CAMPIONAMENTI){
    if (output > oldOutput){    //sta ancora salendo, aggiorno il valore di picco
      flagTuninInterno = 1;
    }
    else if ((output < oldOutput) && flagTuninInterno == 1){   //è iniziata la discesa
      campionamenti[tuningIndex] = tempo;    //salvo il valore del tempo associato al picco (loop precedente)
      tuningIndex++;
      flagTuninInterno = 0;
    }
  
    //calcolo la media dei periodi di oscillazione
    if (tuningIndex > 2){
      tOscillazione = campionamenti[tuningIndex-1] - campionamenti[tuningIndex-2];
      media = (tOscillazione + media) / 2;
    }
    else if (tuningIndex == 2){
       tOscillazione = campionamenti[tuningIndex-1] - campionamenti[tuningIndex-2];
       media = tOscillazione;
    }
}

  DEBUG("PERIOD OF OSCILLATION");
  DEBUG(media);
  
  oldOutput = output;

}*/
