#include "PID.h"
#include "functions.h"

PID::PID(float kp, float ki, float kd) {
  KP = kp;
  KI = ki;
  KD = kd;

  errorI = 0;
  oldError = 0;
  tempo = millis();
}

void PID::updateReferenceValue(float ref) {
  DEBUG(ref);
  referenceValue = ref;
}

void PID::updateFeedback(float fb) {
  DEBUG(fb);
  oldFeedback = feedback;
  feedback = fb;
}

float PID::calculate() {
  float errorP=0, errorD=0;

  int dt = millis() - tempo;

  
  //calcolo errori dei 3 contributi
  errorP = referenceValue - feedback;
  //errorI += error * DT / 1000;            //contributo integrativo 
  errorD = 1000 * (errorP - oldError) / dt;   //contributo derivativo

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
  DEBUG("PROPORTIONAL");
  DEBUG(KP * errorP);
  DEBUG(KP);
  DEBUG(KI);
  DEBUG(KD);
  DEBUG("INTEGRATIVE");
  DEBUG(KI * errorI);
  DEBUG("DERIVATIVE");
  DEBUG(KD * errorD);

  oldError = errorP;


  //richiama la funzione per il calcolo del periodo di oscillazione e print della media - Tuning
  if (FLAG_TUNING == 1)
    mediaTuning(output, tempo);
  
  tempo = millis();
  
  return output;

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

}
