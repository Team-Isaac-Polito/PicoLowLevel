#include "functions.h"
#include "Arduino.h"
#include "config.h"

extern unsigned long countStepsLeft;
extern unsigned long countStepsRight;
extern int refTrLeft;
extern int refTrRight;
extern int refYaw;

void receive(int numBytes){
  refTrLeft = Wire.read();
  refTrRight = Wire.read();
  resYaw = Wire.read();
}

//ISR dell'interrupt motore R, viene richiamata ad ogni fronte di salita del segnale nell' encoder1A (RISING)
void handleEncoderLeft()
{
    countStepsLeft += digitalRead(ENC_TR_LEFT_B) == HIGH ? 1 : -1; // aggiungo o tolgo un passo in base al segnale del pin direzione
}

void handleEncoderRight(){
    countStepsRight += digitalRead(ENC_TR_RIGHT_B) == HIGH ? 1 : -1; // aggiungo o tolgo un passo in base al segnale del pin direzione
}



float getLeftEncoderData() {
  static int tempo;
  noInterrupts(); // AG INIZIO operazione atomica - non può essere interrotta da interrupt

  DEBUG("STEPS COUNTED");
  DEBUG(countStepsLeft);

  float rpm = countStepsLeft * 1000 / (1.9*(millis()-tempo));

  DEBUG("CALCULATED RPM");
  DEBUG(rpm);

  countStepsLeft = 0;
  tempo = millis();

  interrupts();
  
  return rpm;      
}

float getRightEncoderData() {
  static int tempo;
  noInterrupts(); // AG INIZIO operazione atomica - non può essere interrotta da interrupt

  DEBUG("STEPS COUNTED");
  DEBUG(countStepsRight);

  float rpm = countStepsRight * 1000 / (1.9*(millis()-tempo));

  DEBUG("CALCULATED RPM");
  DEBUG(rpm);

  countStepsRight = 0;
  tempo = millis();

  interrupts();
  
  return rpm;      
}