#include "functions.h"
#include "Arduino.h"
#include "config.h"

extern unsigned long countStepsLeft;
extern unsigned long countStepsRight;

void receive(int numBytes){}

//ISR dell'interrupt motore R, viene richiamata ad ogni fronte di salita del segnale nell' encoder1A (RISING)
void handleEncoderLeft()
{
    countStepsLeft += digitalRead(ENC_TR_LEFT_B) == HIGH ? 1 : -1; // aggiungo o tolgo un passo in base al segnale del pin direzione
}

void handleEncoderRight(){
    countStepsRight += digitalRead(ENC_TR_RIGHT_B) == HIGH ? 1 : -1; // aggiungo o tolgo un passo in base al segnale del pin direzione
}



float getLeftEncoderData() {
    ////calcolo numero di giri albero////
    cli();                                                              // AG INIZIO operazione atomica - non può essere interrotta da interrupt
    float rpm = ((float(countStepsLeft) / 12.0) * (60.0 * 1000.0 /DT)) / 74.83;

    ///* quando count=12 il motore effettua un giro (interno, non dell'albero)[trasformo gli impulsi in giri]
    //(60.0 * 1000 / DT) passo da ms a minuti
    ///74.83) divido per il rapporto di trasmissione del motore
    countStepsLeft = 0;
    sei();
    return rpm;      
}

float getRightEncoderData() {
    ////calcolo numero di giri albero////
    cli();                                                              // AG INIZIO operazione atomica - non può essere interrotta da interrupt
    float rpm = ((float(countStepsRight) / 12.0) * (60.0 * 1000.0 /DT)) / 74.83;

    ///* quando count=12 il motore effettua un giro (interno, non dell'albero)[trasformo gli impulsi in giri]
    //(60.0 * 1000 / DT) passo da ms a minuti
    ///74.83) divido per il rapporto di trasmissione del motore
    countStepsRight = 0;
    sei();
    return rpm;      
}
