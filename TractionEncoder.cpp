#include "TractionEncoder.h"

/**
 * Creates object.
 */
TractionEncoder::TractionEncoder(byte pin_a, byte pin_b) {
    this->pin_a = pin_a;
    this->pin_b = pin_b;

    tempo = millis();
}

/**
 * Sets the encoder's pins and attach the interrupt.
 */
void TractionEncoder::begin() {
    pinMode(pin_a,INPUT);
    pinMode(pin_b,INPUT);
    attachInterrupt(digitalPinToInterrupt(pin_a), &ISR_wrapper, RISING, this);
}

/**
 * Computes and returns the last detected speed.
 * @return The speed in RPMs.
 */
float TractionEncoder::getSpeed() {
    float rpm;

    Debug.print("TRACTIONENCODER - Steps Counted - ", Levels::DEBUG);
    Debug.println(countSteps, Levels::DEBUG);

    noInterrupts();     // AG INIZIO operazione atomica - non può essere interrotta da interrupt
    rpm = countSteps;
    countSteps = 0;
    interrupts();

    rpm = rpm * 1000 / (1.9*(millis()-tempo));

    Debug.print("TRACTIONENCODER - Calculated RPM - ", Levels::DEBUG);
    Debug.println(rpm, Levels::DEBUG);

    tempo = millis();

    return rpm; 
}

/**
 * Interrupt Service Routing for the encoder.
 * This registers a new rotation has happened every time it's called. 
 */
void TractionEncoder::ISR() {   
    countSteps += digitalRead(pin_b) == HIGH ? 1 : -1; // aggiungo o tolgo un passo in base al segnale del pin direzione
}

/**
 * Static wrapper for ISR(), allowing it to be used in attachInterrupt().
 * @param te The instance of the TractionEncoder class to call ISR() for.
 */
void TractionEncoder::ISR_wrapper(TractionEncoder* te) {
  te->ISR();
}
