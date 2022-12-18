#include "TractionEncoder.h"

/**
 * Creates object.
 */
TractionEncoder::TractionEncoder(byte pin_a, byte pin_b) {
    this->pin_a = pin_a;
    this->pin_b = pin_b;

    time = micros();
}

/**
 * Sets the encoder's pins and attach the interrupt.
 */
void TractionEncoder::begin() {
    pinMode(pin_a,INPUT);
    pinMode(pin_b,INPUT);
    attachInterrupt(digitalPinToInterrupt(pin_a), &ISR_wrapper, CHANGE, this);
    attachInterrupt(digitalPinToInterrupt(pin_b), &ISR_wrapper, CHANGE, this);
}

/**
 * Computes and returns the last detected speed.
 * @return The speed in milliRPMs.
 */
int TractionEncoder::getSpeed() {
    long rpm;

    Debug.print("TRACTIONENCODER - Steps Counted  ", Levels::DEBUG);
    Debug.println(countSteps, Levels::DEBUG);

    noInterrupts();     // AG INIZIO operazione atomica - non può essere interrotta da interrupt
    rpm = countSteps;
    countSteps = 0;
    interrupts();

    // check definitions.h to understand what the constant is
    rpm = rpm * ENC_TR_CONVERSION / (long)(micros()-time);
  
    Debug.print("TRACTIONENCODER - Calculated RPM  ", Levels::DEBUG);
    Debug.println(rpm, Levels::DEBUG);

    time = micros();

    return rpm; 
}

/**
 * Interrupt Service Routing for the encoder.
 * This registers a new rotation has happened every time it's called. 
 */
void TractionEncoder::ISR() {  
    // saving state of pin_a for next iteration
    old_a = digitalRead(pin_a);
    bool val_b = digitalRead(pin_b);

    // direction detection can be simplified to a XOR between current state of pin b and old state of pin a
    bool ccw = val_b ^ old_a;
    
    // increase steps for ccw, decrease for cw
    countSteps += ccw ? +1 : -1;
}

/**
 * Static wrapper for ISR(), allowing it to be used in attachInterrupt().
 * @param te The instance of the TractionEncoder class to call ISR() for.
 */
void TractionEncoder::ISR_wrapper(TractionEncoder* te) {
  te->ISR();
}
