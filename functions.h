#ifndef FUNCTIONS_H
#define FUNCTIONS_H

//macro per stampare su seriale solo se SERIAL_DEBUG_ATTIVATO è 1
#if SERIAL_DEBUG_ATTIVATO 
#define DEBUG(arg) Serial.println(arg);
#else 
#define DEBUG(arg) {}
#endif


// ToDo
void receive(int numBytes); // function that runs when data is received
void handleEncoderLeft();
void handleEncoderRight();

float getLeftEncoderData();
float getRightEncoderData();


#endif
