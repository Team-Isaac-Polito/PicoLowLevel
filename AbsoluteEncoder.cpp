#include "AbsoluteEncoder.h"

/**
 * Creates object - nothing needed
 *
 */
AbsoluteEncoder::AbsoluteEncoder() {
}

void AbsoluteEncoder::begin() {
  Debug.println("ABSOLUTE ENCODER - BEGIN ", Levels::INFO);

  AMS_AS5048B::begin();
  
  setClockWise(true); 

  setZero();
 }

void AbsoluteEncoder::setZero() {
  Debug.println("ABSOLUTE ENCODER - SET ZERO", Levels::DEBUG);

  zeroRegW(0x0);
}

float AbsoluteEncoder::readAngle() {
  float angle = angleR(U_DEG, false);
  
  Debug.print("ABSOLUTE ENCODER - READ ANGLE ", Levels::DEBUG);
  Debug.print(angle, Levels::DEBUG);

  return angle;
}

void AbsoluteEncoder::update() {
  updateMovingAvgExp();
}
