#include "AbsoluteEncoder.h"

/**
 * Creates object - nothing needed
 *
 */
AbsoluteEncoder::AbsoluteEncoder() {
}

void AbsoluteEncoder::begin() {
  AMS_AS5048B::begin();
  setClockWise(true); 

  setZero();
 }

void AbsoluteEncoder::setZero() {
  zeroRegW(0x0);
}

float AbsoluteEncoder::readAngle() {
  return angleR(U_DEG, false);
}

void AbsoluteEncoder::update() {
  updateMovingAvgExp();
}
