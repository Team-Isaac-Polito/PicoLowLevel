#include "AbsoluteEncoder.h"

/**
 * Creates object - nothing needed
 *
 */
AbsoluteEncoder::AbsoluteEncoder() {
}

void AbsoluteEncoder::begin() {
  absEncoder.begin();
  absEncoder.setClockWise(true); 

  setZero();
 }

void AbsoluteEncoder::setZero() {
  absEncoder.zeroRegW(0x0);
}

float AbsoluteEncoder::readAngle() {
  return absEncoder.angleR(U_DEG, false);
}

void AbsoluteEncoder::update() {
  absEncoder.updateMovingAvgExp();
}