#include "AbsoluteEncoder.h"

/**
 * Creates object - nothing needed
 *
 */
AbsoluteEncoder::AbsoluteEncoder() {
}

void AbsoluteEncoder::begin() {
  AMS_AS5048B::begin();
  AMS_AS5048B::setClockWise(true); 

  setZero();
 }

void AbsoluteEncoder::setZero() {
  AMS_AS5048B::zeroRegW(0x0);
}

float AbsoluteEncoder::readAngle() {
  return AMS_AS5048B::angleR(U_DEG, false);
}

void AbsoluteEncoder::update() {
  AMS_AS5048B::updateMovingAvgExp();
}