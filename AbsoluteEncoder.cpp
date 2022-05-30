#include "AbsoluteEncoder.h"

/**
 * Creates object - nothing needed
 *
 */
AbsoluteEncoder::AbsoluteEncoder(byte i2c_sda, byte i2c_scl) {
  this->i2c_sda = i2c_sda;
  this->i2c_scl = i2c_scl;
}

void AbsoluteEncoder::begin() {
  Wire1.setSDA(i2c_sda);
  Wire1.setSCL(i2c_scl);
  absEncoder.begin();
  absEncoder.setClockWise(true); 

  this->setZero();
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