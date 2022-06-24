#include "AbsoluteEncoder.h"

/**
 * Creates object - nothing needed
 */
AbsoluteEncoder::AbsoluteEncoder(byte addr) : AMS_AS5048B(addr) {
  
}

/**
 * Start the encoder at zero with the default direction.
 */
void AbsoluteEncoder::begin() {
  Debug.println("ABSOLUTE ENCODER - BEGIN ", Levels::INFO);

  AMS_AS5048B::begin();
  
  setClockWise(true);
}

/**
 * Sets the encoder current position to zero.
 */
void AbsoluteEncoder::setZero() {
  Debug.println("ABSOLUTE ENCODER - SET ZERO", Levels::DEBUG);

  zeroRegW(0x0);
}

/**
 * Reads the current measured angle.
 * @return Angle in degrees.
 */
float AbsoluteEncoder::readAngle() {
#ifdef ABS_ENC_OFFSET
  float angle = angleR(U_DEG, false) - ABS_ENC_OFFSET;
#else
  float angle = angleR(U_DEG, false);
#endif
  Debug.print("ABSOLUTE ENCODER - READ ANGLE ", Levels::DEBUG);
  Debug.print(angle, Levels::DEBUG);

  return angle;
}

/**
 * Updates the computations to keep the angle estimation correct.
 */
void AbsoluteEncoder::update() {
  updateMovingAvgExp();
}
