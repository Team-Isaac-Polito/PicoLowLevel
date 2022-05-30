#ifndef ABSOLUTE_ENCODER_H
#define ABSOLUTE_ENCODER_H

#include <Arduino.h>
#include "Debug.h"
#include "definitions.h"
#include "ams_as5048b.h"

/**
 * Class used as a wrapper for the ams_as5048b library to read data from the absolute encoder.
 */
class AbsoluteEncoder {
  public:
    AbsoluteEncoder();
    void begin();
    void setZero();
    float readAngle();
    void update();

  private:
    AMS_AS5048B absEncoder;
};

#endif
