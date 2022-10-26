#ifndef Temperature_h
#define Temperature_h

#include <Arduino.h>
#include <Wire.h>

#define MLX90614_TA 0x06
#define MLX90614_TOBJ1 0x07

class Temperature  {
  public:
    Temperature(uint8_t address);
    bool check();
    float read();
    float readAmb();
  private:
    float readTemp(uint8_t reg);
    uint8_t address;
    uint16_t read16(uint8_t reg);
};

#endif
