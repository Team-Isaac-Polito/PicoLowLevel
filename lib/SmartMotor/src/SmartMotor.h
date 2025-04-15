#ifndef SMART_MOTOR_H
#define SMART_MOTOR_H

#include <Arduino.h>

#include "Motor.h"
#include "TractionEncoder.h"
#include "PID.h"
#include "MovingAvgFilter.h"
#include "Debug.h"
#include <Adafruit_ADS1X15.h>

/**
 * Class used to control DC motors at a constant speed.
 * Relies on a PID controller receiving data from a rotary encoder that uses a PIO state machine to reduce CPU usage and increase reliability.
 */
class SmartMotor {
  public:
    SmartMotor(byte pwm, byte dir, byte enc_a, byte enc_b, Adafruit_ADS1115& adc, int base_adc_channel, bool invert = false,  PIO pio = pio0);
    void begin();
    void update();

    void setSpeed(float value);
    float getSpeed();
    float getCurrent();
    float getTemperature();
    void stop();

    void calibrate(float target = 45.f);

  private:
    int speedToPower(float speed);

    Motor motor;
    TractionEncoder encoder;
    PID pid;

    Adafruit_ADS1115& adc;
    int base_adc_channel;
    bool invert;
    float speed;
    float current;
    float temperature;
    unsigned long enc_last_speed;
    unsigned long current_last;
    unsigned long temperature_last;
    unsigned long pid_last;
};

#endif
