#include "Motor.h"

/**
 * Create object and set motor pins.
 * @param pwm PWM pin.
 * @param dir Direction pin.
 */
Motor::Motor(byte pwm, byte dir) {
  this->pwm = pwm;
  this->dir = dir;
}

/**
 * Initialize motors.
 */
void Motor::begin() {
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
}

/**
 * Sets the motor speed, value from 0 (OFF) to maximum PWM value.
 */
void Motor::write(float value) {
  int vli = (int) value;
  int mot = constrain(abs(vli), 0, MAX_OUTPUT);
  Serial.println(vli);
  analogWrite(pwm, mot);
  digitalWrite(dir, vli < 0);
}
