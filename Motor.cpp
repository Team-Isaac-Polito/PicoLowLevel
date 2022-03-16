#include "Motor.h"
#include "functions.h"

Motor::Motor(byte pwm, byte dir) {
  this->pwm = pwm;
  this->dir = dir;
}

void Motor::begin() {
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
}

void Motor::write(float value) {
  DEBUG(value);
  analogWrite(pwm, constrain(abs(value), 0, 255));
  digitalWrite(dir, value > 0);
}
