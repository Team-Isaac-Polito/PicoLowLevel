#include "config.h"
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
  DEBUG("PID VALUE");
  DEBUG(value);
  DEBUG("MOTOR VALUE");
  DEBUG(constrain(abs(value), 0, MAX_OUTPUT));
  analogWrite(pwm, constrain(abs(value), 0, MAX_OUTPUT));
  digitalWrite(dir, value > 0);
}
