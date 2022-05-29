#include "Motor.h"

Motor::Motor(byte pwm, byte dir) {
  this->pwm = pwm;
  this->dir = dir;
}

void Motor::begin() {
  pinMode(pwm, OUTPUT);
  pinMode(dir, OUTPUT);
}

void Motor::write(float value) {
  int vli = (int) value;
  int mot = constrain(abs(vli), 0, MAX_OUTPUT);
  Serial.println(vli);
  analogWrite(pwm, mot);
  digitalWrite(dir, vli < 0);
}
