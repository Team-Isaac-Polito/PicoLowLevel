#include "PID.h"

PID::PID(float kp, float ki, float kd, float max_output, float alpha) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->max_output = max_output;
  this->alpha = alpha;

  old_integral = 0;
  old_fe = 0;
  tempo = millis();
}

void PID::updateReferenceValue(float ref) {
  referenceValue = ref;
}

void PID::updateFeedback(float fb) {
  feedback = fb;
}

void PID::setConstants(float kp, float ki, float kd) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
}

float PID::getOutput() {
  return output;
}

float PID::getReferenceValue() {
  return referenceValue;
}

void PID::resetState() {
  old_fe = 0.f;
  old_integral = 0.f;
}

void PID::calculate() {
  unsigned long dt;
  float error, fe, derivative, integral;
  
  dt = millis() - tempo; // sarà fatto a frequenza fissata, andrà eliminato e magari aggiunto come parametro al costruttore

  // e[k] = r[k] - y[k], error between setpoint and true position
  error = referenceValue - feedback;

  // e_f[k] = α e[k] + (1-α) e_f[k-1], filtered error
  fe = alpha * error + (1 - alpha) * old_fe;
  
  // e_d[k] = (e_f[k] - e_f[k-1]) / Tₛ, filtered derivative
  derivative = (fe - old_fe) / dt;
  
  // e_i[k+1] = e_i[k] + Tₛ e[k], integral
  integral = old_integral + error * dt;

  // PID formula:
  // u[k] = Kp e[k] + Ki e_i[k] + Kd e_d[k], control signal
  output = referenceValue + kp * error + ki * old_integral + kd * derivative;

  // Clamp the output
  if (output > max_output)
    output = max_output;
  else if (output < -max_output)
    output = -max_output;
  else // Anti-windup
    old_integral = integral;

  // store the state for the next iteration
  old_fe = fe;
  tempo = millis();
}
