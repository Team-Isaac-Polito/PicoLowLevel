#include "PID.h"

PID::PID(float kp, float ki, float kd, float max_output, float alpha) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->max_output = max_output;
  this->alpha = alpha;

  integral = 0;
  old_ef = 0;
  tempo = millis();
}

void PID::updateReferenceValue(float ref) {
  Debug.print("PID - Updated reference value  ", Levels::DEBUG);
  Debug.println(ref, Levels::DEBUG);

  referenceValue = ref;
}

void PID::updateFeedback(float fb) {  
  Debug.print("PID - Updated feedback  ", Levels::DEBUG);
  Debug.println(fb, Levels::DEBUG);
  
  feedback = fb;
}

float PID::getOutput() {
  return output;
}

float PID::getReferenceValue() {
  return referenceValue;
}

void PID::calculate() {
  int dt;
  float error,ef,derivative,new_integral;
  
  dt = millis() - tempo; // sarà fatto a frequenza fissata, andrà eliminato e magari aggiunto come parametro al costruttore

  // e[k] = r[k] - y[k], error between setpoint and true position
  error = referenceValue - feedback;

  // e_f[k] = α e[k] + (1-α) e_f[k-1], filtered error
  ef = alpha * error + (1 - alpha) * old_ef;
  
  // e_d[k] = (e_f[k] - e_f[k-1]) / Tₛ, filtered derivative
  derivative = (ef - old_ef) / dt;
  
  // e_i[k+1] = e_i[k] + Tₛ e[k], integral
  new_integral = integral + error * dt;

  // PID formula:
  // u[k] = Kp e[k] + Ki e_i[k] + Kd e_d[k], control signal
  output = referenceValue + kp * error + ki * integral + kd * derivative;

  // Clamp the output
  if (output > max_output)
      output = max_output;
  else if (output < -max_output)
      output = -max_output;
  else // Anti-windup
      integral = new_integral;

  Debug.print("PID - Calculated output  ", Levels::DEBUG);
  Debug.println(output, Levels::DEBUG);

  
  // store the state for the next iteration
  old_ef = ef;
  tempo = millis();
}
