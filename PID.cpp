#include "PID.h"

PID::PID(float kp, float ki, float kd) {
	 
    KP = kp;
  KI = ki;
  KP = kd;

  //ToDo inizializzare altre variabili, sumErr, [old]feeback
    
}

void PID::updateReferenceValue(float ref) {
	referenceValue = ref;

	// azzerare sumErr?
}

void PID::updateFeedback(float fb) {
	oldFeedback = feedback;
	feedback = fb;
}

float PID::calculate() {
	float output;
	//velocità di output Encoder Relativo
	output = (referenceValue - feedback) * KP;   // contributo proporzionale
	output += (sumErr * KI * DT / 1000);         // contributo integrativo
	output +=  -(1000 * KD * (feedback - oldFeedback) / DT ); //contributo derivativo

	sumErr += referenceValue - feedback;
}
