//ToDo add description of what the code does
#include <Wire.h>
#include "config.h"
#include "PID.h"
#include "Motor.h"
#include "ams_as5048b.h"
#include "functions.h"

// global variables
PID pidTrLeft(TR_LEFT_KP, TR_LEFT_KI, TR_LEFT_KD),
	pidTrRight(TR_RIGHT_KP, TR_RIGHT_KI, TR_RIGHT_KD),
	pidYaw(YAW_KP, YAW_KI, YAW_KD);

Motor motorTrLeft(DRV_TR_LEFT_PWM,DRV_TR_LEFT_DIR),
	motorTrRight(DRV_TR_RIGHT_PWM,DRV_TR_RIGHT_DIR),
	motorYaw(DRV_YAW_PWM,DRV_YAW_DIR);

AMS_AS5048B absEncoder;

unsigned long tempo,countStepsLeft = 0,countStepsRight = 0;


void setup() {
	//I2C setup
	Wire.begin(I2C_ADDRESS);     // join I2C bus with respective address
	Wire.onReceive(receive); // receive data function

	// PID setup reference value
	pidTrLeft.updateReferenceValue(START_TR_LEFT);
	pidTrLeft.updateReferenceValue(START_TR_RIGHT);
	pidYaw.updateReferenceValue(START_YAW);

	// MOTORS SETUP
	motorTrLeft.begin();
	motorTrRight.begin();
	motorYaw.begin();

	// ENCODER ASSOLUTO
	//init AMS_AS5048B object
  // ToDo check if necessary to change to wire1 inside library
	absEncoder.begin();

	//set clock wise counting
	// AG needed? 
	absEncoder.setClockWise(true); 

	//set the 0 to the sensor
	//absEncoder.zeroRegW(0x0);

	// ENCODER TRAZIONE
	pinMode(ENC_TR_LEFT_A,INPUT);
	pinMode(ENC_TR_LEFT_B,INPUT);
	
  attachInterrupt(digitalPinToInterrupt(ENC_TR_LEFT_A), handleEncoderLeft, RISING);

	pinMode(ENC_TR_RIGHT_A,INPUT);
	pinMode(ENC_TR_RIGHT_B,INPUT);

	attachInterrupt(digitalPinToInterrupt(ENC_TR_RIGHT_A), handleEncoderRight, RISING);

	// can be useful
  pinMode(LED_BUILTIN, OUTPUT);

	// ToDo:
	//  setup other output/input pins

}

void loop() {
	if(millis() - tempo < DT) return;	// eseguo solo quando è passato DT tempo - ToDo check if working on pico
	tempo = millis();

	// TODO
	// read serial data   -> pid.updatereferencevalue

	// read absolute encoder 
  absEncoder.updateMovingAvgExp();
	pidYaw.updateFeedback(absEncoder.angleR(U_DEG, false));

	pidTrLeft.updateFeedback(getLeftEncoderData());
	pidTrRight.updateFeedback(getRightEncoderData());

	// PID calculations and motor control
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on to manually check calculation and writing data time use

	motorTrLeft.write(pidTrLeft.calculate());
	motorTrRight.write(pidTrRight.calculate());
	motorYaw.write(pidYaw.calculate());

	digitalWrite(LED_BUILTIN, LOW); // turn off the led

}
