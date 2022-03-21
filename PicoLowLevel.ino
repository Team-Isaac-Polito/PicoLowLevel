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

long tempo, 
  countStepsLeft = 0, 
  countStepsRight = 0;

int refTrLeft = START_TR_LEFT,
    refTrRight = START_TR_RIGHT,
    refYaw = START_YAW;

void setup() {
  // Serial Debug
#if SERIAL_DEBUG_ATTIVATO 
  Serial.begin(115200);
#endif
  DEBUG("START SETUP");

  //I2C setup
  Wire.begin(I2C_ADDRESS);     // join I2C bus with respective address
  Wire.onReceive(receive); // receive data function

  // MOTORS SETUP
  motorTrLeft.begin();
  motorTrRight.begin();
  motorYaw.begin();

  // ENCODER ASSOLUTO
  //init AMS_AS5048B object
  // ToDo check if necessary to change to wire1 inside library - done, hope it works
  Wire1.setSDA(2);
  Wire1.setSCL(3);
  absEncoder.begin();

  //set clock wise counting
  // AG needed? 
  absEncoder.setClockWise(true); 

  //set the 0 to the sensor
  absEncoder.zeroRegW(0x0); // ToDo, non dobbiamo partire da 0

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

  tempo = millis();


  DEBUG("END SETUP");
}

void loop() {
  if(millis() - tempo < DT) return; // eseguo solo quando è passato DT tempo
  tempo = millis();

  DEBUG("EXECUTING LOOP");
  DEBUG(tempo);

  DEBUG(" - SETPOINT - ");
  DEBUG("TR LEFT");
  pidTrLeft.updateReferenceValue(refTrLeft * 4);
  DEBUG("TR RIGHT");
  pidTrRight.updateReferenceValue(refTrRight * 4);
  DEBUG("YAW");
  pidYaw.updateReferenceValue(refYaw);
  DEBUG("END SETPOINT");

  DEBUG("READING ABSOLUTE ENCODER ANGLE");
  // read absolute encoder 
  absEncoder.updateMovingAvgExp();
  pidYaw.updateFeedback(absEncoder.angleR(U_DEG, false));
  DEBUG("END READING ABSOLUTE ENCODER ANGLE");

  DEBUG("READING TRACTION ENCODERS DATA");
  DEBUG("TRACTION LEFT");
  pidTrLeft.updateFeedback(getLeftEncoderData());
  DEBUG("TRACTION RIGHT");
  pidTrRight.updateFeedback(getRightEncoderData());
  DEBUG("END READING TRACTION ENCODERS DATA");


  DEBUG("CALCULATING PIDS VALUES AND WRITING MOTORS");
  // PID calculations and motor control
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on to manually check calculation and writing data time use

  DEBUG("TRACTION LEFT");
  motorTrLeft.write(pidTrLeft.calculate());
  DEBUG("TRACTION RIGHT");
  motorTrRight.write(pidTrRight.calculate());
  DEBUG("YAW");
  motorYaw.write(pidYaw.calculate());

  digitalWrite(LED_BUILTIN, LOW); // turn off the led
  DEBUG("END CALCULATING PIDS VALUES AND WRITING MOTORS");
}
