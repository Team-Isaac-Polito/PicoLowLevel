//ToDo add description of what the code does
#include <Wire.h>
#include "config.h"
#include "PID.h"
#include "Motor.h"


// global variables
PID pidTrLeft(TR_LEFT_KP, TR_LEFT_KI, TR_LEFT_KD),
  pidTrRight(TR_RIGHT_KP, TR_RIGHT_KI, TR_RIGHT_KD),
  pidYaw(YAW_KP, YAW_KI, YAW_KD);

Motor motorTrLeft(DRV_TR_LEFT_PWM,DRV_TR_LEFT_DIR),
  motorTrRight(DRV_TR_RIGHT_PWM,DRV_TR_RIGHT_DIR),
  motorYaw(DRV_YAW_PWM,DRV_YAW_DIR);


// ToDo
void receive(int numBytes){} // function that runs when data is received


void setup() {
  //I2C setup
  Wire.begin(I2C_ADDRESS);     // join I2C bus with respective address
  Wire.onReceive(receive); // receive data function
  
  // PID setup reference value
  pidTrLeft.updateReferenceValue(START_TR_LEFT);
  pidTrLeft.updateReferenceValue(START_TR_RIGHT);
  pidYaw.updateReferenceValue(START_YAW);

  // Motors setup
  motorTrLeft.begin();
  motorTrRight.begin();
  motorYaw.begin();

  // ToDo:
  //  setup other output/input pins

}

void loop() {
  // ToDo
  // read encoders data -> pid.update feedback
  // read serial data   -> pid.updatereferencevalue
  // 

  // PID calculations and motor control
  motorTrLeft.write(pidTrLeft.calculate());
  motorTrRight.write(pidTrRight.calculate());
  motorYaw.write(pidYaw.calculate());

}
