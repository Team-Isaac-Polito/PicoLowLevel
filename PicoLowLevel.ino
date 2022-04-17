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

union serialData_t {
  byte valueBuffer[12];
  float value[3];
} serialData;

enum {traction_left, traction_right, yaw};

int motorTestDir = 1,
  motorTestValue = 0;



void receive(int byteCount){
    DEBUG("RECEIVED DATA");

    for(uint8_t index = 0; index<byteCount; index++){
        serialData.valueBuffer[index] = Wire.read();
    }
    
    return;
}

void setup() {
  // Serial Debug
#if SERIAL_DEBUG_ATTIVATO 
  Serial.begin(115200);
#endif
  DEBUG("START SETUP");

  //I2C setup
  Wire.setSDA(I2C_PIN_SDA);
  Wire.setSCL(I2C_PIN_SDA);
  Wire.begin(I2C_ADDRESS);     // join I2C bus with respective address
  Wire.onReceive(receive); // receive data function

  // MOTORS SETUP
  motorTrLeft.begin();
  motorTrRight.begin();
  motorYaw.begin();

  serialData.value[traction_left] = START_TR_LEFT;
  serialData.value[traction_right] = START_TR_RIGHT;
  serialData.value[yaw] = START_YAW;
  
  // ENCODER ASSOLUTO
  //init AMS_AS5048B object
  Wire1.setSDA(I2C_ENC_PIN_SDA);
  Wire1.setSCL(I2C_ENC_PIN_SCL);
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

  DEBUG("WRITING DATA TO MOTORS");  
  DEBUG("TRACTION LEFT");
  motorTrLeft.write(motorTestValue);
  DEBUG("TRACTION RIGHT");
  motorTrRight.write(motorTestValue);
  DEBUG("YAW");
  motorYaw.write(motorTestValue);

  motorTestValue += motorTestDir;

  if(motorTestValue == 1000)
    motorTestDir = -1;
  else if(motorTestValue == -1000)
    motorTestDir = 1;

}
