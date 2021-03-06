#include <Wire.h>
#include <SPI.h>
#include "Motor.h"
#include "AbsoluteEncoder.h"
#include "Battery.h"
#include "TractionEncoder.h"
#include "DynamixelSerial.h"
#include "PID.h"
#include "definitions.h"
#include "mod_config.h"
#include "Debug.h"
#include "mcp2515.h"
#include "communication.h"

int time_enc = 0;
int time_bat = 0;
int time_data = 0;

struct can_frame canMsg;
MCP2515 mcp2515(5);

Motor motorTrLeft(DRV_TR_LEFT_PWM,DRV_TR_LEFT_DIR);
Motor motorTrRight(DRV_TR_RIGHT_PWM,DRV_TR_RIGHT_DIR);

TractionEncoder encoderTrLeft(ENC_TR_LEFT_A,ENC_TR_LEFT_B);
TractionEncoder encoderTrRight(ENC_TR_RIGHT_A,ENC_TR_RIGHT_B);

#ifdef MODC_YAW
Motor motorYaw(DRV_YAW_PWM,DRV_YAW_DIR);
AbsoluteEncoder encoderYaw(ABSOLUTE_ENCODER_ADDRESS);
PID pidYaw(PID_YAW_KP,PID_YAW_KI,PID_YAW_KD ,PID_YAW_MAX_OUTPUT,PID_YAW_EMA_ALPHA);
#endif

Battery battery;

void setup() {
  Serial.begin(115200);

  // CAN initialization
  mcp2515.begin();
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  // initializing PWM
  analogWriteFreq(PWM_FREQUENCY); // switching frequency to 50kHz
  analogWriteRange(PWM_MAX_VALUE); // analogWrite range from 0 to 512, default is 255

  // motor initialization
  motorTrLeft.begin();
  motorTrRight.begin();

  // encoder initialization
  encoderTrLeft.begin();
  encoderTrRight.begin();

#ifdef MODC_YAW
  Wire1.setSDA(I2C_SENS_SDA);
  Wire1.setSCL(I2C_SENS_SCL);
  Wire1.begin();
  motorYaw.begin();
  encoderYaw.begin();

  pidYaw.updateReferenceValue(0);
#endif

  // Dynamixel ServoMotor Pitch
  Serial1.setRX(1);
  Serial1.setTX(0);
  Dynamixel.setSerial(&Serial1);
  Dynamixel.begin(19200);


  Debug.println("BEGIN", Levels::INFO);
}

void loop() {
  int time_cur = millis();

  // pid routine, to be executed every DT milliseconds
  if (time_cur - time_enc > DT) { 
    time_enc = time_cur;



    // debug output
    Debug.println("ENCODER");
    Debug.print("LEFT \t- ");
    Debug.println(encoderTrLeft.getSpeed());
    Debug.print("RIGHT \t- ");
    Debug.println(encoderTrRight.getSpeed());
#ifdef MODC_YAW    
    // yaw setting
    encoderYaw.update();
    pidYaw.updateFeedback(encoderYaw.readAngle());
    pidYaw.calculate();
    
    int outPid = pidYaw.getOutput();

    if (abs(outPid) < 60) outPid = 0;
    motorYaw.write(outPid);

    Debug.print("READ ANGLE \t- ");
    Debug.println(encoderYaw.readAngle());
    Debug.print("YAW MOTOR \t- ");
    Debug.println(pidYaw.getOutput());
#endif
  }

  // read battery voltage every second
  if (time_cur - time_bat > 1000) {
    time_bat = millis();

    Debug.print("Battery voltage is: ");
    Debug.println(battery.readVoltage());
  }

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK && (canMsg.can_id == CAN_ID)) {
    Debug.println("RECEIVED CANBUS DATA");

    int16_t data;

    time_data = time_cur;

    switch (canMsg.data[0]) {
      case DATA_TRACTION_LEFT:
        data = canMsg.data[1] | canMsg.data[2]<<8;
        motorTrLeft.write(data);

        Debug.print("TRACTION LEFT DATA :\t");
        Debug.println(data);
        break;
      case DATA_TRACTION_RIGHT:
        data = canMsg.data[1] | canMsg.data[2]<<8;
        data = -data; // one side needs to rotate on the opposite direction
        motorTrRight.write(data);

        Debug.print("TRACTION RIGHT DATA :\t");
        Debug.println(data);
        break;
      case DATA_YAW:  
        data = canMsg.data[1] | canMsg.data[2]<<8;
#ifdef MODC_YAW  
        pidYaw.updateReferenceValue(data);
#endif
        Debug.print("YAW DATA :\t");
        Debug.println(data);
        break;
      case DATA_PITCH:
        Debug.print("TODO");
        break;
      case SEND_STATUS:
        Debug.print("TODO");
        break;        
      case SEND_IMU_DATA:
        Debug.print("TODO");
        break;
      case SEND_YAW_ENCODER:
        Debug.print("TODO");
        break;
      case SEND_TRACTION_LEFT_SPEED:
        Debug.print("TODO");
        break;
      case SEND_TRACTION_RIGHT_SPEED:
        Debug.print("TODO");
        break;
      case SEND_BATTERY_VOLTAGE:
        Debug.print("TODO");
        break;
    }
    
  } else if (time_cur - time_data > 1000 && time_data != -1) { //if we do not receive data for more than a second stop motors
    time_data = -1;
    motorTrLeft.write(0);
    motorTrRight.write(0);
  }
 
}
