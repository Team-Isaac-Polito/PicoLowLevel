#include <Wire.h>
#include <SPI.h>
#include "SmartMotor.h"
#include "AbsoluteEncoder.h"
#include "Battery.h"
#include "DynamixelSerial.h"
#include "definitions.h"
#include "mod_config.h"
#include "Debug.h"
#include "mcp2515.h"
#include "communication.h"
#include "WebManagement.h"
#include "Display.h"

int time_bat = 0;
int time_tel = 0;
int time_data = 0;
int time_tel_avg = DT_TEL;

struct can_frame canMsg;
MCP2515 mcp2515(5, 10000000UL, &SPI); // passing all parameters avoids premature initialization of SPI, which should be done in setup()

SmartMotor motorTrLeft(DRV_TR_LEFT_PWM, DRV_TR_LEFT_DIR, ENC_TR_LEFT_A, ENC_TR_LEFT_B, false);
SmartMotor motorTrRight(DRV_TR_RIGHT_PWM, DRV_TR_RIGHT_DIR, ENC_TR_RIGHT_A, ENC_TR_RIGHT_B, true);


#ifdef MODC_YAW
AbsoluteEncoder encoderYaw(ABSOLUTE_ENCODER_ADDRESS);
#endif


#ifdef MODC_EE
DynamixelMotor motorPitch(SERVO_PITCH_ID);
DynamixelMotor motorPitch2(SERVO_PITCH2_ID);
DynamixelMotor motorRoll(SERVO_ROLL_ID);
#endif

#ifdef MODC_PITCH
DynamixelMotor motorPitchA(SERVO_A_ID);
DynamixelMotor motorPitchB(SERVO_B_ID);
#endif

float oldAngle;

WebManagement wm(CONF_PATH);

Display display;

void okInterrupt() {
  display.okInterrupt();
}

void navInterrupt() {
  display.navInterrupt();
}

void sendTelemetry() {
  canMsg.can_id = 0x14;
  canMsg.can_dlc = 5;
  
  // sending right encoder as float
  float speedR = motorTrRight.getSpeed();
  canMsg.data[0] = SEND_TRACTION_RIGHT_SPEED;
  canMsg.data[1] = ((uint8_t*)&speedR)[3];
  canMsg.data[2] = ((uint8_t*)&speedR)[2];
  canMsg.data[3] = ((uint8_t*)&speedR)[1];
  canMsg.data[4] = ((uint8_t*)&speedR)[0];
  mcp2515.sendMessage(&canMsg);

  // sending left encoder as float
  float speedL = motorTrLeft.getSpeed();
  canMsg.data[0] = SEND_TRACTION_LEFT_SPEED;
  canMsg.data[1] = ((uint8_t*)&speedL)[3];
  canMsg.data[2] = ((uint8_t*)&speedL)[2];
  canMsg.data[3] = ((uint8_t*)&speedL)[1];
  canMsg.data[4] = ((uint8_t*)&speedL)[0];
  mcp2515.sendMessage(&canMsg);

  // if we have an absolute encoder connected send it's value as float
#ifdef MODC_YAW  
  encoderYaw.update();
  float angle = encoderYaw.readAngle();
#if defined(MOD_TAIL)
  canMsg.data[0] = SEND_YAW_ENCODER_TAIL;
#elif defined(MOD_MIDDLE)
  canMsg.data[0] = SEND_YAW_ENCODER_MIDDLE;
#endif
  canMsg.data[1] = ((uint8_t*)&angle)[3];
  canMsg.data[2] = ((uint8_t*)&angle)[2];
  canMsg.data[3] = ((uint8_t*)&angle)[1];
  canMsg.data[4] = ((uint8_t*)&angle)[0];
  mcp2515.sendMessage(&canMsg);
#endif
}

void setup() {
  Serial.begin(115200);
  Debug.setLevel(Levels::INFO); // comment to set debug verbosity to debug
  Wire1.setSDA(I2C_SENS_SDA);
  Wire1.setSCL(I2C_SENS_SCL);
  Wire1.begin();

  SPI.setRX(4);
  SPI.setCS(5);
  SPI.setSCK(6);
  SPI.setTX(7);
  SPI.begin();
  
  LittleFS.begin();

  String hostname = WIFI_HOSTBASE+String(CAN_ID);
  wm.begin(WIFI_SSID, WIFI_PWD, hostname.c_str());

  // CAN initialization
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ);

  mcp2515.setConfigMode(); // tell the MCP2515 next instructions are for configuration
  // enable filtering for 11 bit address on both RX buffers
  mcp2515.setFilterMask(MCP2515::MASK0, false, 0x03FF);
  mcp2515.setFilterMask(MCP2515::MASK1, false, 0x03FF);
  // set all filters to module's ID, so only packets for us get through
  mcp2515.setFilter(MCP2515::RXF0, false, CAN_ID);
  mcp2515.setFilter(MCP2515::RXF1, false, CAN_ID);
  mcp2515.setFilter(MCP2515::RXF2, false, CAN_ID);
  mcp2515.setFilter(MCP2515::RXF3, false, CAN_ID);
  mcp2515.setFilter(MCP2515::RXF4, false, CAN_ID);
  mcp2515.setFilter(MCP2515::RXF5, false, CAN_ID);
  mcp2515.setNormalMode();

  // initializing PWM
  analogWriteFreq(PWM_FREQUENCY); // switching frequency to 15kHz
  analogWriteRange(PWM_MAX_VALUE); // analogWrite range from 0 to 512, default is 255

  // initializing ADC
  analogReadResolution(12); // set precision to 12 bits, 0-4095 input

  // motor initialization
  motorTrLeft.begin();
  motorTrRight.begin();

  motorTrLeft.calibrate();
  motorTrRight.calibrate();

#if defined MODC_PITCH || defined MODC_EE
  Serial1.setRX(1);
  Serial1.setTX(0);
  Dynamixel.setSerial(&Serial1);
  Dynamixel.begin(19200);
#endif

  Debug.println("BEGIN", Levels::INFO);

#ifdef MODC_YAW    
  encoderYaw.update();
  oldAngle = encoderYaw.readAngle();
  encoderYaw.setZero();
#endif

  // Display initialization
  display.begin();

  // Buttons initialization
  pinMode(BTNOK, INPUT_PULLUP);
  pinMode(BTNNAV, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTNOK), okInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTNNAV), navInterrupt, FALLING);

}

void loop() {
  int time_cur = millis();

  // update motors
  motorTrLeft.update();
  motorTrRight.update();

  // health checks
  if (time_cur - time_bat >= DT_BAT) {
    time_bat = time_cur;

    if (time_tel_avg > DT_TEL) Debug.println("Telemetry frequency below required: " + String(1000/time_tel_avg) + " Hz", Levels::WARN);

    if(!battery.charged()) Debug.println("Battery voltage low! " + String(battery.readVoltage()) + "v", Levels::WARN);
  }

  // send telemetry
  if (time_cur - time_tel >= DT_TEL) {
    time_tel_avg = (time_tel_avg + (time_cur - time_tel)) / 2;
    time_tel = time_cur;
    
    sendTelemetry();
  }

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    time_data = time_cur;

    Debug.println("RECEIVED CANBUS DATA");
    int16_t data;
    byte buf[4];

    switch (canMsg.data[0]) {
      case DATA_TRACTION_LEFT:
        data = canMsg.data[1] | canMsg.data[2]<<8;
        motorTrLeft.setSpeed((float)(data)/100.f);

        Debug.print("TRACTION LEFT DATA :\t");
        Debug.println(data);
        break;
      case DATA_TRACTION_RIGHT:
        data = canMsg.data[1] | canMsg.data[2]<<8;
        motorTrRight.setSpeed((float)(data)/100.f);
        
        Debug.print("TRACTION RIGHT DATA :\t");
        Debug.println(data);
        break;
      case DATA_PITCH:
        data = canMsg.data[1] | canMsg.data[2]<<8;
#ifdef MODC_PITCH
        data = map(data, 0, 1023, SERVO_MIN, SERVO_MAX);
        motorPitchA.moveSpeed(data, SERVO_SPEED);
        motorPitchB.moveSpeed(motorPitchB.readPosition() + motorPitchA.readPosition() - data, SERVO_SPEED);
#endif
        Debug.print("PITCH MOTOR DATA : \t");
        Debug.println(data);
        break;

      case DATA_EE_PITCH:
        data = canMsg.data[1] | canMsg.data[2]<<8;
#ifdef MODC_EE
        data = map(data, 0, 1023, SERVO_MIN, SERVO_MAX);
        motorPitch.moveSpeed(data, SERVO_SPEED);
#endif
        Debug.print("PITCH END EFFECTOR MOTOR DATA : \t");
        Debug.println(data);
        break;

      case DATA_EE_PITCH2:
        data = canMsg.data[1] | canMsg.data[2]<<8;
#ifdef MODC_EE
        data = map(data, 0, 1023, SERVO_MIN, SERVO_MAX);
        motorPitch2.moveSpeed(data, SERVO_SPEED);
#endif
        Debug.print("PITCH2 END EFFECTOR MOTOR DATA : \t");
        Debug.println(data);
        break;

      case DATA_EE_ROLL:
        data = canMsg.data[1] | canMsg.data[2]<<8;
#ifdef MODC_EE
        data = map(data, 0, 1023, SERVO_MIN, SERVO_MAX);
        motorRoll.moveSpeed(data, SERVO_SPEED);
#endif
        Debug.print("ROLL END EFFECTOR MOTOR DATA : \t");
        Debug.println(data);
        break;
    }
    
  } else if (time_cur - time_data > 1000 && time_data != -1) { //if we do not receive data for more than a second stop motors
    time_data = -1;
    Debug.println("Stopping motors after timeout.", Levels::INFO);
    motorTrLeft.stop();
    motorTrRight.stop();
  }

  wm.handle();
  display.handleGUI();
}
