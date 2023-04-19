#include <Wire.h>
#include <SPI.h>
#include "Motor.h"
#include "AbsoluteEncoder.h"
#include "MovingAvgFilter.h"
#include "Battery.h"
#include "TractionEncoder.h"
#include "DynamixelSerial.h"
#include "PID.h"
#include "definitions.h"
#include "mod_config.h"
#include "Debug.h"
#include "mcp2515.h"
#include "communication.h"
#include "WebManagement.h"
#include "Display.h"

int time_pid = 0;
int time_bat = 0;
int time_tel = 0;
int time_data = 0;
int time_enc = 0;
int time_pid_avg = DT_PID;
int time_tel_avg = DT_TEL;

struct can_frame canMsg;
MCP2515 mcp2515(5);

Motor motorTrLeft(DRV_TR_LEFT_PWM,DRV_TR_LEFT_DIR);
Motor motorTrRight(DRV_TR_RIGHT_PWM,DRV_TR_RIGHT_DIR);

TractionEncoder encoderTrLeft(ENC_TR_LEFT_A,ENC_TR_LEFT_B, new MovingAvgFilter<int>(ENC_TR_SAMPLES));
TractionEncoder encoderTrRight(ENC_TR_RIGHT_A,ENC_TR_RIGHT_B, new MovingAvgFilter<int>(ENC_TR_SAMPLES));

PID pidTrLeft(PID_TR_KP,PID_TR_KI,PID_TR_KD ,PID_TR_MAX_OUTPUT,PID_TR_EMA_ALPHA);
PID pidTrRight(PID_TR_KP,PID_TR_KI,PID_TR_KD ,PID_TR_MAX_OUTPUT,PID_TR_EMA_ALPHA);


#ifdef MODC_YAW
Motor motorYaw(DRV_YAW_PWM,DRV_YAW_DIR);
AbsoluteEncoder encoderYaw(ABSOLUTE_ENCODER_ADDRESS);
PID pidYaw(PID_YAW_KP,PID_YAW_KI,PID_YAW_KD ,PID_YAW_MAX_OUTPUT,PID_YAW_EMA_ALPHA);
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

int motorCurrent(bool rightSide) {
  int val = analogRead(rightSide?DRV_TR_RIGHT_CURR:DRV_TR_LEFT_CURR);
  val = map(val, 0, 4095, 0, 3300000);
  return (val-50)/40;
}

void sendTelemetry() {
  canMsg.can_id = 0x14;
  canMsg.can_dlc = 5;
  
  // sending right encoder as float
  float speedR = - encoderTrRight.getSpeed(); // invert
  canMsg.data[0] = SEND_TRACTION_RIGHT_SPEED;
  canMsg.data[1] = ((uint8_t*)&speedR)[3];
  canMsg.data[2] = ((uint8_t*)&speedR)[2];
  canMsg.data[3] = ((uint8_t*)&speedR)[1];
  canMsg.data[4] = ((uint8_t*)&speedR)[0];
  mcp2515.sendMessage(&canMsg);

  // sending left encoder as float
  float speedL = encoderTrLeft.getSpeed();
  canMsg.data[0] = SEND_TRACTION_LEFT_SPEED;
  canMsg.data[1] = ((uint8_t*)&speedL)[3];
  canMsg.data[2] = ((uint8_t*)&speedL)[2];
  canMsg.data[3] = ((uint8_t*)&speedL)[1];
  canMsg.data[4] = ((uint8_t*)&speedL)[0];
  mcp2515.sendMessage(&canMsg);

  // sending motor currents as two unsigned integers of 16bit
  int currL = motorCurrent(false);
  int currR = motorCurrent(true);
  canMsg.data[0] = SEND_CURRENT;
  canMsg.data[1] = currL;
  canMsg.data[2] = currL>>8;
  canMsg.data[3] = currR;
  canMsg.data[4] = currR>>8;
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

void updatePID() {
  float speed, outPid;
  // LEFT TRACTION PID 
  Debug.println("\n\n\n\n");
  speed = encoderTrLeft.getSpeed();
  Debug.print("LEFT ENCODER \t ");
  Debug.println(speed);
  Debug.print("LEFT REF VAL ");
  Debug.println(pidTrLeft.getReferenceValue());

  pidTrLeft.updateFeedback(speed);
  pidTrLeft.calculate();
  
  outPid = pidTrLeft.getOutput() * RPM_TO_PWM;
  //if (abs(outPid) < 120) outPid = 0;

  motorTrLeft.write(outPid);
  Debug.print("LEFT MOTOR OUTPUT \t ");
  Debug.println(outPid);

  Debug.println("--------------------------------------------");

  // RIGHT TRACTION PID
  speed = - encoderTrRight.getSpeed(); // invert
  Debug.print("RIGHT ENCODER \t ");
  Debug.println(speed);
  Debug.print("RIGHT REF VAL ");
  Debug.println(pidTrRight.getReferenceValue());

  pidTrRight.updateFeedback(speed);
  pidTrRight.calculate();
  
  outPid = - pidTrRight.getOutput() * RPM_TO_PWM; // invert
  //if (abs(outPid) < 120) outPid = 0;

  motorTrRight.write(outPid);
  Debug.print("RIGHT MOTOR OUTPUT \t ");
  Debug.println(outPid);

  // YAW PID
  /*
#ifdef MODC_YAW    
  // yaw setting
  encoderYaw.update();
  float angle = encoderYaw.readAngle();
  if(angle != ANGLE_READ_ERROR && abs(angle - oldAngle) < 30) {
    pidYaw.updateFeedback(angle);
    oldAngle = angle;
  }
  pidYaw.calculate();
  
  outPid = -pidYaw.getOutput();

  if (abs(outPid) < 60) outPid = 0;
  motorYaw.write(outPid);
  Debug.print("YAW REF VALUE ");
  Debug.println(pidYaw.getReferenceValue());
  Debug.print("READ ANGLE ");
  Debug.println(encoderYaw.readAngle());
  Debug.print("YAW MOTOR ");
  Debug.println(outPid);
#endif
*/
}

void setup() {
  Serial.begin(115200);
  Debug.setLevel(Levels::INFO); // comment to set debug verbosity to debug
  Wire1.setSDA(I2C_SENS_SDA);
  Wire1.setSCL(I2C_SENS_SCL);
  Wire1.begin();
  
  LittleFS.begin();

  String hostname = WIFI_HOSTBASE+String(CAN_ID);
  wm.begin(WIFI_SSID, WIFI_PWD, hostname.c_str());

  // CAN initialization
  mcp2515.begin();
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

  // encoder initialization
  encoderTrLeft.begin();
  encoderTrRight.begin();

  // PID initialization
  pidTrLeft.updateReferenceValue(0);
  pidTrRight.updateReferenceValue(0);

#ifdef MODC_YAW
  motorYaw.begin();
  encoderYaw.begin();

  pidYaw.updateReferenceValue(0);
#endif

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

  // PID routine
  if (time_cur - time_pid >= DT_PID) { 
    time_pid_avg = (time_pid_avg + (time_cur - time_pid)) / 2;
    time_pid = time_cur;
    updatePID();
  }

  if (time_cur - time_enc >= DT_ENC) {
    time_enc = time_cur;
    encoderTrRight.update();
    encoderTrLeft.update();
  }

  // health checks
  if (time_cur - time_bat >= DT_BAT) {
    time_bat = time_cur;

    if (time_tel_avg > DT_TEL) Debug.println("Telemetry frequency below required: " + String(1000/time_tel_avg) + " Hz", Levels::WARN);
    if (time_tel_avg > DT_PID) Debug.println("Average PID frequency below required: " + String(1000/time_pid_avg) + " Hz", Levels::WARN);

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
        // motorTrLeft.write(data);
        pidTrLeft.updateReferenceValue(data);

        Debug.print("TRACTION LEFT DATA :\t");
        Debug.println(data);
        break;
      case DATA_TRACTION_RIGHT:
        data = canMsg.data[1] | canMsg.data[2]<<8;
        // data = -data; // one side needs to rotate on the opposite direction
        // motorTrRight.write(data);
        pidTrRight.updateReferenceValue(data);
        
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

      case DATA_PID_KP:
        for (int i = 0; i<4; i++) buf[i] = canMsg.data[i+1];
        float kp;
        memcpy(&kp, &buf, sizeof(kp));
        pidTrLeft.setKp(kp);
        pidTrRight.setKp(kp);
        break;
      case DATA_PID_KD:
        for (int i = 0; i<4; i++) buf[i] = canMsg.data[i+1];
        float kd;
        memcpy(&kd, &buf, sizeof(kd));
        pidTrLeft.setKd(kp);
        pidTrRight.setKd(kp);
        break;
      case DATA_PID_KI:
        for (int i = 0; i<4; i++) buf[i] = canMsg.data[i+1];
        float ki;
        memcpy(&ki, &buf, sizeof(ki));
        pidTrLeft.setKi(ki);
        pidTrRight.setKi(ki);
        break;
    }
    
  } else if (time_cur - time_data > 1000 && time_data != -1) { //if we do not receive data for more than a second stop motors
    time_data = -1;
    Debug.println("Stopping motors after timeout.", Levels::INFO);
    pidTrLeft.updateReferenceValue(0);
    pidTrRight.updateReferenceValue(0);
  }

  wm.handle();
  display.handleGUI();
}
