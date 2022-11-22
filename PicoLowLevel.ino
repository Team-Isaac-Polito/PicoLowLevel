#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
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
#include "bitmap_logos.h"
#include "WebManagement.h"

int time_enc = 0;
int time_bat = 0;
int time_data = 0;

// Menu handling variables
int ok = 0;
int lastok = 0;
int nav = 0;
int lastnav = 0;
int menupos = 0;
int menutime = 0;

struct can_frame canMsg;
MCP2515 mcp2515(5);

Motor motorTrLeft(DRV_TR_LEFT_PWM,DRV_TR_LEFT_DIR);
Motor motorTrRight(DRV_TR_RIGHT_PWM,DRV_TR_RIGHT_DIR);

TractionEncoder encoderTrLeft(ENC_TR_LEFT_A,ENC_TR_LEFT_B);
TractionEncoder encoderTrRight(ENC_TR_RIGHT_A,ENC_TR_RIGHT_B);

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

Battery battery;

Adafruit_SH1106G display = Adafruit_SH1106G(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire1, -1);

float oldAngle;

WebManagement wm(CONF_PATH);

void showLogo() {
  display.clearDisplay();
  display.drawBitmap(44, 4,  bitmap_logo_isaac, 41, 58, 1);
  display.display();
}

void showWifi() {
  display.clearDisplay();
  display.drawBitmap(0, 0,  bitmap_logo_wifi, 26, 21, 1);
  display.setCursor(0, 32);
  display.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  display.display();
}

void showBattery() {
  display.clearDisplay();
  display.drawBitmap(0, 0,  bitmap_logo_bat, 23, 11, 1);
  display.setCursor(0, 24);
  display.printf("Voltage:  %.2fV\n\n", battery.readVoltage());
  display.printf("Charge:   %.1f%%\n", battery.chargePercent());
  display.display();
}

void showVersion() {
  display.clearDisplay();
  display.drawBitmap(0, 0,  bitmap_logo_upd, 24, 24, 1);
  display.setCursor(0, 32);
  display.printf("Version: %s\n\n", VERSION);
  display.printf("Can ID:  %#04X",CAN_ID);
  display.display();
}

void handleGUI() {
  bool change = false;

  if(nav > 0) {
    change = true;
    nav--;
    menupos++;
    if (menupos >= NMENUS) menupos = 0;
    else menutime = millis();
  } else if(millis() - menutime > (MENUTIMEOUT * 1000)) {
    change = true;
    menupos = 0;
  }

  switch (menupos) {
    case 0:
      if(change) showLogo();
      break;
    case 1:
      if (change) showBattery();
      break;
    case 2:
      if(change) showWifi();
      break;
    case 3:
      if (change) showVersion();
      break;
  }
}

void okInterrupt() {
  int now = millis();
  if (now - lastok > DEBOUNCE) {
    ok++;
    lastok = now;
  }
}

void navInterrupt() {
  int now = millis();
  if (now - lastnav > DEBOUNCE) {
    nav++;
    lastnav = now;
  }
}

int motorCurrent(bool rightSide) {
  int val = analogRead(rightSide?DRV_TR_RIGHT_CURR:DRV_TR_LEFT_CURR);
  val = map(val, 0, 4095, 0, 3300000);
  return (val-50)/40;
}

void sendTelemetry() {
  canMsg.can_id = 0x14;
  canMsg.can_dlc = 5;

  int currL = motorCurrent(false);
  int currR = motorCurrent(true);
  canMsg.data[0] = SEND_CURRENT;
  canMsg.data[1] = currL;
  canMsg.data[2] = currL>>8;
  canMsg.data[3] = currR;
  canMsg.data[4] = currR>>8;
  mcp2515.sendMessage(&canMsg);
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
  
  outPid = pidTrLeft.getOutput();
  if (abs(outPid) < 120) outPid = 0;

  motorTrLeft.write(outPid);
  Debug.print("LEFT MOTOR OUTPUT \t ");
  Debug.println(outPid);

  Debug.println("--------------------------------------------");

  // RIGHT TRACTION PID
  speed = encoderTrRight.getSpeed();
  Debug.print("RIGHT ENCODER \t ");
  Debug.println(speed);
  Debug.print("RIGHT REF VAL ");
  Debug.println(pidTrRight.getReferenceValue());

  pidTrRight.updateFeedback(speed);
  pidTrRight.calculate();
  
  outPid = pidTrRight.getOutput();
  if (abs(outPid) < 120) outPid = 0;

  motorTrRight.write(outPid);
  Debug.print("RIGHT MOTOR OUTPUT \t ");
  Debug.println(outPid);

  // YAW PID
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
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  // initializing PWM
  //analogWriteFreq(PWM_FREQUENCY); // switching frequency to 50kHz
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
#endif

  // Display initialization
  display.begin(DISPLAY_ADDR, true);
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.display();

  // Buttons initialization
  pinMode(BTNOK, INPUT_PULLUP);
  pinMode(BTNNAV, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTNOK), okInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTNNAV), navInterrupt, FALLING);

  // Show ISAAC's logo
  showLogo();
}

void loop() {
  int time_cur = millis();

  // pid routine, to be executed every DT milliseconds
  if (time_cur - time_enc > DT) { 
    time_enc = time_cur;
    updatePID();
  }

  // read battery voltage every second
  if (time_cur - time_bat > 1000) {
    time_bat = millis();

    Debug.print("Battery voltage is: ");
    Debug.println(battery.readVoltage());

    sendTelemetry();
  }

  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK && (canMsg.can_id == CAN_ID)) {
    time_data = time_cur;

    Debug.println("RECEIVED CANBUS DATA");
    int16_t data;

    switch (canMsg.data[0]) {
      case DATA_TRACTION_LEFT:
        data = canMsg.data[1] | canMsg.data[2]<<8;
        //motorTrLeft.write(data);
        pidTrLeft.updateReferenceValue(data);

        Debug.print("TRACTION LEFT DATA :\t");
        Debug.println(data);
        break;
      case DATA_TRACTION_RIGHT:
        data = canMsg.data[1] | canMsg.data[2]<<8;
        data = -data; // one side needs to rotate on the opposite direction
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
    }
    
  } else if (time_cur - time_data > 1000 && time_data != -1) { //if we do not receive data for more than a second stop motors
    time_data = -1;
    Debug.println("Stopping motors after timeout.", Levels::INFO);
    pidTrLeft.updateReferenceValue(0);
    pidTrRight.updateReferenceValue(0);
  }

  wm.handle();
  // handleGUI(); // leads to lag in motor driving, needs improvements
}
