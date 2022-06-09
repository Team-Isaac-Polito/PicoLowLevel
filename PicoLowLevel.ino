#include <Wire.h>
#include "Motor.h"
#include "AbsoluteEncoder.h"
#include "Battery.h"
#include "TractionEncoder.h"
#include "PID.h"
#include "definitions.h"
#include "Debug.h"

bool updm = false;

int time_enc = 0;
int time_bat = 0;

// used to convert from byte to float, both arrays share the same memory
union serialData_t {
  byte valueBuffer[8];
  float value[2];
} serialData;

// field identifier for serialData values
enum {traction_left, traction_right};

Motor motorTrLeft(DRV_TR_LEFT_PWM,DRV_TR_LEFT_DIR);
Motor motorTrRight(DRV_TR_RIGHT_PWM,DRV_TR_RIGHT_DIR);
Motor motorYaw(DRV_YAW_PWM,DRV_YAW_DIR);

TractionEncoder encoderTrLeft(ENC_TR_LEFT_A,ENC_TR_LEFT_B);
TractionEncoder encoderTrRight(ENC_TR_RIGHT_A,ENC_TR_RIGHT_B);
AbsoluteEncoder encoderYaw;

PID pidYaw(.5,0.0001,0.35 ,1023,0.5);

Battery battery;

// event on incoming I²C data
void receive(int byteCount) {
  updm = true;
  Debug.println("RECEIVED DATA");
  for (uint8_t index = 0; index < byteCount && index < 8; index++) {
    serialData.valueBuffer[index] = Wire.read();
  }
}


void setup() {
  Serial.begin(115200);

  // I²C initialization
  Wire.setSDA(I2C_PIN_SDA);
  Wire.setSCL(I2C_PIN_SCL);
  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receive);
  Wire1.setSDA(I2C_SENS_SDA);
  Wire1.setSCL(I2C_SENS_SCL);
  Wire1.begin();

  // initializing PWM
  analogWriteFreq(PWM_FREQUENCY); // switching frequency to 50kHz
  analogWriteRange(PWM_MAX_VALUE); // analogWrite range from 0 to 512, default is 255

  // motor initialization
  motorTrLeft.begin();
  motorTrRight.begin();
  motorYaw.begin();

  // encoder initialization
  encoderTrLeft.begin();
  encoderTrRight.begin();
  encoderYaw.begin();

  // set yaw to zero
  encoderYaw.setZero();
  pidYaw.updateReferenceValue(180);

  Debug.println("BEGIN", Levels::INFO);
}

void loop() {
  int time_cur = millis();

  // pid routine, to be executed every DT milliseconds
  if (time_cur - time_enc > DT) { 
    time_enc = millis();

    // yaw setting
    encoderYaw.update();
    pidYaw.updateFeedback(encoderYaw.readAngle());
    pidYaw.calculate();
    motorYaw.write(pidYaw.getOutput());

    // debug output
    Debug.println("ENCODER");
    Debug.print("LEFT \t- ");
    Debug.println(encoderTrLeft.getSpeed());
    Debug.print("RIGHT \t- ");
    Debug.println(encoderTrRight.getSpeed());
    Debug.print("AGLE \t- ");
    Debug.println(encoderYaw.readAngle());
    Debug.print("EENCODER OUTPUT \t- ");
    Debug.println(pidYaw.getOutput());
  }

  // read battery voltage every second
  if (time_cur - time_bat > 1000) {
    time_bat = millis();

    Debug.print("Battery voltage is: ");
    Debug.println(battery.readVoltage());
  }

  // only set motor speed if we received new data
  if (updm) {
    Debug.print("TRACTION LEFT:\t");
    motorTrLeft.write(serialData.value[traction_left]);
    Debug.print("TRACTION RIGHT:\t");
    motorTrRight.write(-serialData.value[traction_right]);
    updm = false;
  }

}
