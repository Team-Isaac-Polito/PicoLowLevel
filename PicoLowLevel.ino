#include <Wire.h>
#include "Motor.h"
#include "AbsoluteEncoder.h"
#include "Battery.h"
#include "TractionEncoder.h"
#include "DynamixelSerial.h"
#include "PID.h"
#include "definitions.h"
#include "mod_config.h"
#include "Debug.h"

bool updm = false;

int time_enc = 0;
int time_bat = 0;

// used to convert from byte to float, both arrays share the same memory
union serialData_t {
  byte valueBuffer[12];
  float value[3];
} serialData;

// field identifier for serialData values
enum {traction_left, traction_right, yaw};

Motor motorTrLeft(DRV_TR_LEFT_PWM,DRV_TR_LEFT_DIR);
Motor motorTrRight(DRV_TR_RIGHT_PWM,DRV_TR_RIGHT_DIR);

TractionEncoder encoderTrLeft(ENC_TR_LEFT_A,ENC_TR_LEFT_B);
TractionEncoder encoderTrRight(ENC_TR_RIGHT_A,ENC_TR_RIGHT_B);

#ifdef MODC_YAW
Motor motorYaw(DRV_YAW_PWM,DRV_YAW_DIR);
AbsoluteEncoder encoderYaw;
PID pidYaw(PID_YAW_KP,PID_YAW_KI,PID_YAW_KD ,PID_YAW_MAX_OUTPUT,PID_YAW_EMA_ALPHA);
#endif

Battery battery;

// event on incoming I²C data
void receive(int byteCount) {
  updm = true;
  Debug.println("RECEIVED DATA");
  for (uint8_t index = 0; index < byteCount && index < 12; index++) {
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
    motorYaw.write(pidYaw.getOutput());
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

  // only set motor speed if we received new data
  if (updm) {
    Debug.println("UPDATE RECEIVED DATA.");
#ifdef MODC_YAW
    Debug.println("YAW:");
    pidYaw.updateReferenceValue(serialData.value[yaw]);
#endif
    Debug.print("TRACTION LEFT:\t");
    motorTrLeft.write(serialData.value[traction_left]);
    Debug.print("TRACTION RIGHT:\t");
    motorTrRight.write(-serialData.value[traction_right]);
    updm = false;
  }

}
