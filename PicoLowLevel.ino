#include <Wire.h>
#include "Motor.h"

#define I2C_ADDRESS    0x15
#define I2C_PIN_SDA   20
#define I2C_PIN_SCL   21

#define DRV_TR_LEFT_DIR  15
#define DRV_TR_LEFT_PWM 14

#define DRV_TR_RIGHT_DIR  9
#define DRV_TR_RIGHT_PWM  8

bool updm = false;

// used to convert from byte to float, both arrays share the same memory
union serialData_t {
  byte valueBuffer[8];
  float value[2];
} serialData;

// field identifier for serialData values
enum {traction_left, traction_right};

Motor motorTrLeft(DRV_TR_LEFT_PWM,DRV_TR_LEFT_DIR);
Motor motorTrRight(DRV_TR_RIGHT_PWM,DRV_TR_RIGHT_DIR);

// event on incoming I²C data
void receive(int byteCount) {
  updm = true;
  Serial.println("RECEIVED DATA");
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

  // initializing PWM
  analogWriteFreq(50000); // switching frequency to 50kHz
  analogWriteRange(512); // analogWrite range from 0 to 512, default is 255

  // motor initialization
  motorTrLeft.begin(); 
  motorTrRight.begin();

  Serial.println("BEGIN");
}

void loop() {
  // only set motor speed if we received new data
  if (updm) {
    Serial.print("TRACTION LEFT:\t");
    motorTrLeft.write(serialData.value[traction_left]);
    Serial.print("TRACTION RIGHT:\t");
    motorTrRight.write(-serialData.value[traction_right]);
    updm = false;
  }

}
