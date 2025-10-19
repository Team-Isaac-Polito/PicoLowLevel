#include "Dynamixel_ll.h"
#include "include/mod_config.h"

#define ProfileAcceleration 10
#define ProfileVelocity 20






const uint8_t motorIDs[] = {210, 211};
const uint8_t numMotors = sizeof(motorIDs) / sizeof(motorIDs[0]);

// variabili per la posizione iniziale
int32_t pos0_mot_2 = 0;
int32_t pos0_mot_3 = 0;
int32_t pos0_mot_4 = 0;
int32_t pos0_mot_5 = 0;
int32_t pos0_mot_6 = 0;
int32_t getpositions0[2] = {0, 0};






#define ProfileAcceleration 10
#define ProfileVelocity 20

int16_t presentLoad_mot_6 = 0;

DynamixelLL dxl(Serial1, 0);
DynamixelLL mot_Left_1(Serial1, motorIDs[0]);
DynamixelLL mot_Right_1(Serial1, motorIDs[1]);
DynamixelLL mot_2(Serial1, 112);
DynamixelLL mot_3(Serial1, 113);
DynamixelLL mot_4(Serial1, 214);
DynamixelLL mot_5(Serial1, 215);
DynamixelLL mot_6(Serial1, 216);




void setup() {

   Serial.begin(115200);
   while(!Serial) {
    delay(10); // Wait for Serial to be ready
    // Wait for Serial to be ready
    }
   Serial1.setTX(0);
  Serial1.setRX(1);


    // Set the baud rate for Dynamixel communication
  dxl.begin_dxl(2000000);
  mot_Left_1.begin_dxl(2000000);
  mot_Right_1.begin_dxl(2000000);
  mot_2.begin_dxl(2000000);
  mot_3.begin_dxl(2000000);
  mot_4.begin_dxl(2000000);
  mot_5.begin_dxl(2000000);
  mot_6.begin_dxl(2000000);

  mot_Right_1.setTorqueEnable(false); // Disable torque for safety
  mot_Left_1.setTorqueEnable(false);
  mot_2.setTorqueEnable(false);
  mot_3.setTorqueEnable(false);
  mot_4.setTorqueEnable(false);
  mot_5.setTorqueEnable(false);
  mot_6.setTorqueEnable(false);

  delay(10);

 dxl.setStatusReturnLevel(2); // Set status return level for the main motor
  mot_Left_1.setStatusReturnLevel(2);
  mot_Right_1.setStatusReturnLevel(2);
  mot_2.setStatusReturnLevel(2);
  mot_3.setStatusReturnLevel(2);
  mot_4.setStatusReturnLevel(2);
  mot_5.setStatusReturnLevel(2);
  mot_6.setStatusReturnLevel(2);
  delay(10);

  // Enable or disable debug mode for troubleshooting
  mot_Left_1.setDebug(false);
  mot_Right_1.setDebug(false);
  mot_2.setDebug(false);
  mot_3.setDebug(false);
  mot_4.setDebug(false);
  mot_5.setDebug(false);
  mot_6.setDebug(false);
  dxl.setDebug(false);

  // Enable sync mode for multiple motor control.
  dxl.enableSync(motorIDs, numMotors);

  // Configure Drive Mode for each motor:
  mot_Left_1.setDriveMode(false, false, false);
  mot_Right_1.setDriveMode(false, false, false);
  mot_2.setDriveMode(false, false, false);
  mot_3.setDriveMode(false, false, false);
  mot_4.setDriveMode(false, false, false);
  mot_5.setDriveMode(false, false, false);
  mot_6.setDriveMode(false, false, false);

  // Set Operating Mode for each motor:
  dxl.setOperatingMode(4); // Extended Position Mode
  mot_2.setOperatingMode(4);
  mot_3.setOperatingMode(4);
  mot_4.setOperatingMode(4);
  mot_5.setOperatingMode(4);
  mot_6.setOperatingMode(4);







  delay(10);


}

void loop() {
 /*
   getpositions0[0] = 2695; // Initialize positions to 0
  getpositions0[1] = 813;  // Initialize positions to 0
  pos0_mot_2 = 4851;
  pos0_mot_3 = -1895;
  pos0_mot_4 = 3209;
  pos0_mot_5 = 7181;
  pos0_mot_6 = -1009; // Initialize positions to 0*/

   dxl.getPresentPosition(getpositions0);
  mot_2.getPresentPosition(pos0_mot_2);
  mot_3.getPresentPosition(pos0_mot_3);
  mot_4.getPresentPosition( pos0_mot_4);
  mot_5.getPresentPosition( pos0_mot_5);
  mot_6.getPresentPosition(pos0_mot_6);

  Serial.println("initial positions:");
  Serial.print("getpositions0[0] = ");
  Serial.print(getpositions0[0]);
  Serial.println(";");
  Serial.print("getpositions0[1] = ");
  Serial.print(getpositions0[1]);
  Serial.println(";");
  Serial.print("pos0_mot_2 = ");
  Serial.print(pos0_mot_2);
  Serial.println(";");
  Serial.print("pos0_mot_3 = ");
  Serial.print(pos0_mot_3);
  Serial.println(";");
  Serial.print("pos0_mot_4 = ");
  Serial.print(pos0_mot_4);
  Serial.println(";");
  Serial.print("pos0_mot_5 = ");
  Serial.print(pos0_mot_5);
  Serial.println(";");
  Serial.print("pos0_mot_6 = ");
  Serial.print(pos0_mot_6);
  Serial.println(";");


  delay(1000); // Wait for a bit before the next loop.

  // Repeat loop...
}
