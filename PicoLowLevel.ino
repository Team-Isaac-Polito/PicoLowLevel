
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "AbsoluteEncoder.h"
#include "Battery.h"
// #include "DynamixelSerial.h"
// #include "TractionEncoder.h"
// #include "MovingAvgFilter.h"
// #include "ExpSmoothingFilter.h"
#include "Debug.h"
#include "mcp2515.h"
#include "Display.h"
// #include "SmartMotor.h"
// #include "Motor.h"
// #include "PID.h"
#include "CanWrapper.h"

#include "include/definitions.h"
#include "include/mod_config.h"
#include "include/communication.h"
#include <string.h> // strncpy, strlen
#include <LittleFS.h>

#include "Dynamixel_ll.h"
#include "IMU.h"

#include "debug_log.h"
// #define DEBUG_LOG_ENABLED // Uncomment to enable debug logging

#define DXL_TO_RAD 0.00153398f // Conversion factor: 2pi / 4096
#define RAD_TO_DXL 651.899f // Conversion factor: 4096 / 2pi

void okInterrupt();
void navInterrupt();
void sendFeedback();
void handleSetpoint(uint8_t msg_id, const byte *msg_data);
void DXL_TRACTION_INIT();
#ifdef MODC_ARM
void MODC_ARM_INIT();
void RESET_ARM_INITIAL_POSITION();
bool loadHomePositions();
void saveHomePositions();
#endif

#define BaudRateDXL 2000000

struct DxlStatsPayload {
    uint32_t txCount;
    uint32_t rxSuccess;
    uint32_t rxTimeout;
    uint32_t rxCrcError;
    uint32_t rxRetries;
};

int time_bat = 0;
int time_tel = 0;
int time_data = 0;
int time_tel_avg = DT_TEL;
int time_DXL_check = 0;

int errorCount = 0;

CanWrapper canW(5, 20000000UL, &SPI);

bool led_status = false;

uint8_t error_var = 0;

//================ Traction Motors =================
DynamixelLL dxl_traction(Serial1, 0);
const uint8_t motorIDs_traction[] = {212, 114};
// NOTE: variable names mot_Left/mot_Right are swapped relative to physical
// motor positions (212 = physical right, 114 = physical left).
// The robot still drives correctly because the memcpy order inside
// MOTOR_SETPOINT compensates: speeds_dxl[0] receives the CAN "right"
// value and is sync-written to ID 212 (physical right), speeds_dxl[1]
// receives "left" and goes to ID 114 (physical left).
// setDriveMode(reverse=true) on ID 212 only corrects for the opposite
// motor mounting orientation — it does NOT fix the naming mismatch.
// The debug print labels ("left: speeds_dxl[0]") are also swapped.
const uint8_t numMotors_traction = sizeof(motorIDs_traction) / sizeof(motorIDs_traction[0]);
DynamixelLL mot_Left_traction(Serial1, motorIDs_traction[0]);   // ID 212 — physical RIGHT motor
DynamixelLL mot_Right_traction(Serial1, motorIDs_traction[1]);  // ID 114 — physical LEFT motor
float speeds_dxl[2] = {0.0f, 0.0f};
float old_speeds_dxl[2] = {0.0f, 0.0f};
float delta_speeds_dxl = 2.0f;
uint8_t data_dxl_traction[8];

uint8_t ErrorStatus_traction[2] = {0, 0};

int32_t currentSpeeds_left;  // Current speeds of the left traction motor for feedback
int32_t currentSpeeds_right; // Current speeds of the right traction motor for feedback

float currentSpeeds_left_float = 0.0f;
float currentSpeeds_right_float = 0.0f;

int32_t servo_data;

static unsigned long lastStatsTx = 0;
const unsigned long STATS_INTERVAL_MS = 1000; // millis interval for transmission (in this case 1 Hz = 1 s)

#ifdef MODC_YAW
AbsoluteEncoder encoderYaw(ABSOLUTE_ENCODER_ADDRESS);
#endif

#ifdef MODC_EE
DynamixelMotor motorEEPitch(SERVO_EE_PITCH_ID);
DynamixelMotor motorEEHeadPitch(SERVO_EE_HEAD_PITCH_ID);
DynamixelMotor motorEEHeadRoll(SERVO_EE_HEAD_ROLL_ID);
#endif


//================ motori del braccio =================
#ifdef MODC_ARM
// variabili per lettura dal CAN della posizione desiderata dei motori
#define ProfileAcceleration 10
#define ProfileVelocity 20

float servo_data_1a = 0.0f;
float servo_data_1b = 0.0f;
float servo_data_float = 0.0f;

const uint8_t motorIDs_ARM[] = {210, 211};
const uint8_t numMotors_ARM = sizeof(motorIDs_ARM) / sizeof(motorIDs_ARM[0]);

// variabili per la posizione iniziale
int32_t ARM_pos0_mot_1LR[2] = {0, 0};
int32_t ARM_pos0_mot_2 = 0;
int32_t ARM_pos0_mot_3 = 0;
int32_t ARM_pos0_mot_4 = 0;
int32_t ARM_pos0_mot_5 = 0;
int32_t ARM_pos0_mot_6 = 0;

// variabili per la posizione attuale da mandare ai motori
int32_t ARM_pos_mot_1LR[2] = {0, 0};
int32_t ARM_pos_mot_2 = 0;
int32_t ARM_pos_mot_3 = 0;
int32_t ARM_pos_mot_4 = 0;
int32_t ARM_pos_mot_5 = 0;
int32_t ARM_pos_mot_6 = 0;
int32_t ARM_pos_mot_6_actual = 0;
float ARM_theta_dxl = 0.0f;
float ARM_phi_dxl = 0.0f;
int32_t ARM_valueToSend = 0;

int32_t ARM_old_pos_mot_2 = 0;
int32_t ARM_old_pos_mot_3 = 0;
int32_t ARM_old_pos_mot_4 = 0;
int32_t ARM_old_pos_mot_5 = 0;
int32_t ARM_old_pos_mot_1LR[2] = {0, 0};

#define ARM_de_can_dxl 10

// delta from start position
int32_t ARM_delta_pos0_mot_2 = 0;
int32_t ARM_delta_pos0_mot_3 = 0;
int32_t ARM_delta_pos0_mot_4 = 0;
int32_t ARM_delta_pos0_mot_5 = 0;
int32_t ARM_delta_pos0_mot_6 = 0;
int32_t ARM_delta_pos0_mot_1LR[2] = {0, 0};

int32_t ARM_servo_data_mot_6 = 0; // variabile per la lettura della posizione del motore 6 dal CAN

// variabili per il feedback
int32_t ARM_posf_1a1b[2] = {0, 0};
int32_t ARM_posf_2 = 0;
int32_t ARM_posf_3 = 0;
int32_t ARM_posf_4 = 0;
int32_t ARM_posf_5 = 0;
int32_t ARM_posf_6 = 0;

float ARM_posf_1a1b_float[2] = {0.0f, 0.0f};
float ARM_posf_2_float = 0.0f;
float ARM_posf_3_float = 0.0f;
float ARM_posf_4_float = 0.0f;
float ARM_posf_5_float = 0.0f;
float ARM_posf_6_float = 0.0f;

float ARM_phif_dxl = 0.0f;
float ARM_thetaf_dxl = 0.0f;

float ARM_vel_mot1a1b[2] = {0.0f, 0.0f};
float ARM_vel_mot2 = 0.0f;
float ARM_vel_mot3 = 0.0f;
float ARM_vel_mot4 = 0.0f;
float ARM_vel_mot5 = 0.0f;
float ARM_vel_mot6 = 0.0f;

float ARM_phif_dxl_vel = 0.0f;
float ARM_thetaf_dxl_vel = 0.0f;

int16_t ARM_presentLoad_mot_6 = 0;

#define HOME_POSITIONS_FILE "/home_pos.bin"
static const int32_t ARM_DEFAULT_HOME[] = {2183, 1234, 3543, 3206, 2035, 3477, 141};

uint8_t ErrorStatusArm[7] = {0, 0, 0, 0, 0, 0, 0};

DynamixelLL ARM_dxl(Serial1, 0);
DynamixelLL mot_Left_1_ARM(Serial1, motorIDs_ARM[0]);
DynamixelLL mot_Right_1_ARM(Serial1, motorIDs_ARM[1]);
DynamixelLL ARM_mot_2(Serial1, 112);
DynamixelLL ARM_mot_3(Serial1, 113);
DynamixelLL ARM_mot_4(Serial1, 214);
DynamixelLL ARM_mot_5(Serial1, 215);
DynamixelLL ARM_mot_6(Serial1, 216);

// Beak/gripper (motor 6, XL430-W250) configuration
// Positions calibrated with dxl_get_position.ino in Extended Position Mode
const int32_t BEAK_POS_OPEN = -154;
const int32_t BEAK_POS_CLOSE = 154;
const int16_t BEAK_LOAD_THRESHOLD = 150;  // 0.1% units (~15% max torque), triggers grip detection
const int32_t BEAK_POS_TOLERANCE = 20;    // encoder units (~1.7°)
const uint32_t BEAK_TIMEOUT_MS = 3000;    // max time for motion before giving up
const int16_t BEAK_HOLD_PWM = 250;        // ~28% max PWM for holding torque after grip
const int16_t BEAK_FULL_PWM = 885;        // 100% max PWM for free movement
const uint8_t BEAK_TEMP_LIMIT = 65;       // °C, reduce holding force before 72° shutdown

enum BeakState { BEAK_IDLE, BEAK_CLOSING, BEAK_OPENING, BEAK_HOLDING };
BeakState beak_state = BEAK_IDLE;
uint32_t beak_motion_start_ms = 0;
uint32_t beak_temp_check_ms = 0;

#endif

#ifdef MODC_JOINT

// variabili per lettura dal CAN della posizione desiderata dei motori
#define ProfileAcceleration 10
#define ProfileVelocity 20

float servo_data_1a = 0.0f;
float servo_data_1b = 0.0f;
float servo_data_float = 0.0f;



const uint8_t motorIDs_JOINT[] = {100,120};
//100 --> motor left 
//120 --> motor right
const uint8_t numMotors_JOINT = sizeof(motorIDs_JOINT) / sizeof(motorIDs_JOINT[0]);

DynamixelLL JOINT_dxl(Serial1, 0);
DynamixelLL JOINT_mot_Left_1(Serial1, motorIDs_JOINT[0]);
DynamixelLL JOINT_mot_Right_1(Serial1, motorIDs_JOINT[1]);
DynamixelLL JOINT_mot_2(Serial1, 123);
float JOINT_theta_dxl = 0.0f;
float JOINT_phi_dxl = 0.0f;
int32_t JOINT_valueToSend = 0;

int32_t JOINT_pos0_mot_1LR[2] = {0, 0};
int32_t JOINT_pos0_mot_2 = 0;

int32_t JOINT_pos_mot_1LR[2] = {0, 0};
int32_t JOINT_pos_mot_2 = 0;

int32_t JOINT_old_pos_mot_1LR[2] = {0, 0};

int32_t JOINT_posf_1a1b[2] = {0, 0};
int32_t JOINT_posf_2 = 0;

// yaw = theta (JOINT_yaw_pitch_float[0]), pitch = phi (JOINT_yaw_pitch_float[1])
float JOINT_yaw_pitch_float[2] = {0.0f, 0.0f};
float JOINT_posf_2_float = 0.0f;

float JOINT_phif_dxl = 0.0f;
float JOINT_thetaf_dxl = 0.0f;
#endif

// IMU
#ifdef MODC_IMU
IMU imu;
#endif


Display display;




void setup()
{

  Serial.begin(115200);
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed");
  }

  Debug.setLevel(Levels::INFO); // comment to set debug verbosity to debug
  Debug.println("BEGIN", Levels::INFO);
  Wire1.setSDA(I2C_SENS_SDA);
  Wire1.setSCL(I2C_SENS_SCL);
  Wire1.begin();

  SPI.setRX(4);
  SPI.setCS(5);
  SPI.setSCK(6);
  SPI.setTX(7);
  SPI.begin();

  Serial1.setTX(0);
  Serial1.setRX(1);
  Serial1.begin(BaudRateDXL); // Set baud rate for Dynamixel communication

  // CAN initialization
  canW.begin();

  // initializing PWM
  analogWriteFreq(PWM_FREQUENCY);  // switching frequency to 15kHz
  analogWriteRange(PWM_MAX_VALUE); // analogWrite range from 0 to 512, default is 255

  // initializing ADC
  analogReadResolution(12); // set precision to 12 bits, 0-4095 input
  // Buttons initialization
  pinMode(BTNOK, INPUT_PULLUP);
  pinMode(BTNNAV, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTNOK), okInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTNNAV), navInterrupt, FALLING);

  // Display initialization
  display.begin(); // has showLogo

  DXL_TRACTION_INIT();

#ifdef MODC_YAW
  encoderYaw.update();
  encoderYaw.readAngle();
  encoderYaw.setZero();
#endif

#ifdef MODC_ARM
  MODC_ARM_INIT();
#endif

#ifdef MODC_JOINT
  MODC_JOINT_INIT();
#endif

#ifdef MODC_IMU
  imu.begin(Wire1, 0x6A);
  if (!imu.checkID()){
    Serial.println("IMU not found!");
    display.addError("IMU not found!", 16);
  }
  else {
    imu.enableGyro();
    imu.enableAccel();
    Serial.println("IMU Initialized");
    imu.calibrateAccel();
    imu.calibrateGyro();
    Serial.println("IMU Calibration Complete!");
  }
#endif

  Serial.println("Setup complete. Waiting for CAN messages...");
#ifdef DEBUG_LOG_ENABLED
  dbg("Setup complete. Waiting for CAN messages...\n");
#endif

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  unsigned long currentMillis = millis();
  unsigned long prevMillis_status, interval_status;
  int time_cur = millis();
  uint8_t msg_id;
  byte msg_data[8];

  if (currentMillis - lastStatsTx >= STATS_INTERVAL_MS)
  {
    sendFeedback();
    prevMillis_status = currentMillis;
  }

  
  static unsigned long prevMillis_dxl_stats = 0;
  const unsigned long interval_dxl_stats = 1000; // Send stats every 1 second

  if (currentMillis - prevMillis_dxl_stats >= interval_dxl_stats)
  {
    sendDxlStats();
    prevMillis_dxl_stats = currentMillis;
  }

  // health checks
  if (time_cur - time_bat >= DT_BAT)
  {
    time_bat = time_cur;

    if (time_tel_avg > DT_TEL)
    {
      char buf[64];
      snprintf(buf, sizeof(buf), "Telemetry frequency below required: %d ms", (int)time_tel_avg);
      Debug.println(buf, Levels::WARN);
      display.addError("Telemetry frequency below required", 16);
    }
    if (!battery.charged())
    {
      char buf[64];
      snprintf(buf, sizeof(buf), "Battery voltage low! %f v", battery.readVoltage());
      Debug.println(buf, Levels::WARN);
      display.addError("Low battery voltage!", 16);
    }
  }

  // send telemetry
  if (time_cur - time_tel >= DT_TEL)
  {
    time_tel_avg = (time_tel_avg + (time_cur - time_tel)) / 2;
    time_tel = time_cur;

    sendFeedback();
  }

  if (canW.readMessage(&msg_id, msg_data))
  {

    // Received CAN message with setpoint
    time_data = time_cur;
    handleSetpoint(msg_id, msg_data);
  }
  else if (time_cur - time_data > CAN_TIMEOUT && time_data != -1)
  {
    // if we do not receive data for more than a second stop motors
    time_data = -1;

    // motorTrLeft.stop();
    speeds_dxl[0] = 0.0f;
    speeds_dxl[1] = 0.0f;

    dxl_traction.setGoalVelocity_RPM(speeds_dxl); // Stop both motors
    display.addError("CANBUS ERROR!", 8);
  }
  else
  {
  }

  // wm.handle();
  display.handleGUI();

  // Beak gripper state machine (motor 6)
#ifdef MODC_ARM
  if (beak_state == BEAK_CLOSING)
  {
    ARM_mot_6.getCurrentLoad(ARM_presentLoad_mot_6);
    ARM_mot_6.getPresentPosition(ARM_pos_mot_6_actual);

    bool load_detected = abs(ARM_presentLoad_mot_6) >= BEAK_LOAD_THRESHOLD;
    bool pos_reached = abs(ARM_pos_mot_6_actual - BEAK_POS_CLOSE) <= BEAK_POS_TOLERANCE;
    bool timed_out = (millis() - beak_motion_start_ms) > BEAK_TIMEOUT_MS;

    if (load_detected || pos_reached || timed_out)
    {
      // Freeze at current position and limit PWM to hold gently
      ARM_mot_6.getPresentPosition(ARM_pos_mot_6_actual);
      ARM_mot_6.setGoalPosition_EPCM(ARM_pos_mot_6_actual);
      ARM_mot_6.setGoalPWM(BEAK_HOLD_PWM);
      beak_state = BEAK_HOLDING;
      beak_temp_check_ms = millis();
    }
  }
  else if (beak_state == BEAK_OPENING)
  {
    ARM_mot_6.getCurrentLoad(ARM_presentLoad_mot_6);
    ARM_mot_6.getPresentPosition(ARM_pos_mot_6_actual);

    bool pos_reached = abs(ARM_pos_mot_6_actual - BEAK_POS_OPEN) <= BEAK_POS_TOLERANCE;
    bool timed_out = (millis() - beak_motion_start_ms) > BEAK_TIMEOUT_MS;

    if (pos_reached || timed_out)
    {
      ARM_mot_6.setGoalPosition_EPCM(ARM_pos_mot_6_actual);
      ARM_mot_6.setGoalPWM(BEAK_FULL_PWM);
      beak_state = BEAK_IDLE;
    }
  }
  else if (beak_state == BEAK_HOLDING)
  {
    // Periodic temperature check to prevent overheating (~every 500ms)
    if (millis() - beak_temp_check_ms > 500)
    {
      beak_temp_check_ms = millis();
      uint8_t temp = 0;
      ARM_mot_6.getPresentTemperature(temp);
      if (temp >= BEAK_TEMP_LIMIT)
        ARM_mot_6.setGoalPWM(BEAK_HOLD_PWM / 2);

      uint8_t hwErr = 0;
      ARM_mot_6.getHardwareErrorStatus(hwErr);
      if (hwErr != 0)
        beak_state = BEAK_IDLE;
    }
  }
#endif

  if (time_cur - time_DXL_check >= DT_DXL_CHECK)
  {
    digitalWrite(LED_BUILTIN, led_status);
    led_status = !led_status;
    time_DXL_check = time_cur;
    char buf[64];

    mot_Right_traction.getHardwareErrorStatus(ErrorStatus_traction[0]);
    mot_Left_traction.getHardwareErrorStatus(ErrorStatus_traction[1]);
#ifdef MODC_ARM
    error_var++;
    if (error_var == 1)
    {
      mot_Left_1_ARM.getHardwareErrorStatus(ErrorStatusArm[0]);
      mot_Right_1_ARM.getHardwareErrorStatus(ErrorStatusArm[1]);
    };
    if (error_var == 2)
    {
      ARM_mot_2.getHardwareErrorStatus(ErrorStatusArm[2]);
      ARM_mot_3.getHardwareErrorStatus(ErrorStatusArm[3]);
    };
    if (error_var == 3)
    {
      ARM_mot_4.getHardwareErrorStatus(ErrorStatusArm[4]);
      ARM_mot_5.getHardwareErrorStatus(ErrorStatusArm[5]);
    }
    if (error_var == 4)
    {
      ARM_mot_6.getHardwareErrorStatus(ErrorStatusArm[6]);
      error_var = 0;
    };

    for (int i = 0; i < 7; i++)
    {
      snprintf(buf, sizeof(buf), " motor%d %d", i + 1, ErrorStatusArm[i]);
      display.addStatus(buf, 16, i + 2);
      memset(buf, 0, sizeof(buf));
    }

#endif

    snprintf(buf, sizeof(buf), " motor_tr%d %d", 0, ErrorStatus_traction[0]);
    display.addStatus(buf, 16, 0);
    memset(buf, 0, sizeof(buf));

    snprintf(buf, sizeof(buf), " motor%d %d", 1, ErrorStatus_traction[1]);
    display.addStatus(buf, 16, 1);
    memset(buf, 0, sizeof(buf));
  }
}

/**
 * @brief Handles the setpoint messages received via CAN bus.
 * @param msg_id ID of the received message.
 * @param msg_data Pointer to the message data.
 */
void handleSetpoint(uint8_t msg_id, const byte *msg_data)
{

  switch (msg_id)
  {

  //========================================================
  case MOTOR_SETPOINT:
  {

    memcpy(&speeds_dxl[1], msg_data, 4);
    memcpy(&speeds_dxl[0], msg_data + 4, 4);

    dxl_traction.setGoalVelocity_RPM(speeds_dxl);
    Debug.println("TRACTION DATA :\tleft: \t" + String(speeds_dxl[0]) + "\tright: \t" + String(speeds_dxl[1]));
    break;
  }

#ifdef MODC_ARM
  //========================================================
  case ARM_PITCH_1a1b_SETPOINT:
    memcpy(&servo_data_1a, msg_data, 4);
    memcpy(&servo_data_1b, msg_data + 4, 4);
    ARM_theta_dxl = servo_data_1a;
    ARM_phi_dxl = servo_data_1b;

    ARM_pos_mot_1LR[0] = (int32_t)(((ARM_theta_dxl * RAD_TO_DXL) + (ARM_phi_dxl * RAD_TO_DXL))) + ARM_pos0_mot_1LR[0];
    ARM_pos_mot_1LR[1] = (int32_t)(((ARM_theta_dxl * RAD_TO_DXL) - (ARM_phi_dxl * RAD_TO_DXL))) + ARM_pos0_mot_1LR[1];

    if (abs(ARM_pos_mot_1LR[0] - ARM_old_pos_mot_1LR[0]) > ARM_de_can_dxl || abs(ARM_pos_mot_1LR[1] - ARM_old_pos_mot_1LR[1]) > ARM_de_can_dxl)
    {
      ARM_dxl.setGoalPosition_EPCM(ARM_pos_mot_1LR);
      Serial.print("time");
      Serial.println(millis());
      ARM_old_pos_mot_1LR[0] = ARM_pos_mot_1LR[0];
      ARM_old_pos_mot_1LR[1] = ARM_pos_mot_1LR[1];
    }

    Debug.print("PITCH ARM 1a MOTOR DATA : \t");
    Debug.println(ARM_pos_mot_1LR[0]);
    Debug.print("PITCH ARM 1b MOTOR DATA : \t");
    Debug.println(ARM_pos_mot_1LR[1]);
    break;

    //========================================================
  case ARM_PITCH_2_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    ARM_valueToSend = (int32_t)(servo_data_float * RAD_TO_DXL);
    ARM_pos_mot_2 = ARM_valueToSend + ARM_pos0_mot_2;

    if (abs(ARM_pos_mot_2 - ARM_old_pos_mot_2) > ARM_de_can_dxl)
    {
      ARM_mot_2.setGoalPosition_EPCM(ARM_pos_mot_2);
      ARM_old_pos_mot_2 = ARM_pos_mot_2;
    }
    Debug.print("PITCH ARM 2 MOTOR DATA : \t");
    Debug.println(ARM_pos_mot_2);
    break;

    //========================================================
  case ARM_ROLL_3_SETPOINT:

    memcpy(&servo_data_float, msg_data, 4);
    ARM_valueToSend = (int32_t)(servo_data_float * RAD_TO_DXL);
    ARM_pos_mot_3 = ARM_valueToSend + ARM_pos0_mot_3;
    if (abs(ARM_pos_mot_3 - ARM_old_pos_mot_3) > ARM_de_can_dxl)
    {
      
      ARM_mot_3.setGoalPosition_EPCM(ARM_pos_mot_3);
      ARM_old_pos_mot_3 = ARM_pos_mot_3;
    }
    Debug.print("ROLL ARM 3 MOTOR DATA : \t");
    Debug.println(ARM_pos_mot_3);
    break;

    //========================================================
  case ARM_PITCH_4_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    ARM_valueToSend = (int32_t)(servo_data_float * RAD_TO_DXL);
    ARM_pos_mot_4 = ARM_pos0_mot_4 + ARM_valueToSend;
    if (abs(ARM_pos_mot_4 - ARM_old_pos_mot_4) > ARM_de_can_dxl)
    {
      ARM_mot_4.setGoalPosition_EPCM(ARM_pos_mot_4);
      ARM_old_pos_mot_4 = ARM_pos_mot_4;
    }
    Debug.print("PITCH ARM 4 MOTOR DATA : \t");
    Debug.println(ARM_pos_mot_4);
    break;

    //========================================================
  case ARM_ROLL_5_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    ARM_valueToSend = (int32_t)(servo_data_float * RAD_TO_DXL);
    ARM_pos_mot_5 = ARM_pos0_mot_5 - ARM_valueToSend;
    if (abs(ARM_pos_mot_5 - ARM_old_pos_mot_5) > ARM_de_can_dxl)
    {
      ARM_mot_5.setGoalPosition_EPCM(ARM_pos_mot_5);
      ARM_old_pos_mot_5 = ARM_pos_mot_5;
    }
    Debug.print("ROLL ARM 5 MOTOR DATA : \t");
    Debug.println(ARM_pos_mot_5);
    break;

    //========================================================

  case ARM_ROLL_6_SETPOINT:

    memcpy(&ARM_servo_data_mot_6, msg_data, 4);
    if (ARM_servo_data_mot_6 == 0)
    {
      // Close beak: full PWM for approach, then PWM-limited hold on contact
      ARM_mot_6.setGoalPWM(BEAK_FULL_PWM);
      ARM_mot_6.setGoalPosition_EPCM(BEAK_POS_CLOSE);
      beak_motion_start_ms = millis();
      beak_state = BEAK_CLOSING;
    }
    else if (ARM_servo_data_mot_6 == 1)
    {
      // Open beak: restore full PWM and move to open position
      ARM_mot_6.setGoalPWM(BEAK_FULL_PWM);
      ARM_mot_6.setGoalPosition_EPCM(BEAK_POS_OPEN);
      beak_motion_start_ms = millis();
      beak_state = BEAK_OPENING;
    }
    break;

  case RESET_ARM:
    RESET_ARM_INITIAL_POSITION();
    break;

  case REBOOT_ARM:
    mot_Left_1_ARM.reboot();
    mot_Right_1_ARM.reboot();
    ARM_mot_2.reboot();
    ARM_mot_3.reboot();
    ARM_mot_4.reboot();
    ARM_mot_5.reboot();
    ARM_mot_6.reboot();
    MODC_ARM_INIT();
    break;

  case SET_HOME:
  {
    bool persist = (msg_data[0] == 1);

    // Read current positions as new home
    uint8_t read_err = 0;
    read_err |= ARM_dxl.getPresentPosition(ARM_pos0_mot_1LR);
    read_err |= ARM_mot_2.getPresentPosition(ARM_pos0_mot_2);
    read_err |= ARM_mot_3.getPresentPosition(ARM_pos0_mot_3);
    read_err |= ARM_mot_4.getPresentPosition(ARM_pos0_mot_4);
    read_err |= ARM_mot_5.getPresentPosition(ARM_pos0_mot_5);
    read_err |= ARM_mot_6.getPresentPosition(ARM_pos0_mot_6);

    if (read_err != 0) {
      Debug.println("SET_HOME aborted: failed to read motor positions", Levels::WARN);
      break;
    }

    // Reset deltas
    ARM_delta_pos0_mot_1LR[0] = 0;
    ARM_delta_pos0_mot_1LR[1] = 0;
    ARM_delta_pos0_mot_2 = 0;
    ARM_delta_pos0_mot_3 = 0;
    ARM_delta_pos0_mot_4 = 0;
    ARM_delta_pos0_mot_5 = 0;
    ARM_delta_pos0_mot_6 = 0;

    if (persist) {
      saveHomePositions();
      Debug.println("SET_HOME: permanent — saved to flash", Levels::INFO);
    } else {
      Debug.println("SET_HOME: interim — session only", Levels::INFO);
    }

    RESET_ARM_INITIAL_POSITION();
    break;
  }

#endif

#ifdef MODC_JOINT
    //========================================================
  case JOINT_PITCH_1a1b_SETPOINT:
    memcpy(&servo_data_1a, msg_data, 4);
    memcpy(&servo_data_1b, msg_data + 4, 4);
    JOINT_theta_dxl = servo_data_1a;
    JOINT_phi_dxl = servo_data_1b;

    // 100 -> left motor (JOINT_pos_mot_1LR[0])
    JOINT_pos_mot_1LR[0] = (int32_t)((JOINT_theta_dxl * RAD_TO_DXL) + (JOINT_phi_dxl * RAD_TO_DXL)) + JOINT_pos0_mot_1LR[0];
    // 120 -> right motor (JOINT_pos_mot_1LR[1])
    JOINT_pos_mot_1LR[1] = (int32_t)((JOINT_theta_dxl * RAD_TO_DXL) - (JOINT_phi_dxl * RAD_TO_DXL)) + JOINT_pos0_mot_1LR[1];

      JOINT_dxl.setGoalPosition_EPCM(JOINT_pos_mot_1LR);
      JOINT_old_pos_mot_1LR[0] = JOINT_pos_mot_1LR[0];
      JOINT_old_pos_mot_1LR[1] = JOINT_pos_mot_1LR[1];
    break;



    case JOINT_ROLL_2_SETPOINT:
    memcpy(&servo_data_float, msg_data, 4);
    JOINT_valueToSend = (int32_t)(servo_data_float * RAD_TO_DXL);
    JOINT_pos_mot_2 = JOINT_pos0_mot_2 + JOINT_valueToSend;

      JOINT_mot_2.setGoalPosition_EPCM(JOINT_pos_mot_2);
    break;
    
#endif
    //========================================================
  case MOTOR_TRACTION_REBOOT:
    mot_Left_traction.reboot();
    mot_Right_traction.reboot();

    Debug.println("Traction motors rebooted.");
    break;

  default:

    Debug.print("\tUnknown message ID:\t");

    Debug.print(msg_id);

    Debug.print("\tData:\t");
    for (int i = 0; i < 8; i++)
    {
      Debug.print(msg_data[i]);
      if (i < 7)
        Debug.print("\t");
    }
    Debug.print("\n");
    break;
  }
}

/**
 * @brief Sends feedback data over CAN bus.
 *
 * This function sends various feedback data including motor speeds, yaw angle, and end effector positions
 * if the respective modules are enabled.
 *
 * @note The function uses conditional compilation to include/exclude parts of the code based on the presence of specific modules.
 */
void sendFeedback()
{
  float speed_fb[2] = {currentSpeeds_left_float, currentSpeeds_right_float};
  dxl_traction.getPresentVelocity_RPM(speed_fb);

  memcpy(&data_dxl_traction[0], &speed_fb[0], 4); // copia il primo float nei primi 4 byte
  memcpy(&data_dxl_traction[4], &speed_fb[1], 4); // copia il secondo float nei secondi 4 byte

  canW.sendMessage(MOTOR_FEEDBACK, data_dxl_traction, 8);
  canW.sendMessage(MOTOR_TRACTION_ERROR_STATUS, ErrorStatus_traction, 2);

  // send yaw angle of the joint if this module has one
  #ifdef MODC_YAW
    encoderYaw.update();
    float angle = encoderYaw.readAngle();
    canW.sendMessage(JOINT_YAW_FEEDBACK, &angle, 4);
  #endif

    // Send the present position data of the arm motors

  #ifdef MODC_ARM

    ARM_dxl.getPresentPosition(ARM_posf_1a1b);
    ARM_phif_dxl = -(float)(((ARM_posf_1a1b[0] - ARM_pos0_mot_1LR[0]) + (ARM_posf_1a1b[1] - ARM_pos0_mot_1LR[1])) / 2.0f ) * DXL_TO_RAD;
    ARM_thetaf_dxl = (float)(((ARM_posf_1a1b[0] - ARM_pos0_mot_1LR[0]) - (ARM_posf_1a1b[1] - ARM_pos0_mot_1LR[1])) / 2.0f ) * DXL_TO_RAD;
    ARM_posf_1a1b_float[0] = ARM_thetaf_dxl;
    ARM_posf_1a1b_float[1] = ARM_phif_dxl;
    canW.sendMessage(ARM_PITCH_1a1b_FEEDBACK, ARM_posf_1a1b_float, sizeof(ARM_posf_1a1b));
    ARM_mot_2.getPresentPosition(ARM_posf_2);
    ARM_posf_2_float = (float)(ARM_posf_2 - ARM_pos0_mot_2) * DXL_TO_RAD;
    canW.sendMessage(ARM_PITCH_2_FEEDBACK, &ARM_posf_2_float, sizeof(ARM_posf_2));
    ARM_mot_3.getPresentPosition(ARM_posf_3);
    ARM_posf_3_float = (float)(ARM_posf_3 - ARM_pos0_mot_3) * DXL_TO_RAD;
    canW.sendMessage(ARM_ROLL_3_FEEDBACK, &ARM_posf_3_float, sizeof(ARM_posf_3));
    ARM_mot_4.getPresentPosition(ARM_posf_4);
    ARM_posf_4_float = (float)(ARM_posf_4 - ARM_pos0_mot_4) * DXL_TO_RAD;
    canW.sendMessage(ARM_PITCH_4_FEEDBACK, &ARM_posf_4_float, sizeof(ARM_posf_4));
    ARM_mot_5.getPresentPosition(ARM_posf_5);
    ARM_posf_5_float = (float)(ARM_posf_5 - ARM_pos0_mot_5) * DXL_TO_RAD;
    canW.sendMessage(ARM_ROLL_5_FEEDBACK, &ARM_posf_5_float, sizeof(ARM_posf_5));
    ARM_mot_6.getPresentPosition(ARM_posf_6);
    ARM_posf_6_float = (float)(ARM_posf_6 - ARM_pos0_mot_6) * DXL_TO_RAD;
    canW.sendMessage(ARM_ROLL_6_FEEDBACK, &ARM_posf_6_float, sizeof(ARM_posf_6));

    // Arm velocity feedback (RPM → rad/s)
    ARM_dxl.getPresentVelocity_RPM(ARM_vel_mot1a1b);

    ARM_phif_dxl_vel = -(float)((ARM_vel_mot1a1b[0]) + (ARM_vel_mot1a1b[1])) * 2 * M_PI / 60;
    ARM_thetaf_dxl_vel = (float)(-(ARM_vel_mot1a1b[0]) + (ARM_vel_mot1a1b[1])) * 2 * M_PI / 60;
    ARM_vel_mot1a1b[0] = ARM_thetaf_dxl_vel;
    ARM_vel_mot1a1b[1] = ARM_phif_dxl_vel;

    canW.sendMessage(ARM_PITCH_1a1b_FEEDBACK_VEL, &ARM_vel_mot1a1b, sizeof(ARM_vel_mot1a1b));

    ARM_mot_2.getPresentVelocity_RPM(ARM_vel_mot2);
    ARM_vel_mot2 = ARM_vel_mot2 * 2 * M_PI / 60;
    canW.sendMessage(ARM_PITCH_2_FEEDBACK_VEL, &ARM_vel_mot2, sizeof(ARM_vel_mot2));

    ARM_mot_3.getPresentVelocity_RPM(ARM_vel_mot3);
    ARM_vel_mot3 = ARM_vel_mot3 * 2 * M_PI / 60;
    canW.sendMessage(ARM_ROLL_3_FEEDBACK_VEL, &ARM_vel_mot3, sizeof(ARM_vel_mot3));

    ARM_mot_4.getPresentVelocity_RPM(ARM_vel_mot4);
    ARM_vel_mot4 = ARM_vel_mot4 * 2 * M_PI / 60;
    canW.sendMessage(ARM_PITCH_4_FEEDBACK_VEL, &ARM_vel_mot4, sizeof(ARM_vel_mot4));

    ARM_mot_5.getPresentVelocity_RPM(ARM_vel_mot5);
    ARM_vel_mot5 = ARM_vel_mot5 * 2 * M_PI / 60;
    canW.sendMessage(ARM_ROLL_5_FEEDBACK_VEL, &ARM_vel_mot5, sizeof(ARM_vel_mot5));

    ARM_mot_6.getPresentVelocity_RPM(ARM_vel_mot6);
    ARM_vel_mot6 = ARM_vel_mot6 * 2 * M_PI / 60;
    canW.sendMessage(ARM_ROLL_6_FEEDBACK_VEL, &ARM_vel_mot6, sizeof(ARM_vel_mot6));

    canW.sendMessage(MOTOR_ARM_ERROR_STATUS, ErrorStatusArm, 7);

  #endif

  #ifdef MODC_IMU
    //OPTION A - Accelerometer Only (No Sensor Fusion)
    imu.update();
    float imu_roll = imu.getRoll();
    float imu_pitch = imu.getPitch();
    canW.sendMessage(JOINT_ROLL_FEEDBACK, &imu_roll, sizeof(imu_roll));
    delayMicroseconds(1200); // Wait for 1 CAN frame at 125kbps to free a TX buffer
    canW.sendMessage(JOINT_PITCH_FEEDBACK, &imu_pitch, sizeof(imu_pitch));

    // OPTION B - Accelerometer + Gyroscope (Sensor Fusion)
    // imu.updateFused();
    // float imu_roll_fusion = imu.getFusedRoll();
    // float imu_pitch_fusion = imu.getFusedPitch();
    // canW.sendMessage(JOINT_ROLL_FEEDBACK, &imu_roll_fusion, sizeof(imu_roll_fusion));
    // delayMicroseconds(1200);
    // canW.sendMessage(JOINT_PITCH_FEEDBACK, &imu_pitch_fusion, sizeof(imu_pitch_fusion));
    // #endif
  #endif
    // Send the present position data of the joint motors
  #ifdef MODC_JOINT

  JOINT_dxl.getPresentPosition(JOINT_posf_1a1b);
  JOINT_thetaf_dxl = ((float)((JOINT_posf_1a1b[0] - JOINT_pos0_mot_1LR[0]) + (JOINT_posf_1a1b[1] - JOINT_pos0_mot_1LR[1])) / 2.0f) * DXL_TO_RAD;
  JOINT_phif_dxl   = ((float)((JOINT_posf_1a1b[0] - JOINT_pos0_mot_1LR[0]) - (JOINT_posf_1a1b[1] - JOINT_pos0_mot_1LR[1])) / 2.0f) * DXL_TO_RAD;
  JOINT_yaw_pitch_float[0] = JOINT_thetaf_dxl;
  JOINT_yaw_pitch_float[1] = JOINT_phif_dxl;
  canW.sendMessage(JOINT_PITCH_1a1b_FEEDBACK, JOINT_yaw_pitch_float, sizeof(JOINT_yaw_pitch_float));
  JOINT_mot_2.getPresentPosition(JOINT_posf_2);
  JOINT_posf_2_float = (float)(JOINT_posf_2 - JOINT_pos0_mot_2) * DXL_TO_RAD;
  canW.sendMessage(JOINT_ROLL_2_FEEDBACK, &JOINT_posf_2_float, sizeof(JOINT_posf_2_float));
#endif

}

void sendDxlStats() {
    // 1. Usiamo 'auto': il compilatore capisce da solo il tipo di dato restituito dalla libreria!
    const auto& stats = dxl_traction.getStats();

    // 2. Mappiamo i valori nella nostra struct per il CAN bus
    DxlStatsPayload payload;
    payload.txCount    = stats.txCount;
    payload.rxSuccess  = stats.rxSuccess;   
    payload.rxTimeout  = stats.timeouts;    
    payload.rxCrcError = stats.crcErrors;   
    payload.rxRetries  = stats.retries;     

    canW.sendMessage(DXL_COMM_STATS, (byte*)&payload, sizeof(payload)); 
    
    #ifdef DEBUG_LOG_ENABLED
    Serial.println("Sent DXL Stats over CAN.");
    #endif
}

void okInterrupt()
{
  display.okInterrupt();
}

void navInterrupt()
{
  display.navInterrupt();
}

#ifdef MODC_ARM

bool loadHomePositions() {
  File f = LittleFS.open(HOME_POSITIONS_FILE, "r");
  if (!f) {
    Debug.println("No saved home positions, using defaults", Levels::INFO);
    return false;
  }

  int32_t positions[7];
  if (f.read((uint8_t*)positions, sizeof(positions)) != sizeof(positions)) {
    f.close();
    Debug.println("Corrupted home positions file, using defaults", Levels::WARN);
    return false;
  }
  f.close();

  ARM_pos0_mot_1LR[0] = positions[0];
  ARM_pos0_mot_1LR[1] = positions[1];
  ARM_pos0_mot_2 = positions[2];
  ARM_pos0_mot_3 = positions[3];
  ARM_pos0_mot_4 = positions[4];
  ARM_pos0_mot_5 = positions[5];
  ARM_pos0_mot_6 = positions[6];

  Debug.println("Home positions loaded from flash", Levels::INFO);
  return true;
}

void saveHomePositions() {
  int32_t positions[7] = {
    ARM_pos0_mot_1LR[0], ARM_pos0_mot_1LR[1],
    ARM_pos0_mot_2, ARM_pos0_mot_3,
    ARM_pos0_mot_4, ARM_pos0_mot_5, ARM_pos0_mot_6
  };

  File f = LittleFS.open(HOME_POSITIONS_FILE, "w");
  if (!f) {
    Debug.println("Failed to save home positions", Levels::WARN);
    return;
  }
  size_t written = f.write((uint8_t*)positions, sizeof(positions));
  if (written != sizeof(positions)) {
    Debug.println("Partial write, removing corrupt file", Levels::WARN);
    f.close();
    LittleFS.remove(HOME_POSITIONS_FILE);
    return;
  }
  f.close();

  Debug.println("Home positions saved to flash", Levels::INFO);
}

void MODC_ARM_INIT()
{ // Initialize Dynamixel motors for the arm

  // Set the baud rate for Dynamixel communication
  ARM_dxl.begin_dxl(BaudRateDXL);
  mot_Left_1_ARM.begin_dxl(BaudRateDXL);
  mot_Right_1_ARM.begin_dxl(BaudRateDXL);
  ARM_mot_2.begin_dxl(BaudRateDXL);
  ARM_mot_3.begin_dxl(BaudRateDXL);
  ARM_mot_4.begin_dxl(BaudRateDXL);
  ARM_mot_5.begin_dxl(BaudRateDXL);
  ARM_mot_6.begin_dxl(BaudRateDXL);

  mot_Right_1_ARM.setTorqueEnable(false); // Disable torque for safety
  mot_Left_1_ARM.setTorqueEnable(false);
  ARM_mot_2.setTorqueEnable(false);
  ARM_mot_3.setTorqueEnable(false);
  ARM_mot_4.setTorqueEnable(false);
  ARM_mot_5.setTorqueEnable(false);
  ARM_mot_6.setTorqueEnable(false);

  delay(10);

  ARM_dxl.setStatusReturnLevel(2); // Set status return level for the main motor
  mot_Left_1_ARM.setStatusReturnLevel(2);
  mot_Right_1_ARM.setStatusReturnLevel(2);
  ARM_mot_2.setStatusReturnLevel(2);
  ARM_mot_3.setStatusReturnLevel(2);
  ARM_mot_4.setStatusReturnLevel(2);
  ARM_mot_5.setStatusReturnLevel(2);
  ARM_mot_6.setStatusReturnLevel(2);
  delay(10);

  // Enable or disable debug mode for troubleshooting
  mot_Left_1_ARM.setDebug(false);
  mot_Right_1_ARM.setDebug(false);
  ARM_mot_2.setDebug(false);
  ARM_mot_3.setDebug(false);
  ARM_mot_4.setDebug(false);
  ARM_mot_5.setDebug(false);
  ARM_mot_6.setDebug(false);
  ARM_dxl.setDebug(false);

  // Enable sync mode for multiple motor control.
  ARM_dxl.enableSync(motorIDs_ARM, numMotors_ARM);

  // Configure Drive Mode for each motor:
  mot_Left_1_ARM.setDriveMode(false, false, false);
  mot_Right_1_ARM.setDriveMode(false, false, false);
  ARM_mot_2.setDriveMode(false, false, false);
  ARM_mot_3.setDriveMode(false, false, false);
  ARM_mot_4.setDriveMode(false, false, false);
  ARM_mot_5.setDriveMode(false, false, false);
  ARM_mot_6.setDriveMode(false, false, false);

  // Set Operating Mode for each motor:
  ARM_dxl.setOperatingMode(4); // Extended Position Mode
  ARM_mot_2.setOperatingMode(4);
  ARM_mot_3.setOperatingMode(4);
  ARM_mot_4.setOperatingMode(4);
  ARM_mot_5.setOperatingMode(4);
  ARM_mot_6.setOperatingMode(4);

  delay(10);
  // Set Profile Velocity and Profile Acceleration for smooth motion.
  mot_Left_1_ARM.setProfileVelocity(ProfileVelocity);
  mot_Left_1_ARM.setProfileAcceleration(ProfileAcceleration);
  mot_Right_1_ARM.setProfileVelocity(ProfileVelocity);
  mot_Right_1_ARM.setProfileAcceleration(ProfileAcceleration);
  ARM_mot_2.setProfileVelocity(ProfileVelocity);
  ARM_mot_2.setProfileAcceleration(ProfileAcceleration);
  ARM_mot_3.setProfileVelocity(ProfileVelocity);
  ARM_mot_3.setProfileAcceleration(ProfileAcceleration);
  ARM_mot_4.setProfileVelocity(ProfileVelocity);
  ARM_mot_4.setProfileAcceleration(ProfileAcceleration);
  ARM_mot_5.setProfileVelocity(ProfileVelocity);
  ARM_mot_5.setProfileAcceleration(ProfileAcceleration);
  ARM_mot_6.setProfileVelocity(ProfileVelocity);
  ARM_mot_6.setProfileAcceleration(ProfileAcceleration);

  delay(10);

  // Read current motor positions BEFORE enabling torque to prevent violent
  // startup motion. Set goal = current so motors stay in place when torque
  // is enabled, then smoothly move to home position via profile velocity.
  int32_t current_pos_1LR[2];
  int32_t current_pos_2, current_pos_3, current_pos_4, current_pos_5, current_pos_6;

  bool posReadOk =
    ARM_dxl.getPresentPosition(current_pos_1LR) == 0 &&
    ARM_mot_2.getPresentPosition(current_pos_2) == 0 &&
    ARM_mot_3.getPresentPosition(current_pos_3) == 0 &&
    ARM_mot_4.getPresentPosition(current_pos_4) == 0 &&
    ARM_mot_5.getPresentPosition(current_pos_5) == 0 &&
    ARM_mot_6.getPresentPosition(current_pos_6) == 0;

  if (!posReadOk) {
    Debug.println("ARM init: position reads failed, torque disabled", Levels::WARN);
    return;
  }

  // Pre-load goal position registers with current positions (while torque is off)
  ARM_dxl.setGoalPosition_EPCM(current_pos_1LR);
  ARM_mot_2.setGoalPosition_EPCM(current_pos_2);
  ARM_mot_3.setGoalPosition_EPCM(current_pos_3);
  ARM_mot_4.setGoalPosition_EPCM(current_pos_4);
  ARM_mot_5.setGoalPosition_EPCM(current_pos_5);
  ARM_mot_6.setGoalPosition_EPCM(current_pos_6);

  delay(10);

  // NOW enable torque safely — motors stay in place since goal ≈ current
  ARM_dxl.setTorqueEnable(true);
  mot_Left_1_ARM.setTorqueEnable(true);
  mot_Right_1_ARM.setTorqueEnable(true);
  ARM_mot_2.setTorqueEnable(true);
  ARM_mot_3.setTorqueEnable(true);
  ARM_mot_3.setLED(true); // Enable LED for visual feedback
  ARM_mot_4.setTorqueEnable(true);
  ARM_mot_5.setTorqueEnable(true);
  ARM_mot_6.setTorqueEnable(true);
  // Load home positions from flash, fall back to compiled defaults
  delay(10);
  if (!loadHomePositions()) {
    ARM_pos0_mot_1LR[0] = ARM_DEFAULT_HOME[0];
    ARM_pos0_mot_1LR[1] = ARM_DEFAULT_HOME[1];
    ARM_pos0_mot_2 = ARM_DEFAULT_HOME[2];
    ARM_pos0_mot_3 = ARM_DEFAULT_HOME[3];
    ARM_pos0_mot_4 = ARM_DEFAULT_HOME[4];
    ARM_pos0_mot_5 = ARM_DEFAULT_HOME[5];
    ARM_pos0_mot_6 = ARM_DEFAULT_HOME[6];
  }

  RESET_ARM_INITIAL_POSITION();
}

void RESET_ARM_INITIAL_POSITION()
{

  ARM_pos0_mot_1LR[0] = ARM_pos0_mot_1LR[0] + ARM_delta_pos0_mot_1LR[0];
  ARM_pos0_mot_1LR[1] = ARM_pos0_mot_1LR[1] + ARM_delta_pos0_mot_1LR[1];
  ARM_pos0_mot_2 = ARM_pos0_mot_2 + ARM_delta_pos0_mot_2;
  ARM_pos0_mot_3 = ARM_pos0_mot_3 + ARM_delta_pos0_mot_3;
  ARM_pos0_mot_4 = ARM_pos0_mot_4 + ARM_delta_pos0_mot_4;
  ARM_pos0_mot_5 = ARM_pos0_mot_5 + ARM_delta_pos0_mot_5;
  ARM_pos0_mot_6 = ARM_pos0_mot_6 + ARM_delta_pos0_mot_6;
  ARM_dxl.setGoalPosition_EPCM(ARM_pos0_mot_1LR);
  ARM_mot_2.setGoalPosition_EPCM(ARM_pos0_mot_2);
  ARM_mot_3.setGoalPosition_EPCM(ARM_pos0_mot_3);
  ARM_mot_4.setGoalPosition_EPCM(ARM_pos0_mot_4);
  ARM_mot_5.setGoalPosition_EPCM(ARM_pos0_mot_5);
  ARM_mot_6.setGoalPosition_EPCM(ARM_pos0_mot_6);
}

#endif


#ifdef MODC_JOINT
void MODC_JOINT_INIT()
{ // Initialize Dynamixel motors for the joint

  // Set the baud rate for Dynamixel communication
  JOINT_dxl.begin_dxl(BaudRateDXL);
  JOINT_mot_Left_1.begin_dxl(BaudRateDXL);
  JOINT_mot_Right_1.begin_dxl(BaudRateDXL);


  JOINT_mot_Left_1.setTorqueEnable(false); // Disable torque for safety
  JOINT_mot_Right_1.setTorqueEnable(false);


  delay(10);

  JOINT_dxl.setStatusReturnLevel(2); // Set status return level for the main motor
  JOINT_mot_Left_1.setStatusReturnLevel(2);
  JOINT_mot_Right_1.setStatusReturnLevel(2);

  delay(10);

  // Enable or disable debug mode for troubleshooting
  JOINT_mot_Left_1.setDebug(false);
  JOINT_mot_Right_1.setDebug(false);
  JOINT_dxl.setDebug(false);

  // Enable sync mode for multiple motor control.
  JOINT_dxl.enableSync(motorIDs_JOINT, numMotors_JOINT);

  // Configure Drive Mode for each motor:
  JOINT_mot_Left_1.setDriveMode(false, false, false);
  JOINT_mot_Right_1.setDriveMode(false, false, false);
 
  // Set Operating Mode for each motor:
  JOINT_dxl.setOperatingMode(4); // Extended Position Mode
  JOINT_mot_Left_1.setOperatingMode(4);
  JOINT_mot_Right_1.setOperatingMode(4);
 

  delay(10);
  // Set Profile Velocity and Profile Acceleration for smooth motion.
  JOINT_mot_Left_1.setProfileVelocity(ProfileVelocity);
  JOINT_mot_Left_1.setProfileAcceleration(ProfileAcceleration);
  JOINT_mot_Right_1.setProfileVelocity(ProfileVelocity);
  JOINT_mot_Right_1.setProfileAcceleration(ProfileAcceleration);
  

  delay(10);
  // Enable torque for all motors.
  JOINT_dxl.setTorqueEnable(true);
  JOINT_mot_Left_1.setTorqueEnable(true);
  JOINT_mot_Right_1.setTorqueEnable(true);
 
  // Insert here the initial positions
  delay(10);
  JOINT_dxl.getPresentPosition(JOINT_pos0_mot_1LR);  
  JOINT_mot_2.getPresentPosition(JOINT_pos0_mot_2);
 
}

#endif


void DXL_TRACTION_INIT()
{

  // Set the baud rate for Dynamixel communication
  dxl_traction.begin_dxl(BaudRateDXL);
  mot_Left_traction.begin_dxl(BaudRateDXL);
  mot_Right_traction.begin_dxl(BaudRateDXL);

  mot_Right_traction.setTorqueEnable(false); // Disable torque for safety
  mot_Left_traction.setTorqueEnable(false);

  delay(10);

  dxl_traction.setStatusReturnLevel(2); // Set status return level for the main motor
  mot_Left_traction.setStatusReturnLevel(2);
  mot_Right_traction.setStatusReturnLevel(2);

  delay(10);

  // Enable or disable debug mode for troubleshooting
  mot_Left_traction.setDebug(false);
  mot_Right_traction.setDebug(false);
  dxl_traction.setDebug(false);

  // Enable sync mode for multiple motor control.
  dxl_traction.enableSync(motorIDs_traction, numMotors_traction);

  // Configure Drive Mode for each motor:
  mot_Left_traction.setDriveMode(false, false, true);
  mot_Right_traction.setDriveMode(false, false, false);

  // Set Operating Mode for each motor:
  dxl_traction.setOperatingMode(1); // Extended Position Mode

  delay(10);
  // Set Profile Velocity and Profile Acceleration for smooth motion.

  // Enable torque for all motors.
  dxl_traction.setTorqueEnable(true);
  mot_Left_traction.setTorqueEnable(true);
  mot_Right_traction.setTorqueEnable(true);
}