#ifndef definitions_h
#define definitions_h

#define DT 100

// PWM configuration
#define PWM_MAX_VALUE 1023
#define PWM_FREQUENCY 15000

// I²C configuration
#define I2C_SENS_SDA   18
#define I2C_SENS_SCL   19

#define ABSOLUTE_ENCODER_ADDRESS 0x40 // ToDo check, maybe 0x44

// Battery configuration
#define BAT_LOW 11.1f
#define BAT_NOM 12.6f
#define BAT_PIN 28
#define BAT_R1 10000
#define BAT_R2 3300

// Motors pins
#define DRV_TR_LEFT_DIR  15
#define DRV_TR_LEFT_PWM 14
#define DRV_TR_LEFT_CURR A0

#define DRV_TR_RIGHT_DIR  9
#define DRV_TR_RIGHT_PWM  8
#define DRV_TR_RIGHT_CURR A1

#define DRV_YAW_DIR  3
#define DRV_YAW_PWM  2

// Encoder pins
#define ENC_TR_LEFT_A   12
#define ENC_TR_LEFT_B   13

#define ENC_TR_RIGHT_A  10
#define ENC_TR_RIGHT_B  11

// Encoder conversion constant
// K = 1000          *       10^6        *       60      / (     12   *             74,83                    *       2 )
//      milliRPM           microsecToSec        SecToMin       intPerRotation        transmissionRatio             transmissionRatio2
#define ENC_TR_CONVERSION 33783784
// (milliRPM/1000) : maxRPM = PWM : 1023
// maxRPM = 130 / 2 (2 = transmissionRatio)
// PWM = milliRPM * 1023 / (1000 * maxRPM)
#define RPM_TO_PWM(rpm) rpm*.015738461538

// YAW MIN/MAX
#define YAW_MAX_ANGLE 30
#define YAW_MIN_ANGLE -YAW_MAX_ANGLE


// PID configuration
#define PID_YAW_KP			13
#define PID_YAW_KI			.005
#define PID_YAW_KD			1000
#define PID_YAW_MAX_OUTPUT	1023
#define PID_YAW_EMA_ALPHA	.5

#define PID_TR_KP			0.1
#define PID_TR_KI			.0005
#define PID_TR_KD			0.0
#define PID_TR_MAX_OUTPUT	1023
#define PID_TR_EMA_ALPHA	1

// Display
#define DISPLAY_ADDR 0x3c
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64

// Interface definitions
#define NMENUS 4
#define MENUTIMEOUT 18
#define BTNOK 16
#define BTNNAV 17
#define DEBOUNCE 300

// Versioning
#ifndef VERSION
  #define VERSION "testing"
#endif

// OTA configuration
#define OTA_PWD "ciaociao"
#define WIFI_SSID "iswifi"
#define WIFI_PWD "ciaociao"
#define WIFI_HOSTBASE "picow-"
#define CONF_PATH "/config.txt"

#endif
