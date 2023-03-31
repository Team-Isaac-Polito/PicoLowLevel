#ifndef definitions_h
#define definitions_h

// Main loop timings
#define DT_BAT 1000   // 1000ms / 1000 = 1 Hz
#define DT_PID 10     // 1000ms / 100   = 100 Hz
#define DT_TEL 10     // 1000ms / 10   = 100 Hz
#define DT_ENC 20     // 1000ms / 20   = 50 Hz

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
// K = 100          *       10^6        *       60      / (     48   *             74,83                    *       2 )
//      centiRPM           microsecToSec        SecToMin       intPerRotation        transmissionRatio             transmissionRatio2
#define ENC_TR_CONVERSION (835226)
// (centiRPM/100) : maxRPM = PWM : 1023
// maxRPM = 130 / 2 (2 = transmissionRatio)
// PWM = centiRPM * 1023 / (100 * maxRPM)
#define RPM_TO_PWM .15738461538

#define ENC_TR_SAMPLES 5

// YAW MIN/MAX
#define YAW_MAX_ANGLE 30
#define YAW_MIN_ANGLE -YAW_MAX_ANGLE


// PID configuration
#define PID_YAW_KP			13
#define PID_YAW_KI			.005
#define PID_YAW_KD			1000
#define PID_YAW_MAX_OUTPUT	1023
#define PID_YAW_EMA_ALPHA	.5

#define PID_TR_KP			0.25
#define PID_TR_KI			5.0
#define PID_TR_KD			0.0
#define PID_TR_MAX_OUTPUT	6500
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
