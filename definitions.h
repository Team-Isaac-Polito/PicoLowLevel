#ifndef definitions_h
#define definitions_h

#define DT 100

// PWM configuration
#define PWM_MAX_VALUE 512
#define PWM_FREQUENCY 50000

// I²C configuration
#define I2C_SENS_SDA   18
#define I2C_SENS_SCL   19

#define ABSOLUTE_ENCODER_ADDRESS 0x40 // ToDo check, maybe 0x44

// Battery configuration
#define BAT_LOW 11.1f
#define BAT_PIN 28
#define BAT_R1 10000
#define BAT_R2 3300

// Motors pins
#define DRV_TR_LEFT_DIR  15
#define DRV_TR_LEFT_PWM 14

#define DRV_TR_RIGHT_DIR  9
#define DRV_TR_RIGHT_PWM  8

#define DRV_YAW_DIR  3
#define DRV_YAW_PWM  2

// Encoder pins
#define ENC_TR_LEFT_A   13
#define ENC_TR_LEFT_B   12

#define ENC_TR_RIGHT_A  10
#define ENC_TR_RIGHT_B  11

// YAW MIN/MAX
#define YAW_MAX_ANGLE 30
#define YAW_MIN_ANGLE -YAW_MAX_ANGLE


// PID configuration
#define PID_YAW_KP			9.5
#define PID_YAW_KI			.001
#define PID_YAW_KD			.35
#define PID_YAW_MAX_OUTPUT	1023
#define PID_YAW_EMA_ALPHA	.5


// DYN servo config 
// - maybe it will be added in mod_config as it can vary between modules
#define DYN_ADDR    2

#endif
