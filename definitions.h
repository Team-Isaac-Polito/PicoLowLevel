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
#define ENC_TR_LEFT_A   12
#define ENC_TR_LEFT_B   13

#define ENC_TR_RIGHT_A  10
#define ENC_TR_RIGHT_B  11


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

#endif
