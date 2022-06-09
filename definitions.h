#ifndef definitions_h
#define definitions_h

#define DT 100

// PWM configuration
#define PWM_MAX_VALUE 512
#define PWM_FREQUENCY 50000

// I²C configuration
#define I2C_ADDRESS    0x15
#define I2C_PIN_SDA   20
#define I2C_PIN_SCL   21

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

//Encoder pins
#define ENC_TR_LEFT_A   13
#define ENC_TR_LEFT_B   12

#define ENC_TR_RIGHT_A  10
#define ENC_TR_RIGHT_B  11


#endif
