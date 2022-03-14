// configuration file

// I2C config
#define I2C_ADDRESS		0x15


//  - - - PID control constants
//  

#define DT				19

#define START_TR_LEFT	0
#define START_TR_RIGHT	START_TR_LEFT
#define START_YAW		12 //TODO

// traction left
#define TR_LEFT_KP		1
#define TR_LEFT_KI		2
#define TR_LEFT_KD		3

// traction right
#define TR_RIGHT_KP 	TR_LEFT_KP
#define TR_RIGHT_KI 	TR_LEFT_KI
#define TR_RIGHT_KD 	TR_LEFT_KD

// yaw
#define YAW_KP			1
#define YAW_KI			1
#define YAW_KD			1


//  - - - DRIVER pins

// traction left
#define DRV_TR_LEFT_DIR	1
#define DRV_TR_LEFT_PWM	1

// traction right
#define DRV_TR_RIGHT_DIR	1
#define DRV_TR_RIGHT_PWM	1

// yaw
#define DRV_YAW_DIR		1
#define DRV_YAW_PWM		1


// ENCODER PINS
#define ENC_TR_LEFT_A	23
#define ENC_TR_LEFT_B	23

#define ENC_TR_RIGHT_A	23
#define ENC_TR_RIGHT_B	23
