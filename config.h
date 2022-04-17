// configuration file 


// 
#define SERIAL_DEBUG_ATTIVATO 1


// I2C config
#define I2C_ADDRESS		0x15
#define I2C_PIN_SDA		20
#define I2C_PIN_SCL		21

#define I2C_ENC_PIN_SDA		2
#define I2C_ENC_PIN_SCL		3


//  - - - PID control constants
//  

#define DT				50

#define START_TR_LEFT	0
#define START_TR_RIGHT	START_TR_LEFT
#define START_YAW		0 //TODO

#define MAX_OUTPUT		1023
#define MIN_OUTPUT		-MAX_OUTPUT

// traction left
#define TR_LEFT_KP		.91
#define TR_LEFT_KI		.1
#define TR_LEFT_KD		.0

// traction right
#define TR_RIGHT_KP 	TR_LEFT_KP
#define TR_RIGHT_KI 	TR_LEFT_KI
#define TR_RIGHT_KD 	TR_LEFT_KD

// yaw
#define YAW_KP			3
#define YAW_KI			.3
#define YAW_KD			.35


//  - - - DRIVER pins

// traction left
#define DRV_TR_LEFT_DIR	16
#define DRV_TR_LEFT_PWM	17
#define DRV_TR_LEFT_SLP	
#define DRV_TR_LEFT_FLT	
#define DRV_TR_LEFT_CS		// current sense

// traction right
#define DRV_TR_RIGHT_DIR	14
#define DRV_TR_RIGHT_PWM	15
#define DRV_TR_LEFT_SLP
#define DRV_TR_LEFT_FLT
#define DRV_TR_LEFT_CS

// yaw
#define DRV_YAW_DIR		4
#define DRV_YAW_PWM		5
#define DRV_TR_LEFT_SLP
#define DRV_TR_LEFT_FLT
#define DRV_TR_LEFT_CS

// ENCODER PINS
#define ENC_TR_LEFT_A	11
#define ENC_TR_LEFT_B	10

#define ENC_TR_RIGHT_A	12
#define ENC_TR_RIGHT_B	13
