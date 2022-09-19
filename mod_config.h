#ifndef module_configuration_h
#define module_configuration_h


// select here which module to build for
// teoricamente definiti dal compilatore
//#define MOD_HEAD
#define MOD_MIDDLE
//#define MOD_TAIL


#if defined(MOD_HEAD)
#define CAN_ID    0x15  // HEAD
#define MODC_PITCH
#define SERVO_ID 1
#define SERVO_MIN 200
#define SERVO_MAX 800
#define SERVO_SPEED 200

#elif defined(MOD_MIDDLE)
#define CAN_ID    0x16  // MIDDLE
#define MODC_YAW
#define ABS_ENC_OFFSET 285 // TODO

#elif defined(MOD_TAIL)
#define CAN_ID    0x17  // TAIL
#define MODC_YAW
#define ABS_ENC_OFFSET 169 // TODO

#endif

#endif
