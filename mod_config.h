#ifndef module_configuration_h
#define module_configuration_h


// select here which module to build for
// teoricamente definiti dal compilatore
//#define MOD_HEAD
//#define MOD_MIDDLE
//#define MOD_TAIL


#elif defined(MOD_HEAD)
#define CAN_ID    0x15  // HEAD

#elif defined(MOD_MIDDLE)
#define CAN_ID    0x16  // MIDDLE
#define MODC_YAW
#define ABS_ENC_OFFSET 160 // TODO

#if defined(MOD_TAIL)
#define CAN_ID    0x17  // TAIL
#define MODC_YAW
#define ABS_ENC_OFFSET 169 // TODO

#endif

#endif
