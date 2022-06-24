#ifndef module_configuration_h
#define module_configuration_h


// select here which module to build for
// teoricamente definiti dal compilatore
//#define MOD_HEAD
//#define MOD_TAIL


#if defined(MOD_TAIL)
#define CAN_ID    0x15  // TAIL
#define MODC_YAW
#define ABS_ENC_OFFSET 200 // TODO



#elif defined(MOD_MIDDLE)
#define CAN_ID    0x16  // MIDDLE
#define MODC_YAW
#define ABS_ENC_OFFSET 200 // TODO

#elif defined(MOD_HEAD)
#define CAN_ID    0x17  // HEAD

#endif

#endif
