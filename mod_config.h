#ifndef module_configuration_h
#define module_configuration_h


// select here which module to build for

//#define MOD_HEAD
#define MOD_MIDDLE
//#define MOD_TAIL


#if defined(MOD_TAIL)
#define I2C_ADDRESS    0x15  // TAIL
#define MODC_YAW

#elif defined(MOD_MIDDLE)
#define I2C_ADDRESS    0x16  // MIDDLE
#define MODC_YAW

#elif defined(MOD_HEAD)
#define I2C_ADDRESS    0x17  // HEAD

#endif

#endif
