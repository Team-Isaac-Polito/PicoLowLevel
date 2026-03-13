/**
 * IMU.h - LSM6DSL Driver (C++ / Arduino Wire)
 * 
 * Converted from the original C driver for compatibility
 * with the PicoLowLevel Arduino firmware.
 * Original C driver: Located at the end of the file, commented
  */
 
#ifndef IMU_H
#define IMU_H
 
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
 
/* I2C Address (SA0/SDO pin connected to GND) */
#define LSM6DSL_ADDR    0x6A
 
// Bandwidth
#define LSM6DSL_BW_400HZ  0x00
#define LSM6DSL_BW_200HZ  0x01
#define LSM6DSL_BW_100HZ  0x02
#define LSM6DSL_BW_50HZ   0x03
 
// Sensitivity (from datasheet)
#define LSM6DSL_SENSITIVITY_ACCEL  0.061f     // mg/LSB at +/-2g
#define LSM6DSL_SENSITIVITY_GYRO   0.004375f  // dps/LSB at 125dps
#define CALIBRATION_DATA_SIZE      1000
 
// Key Registers
#define LSM6DSL_WHO_AM_I    0x0F
 
// Control Registers
#define LSM6DSL_CTRL1_XL    0x10    // Accelerometer control
#define LSM6DSL_CTRL2_G     0x11    // Gyroscope control
 
// Accelerometer Data Registers
#define LSM6DSL_OUTX_L_XL   0x28
#define LSM6DSL_OUTX_H_XL   0x29
#define LSM6DSL_OUTY_L_XL   0x2A
#define LSM6DSL_OUTY_H_XL   0x2B
#define LSM6DSL_OUTZ_L_XL   0x2C
#define LSM6DSL_OUTZ_H_XL   0x2D
 
// Gyroscope Data Registers
#define LSM6DSL_OUTX_L_G    0x22
#define LSM6DSL_OUTX_H_G    0x23
#define LSM6DSL_OUTY_L_G    0x24
#define LSM6DSL_OUTY_H_G    0x25
#define LSM6DSL_OUTZ_L_G    0x26
#define LSM6DSL_OUTZ_H_G    0x27
 
/* Simple structure to hold raw sensor data */
struct SensorData {
    int16_t x;
    int16_t y;
    int16_t z;
};
 
class IMU {
public:
    /**
     * Initialize the IMU driver.
     * @param wire  Reference to a TwoWire instance (e.g. Wire1)
     * @param addr  I2C address (default 0x6A)
     */
    void begin(TwoWire &wire, uint8_t addr = LSM6DSL_ADDR);
 
    /**
     * Check WHO_AM_I register.
     * @return true if sensor responds with 0x6A
     */
    bool checkID();
 
    // ---- Accelerometer ----
    void enableAccel();
    void calibrateAccel();
    void readAccel(SensorData &data);
 
    // ---- Gyroscope ----
    void enableGyro();
    void calibrateGyro();
    void readGyro(SensorData &data);
 
    // ---- Orientation (computed from accelerometer) ----
 
    /** @return pitch angle in radians */
    float getPitch();
 
    /** @return roll angle in radians */
    float getRoll();
 
private:
    TwoWire *_wire;
    uint8_t  _addr;
 
    // Calibration offsets
    int16_t _offsetAccelX = 0;
    int16_t _offsetAccelY = 0;
    int16_t _offsetAccelZ = 0;
 
    int16_t _offsetGyroX = 0;
    int16_t _offsetGyroY = 0;
    int16_t _offsetGyroZ = 0;
 
    // Low-level I2C helpers
    void    writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void    readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
};
 
#endif






// /**
//  * imu.h - Simple LSM6DSL Driver for experimental purposes
//  */

// #ifndef IMU_H
// #define IMU_H

// #include <stdint.h>
// #include <stdbool.h>

// /* I2C Address (when SA0/SDO pin is connected to GND) */
// #define LSM6DSL_ADDR    0x6A

// // Bandwidth
// #define LSM6DSL_BW_400HZ  0x00
// #define LSM6DSL_BW_200HZ  0x01
// #define LSM6DSL_BW_100HZ  0x02
// #define LSM6DSL_BW_50HZ   0x03

// // Sensitivity (From the data sheet)
// #define LSM6DSL_SENSITIVITY_ACCEL 0.061f   // mg/LSB
// #define LSM6DSL_SENSITIVITY_GYRO  0.004375f   // dps/LSB
// #define CALIBRATION_DATA_SIZE 1000  // Use this many samples to calibrate


// // Key Register Definitions
// #define LSM6DSL_WHO_AM_I    0x0F    // Should read 0x6A as we have connected SA0/SDO to GND

// // Control Register Definitions
// #define LSM6DSL_CTRL1_XL    0x10    // Accelerometer control
// #define LSM6DSL_CTRL2_G     0x11    // Gyroscope control

// // Data Register Definitions for accelerometer
// #define LSM6DSL_OUTX_L_XL   0x28    // Accelerometer X low byte
// #define LSM6DSL_OUTX_H_XL   0x29    // Accelerometer X high byte
// #define LSM6DSL_OUTY_L_XL   0x2A    // Accelerometer Y low byte
// #define LSM6DSL_OUTY_H_XL   0x2B    // Accelerometer Y high byte
// #define LSM6DSL_OUTZ_L_XL   0x2C    // Accelerometer Z low byte
// #define LSM6DSL_OUTZ_H_XL   0x2D    // Accelerometer Z high byte

// // Data Register Definitions for gyroscope
// #define LSM6DSL_OUTX_L_G    0x22    // Gyroscope X low byte
// #define LSM6DSL_OUTX_H_G    0x23    // Gyroscope X high byte
// #define LSM6DSL_OUTY_L_G    0x24    // Gyroscope Y low byte
// #define LSM6DSL_OUTY_H_G    0x25    // Gyroscope Y high byte
// #define LSM6DSL_OUTZ_L_G    0x26    // Gyroscope Z low byte
// #define LSM6DSL_OUTZ_H_G    0x27    // Gyroscope Z high byte

// /* Simple structure to hold sensor data */
// typedef struct {
//     int16_t x;
//     int16_t y;
//     int16_t z;
// } sensor_data_t;  // Sensor data parameterized as *data in functions in imu.c

// /* Driver structure */
// typedef struct {
//     void *i2c_inst;     // Pico i2c instance (i2c1 for our case)
//     uint8_t addr;       // I2C address

//     // Accelerometer calibration offsets
//     int16_t offset_x_accel;
//     int16_t offset_y_accel;
//     int16_t offset_z_accel;

//     // Gyroscope calibration offsets
//     int16_t offset_x_gyro;
//     int16_t offset_y_gyro;
//     int16_t offset_z_gyro;
// } lsm6dsl_t;    // LSM6DSL driver paremeterized as *sensor in functions in imu.c

// /* Basic functions */
// // Initialization fucntions with 3 parameters
// // 1. Pointer to driver structure *sensor
// // 2. Pointer to i2c instance *i2c  (either i2c0 or i2c1)
// // 3. I2C address
// void lsm6dsl_init(lsm6dsl_t *sensor, void *i2c, uint8_t addr);

// // Check if sensor is connected
// // Returns true if connected and checks WHO_AM_I which should return 0x6A
// // 1. Pointer to driver structure *sensor (single parameter)
// bool lsm6dsl_check_id(lsm6dsl_t *sensor);

// /*------------Accelerometer related functions----------------*/

// // Enable Accelerometer by writing to CTRL1_XL
// // 1. Pointer to driver structure *sensor
// void lsm6dsl_enable_accel(lsm6dsl_t *sensor);

// // Read Accelerometer data
// // 1. Pointer to driver structure *sensor
// // 2. Pointer to sensor data structure *data
// void lsm6dsl_read_accel(lsm6dsl_t *sensor, sensor_data_t *data);

// // Calibrate Accelerometer
// // 1. Pointer to driver structure *sensor
// void lsm6dsl_calibrate_accel(lsm6dsl_t *sensor);

// /*------------Gyroscope related functions----------------*/

// // Enable Accelerometer by writing to CTRL1_XL
// // 1. Pointer to driver structure *sensor
// void lsm6dsl_enable_gyro(lsm6dsl_t *sensor);

// // Read Accelerometer data
// // 1. Pointer to driver structure *sensor
// // 2. Pointer to sensor data structure *data
// void lsm6dsl_read_gyro(lsm6dsl_t *sensor, sensor_data_t *data);

// // Calibrate Accelerometer
// // 1. Pointer to driver structure *sensor
// void lsm6dsl_calibrate_gyro(lsm6dsl_t *sensor);


// #endif