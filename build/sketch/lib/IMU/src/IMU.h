#line 1 "C:\\Users\\sghio\\Documents\\PicoLowLevel\\PicoLowLevel\\lib\\IMU\\src\\IMU.h"
/**
 * IMU.h - LSM6DSL Driver (C++ / Arduino Wire)
 * 
 * Converted from the original C driver for compatibility
 * with the PicoLowLevel Arduino firmware.
 *
 * Supports two orientation modes:
 *   - Accelerometer-only:       call update(), then getPitch()/getRoll()
 *   - Complementary filter:     call updateFused(), then getFusedPitch()/getFusedRoll()
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
#define DEG_TO_RAD_F  0.017453293f            // pi / 180
#define CALIBRATION_DATA_SIZE      1000

// Complementary filter default coefficient
// 0.98 = 98% gyro trust, 2% accel correction
// Tune for best performance
#define COMPLEMENTARY_FILTER_ALPHA 0.98f
 
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

    // ---- Orientation: Accelerometer-only ----

    /** Read accelerometer once and update cached pitch/roll */
    void update();
 
    /** @return pitch angle in radians (accel-only) */
    float getPitch();
 
    /** @return roll angle in radians (accel-only) */
    float getRoll();

    // ---- Orientation: Complementary filter (accel + gyro) ----

    /**
     * Set the filter coefficient alpha.
     * Higher alpha (e.g. 0.98) = smoother, slower drift correction.
     * Lower alpha (e.g. 0.90) = noisier, faster drift correction.
     * @param alpha  Value between 0.0 and 1.0
     */
    void setAlpha(float alpha);

    /**
     * Read both accel and gyro, apply complementary filter.
     * Must be called at a regular interval (e.g. every sendFeedback cycle).
     */
    void updateFused();

    /** @return pitch angle in radians (fused) */
    float getFusedPitch();

    /** @return roll angle in radians (fused) */
    float getFusedRoll();
 
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

    // Cached angles — accelerometer-only
    float _cachedPitch = 0.0f;
    float _cachedRoll  = 0.0f;

    // Fused angles — complementary filter
    float _fusedPitch = 0.0f;
    float _fusedRoll  = 0.0f;
    float _alpha = COMPLEMENTARY_FILTER_ALPHA;
    unsigned long _lastFusedTime = 0;  // micros() timestamp of last updateFused()
    bool _fusedInitialized = false;    // seed from accel on first call
 
    // Low-level I2C helpers
    void    writeRegister(uint8_t reg, uint8_t value);
    uint8_t readRegister(uint8_t reg);
    void    readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length);
};
 
#endif