/**
 * IMU.cpp - LSM6DSL Driver Implementation (C++ / Arduino Wire)
 *
 * Converted from the original C driver. All register values,
 * calibration logic, and sensor math are identical.
 */

#include "IMU.h"

// ======================== I2C Helpers ========================

void IMU::writeRegister(uint8_t reg, uint8_t value)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->write(value);
    _wire->endTransmission();
}

uint8_t IMU::readRegister(uint8_t reg)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);  // repeated start
    _wire->requestFrom(_addr, (uint8_t)1);
    return _wire->read();
}

void IMU::readRegisters(uint8_t reg, uint8_t *buffer, uint8_t length)
{
    _wire->beginTransmission(_addr);
    _wire->write(reg);
    _wire->endTransmission(false);  // repeated start
    _wire->requestFrom(_addr, length);
    for (uint8_t i = 0; i < length; i++) {
        buffer[i] = _wire->read();
    }
}

// ======================== Init & ID ========================

void IMU::begin(TwoWire &wire, uint8_t addr)
{
    _wire = &wire;
    _addr = addr;
}

bool IMU::checkID()
{
    uint8_t id = readRegister(LSM6DSL_WHO_AM_I);
    return (id == 0x6A);
}

// ======================== Accelerometer ========================

void IMU::enableAccel()
{
    // CTRL1_XL: ODR=104Hz, FS=+/-2g, BW=50Hz
    // Binary: 01000011 = 0x43
    writeRegister(LSM6DSL_CTRL1_XL, 0x43);
}

void IMU::readAccel(SensorData &data)
{
    uint8_t buffer[6];
    readRegisters(LSM6DSL_OUTX_L_XL, buffer, 6);

    // Combine low and high bytes (little-endian)
    data.x = (int16_t)(buffer[1] << 8 | buffer[0]);
    data.y = (int16_t)(buffer[3] << 8 | buffer[2]);
    data.z = (int16_t)(buffer[5] << 8 | buffer[4]);

    // Apply calibration offsets
    data.x -= _offsetAccelX;
    data.y -= _offsetAccelY;
    data.z -= _offsetAccelZ;
}

void IMU::calibrateAccel()
{
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    uint8_t buffer[6];

    for (int i = 0; i < CALIBRATION_DATA_SIZE; i++) {
        readRegisters(LSM6DSL_OUTX_L_XL, buffer, 6);

        sumX += (int16_t)(buffer[1] << 8 | buffer[0]);
        sumY += (int16_t)(buffer[3] << 8 | buffer[2]);
        sumZ += (int16_t)(buffer[5] << 8 | buffer[4]);
    }

    _offsetAccelX = sumX / CALIBRATION_DATA_SIZE;
    _offsetAccelY = sumY / CALIBRATION_DATA_SIZE;
    _offsetAccelZ = (sumZ / CALIBRATION_DATA_SIZE) - 16384;  // Z = +1g when flat
}

// ======================== Gyroscope ========================

void IMU::enableGyro()
{
    // CTRL2_G: ODR=104Hz, FS=125dps
    // Binary: 01000010 = 0x42
    writeRegister(LSM6DSL_CTRL2_G, 0x42);
}

void IMU::readGyro(SensorData &data)
{
    uint8_t buffer[6];
    readRegisters(LSM6DSL_OUTX_L_G, buffer, 6);

    // Combine low and high bytes (little-endian)
    data.x = (int16_t)(buffer[1] << 8 | buffer[0]);
    data.y = (int16_t)(buffer[3] << 8 | buffer[2]);
    data.z = (int16_t)(buffer[5] << 8 | buffer[4]);

    // Apply calibration offsets
    data.x -= _offsetGyroX;
    data.y -= _offsetGyroY;
    data.z -= _offsetGyroZ;
}

void IMU::calibrateGyro()
{
    int32_t sumX = 0, sumY = 0, sumZ = 0;
    uint8_t buffer[6];

    for (int i = 0; i < CALIBRATION_DATA_SIZE; i++) {
        readRegisters(LSM6DSL_OUTX_L_G, buffer, 6);

        sumX += (int16_t)(buffer[1] << 8 | buffer[0]);
        sumY += (int16_t)(buffer[3] << 8 | buffer[2]);
        sumZ += (int16_t)(buffer[5] << 8 | buffer[4]);
    }

    _offsetGyroX = sumX / CALIBRATION_DATA_SIZE;
    _offsetGyroY = sumY / CALIBRATION_DATA_SIZE;
    _offsetGyroZ = sumZ / CALIBRATION_DATA_SIZE;
}

// ======================== Orientation ========================

void IMU::update()
{
    SensorData accel;
    readAccel(accel);

    float ax = accel.x * LSM6DSL_SENSITIVITY_ACCEL * 0.001f;
    float ay = accel.y * LSM6DSL_SENSITIVITY_ACCEL * 0.001f;
    float az = accel.z * LSM6DSL_SENSITIVITY_ACCEL * 0.001f;

    _cachedPitch = atan2f(ax, sqrtf(ay * ay + az * az));
    _cachedRoll  = atan2f(ay, sqrtf(ax * ax + az * az));
}

float IMU::getPitch()
{
    // Call update() first for fresh data, or use cached value
    // update();  // Uncomment for standalone use 
    return _cachedPitch;
}

float IMU::getRoll()
{
    // Call update() first for fresh data, or use cached value
    // update();  // Uncomment for standalone use
    return _cachedRoll;
}

















