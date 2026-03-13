/**
 * IMU.cpp - LSM6DSL Driver Implementation (C++ / Arduino Wire)
 *
 * Converted from the original C driver. All register values,
 * calibration logic, and sensor math are identical.
 *
 * Original C driver: Located at the end of the file, commented
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

float IMU::getPitch()
{
    SensorData accel;
    readAccel(accel);

    // Convert raw to mg, then to g
    float ax = accel.x * LSM6DSL_SENSITIVITY_ACCEL * 0.001f;
    float ay = accel.y * LSM6DSL_SENSITIVITY_ACCEL * 0.001f;
    float az = accel.z * LSM6DSL_SENSITIVITY_ACCEL * 0.001f;

    return atan2f(ax, sqrtf(ay * ay + az * az));
}

float IMU::getRoll()
{
    SensorData accel;
    readAccel(accel);

    // Convert raw to mg, then to g
    float ax = accel.x * LSM6DSL_SENSITIVITY_ACCEL * 0.001f;
    float ay = accel.y * LSM6DSL_SENSITIVITY_ACCEL * 0.001f;
    float az = accel.z * LSM6DSL_SENSITIVITY_ACCEL * 0.001f;

    return atan2f(ay, sqrtf(ax * ax + az * az));
}









// /**
//  * imu.c - Implementation
//  */

// #include "imu.h"
// #include "hardware/i2c.h"

// /* Helper function to write one register */
// // It is a blocking function that writes a single byte to a register
// // It makes use of the write_blocking function from the i2c library
// static void write_register(lsm6dsl_t *sensor, uint8_t reg, uint8_t value)
// {
//     uint8_t data[2] = {reg, value};
//     i2c_write_blocking(sensor->i2c_inst, sensor->addr, data, 2, false);
// }

// /* Helper function to read one register */
// // It is a blocking function that reads a single byte from a register
// // It makes use of the read_blocking function from the i2c library
// // In order to read a single byte from a register, we need to write the register address first
// static uint8_t read_register(lsm6dsl_t *sensor, uint8_t reg)
// {
//     uint8_t value;
//     i2c_write_blocking(sensor->i2c_inst, sensor->addr, &reg, 1, true);
//     i2c_read_blocking(sensor->i2c_inst, sensor->addr, &value, 1, false);
//     return value;
// }

// /* Initialize the sensor structure */
// void lsm6dsl_init(lsm6dsl_t *sensor, void *i2c, uint8_t addr)
// {
//     sensor->i2c_inst = i2c;
//     sensor->addr = addr;
// }

// /* Check if sensor is responding correctly */
// // It is a blocking function that reads the WHO_AM_I register
// // It returns true if the sensor is responding correctly by returning 0x6A
// bool lsm6dsl_check_id(lsm6dsl_t *sensor)
// {
//     uint8_t id = read_register(sensor, LSM6DSL_WHO_AM_I);
//     return (id == 0x6A);  // LSM6DSL returns 0x6A
// }

// /* Enable accelerometer at 104 Hz, +/-2g */
// // It is a blocking function that writes to the CTRL1_XL register
// // It configures the accelerometer to run at 104 Hz and +/-2g by writing the individual bits of the register as per the data sheet
// void lsm6dsl_enable_accel(lsm6dsl_t *sensor)
// {
//     // CTRL1_XL register: [ODR=104Hz(0100)]
//     //                    [FS=+/-2g(00)][BW(00--> 400Hz, 01--> 200Hz, 10--> 100Hz, 11--> 50Hz)]

//     // Sampling rate: 104 Hz
//     // Nyquist frequency: 52 Hz
//     // Cutoff frequency: 50 Hz
//     // Binary: 01000011 = 0x43

//     uint8_t config = (0x04 << 4) |  // ODR = 104 Hz
//                      (0x00 << 2) |  // FS = +/-2g
//                      LSM6DSL_BW_50HZ;
//     write_register(sensor, LSM6DSL_CTRL1_XL, 0x43);
// }


// /* Read accelerometer data (raw values) */
// // It is a blocking function that reads 6 bytes from the OUTX_L_G register
// // It combines the low and high bytes to get the raw values
// // It applies the calibration offsets
// void lsm6dsl_read_accel(lsm6dsl_t *sensor, sensor_data_t *data)
// {
//     uint8_t buffer[6];
//     uint8_t reg = LSM6DSL_OUTX_L_XL;
    
//     // Read 6 bytes starting from OUTX_L_XL
//     i2c_write_blocking(sensor->i2c_inst, sensor->addr, &reg, 1, true);
//     i2c_read_blocking(sensor->i2c_inst, sensor->addr, buffer, 6, false);
    
//     // Combine low and high bytes (little-endian)
//     data->x = (int16_t)(buffer[1] << 8 | buffer[0]);
//     data->y = (int16_t)(buffer[3] << 8 | buffer[2]);
//     data->z = (int16_t)(buffer[5] << 8 | buffer[4]);

//     // Apply Calibration Offsets
//     data->x -= sensor->offset_x_accel;
//     data->y -= sensor->offset_y_accel;
//     data->z -= sensor->offset_z_accel;
// }

// /* Calibrate the Accelerometer */
// // It is a blocking function that reads 6 bytes from the OUTX_L_XL register 1000 times (can be changed for different accuracy)
// // It calculates the offsets by averaging the raw values
// // It stores the offsets in the sensor structure
// void lsm6dsl_calibrate_accel(lsm6dsl_t *sensor)
// {
//     uint8_t buffer[6];
//     uint8_t reg = LSM6DSL_OUTX_L_XL;
//     int32_t sum_x = 0, sum_y = 0, sum_z = 0;  
    
//     for (int i = 0; i < CALIBRATION_DATA_SIZE; i++) {
//         // Read raw data directly (bypass calibration)
//         i2c_write_blocking(sensor->i2c_inst, sensor->addr, &reg, 1, true);
//         i2c_read_blocking(sensor->i2c_inst, sensor->addr, buffer, 6, false);
        
//         int16_t x = (int16_t)(buffer[1] << 8 | buffer[0]);
//         int16_t y = (int16_t)(buffer[3] << 8 | buffer[2]);
//         int16_t z = (int16_t)(buffer[5] << 8 | buffer[4]);
        
//         sum_x += x;
//         sum_y += y;
//         sum_z += z;
//     }
    
//     // Calculate offsets
//     sensor->offset_x_accel = sum_x / CALIBRATION_DATA_SIZE;
//     sensor->offset_y_accel = sum_y / CALIBRATION_DATA_SIZE;
//     sensor->offset_z_accel = (sum_z / CALIBRATION_DATA_SIZE) - 16384;  // Z should be +16384 when flat
// }


// /* Enable gyroscope at 104 Hz, 125dps */
// // It is a blocking function that writes to the CTRL1_G register
// // It configures the gyroscope to run at 104 Hz and 125dps resolution by writing the individual bits of the register as per the data sheet
// // Gyro has an LPF automatically enabled by the ODR setting with a BW of 40Hz for 104Hz Sampling Rate
// void lsm6dsl_enable_gyro(lsm6dsl_t *sensor)
// {
//     // CTRL1_G register: [ODR=104Hz(0100)]
//     //                   [FS=250(00)]
//     //                   [FS=125(1)]
//     //                   [0] -- Unused
//     // Binary: 01000010 = 0x42
//     write_register(sensor, LSM6DSL_CTRL2_G, 0x42);
// }


// /* Read gyroscope data (raw values) */
// // It is a blocking function that reads 6 bytes from the OUTX_L_G register
// // It combines the low and high bytes to get the raw values
// // It applies the calibration offsets
// void lsm6dsl_read_gyro(lsm6dsl_t *sensor, sensor_data_t *data)
// {
//     uint8_t buffer[6];
//     uint8_t reg = LSM6DSL_OUTX_L_G;
    
//     // Read 6 bytes starting from OUTX_L_G
//     i2c_write_blocking(sensor->i2c_inst, sensor->addr, &reg, 1, true);
//     i2c_read_blocking(sensor->i2c_inst, sensor->addr, buffer, 6, false);
    
//     // Combine low and high bytes (little-endian)
//     data->x = (int16_t)(buffer[1] << 8 | buffer[0]);
//     data->y = (int16_t)(buffer[3] << 8 | buffer[2]);
//     data->z = (int16_t)(buffer[5] << 8 | buffer[4]);

//     // Apply Calibration Offsets
//     data->x -= sensor->offset_x_gyro;
//     data->y -= sensor->offset_y_gyro;
//     data->z -= sensor->offset_z_gyro;
// }

// /* Calibrate the Gyroscope */
// // It is a blocking function that reads 6 bytes from the OUTX_L_G register 1000 times (can be changed for different accuracy)
// // It calculates the offsets by averaging the raw values
// // It stores the offsets in the sensor structure
// void lsm6dsl_calibrate_gyro(lsm6dsl_t *sensor)
// {
//     uint8_t buffer[6];
//     uint8_t reg = LSM6DSL_OUTX_L_G;
//     int32_t sum_x = 0, sum_y = 0, sum_z = 0;  
    
//     for (int i = 0; i < CALIBRATION_DATA_SIZE; i++) {
//         // Read raw data directly (bypass calibration)
//         i2c_write_blocking(sensor->i2c_inst, sensor->addr, &reg, 1, true);
//         i2c_read_blocking(sensor->i2c_inst, sensor->addr, buffer, 6, false);
        
//         int16_t x = (int16_t)(buffer[1] << 8 | buffer[0]);
//         int16_t y = (int16_t)(buffer[3] << 8 | buffer[2]);
//         int16_t z = (int16_t)(buffer[5] << 8 | buffer[4]);
        
//         sum_x += x;
//         sum_y += y;
//         sum_z += z;
//     }
    
//     // Calculate offsets
//     sensor->offset_x_gyro = sum_x / CALIBRATION_DATA_SIZE;
//     sensor->offset_y_gyro = sum_y / CALIBRATION_DATA_SIZE;
//     sensor->offset_z_gyro = sum_z / CALIBRATION_DATA_SIZE;
// }

