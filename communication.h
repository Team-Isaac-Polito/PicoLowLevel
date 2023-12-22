#ifndef COMMUNICATION_H
#define COMMUNICATION_H

// CAN bus packet identifiers

#define BATTERY_VOLTAGE       0x11
#define BATTERY_PERCENT       0x12
#define BATTERY_TEMPERATURE   0x13
#define MOTOR_SETPOINT        0x21
#define MOTOR_FEEDBACK			0x22
#define JOINT_YAW_FEEDBACK    0x32

// TODO: update to ROS2 equivalent
#define DATA_PITCH            0x04
#define DATA_EE_PITCH         0x0B
#define DATA_EE_ROLL          0x0C
#define DATA_EE_PITCH2        0x0D

#endif
