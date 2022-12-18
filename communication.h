#ifndef COMMUNICATION_H
#define COMMUNICATION_H

// Commands definition

#define DATA_TRACTION_LEFT          0x01
#define DATA_TRACTION_RIGHT         0x02
#define DATA_YAW                    0x03
#define DATA_PITCH                  0x04

#define SEND_STATUS                 0x05                
#define SEND_IMU_DATA               0x06
#define SEND_YAW_ENCODER            0x07    
#define SEND_TRACTION_LEFT_SPEED    0x08 
#define SEND_TRACTION_RIGHT_SPEED   0x09            
#define SEND_BATTERY_VOLTAGE        0x0A
#define SEND_CURRENT                0x0E
#define SEND_TEMPERATURE            0x0F

#define DATA_EE_PITCH               0x0B
#define DATA_EE_ROLL                0x0C
#define DATA_EE_PITCH2              0x0D

#define DATA_PID_KP                 0x10
#define DATA_PID_KI                 0x11
#define DATA_PID_KD                 0x12

#endif