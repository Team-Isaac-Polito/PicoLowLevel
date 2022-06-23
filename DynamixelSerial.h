#ifndef DynamixelSerial_h
#define DynamixelSerial_h

// EEPROM AREA
#define AX_ID                       3
#define AX_BAUD_RATE                4
#define AX_CW_ANGLE_LIMIT_L         6
#define AX_CCW_ANGLE_LIMIT_L        8
#define AX_LIMIT_TEMPERATURE        11
#define AX_DOWN_LIMIT_VOLTAGE       12
#define AX_MAX_TORQUE_L             14
#define AX_RETURN_LEVEL             16
#define AX_ALARM_LED                17
#define AX_ALARM_SHUTDOWN           18

// RAM AREA
#define AX_TORQUE_ENABLE            24
#define AX_LED                      25
#define AX_CW_COMPLIANCE_MARGIN     26
#define AX_CCW_COMPLIANCE_MARGIN    27
#define AX_CW_COMPLIANCE_SLOPE      28
#define AX_CCW_COMPLIANCE_SLOPE     29
#define AX_GOAL_POSITION_L          30
#define AX_GOAL_SPEED_L             32
#define AX_PRESENT_POSITION_L       36
#define AX_PRESENT_SPEED_L          38
#define AX_PRESENT_LOAD_L           40
#define AX_PRESENT_VOLTAGE          42
#define AX_PRESENT_TEMPERATURE      43
#define AX_REGISTERED_INSTRUCTION   44
#define AX_MOVING                   46
#define AX_LOCK                     47
#define AX_PUNCH_L                  48

// Instruction set
#define AX_PING                     1
#define AX_READ_DATA                2
#define AX_WRITE_DATA               3
#define AX_ACTION                   5
#define AX_RESET                    6

// Constants
#define AX_BROADCAST_ID             254
#define AX_START                    255
#define AX_LOCK						1

// Configurations
#define AX_DEFAULT_CCW_AL			1023
#define AX_TIME_OUT					10

#include <Arduino.h>
#include <HardwareSerial.h>

class DynamixelClass {
private:
	HardwareSerial *serialPort;

	unsigned char Checksum; 
	unsigned char Direction_Pin;
	unsigned char Time_Counter;
	unsigned char Incoming_Byte;               

	void writeByte(byte b);
	bool waitBytes(int n);

	void writeBuf(unsigned char ID, byte* buf, int len);
	int readStatus();
	int readWord();
	int readDWord();

	void writeMode();
	void readMode();
	
public:
	void setSerial(HardwareSerial *sPort);
	
	void begin(long baud);
	void end(void);
	
	int reset(unsigned char ID);
	int ping(unsigned char ID); 
	
	int setID(unsigned char ID, unsigned char newID);
	int setBD(unsigned char ID, long baud);
	
	int move(unsigned char ID, int Position);
	int moveSpeed(unsigned char ID, int Position, int Speed);
	int setEndless(unsigned char ID,bool Status);
	int turn(unsigned char ID, bool SIDE, int Speed);
	
	void action(void);
	
	int setTempLimit(unsigned char ID, unsigned char Temperature);
	int setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit);
	int setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage);
	int setMaxTorque(unsigned char ID, int MaxTorque);
	int setSRL(unsigned char ID, unsigned char SRL);
	int setRDT(unsigned char ID, unsigned char RDT);
	int setLEDAlarm(unsigned char ID, unsigned char LEDAlarm);
	int setShutdownAlarm(unsigned char ID, unsigned char SALARM);
	int setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin);
	int setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope);
	int setPunch(unsigned char ID, int Punch);
	
	int moving(unsigned char ID);
	int lockRegister(unsigned char ID);
	int RWStatus(unsigned char ID);
	
	int readTemperature(unsigned char ID);
	int readVoltage(unsigned char ID);
	int readPosition(unsigned char ID);
	int readSpeed(unsigned char ID);
	int readLoad(unsigned char ID);
	
	int enableTorque(unsigned char ID, bool Status);
	int enableLED(unsigned char ID, bool Status);
};

extern DynamixelClass Dynamixel;

#endif
