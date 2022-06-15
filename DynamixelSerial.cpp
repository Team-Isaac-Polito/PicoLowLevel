#include "DynamixelSerial.h"

void DynamixelClass::writeByte(byte b) {
  serialPort->write(b);
  while (serialPort->available() < 1);
  serialPort->read();
}

bool DynamixelClass::waitBytes(int n) {
  int tc = 0;
  while ((serialPort->available() < n) && (tc < TIME_OUT)) {
    tc++;
    delayMicroseconds(1000);
  }
  
  return tc >= TIME_OUT;
}

void DynamixelClass::writeMode() {
  gpio_set_function(PIN_SERIAL1_TX, GPIO_FUNC_UART);
}

void DynamixelClass::readMode() {
  gpio_set_function(PIN_SERIAL1_TX, GPIO_FUNC_SIO);
}

void DynamixelClass::writeBuf(unsigned char ID, byte* buf, int len) {
  byte out[10] = {AX_START, AX_START, ID, (byte)(len+1)};
  int n = 4;

  for(int i = 0; i < len; i++) out[i+n] = buf[i];
  n+=len;

  byte checksum = 0;
  for(int i = 2; i < n; i++) checksum += out[i];
  out[n++] = ~checksum & 0xFF;

  writeMode();
  serialPort->write(out, n);
  while (serialPort->available() < n);
  for(int i = 0; i < n; i++) serialPort->read();
  readMode();
}

int DynamixelClass::readWord() {
  int out = -1;
  waitBytes(7);

  while (serialPort->available() > 0) {
    unsigned char in = serialPort->read();
    if (in==255 && serialPort->peek()==255) {
      serialPort->read();                            // Start Bytes
      serialPort->read();                            // Ax-12 ID
      serialPort->read();                            // Length
      unsigned char status = serialPort->read();

      if (status!=0) out = -status;
      else out = serialPort->read();
    }
  }
  return out;
}

int DynamixelClass::readDWord() {
  int out = -1;
  waitBytes(8);

  while (serialPort->available() > 0) {
    unsigned char in = serialPort->read();
    if (in == 255 && serialPort->peek() == 255) {
      serialPort->read();  // Start Bytes
      serialPort->read();  // Ax-12 ID
      serialPort->read();  // Length
      unsigned char status = serialPort->read();

      if (status != 0) out = -status;
      else out = serialPort->read() | (serialPort->read() << 8);
    }
  }
  return out;
}

int DynamixelClass::readStatus() {
  int out = -1;
  waitBytes(6);
  
  while (serialPort->available() > 0) {
    Incoming_Byte = serialPort->read();
    if ( (Incoming_Byte == 255) & (serialPort->peek() == 255) ) {
      serialPort->read();                                    // Start Bytes
      serialPort->read();                                    // Ax-12 ID
      serialPort->read();                                    // Length
      out = serialPort->read();
    }
  }
  return out;
}

void DynamixelClass::setSerial(HardwareSerial *sPort) {
  serialPort = sPort;
}

void DynamixelClass::begin(long baud) {
  serialPort->begin(baud);
  readMode();
}

void DynamixelClass::end() {
  serialPort->end();
}

int DynamixelClass::reset(unsigned char ID) {
  byte cmd[] = {AX_RESET};
  writeBuf(ID, cmd, 1);
  return readStatus();
}

int DynamixelClass::ping(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_PING};
  writeBuf(ID, cmd, 2);
  return readStatus();
}

int DynamixelClass::setID(unsigned char ID, unsigned char newID) {
  byte cmd[] = {AX_WRITE_DATA, AX_ID, newID};
  writeBuf(ID, cmd, 3);
  return readStatus();
}

int DynamixelClass::setBD(unsigned char ID, long baud) {
  unsigned char Baud_Rate = (2000000 / baud) - 1;
  
  byte cmd[] = {AX_WRITE_DATA, AX_BAUD_RATE, Baud_Rate};
  writeBuf(ID, cmd, 3);
  return readStatus();
}

int DynamixelClass::move(unsigned char ID, int Position) {
  char Position_H, Position_L;
  Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
  Position_L = Position;

  byte cmd[] = {AX_WRITE_DATA, AX_GOAL_POSITION_L, Position_L, Position_H};
  writeBuf(ID, cmd, 4);
  return readStatus();
}

int DynamixelClass::moveSpeed(unsigned char ID, int Position, int Speed) {
  char Position_H, Position_L, Speed_H, Speed_L;
  Position_H = Position >> 8;
  Position_L = Position;                // 16 bits - 2 x 8 bits variables
  Speed_H = Speed >> 8;
  Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables

  byte cmd[] = {AX_WRITE_DATA, AX_GOAL_POSITION_L, Position_L, Position_H, Speed_L, Speed_H};
  writeBuf(ID, cmd, 6);
  return readStatus();
}

int DynamixelClass::setEndless(unsigned char ID, bool Status) {
  if ( Status ) {
    char AX_CCW_AL_LT = 0;     // Changing the CCW Angle Limits for Full Rotation.

    byte cmd[] = {AX_WRITE_DATA, AX_CCW_ANGLE_LIMIT_L, AX_CCW_AL_LT, AX_CCW_AL_LT};
    writeBuf(ID, cmd, 4);
  }
  else {
    turn(ID, 0, 0);

    byte cmd[] = {AX_WRITE_DATA, AX_CCW_ANGLE_LIMIT_L, AX_CCW_AL_L, AX_CCW_AL_H};
    writeBuf(ID, cmd, 4);
  }
  return readStatus();
}

int DynamixelClass::turn(unsigned char ID, bool SIDE, int Speed) {
  char Speed_H, Speed_L;
  Speed_H = Speed >> 8;
  Speed_L = Speed;
  // side 0 is left
  if (SIDE!=0) Speed_H + 4;

  byte cmd[] = {AX_WRITE_DATA, AX_GOAL_SPEED_L, Speed_L, Speed_H};
  writeBuf(ID, cmd, 4);
  return readStatus();
}

int DynamixelClass::moveRW(unsigned char ID, int Position) {
  char Position_H, Position_L;
  Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
  Position_L = Position;

  byte cmd[] = {AX_REG_WRITE, AX_GOAL_POSITION_L, Position_L, Position_H};
  writeBuf(ID, cmd, 4);
  return readStatus();
}

int DynamixelClass::moveSpeedRW(unsigned char ID, int Position, int Speed) {
  char Position_H, Position_L, Speed_H, Speed_L;
  Position_H = Position >> 8;
  Position_L = Position;                // 16 bits - 2 x 8 bits variables
  Speed_H = Speed >> 8;
  Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables

  byte cmd[] = {AX_REG_WRITE, AX_GOAL_POSITION_L, Position_L, Position_H, Speed_L, Speed_H};
  writeBuf(ID, cmd, 6);
  return readStatus();
}

void DynamixelClass::action() {
  byte cmd[] = {AX_ACTION};
  writeBuf(BROADCAST_ID, cmd, 1);
}

int DynamixelClass::torqueStatus( unsigned char ID, bool Status) {
  byte cmd[] = {AX_WRITE_DATA, AX_TORQUE_ENABLE, Status};
  writeBuf(ID, cmd, 3);
  return readStatus();
}

int DynamixelClass::ledStatus(unsigned char ID, bool Status) {
  byte cmd[] = {AX_WRITE_DATA, AX_LED, Status};
  writeBuf(ID, cmd, 3);
  return readStatus();
}

int DynamixelClass::readTemperature(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_TEMPERATURE, AX_BYTE_READ};
  writeBuf(ID, cmd, 3);
  return readWord();
}

int DynamixelClass::readPosition(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_POSITION_L, AX_BYTE_READ_POS};
  writeBuf(ID, cmd, 3);
  return readDWord();
}

int DynamixelClass::readVoltage(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_VOLTAGE, AX_BYTE_READ};
  writeBuf(ID, cmd, 3);
  return readWord();
}

int DynamixelClass::setTempLimit(unsigned char ID, unsigned char Temperature) {
  byte cmd[] = {AX_WRITE_DATA, AX_LIMIT_TEMPERATURE, Temperature};
  writeBuf(ID, cmd, 3);
  return readStatus();
}

int DynamixelClass::setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage) {
  byte cmd[] = {AX_WRITE_DATA, AX_DOWN_LIMIT_VOLTAGE, DVoltage, UVoltage};
  writeBuf(ID, cmd, 4);
  return readStatus();
}

int DynamixelClass::setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit) {
  char CW_H, CW_L, CCW_H, CCW_L;
  CW_H = CWLimit >> 8;
  CW_L = CWLimit;                // 16 bits - 2 x 8 bits variables
  CCW_H = CCWLimit >> 8;
  CCW_L = CCWLimit;

  byte cmd[] = {AX_WRITE_DATA, AX_CW_ANGLE_LIMIT_L, CW_L, CW_H, AX_CCW_ANGLE_LIMIT_L, CCW_L, CCW_H};
  writeBuf(ID, cmd, 7);
  return readStatus();
}

int DynamixelClass::setMaxTorque(unsigned char ID, int MaxTorque) {
  char MaxTorque_H, MaxTorque_L;
  MaxTorque_H = MaxTorque >> 8;           // 16 bits - 2 x 8 bits variables
  MaxTorque_L = MaxTorque;

  byte cmd[] = {AX_WRITE_DATA, AX_MAX_TORQUE_L, MaxTorque_L, MaxTorque_H};
  writeBuf(ID, cmd, 4);
  return readStatus();
}

int DynamixelClass::setSRL(unsigned char ID, unsigned char SRL) {
  byte cmd[] = {AX_WRITE_DATA, AX_RETURN_LEVEL, SRL};
  writeBuf(ID, cmd, 3);
  return readStatus();
}

int DynamixelClass::setRDT(unsigned char ID, unsigned char RDT) {
  byte cmd[] = {AX_WRITE_DATA, AX_RETURN_LEVEL, (byte)(RDT/2)};
  writeBuf(ID, cmd, 3);
  return readStatus();
}

int DynamixelClass::setLEDAlarm(unsigned char ID, unsigned char LEDAlarm) {
  byte cmd[] = {AX_WRITE_DATA, AX_ALARM_LED, LEDAlarm};
  writeBuf(ID, cmd, 3);
  return readStatus();
}

int DynamixelClass::setShutdownAlarm(unsigned char ID, unsigned char SALARM) {
  byte cmd[] = {AX_WRITE_DATA, AX_ALARM_SHUTDOWN, SALARM};
  writeBuf(ID, cmd, 3);
  return readStatus();
}

int DynamixelClass::setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin) {
  byte cmd[] = {AX_WRITE_DATA, AX_CW_COMPLIANCE_MARGIN, CWCMargin, AX_CCW_COMPLIANCE_MARGIN, CCWCMargin};
  writeBuf(ID, cmd, 5);
  return readStatus();
}

int DynamixelClass::setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope) {
  byte cmd[] = {AX_WRITE_DATA, AX_CW_COMPLIANCE_SLOPE, CWCSlope, AX_CCW_COMPLIANCE_SLOPE, CCWCSlope};
  writeBuf(ID, cmd, 5);
  return readStatus();
}

int DynamixelClass::setPunch(unsigned char ID, int Punch) {
  char Punch_H, Punch_L;
  Punch_H = Punch >> 8;           // 16 bits - 2 x 8 bits variables
  Punch_L = Punch;

  byte cmd[] = {AX_WRITE_DATA, AX_PUNCH_L, Punch_L, Punch_H};
  writeBuf(ID, cmd, 4);
  return readStatus();
}

int DynamixelClass::moving(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_MOVING, AX_BYTE_READ};
  writeBuf(ID, cmd, 3);
  return readWord();
}

int DynamixelClass::lockRegister(unsigned char ID) {
  byte cmd[] = {AX_WRITE_DATA, AX_LOCK, LOCK};
  writeBuf(ID, cmd, 3);
  return readStatus();
}

int DynamixelClass::RWStatus(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_REGISTERED_INSTRUCTION, AX_BYTE_READ};
  writeBuf(ID, cmd, 3);
  return readWord();
}

int DynamixelClass::readSpeed(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_SPEED_L, AX_BYTE_READ_POS};
  writeBuf(ID, cmd, 3);
  return readDWord();
}

int DynamixelClass::readLoad(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_LOAD_L, AX_BYTE_READ_POS};
  writeBuf(ID, cmd, 3);
  return readDWord();
}

DynamixelClass Dynamixel;
