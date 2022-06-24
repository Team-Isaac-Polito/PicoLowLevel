#include "DynamixelSerial.h"

void DynamixelInterface::writeByte(byte b) {
  serialPort->write(b);
  while (serialPort->available() < 1);
  serialPort->read();
}

bool DynamixelInterface::waitBytes(int n) {
  int tc = 0;
  while ((serialPort->available() < n) && (tc < AX_TIME_OUT)) {
    tc++;
    delayMicroseconds(1000);
  }
  
  return tc >= AX_TIME_OUT;
}

void DynamixelInterface::writeMode() {
  gpio_set_function(PIN_SERIAL1_TX, GPIO_FUNC_UART);
}

void DynamixelInterface::readMode() {
  gpio_set_function(PIN_SERIAL1_TX, GPIO_FUNC_SIO);
}

void DynamixelInterface::writeBuf(byte id, byte* buf, int len) {
  byte out[10] = {AX_START, AX_START, id, (byte)(len+1)};
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

int DynamixelInterface::readWord() {
  int out = -1;
  waitBytes(7);

  while (serialPort->available() > 0) {
    byte in = serialPort->read();
    if (in==255 && serialPort->peek()==255) {
      serialPort->read();                            // Start Bytes
      serialPort->read();                            // Ax-12 ID
      serialPort->read();                            // Length
      byte status = serialPort->read();

      if (status!=0) out = -status;
      else out = serialPort->read();
    }
  }
  return out;
}

int DynamixelInterface::readDWord() {
  int out = -1;
  waitBytes(8);

  while (serialPort->available() > 0) {
    byte in = serialPort->read();
    if (in == 255 && serialPort->peek() == 255) {
      serialPort->read();  // Start Bytes
      serialPort->read();  // Ax-12 ID
      serialPort->read();  // Length
      byte status = serialPort->read();

      if (status != 0) out = -status;
      else out = serialPort->read() | (serialPort->read() << 8);
    }
  }
  return out;
}

int DynamixelInterface::readStatus() {
  int out = -1;
  waitBytes(6);
  
  while (serialPort->available() > 0) {
    byte in = serialPort->read();
    if ( (in == 255) & (serialPort->peek() == 255) ) {
      serialPort->read();                                    // Start Bytes
      serialPort->read();                                    // Ax-12 ID
      serialPort->read();                                    // Length
      out = serialPort->read();
    }
  }
  return out;
}

void DynamixelInterface::setSerial(HardwareSerial *sPort) {
  serialPort = sPort;
}

void DynamixelInterface::begin(long baud) {
  serialPort->begin(baud);
  readMode();
}

void DynamixelInterface::end() {
  serialPort->end();
}

int DynamixelClass::reset(unsigned char ID) {
  byte cmd[] = {AX_RESET};
  Dynamixel.writeBuf(ID, cmd, 1);
  return Dynamixel.readStatus();
}

int DynamixelClass::ping(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_PING};
  Dynamixel.writeBuf(ID, cmd, 2);
  return Dynamixel.readStatus();
}

int DynamixelClass::setID(unsigned char ID, unsigned char newID) {
  byte cmd[] = {AX_WRITE_DATA, AX_ID, newID};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelClass::setBD(unsigned char ID, long baud) {
  unsigned char Baud_Rate = (2000000 / baud) - 1;
  
  byte cmd[] = {AX_WRITE_DATA, AX_BAUD_RATE, Baud_Rate};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelClass::move(unsigned char ID, int Position) {
  byte pos_l = Position >> 8;
  byte pos_h = Position;

  byte cmd[] = {AX_WRITE_DATA, AX_GOAL_POSITION_L, pos_l, pos_h};
  Dynamixel.writeBuf(ID, cmd, 4);
  return Dynamixel.readStatus();
}

int DynamixelClass::moveSpeed(unsigned char ID, int Position, int Speed) {
  byte pos_l = Position >> 8;
  byte pos_h = Position;
  byte speed_l = Speed >> 8;
  byte speed_h = Speed;

  byte cmd[] = {AX_WRITE_DATA, AX_GOAL_POSITION_L, pos_l, pos_h, speed_l, speed_h};
  Dynamixel.writeBuf(ID, cmd, 6);
  return Dynamixel.readStatus();
}

int DynamixelClass::setEndless(unsigned char ID, bool Status) {
  int al_ccw = Status?0:AX_DEFAULT_CCW_AL; // endless mode is enabled when both limits are set to 0
  if(!Status) turn(ID, 0, 0); // stop the motor before disabling endless mode

  return setAngleLimit(ID, 0, al_ccw);
}

int DynamixelClass::turn(unsigned char ID, bool SIDE, int Speed) {
  byte speed_l = Speed >> 8;
  byte speed_h = Speed;
  // side 0 is left
  if (SIDE!=0) speed_h += 4; // ???

  byte cmd[] = {AX_WRITE_DATA, AX_GOAL_SPEED_L, speed_l, speed_h};
  Dynamixel.writeBuf(ID, cmd, 4);
  return Dynamixel.readStatus();
}

void DynamixelClass::action() {
  byte cmd[] = {AX_ACTION};
  Dynamixel.writeBuf(AX_BROADCAST_ID, cmd, 1);
}

int DynamixelClass::enableTorque( unsigned char ID, bool Status) {
  byte cmd[] = {AX_WRITE_DATA, AX_TORQUE_ENABLE, Status};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelClass::enableLED(unsigned char ID, bool Status) {
  byte cmd[] = {AX_WRITE_DATA, AX_LED, Status};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelClass::readTemperature(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_TEMPERATURE, 1};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readWord();
}

int DynamixelClass::readPosition(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_POSITION_L, 2};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readDWord();
}

int DynamixelClass::readVoltage(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_VOLTAGE, 1};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readWord();
}

int DynamixelClass::setTempLimit(unsigned char ID, unsigned char Temperature) {
  byte cmd[] = {AX_WRITE_DATA, AX_LIMIT_TEMPERATURE, Temperature};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelClass::setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage) {
  byte cmd[] = {AX_WRITE_DATA, AX_DOWN_LIMIT_VOLTAGE, DVoltage, UVoltage};
  Dynamixel.writeBuf(ID, cmd, 4);
  return Dynamixel.readStatus();
}

int DynamixelClass::setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit) {
  byte al_cw_h = CWLimit >> 8;
  byte al_cw_l = CWLimit;
  byte al_ccw_h = CCWLimit >> 8;
  byte al_ccw_l = CCWLimit;

  byte cmd[] = {AX_WRITE_DATA, AX_CW_ANGLE_LIMIT_L, al_cw_l, al_cw_h, AX_CCW_ANGLE_LIMIT_L, al_ccw_l, al_ccw_h};
  Dynamixel.writeBuf(ID, cmd, 7);
  return Dynamixel.readStatus();
}

int DynamixelClass::setMaxTorque(unsigned char ID, int MaxTorque) {
  byte torque_h = MaxTorque >> 8;
  byte torque_l = MaxTorque;

  byte cmd[] = {AX_WRITE_DATA, AX_MAX_TORQUE_L, torque_l, torque_h};
  Dynamixel.writeBuf(ID, cmd, 4);
  return Dynamixel.readStatus();
}

int DynamixelClass::setSRL(unsigned char ID, unsigned char SRL) {
  byte cmd[] = {AX_WRITE_DATA, AX_RETURN_LEVEL, SRL};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelClass::setRDT(unsigned char ID, unsigned char RDT) {
  byte cmd[] = {AX_WRITE_DATA, AX_RETURN_LEVEL, (byte)(RDT/2)};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelClass::setLEDAlarm(unsigned char ID, unsigned char LEDAlarm) {
  byte cmd[] = {AX_WRITE_DATA, AX_ALARM_LED, LEDAlarm};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelClass::setShutdownAlarm(unsigned char ID, unsigned char SALARM) {
  byte cmd[] = {AX_WRITE_DATA, AX_ALARM_SHUTDOWN, SALARM};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelClass::setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin) {
  byte cmd[] = {AX_WRITE_DATA, AX_CW_COMPLIANCE_MARGIN, CWCMargin, AX_CCW_COMPLIANCE_MARGIN, CCWCMargin};
  Dynamixel.writeBuf(ID, cmd, 5);
  return Dynamixel.readStatus();
}

int DynamixelClass::setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope) {
  byte cmd[] = {AX_WRITE_DATA, AX_CW_COMPLIANCE_SLOPE, CWCSlope, AX_CCW_COMPLIANCE_SLOPE, CCWCSlope};
  Dynamixel.writeBuf(ID, cmd, 5);
  return Dynamixel.readStatus();
}

int DynamixelClass::setPunch(unsigned char ID, int Punch) {
  byte punch_h = Punch >> 8;
  byte punch_l = Punch;

  byte cmd[] = {AX_WRITE_DATA, AX_PUNCH_L, punch_l, punch_h};
  Dynamixel.writeBuf(ID, cmd, 4);
  return Dynamixel.readStatus();
}

int DynamixelClass::moving(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_MOVING, 1};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readWord();
}

int DynamixelClass::lockRegister(unsigned char ID) {
  byte cmd[] = {AX_WRITE_DATA, AX_LOCK, true};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readStatus();
}

int DynamixelClass::RWStatus(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_REGISTERED_INSTRUCTION, 1};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readWord();
}

int DynamixelClass::readSpeed(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_SPEED_L, 2};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readDWord();
}

int DynamixelClass::readLoad(unsigned char ID) {
  byte cmd[] = {AX_READ_DATA, AX_PRESENT_LOAD_L, 2};
  Dynamixel.writeBuf(ID, cmd, 3);
  return Dynamixel.readDWord();
}


DynamixelInterface Dynamixel;
