#ifndef DEBUG_H
#define DEBUG_H

#include <Arduino.h>
#include "Debug.h"

enum class Levels{
  OFF, WARN, INFO, DEBUG
};

class SerialDebug {
public:
  SerialDebug(HardwareSerial* serial = &Serial) : serial(serial), ended(true), level(Levels::DEBUG) {}
  
  void print(String st, Levels level);
  void println(String st, Levels level);
  void print(String st);
  void println(String st);

  // generic functions, accepting all parameters accepted by String
  template<class T>
  void print(T any, Levels level) {print(String(any), level);}
  template<class T>
  void println(T any, Levels level) {println(String(any), level);}
  template<class T>
  void print(T any) {print(String(any));}
  template<class T>
  void println(T any) {println(String(any));}

  void delayd(int t);
  void setLevel(Levels lvl);
  void setSerial(HardwareSerial* serial);
private:
  HardwareSerial* serial;
  String getLevel(Levels level);
  bool ended;
  Levels level;
};

// default instance, extern so any class can use it
extern SerialDebug Debug;

#endif
