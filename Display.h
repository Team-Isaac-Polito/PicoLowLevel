#ifndef Display_h
#define Display_h


#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

#include "definitions.h"

Adafruit_SH1106G display = Adafruit_SH1106G(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire1, -1);

class Display {
public:
  Display(String configFile) : confFile(configFile), server(80) {}
  void begin(const char* ssid, const char* password, const char* hostname);
  void handle();
private:
  void setupServer();
  void setupOTA(const char* hostname);

  void handleUpdate();
  void handleConfig();

  String readFile(String path);

  String confFile;
  File tempFile;
  WebServer server;
};

#endif
