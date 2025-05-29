#ifndef Display_h
#define Display_h

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <WiFi.h>

#include "../../../include/definitions.h"
#include "../../../include/bitmap_logos.h"
#include "../../../include/mod_config.h"
#include "../../Battery/src/Battery.h"

/**
 * Display class, handles display itself and GUI.
 */
class Display {
public:
  Display();
  void begin();

  void handleGUI();
  void okInterrupt();
  void navInterrupt();
  void showLogo();
  void showBattery();

  void showVersion();

  int errorCount = 0;  // error count

  /**
   * @brief adds an error message to the list of errors.
   * @param errorMsg Pointer to the error message string.
   * @param cursorY Y-coordinate for the cursor position on the display.
   */
  void addError(const char* errorMsg, int cursorY);

  /**
   * @brief Shows the current error message based on the index.
   * @param idx Index of the error to be displayed.
   * @param errorMsg Pointer to the error message string.
   * @param cursorY Y-coordinate for the cursor position on the display.
   */
  void showError(const char* errorMsg, int cursorY);

private:
struct Error { // Structure to hold error information
    // Pointer to the error message string
    const char* errorMsg;
    int cursorY;
  };

  Error errorList[10];              // max 10 errors
  int idx = 0;                      // index of shown error
  bool errorTopPrinted = false;     // flag to indicate if the top error is printed
  bool errorBottomPrinted = false;  // flag to indicate if the bottom error is printed

  void showWifi();

  Adafruit_SH1106G display = Adafruit_SH1106G(DISPLAY_WIDTH, DISPLAY_HEIGHT, &Wire1, -1);

  //menu handling variables
  int ok = 0;
  int lastok = 0;
  int nav = 0;
  int lastnav = 0;
  int menupos = 0;
  int menutime = 0;
};

#endif
