#include "Display.h"

Display::Display() {
  
}

/*
 * Initialization of the display.
 * Sets basic graphic settings and shows the team's logo.
 */
void Display::begin() {
  display.begin(DISPLAY_ADDR, true);
  display.setRotation(2);
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.clearDisplay();
  display.display();
  showLogo(); // Show the logo on startup
}

/**
 * Displays the team's logo.
 */
void Display::showLogo() {
  display.clearDisplay();
  display.drawBitmap(44, 4,  bitmap_logo_isaac, 41, 58, 1);
  display.display();
}

/**
 * Displays WiFi status and properties.
 */
void Display::showWifi() {
  display.clearDisplay();
  display.drawBitmap(0, 0,  bitmap_logo_wifi, 26, 21, 1);
  display.setCursor(0, 32);
  display.printf("IP: %s\n", WiFi.localIP().toString().c_str());
  display.display();
}

/**
 * Displays battery status.
 */
void Display::showBattery() {
  display.clearDisplay();
  display.drawBitmap(0, 0,  bitmap_logo_bat, 23, 11, 1);
  display.setCursor(0, 24);
  display.printf("Voltage:  %.2fV\n\n", battery.readVoltage());
  display.printf("Charge:   %.1f%%\n", battery.chargePercent());
  display.display();
}

/**
 * Display software version, and module address.
 */
void Display::showVersion() {
  display.clearDisplay();
  display.drawBitmap(0, 0,  bitmap_logo_upd, 24, 24, 1);
  display.setCursor(0, 32);
  display.printf("Version: %s\n\n", VERSION);
  display.printf("Can ID:  %#04X",CAN_ID);
  display.display();
}

/**
 * Handles display via recorded interrupts. 
 * This function needs to be called as often as possible.
 * If no button is pressed for more than #MENUTIMEOUT automatically returns to the logo.
 */
void Display::handleGUI() {
  bool change = false;

  if(nav > 0) {
    change = true;
    nav--;
    menupos++;
    if (menupos >= NMENUS) menupos = 0;
    else menutime = millis();
  } else if(menupos != 0 && millis() - menutime > (MENUTIMEOUT * 1000)) {
    change = true;
    menupos = 0;
  }

  switch (menupos) {
    case 0:
      if(change) showLogo();
      break;
    case 1:
      if (change) showBattery();
      break;
    case 2:
      if(change) showWifi();
      break;
    case 3:
      if (change) showVersion();
      break;
    case 4:
      if (change) showCurrentError(index);
      break;
  }
}

/**
 * NAV button ISR.
 */
void Display::navInterrupt() {
  int now = millis();
  if (now - lastnav > DEBOUNCE) {
    nav++;
    lastnav = now;
  }
}


/**
 * OK button ISR.
 */
void Display::okInterrupt() {
  int now = millis();
  if (now - lastok > DEBOUNCE) {
    if (menupos == 4 && errorCount > 0) { // in the error menu
      index = (index + 1) % errorCount; // cycle through errors
      showCurrentError(index); // Show the next error message
    } else {
    ok++;
    }
    lastok = now;
  }
}

void Display::showError(const char* errorMsg, int cursorY, const unsigned char* errorMsgCANID, const byte* errorMsgCANData) {
  static int currentY = 0; // Keeps track of the current Y position for the next error message

  if (currentY + 30 > display.height()) { // If the display is full, clear it and reset the position
    display.clearDisplay();
    currentY = 0;
  }

  display.setCursor(0, currentY);
  display.printf(errorMsg);

  if (errorMsgCANID != nullptr) {
      display.setCursor(0, currentY + 10);
      display.printf("CAN ID: ");
      display.print(*errorMsgCANID, HEX);
  }

  if (errorMsgCANData != nullptr) {
      display.setCursor(0, currentY + 20);
      display.print("CAN Data: ");
      for (int i = 0; i < 8; i++) {
          display.print(errorMsgCANData[i], HEX);
          display.printf(" ");
      }
  }

  if (errorCount > 1) {
    // left arrow 
    display.drawBitmap(0, display.height() - 5, bitmap_arrow_left_down, 5, 5, 1);
    // right arrow
    display.drawBitmap(display.width() - 5, display.height() - 5, bitmap_arrow_right_down, 5, 5, 1);
  }

  currentY += 30; // Move to the next line for the next error message
  display.display();
}

void Display::showCurrentError(int index) {
  if (index >= 0 && index < errorCount) {
    const Error& currentError = errorList[index];
    showError(currentError.errorMsg, currentError.cursorY, currentError.errorMsgCANID, currentError.errorMsgCANData);
  } else {
    display.clearDisplay();
    display.setCursor(0, 16);
    display.print("No errors.");
    display.display();
  }
}

void Display::addError(const char* errorMsg, int cursorY, const unsigned char* errorMsgCANID, const byte* errorMsgCANData) {
  if (errorCount < 10) { // Check if there's space for a new error
    errorList[errorCount].errorMsg = errorMsg;
    errorList[errorCount].cursorY = cursorY;
    errorList[errorCount].errorMsgCANID = errorMsgCANID;
    errorList[errorCount].errorMsgCANData = errorMsgCANData;
    errorCount++;
  } else {
    // Handle the case where the error list is full (e.g., overwrite the oldest error or ignore the new one)
    // we'll just ignore the new error in this example.
    Serial.println("Error list is full. Cannot add new error.");
  }
}
