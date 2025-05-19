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
      if (change) showCurrentError(idx);
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
 * Added errors to the list are shown one by one with each button press.
 * If no error is present, the button press is counted as a menu change.
 */
void Display::okInterrupt() {
  int now = millis();
  if (now - lastok > DEBOUNCE) {
    if (menupos == 4 && errorCount > 0) {     // in the error menu and there are errors
      idx = (idx + 1) % errorCount;       // cycle through errors with each press
      showCurrentError(idx);                // Show the next error message 
    } else {
    ok++;
    }
    lastok = now;
  }
}

/* CURRENTLY UNUSED
 * Displays an error message on the screen.
 * If the message is too long, it will be split into multiple lines.  
 * The function also shows the CAN ID and data if provided.
 */
void Display::showError(const char* errorMsg, int cursorY) {
  static int currentY = 0;                  // Keeps track of the current Y position for 
                                            // the next error message

  if (currentY + 30 > display.height()) {   // If the display is full, clear it and reset                                        
    display.clearDisplay();                 // the position
    currentY = 0;
  }

  display.setCursor(0, currentY);
  display.printf(errorMsg);

  if (errorCount > 1) {
    // Drawing arrow to indicate that there are more errors
    // Meaning: (Need to press OK button) 
    // right arrow
    display.drawBitmap(display.width() - 10, display.height() - 10, bitmap_arrow_right, 8, 8, 1);
  }

  currentY += 30; // Move to the next line for the next error message
  display.display();
}

/*
 * Displays the current error message based on the index.
 * Shows two errors at once: the current and the next one below.
 * If index is even and there is a next error, shows both; if not, shows only one.
 * When index reaches the end, wraps around to show from the beginning.
 */
void Display::showCurrentError(int idx) {
    if (errorCount == 0) {
        display.clearDisplay();
        display.setCursor(0, 16);
        display.printf("No errors.");
        display.display();
        return;
    } else if (errorTopPrinted && errorBottomPrinted) {
        errorTopPrinted = false;
        errorBottomPrinted = false;
        display.clearDisplay();
    } 


    // Önce üst error yazılmadıysa yaz ve çık
    if (!errorTopPrinted) {
        display.clearDisplay();
        if (errorCount > 1) {
          // Draw right arrow to indicate more errors are available
          display.drawBitmap(display.width() - 10, display.height() - 10, bitmap_arrow_right, 8, 8, 1);
        }
        display.setCursor(0, 0);
        display.printf("[%d] %s", idx, errorList[idx].errorMsg);
        errorTopPrinted = true;
        display.display();
        return;
    }

    // Sonra alt error yazılmadıysa yaz ve çık
    if (!errorBottomPrinted && (idx < errorCount)) {
        display.setCursor(0, 30);
        display.printf("[%d] %s", idx, errorList[idx].errorMsg);
        errorBottomPrinted = true;
        display.display();
        return;
    }
    
}

/*
 * Adds an error message to the list of errors.
 */
void Display::addError(const char* errorMsg, int cursorY) {
  // Check if the error message is a CAN bus error
  bool isCanBusError = (strstr(errorMsg, "CAN") != nullptr);

  // If it's a CAN bus error, check if one already exists
  if (isCanBusError) {
    for (int i = 0; i < errorCount; ++i) {
      if (strstr(errorList[i].errorMsg, "CAN") != nullptr) {
        // Already have a CAN bus error, do not add another
        return;
      }
    }
  }

  if (errorCount < 10) { // Check if there's space for a new error
    errorList[errorCount].errorMsg = errorMsg;
    errorList[errorCount].cursorY = cursorY;
    errorCount++;
  } else {
    // Error list is full, ignore the new error
    Serial.println("Error list is full. Cannot add new error.");
  }
}
