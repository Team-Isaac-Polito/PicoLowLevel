# System ID Consensus — Project Walkthrough

This document explains in detail the **System ID Consensus** function and all the changes made to ensure security when replacing a microcontroller (MCU).

---

## 1. Project Goal and Problem
The **Rese.Q MK2** robot has 3 Raspberry Pi Pico W microcontrollers (ARM, Middle, Tail). If the ARM module microcontroller is replaced, the old calibration data (`/home_pos.bin`) will not be available. When the system starts up, the robot arm may assume the wrong position as the starting point and break mechanical parts.

**Solution:** A session ID is exchanged between all MCUs via the CAN bus for 10 seconds. Only the **ARM microcontroller** (`MODC_ARM`) strictly checks the compatibility of these IDs.

---

## 2. Changes and Encoder Structure

The changes were made to the following files:

### A. CAN Protocol Extension
* **[communication.h](file:///c:/Users/Asus TUF/Desktop/ISAAC/PicoLowLevel/PicoLowLevel/include/communication.h)**:
* New CAN message IDs added:
```cpp
#define SYSTEM_ID_EXCHANGE 0xA0 // Session ID exchange (startup)
#define SYSTEM_ID_UPDATE 0xA1 // New session ID (after consensus)
```
* **[CanWrapper.h](file:///c:/Users/Asus TUF/Desktop/ISAAC/PicoLowLevel/PicoLowLevel/lib/Can/src/CanWrapper.h)** and **[CanWrapper.cpp](file:///c:/Users/Asus TUF/Desktop/ISAAC/PicoLowLevel/PicoLowLevel/lib/Can/src/CanWrapper.cpp)**:
* Added `sendBroadcast` method to send broadcast to all modules.
* Added `readBroadcastMessage` method to read broadcast messages.
* MCP2515 filters were configured: **RX0** is reserved only for its module ID, and **RX1** is reserved for `0xFF` (broadcast) messages.

### B. New `SystemID` Library
* **[SystemID.h](file:///c:/Users/Asus TUF/Desktop/ISAAC/PicoLowLevel/PicoLowLevel/lib/SystemID/src/SystemID.h)** and **[SystemID.cpp](file:///c:/Users/Asus TUF/Desktop/ISAAC/PicoLowLevel/PicoLowLevel/lib/SystemID/src/SystemID.cpp)**:
* `loadStoredID()`: Reads the `/sys_id.bin` file from LittleFS memory (returns `0` if not found).
* `generateNewID()`: Creates a new session ID using the hardware unique ID of the RP2040 chip and analog noise.
* `saveID()`: Writes the new ID to LittleFS.
* `waitForConsensus()`: Collects IDs from other boards for 10 seconds and checks for at least 60% match.

### C. Core Software Integration
* **[PicoLowLevel.ino](file:///c:/Users/Asus TUF/Desktop/ISAAC/PicoLowLevel/PicoLowLevel/PicoLowLevel.ino)**:
* At the beginning of `setup()`, a consensus check is performed before the motors start.
* **ARM Module Error and Bypass (Button)**: If the ID does not match on the ARM board, the motors will be blocked, ``MCU MISMATCH!'' will be displayed on the display, and the LED will blink rapidly. Only when the user presses the board's **OK button (`BTNOK`)** will the security be bypassed and allowed to start.
* After consensus is successful, a new session ID is created and updated in the LittleFS memory of all boards.

