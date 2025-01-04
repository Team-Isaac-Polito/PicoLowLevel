# PicoLowLevel

This is the code running on the board inside each module of Rese.Q MK1.

## Module components

The components inside each modules are:

- Raspberry Pi Pico W
- CAN transceiver, with MCP2515 and TJA1050
- Two  DC motors
  - controlled by Pololu G2 24v13 drivers
  - rotary encoder with 48 pulses per motor rotation
  - 150:1 gearbox
- One to three Dynamixel AX-12A smart servo motors
- AMS AS5048B absolute encoder
- 64*128 OLED display with SH1106 driver

## Building

To build the project you need PlatformIO. We suggest using [PlatformIO for VSCode](https://platformio.org/install/ide?install=vscode).

### Arduino-Pico

This project is based on the Arduino framework, and in particular uses the Raspberry Pi Pico available [here](https://github.com/earlephilhower/arduino-pico).

### Libraries

Libraries are automatically managed by *PlatformIO*, at the moment we are using:
  -  `adafruit/Adafruit SH110X@^2.1.11` (for the OLED display)

### Build options

These are automatically managed by *PlatformIO*. The code requires the *Raspberry Pi Pico W* (`rpipicow`) board and the flash size should be set to `2MB (Sketch: 1MB, FS: 1MB)`, meaning that half of the microcontroller memory will be dedicated to the program itself, while the other half can be used for storing informations and performing over-the-air (OTA) upgrades.

## Uploading Firmware

### Using GitHub Actions Artifacts

To upload the firmware from the GitHub Actions artifacts, follow these steps:

1. Download the firmware artifact from the GitHub Actions page.
2. Put the Raspberry Pi Pico into BOOTSEL mode by holding down the BOOTSEL button while plugging it into your computer.
3. Drag and drop the downloaded firmware file (`uf2`) onto the RPI-RP2 drive that appears.

### Using PlatformIO

PlatformIO can also be used to upload the firmware directly to the Pico when it is not in BOOTSEL mode. Ensure that PlatformIO is installed and available in your system's PATH.

1. Open a terminal and navigate to the project directory.
2. Use the `Upload` button in the PlatformIO panel

### Using Makefile
A Makefile is provided to build and upload all firmware. This requires PlatformIO and make to be available in your system's PATH.

1. Open a terminal and navigate to the project directory.
2. Run the following command to build and upload the firmware:
   ```sh
   make
   ```

This will compile the firmware for all the modules in `.pio/build/mod<i>/firmware.uf2`
