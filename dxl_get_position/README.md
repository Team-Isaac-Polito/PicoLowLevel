
# MK2 Robot – Reading Initial Robotic Arm Positions

## Description

This program is designed for the **MK2 robot module 1 with a robotic arm**.
Its main purpose is to read the **current positions of the arm motors** and print them on the serial monitor.

This step is essential when the motors are **disconnected and reconnected**, as the reference position `0` may change. Once the positions are read, they can be copied into the main code to properly calibrate the arm.

***

## Operation

1. The code uses the `Dynamixel_ll` library to communicate with the Dynamixel motors via **Serial1**.
2. The main motors are defined as:
    - `dxl`, `mot_Left_1`, `mot_Right_1` → main traction motors
    - `mot_2` … `mot_6` → robotic arm motors
3. During `setup()`:
    - Communication with the PC and motors is initialized at **2 Mbps**
    - Torque is disabled (`TorqueEnable = false`) for safety
    - **Operating modes** (Extended Position Mode) are set
    - **Sync mode** is enabled for simultaneous multi-motor control
    - Debug mode is disabled
4. During `loop()`:
    - The current motor positions are read (`getPresentPosition`)
    - Values are printed to the serial monitor in a readable format, for example:

```
pos0_mot_2 = 4746;
pos0_mot_3 = 987;
...
```

    - The read positions can then be copied into the `modc_arm_init()` function of the main code `picolowlevel.ino` to correctly zero the arm.

***

## Usage Instructions

1. Open the program in VS Code.
2. Connect the Pico of the first module to the PC in BOOTSEL mode.
3. Edit the Makefile to insert the correct Pico board name:
`BOARD_FQBN ?= rp2040:rp2040:rpipico` or
`BOARD_FQBN ?= rp2040:rp2040:rpipicow`, depending on your Pico device.
4. Edit the Makefile to set the correct drive letter in
`DESTINATION ?= 'D:\'` based on how your PC detects the external Pico drive.
5. In the terminal, run **make compile** to build the project and upload it to the Pico with **make upload bootsel**.
6. Open the serial monitor at **115200 baud**.
7. Record or copy the printed values.
8. Insert these values into the `MODC_ARM_INIT()` function inside `picolowlevel > picolowlevel.ino` before executing any arm movement.

# Guide to using the Makefile
The guide to using the Makefile is available inside the repository MAKEFILE-ARDUINO-CLI, accessible at the following link.
https://github.com/Team-Isaac-Polito/MAKEFILE-ARDUINO-CLI