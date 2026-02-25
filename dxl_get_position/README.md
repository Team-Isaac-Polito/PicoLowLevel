# dxl_get_position - Arm Motor Position Reader

## Description

This utility sketch is designed for the **MK2 robot module 1 with the robotic arm**.
Its purpose is to read the **current positions of the arm motors** and print them to the serial monitor.

This step is essential whenever the motors are **disconnected and reconnected**, because the reference position `0` may change. Once the positions are read, they can be copied into the main firmware to correctly calibrate the arm.

---

## How It Works

1. The code uses the `Dynamixel_ll` library to communicate with the Dynamixel motors via **Serial1**.
2. The motors defined are:
   - `mot_Left_1`, `mot_Right_1` - traction motors
   - `mot_2` ... `mot_6` - robotic arm motors
3. During `setup()`:
   - Serial communication is initialized with the PC and the motors at **2 Mbps**
   - Torque is disabled on all motors (`TorqueEnable = false`) for safety
   - **Operating modes** are set (Extended Position Mode)
   - **Sync mode** is enabled for simultaneous multi-motor control
   - Debug mode is disabled

4. During `loop()`:
   - Current motor positions are read via `getPresentPosition()`
   - Positions are printed to the serial monitor in a readable format
   - The printed values can then be copied into the `MODC_ARM_INIT()` function in `PicoLowLevel.ino` to correctly zero the arm

---

## Usage

1. Open the `dxl_get_position` folder in **VS Code**.
2. Connect the Pico of the first module to the PC in **BOOTSEL** mode.
3. Edit the Makefile:
   - Set `BOARD_FQBN` to `rp2040:rp2040:rpipico` or `rp2040:rp2040:rpipicow` depending on your board.
   - Set `DESTINATION` to the drive letter where the PC detects the Pico (e.g., `'D:\'`).
4. In the terminal, run `make compile` to build, then `make upload bootsel` to flash the Pico.
5. Open a serial monitor at **115200 baud**.
6. Note or copy the printed position values.
7. Paste the values into the `MODC_ARM_INIT()` function in `PicoLowLevel/PicoLowLevel.ino` before running any arm movements.

---

## Notes

- This sketch **does not move the motors** - it only reads initial positions.
- It should be run every time the motors are disconnected, to prevent unexpected arm movements.
- The read interval is 1 second (`delay(1000)`) but can be adjusted if needed.
