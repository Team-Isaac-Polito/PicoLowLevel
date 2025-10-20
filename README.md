# General Description
This repository contains the low-level project used during the RoboCupRescue 2025 competition. The Dynamixel XM540-W270-T motors were used for traction, while the Dynamixel XL430-W250-T and XM540-W270-T were used for the robotic arm.

## ⚠️ Important Warning: Motor Zeroing

If the arm motors are disconnected and reconnected, the reference position `0` changes.  
To correctly set the zero position, follow the steps below before using the main control code.

## 0. Ensure all required dependencies from the start guide are installed

## 1. Read the Current Positions

1. Open the `dxl_get_position` folder in VS Code, then compile by running **make compile** from the terminal in that folder.  
2. Upload the program to the robot by running **make upload bootsel** from the terminal in the `dxl_get_position` folder after connecting the Pico of the first module in BOOTSEL mode.  
3. Open the serial monitor.  
4. The current motor positions will be printed on the screen.

**Example serial output:**
```

getpositions0 = 2209;
getpositions0 = 1451;
pos0_mot_2 = 4746;
pos0_mot_3 = 987;
pos0_mot_4 = 3121;
pos0_mot_5 = 1979;
pos0_mot_6 = 0;

```

***

## 2. Copy the Values into the Main Code

1. Open the `picolowlevel` folder and the `picolowlevel.ino` file.  
2. Locate the `MODC_ARM_INIT()` function.  
3. Paste the values read from the serial monitor at the end of the function, replacing any previous ones.

**Example of insertion:**
```

getpositions0 = 2209;
getpositions0 = 1451;
pos0_mot_2 = 4746;
pos0_mot_3 = 987;
pos0_mot_4 = 3121;
pos0_mot_5 = 1979;
pos0_mot_6 = 0;

```

4. Upload the main code.  
   After updating the `modc_arm_init()` function, you can compile and upload `picolowlevel.ino` to the robot. The arm will now be correctly zeroed and ready for use.

# Guide to using the Makefile
The guide to using the Makefile is available inside the repository MAKEFILE-ARDUINO-CLI, accessible at the following link.
https://github.com/Team-Isaac-Polito/MAKEFILE-ARDUINO-CLI
