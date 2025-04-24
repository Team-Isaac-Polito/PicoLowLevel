# Work In Progress – Motors Monitoring: Current and Temperature

## Current State
- [x] Tested current sensor and thermistor individually
- [x] Added new methods to the `SmartMotor` class (`getCurrent` and `getTemperature`)
- [x] Transmitting data from the new methods via CAN
- [x] Possible adjustments after on-field testing
- [x] Set current and temperature limits and stop motors when exceeded
- [x] Add external ADC (ADS1115)
- [x] Change `getTemperature()`, `getCurrent()`, `update()` methods in `SmartMotor` class in order to set limits
- [ ] Adjustments after on-field testing

## Notes

- A redundant call to `analogReadResolution` was necessary in the `SmartMotor` class because `analogRead()` inside class methods did not inherit the resolution set in the `.ino` file. This may be removed if it's confirmed to be unnecessary when using ADS1115 methods.
- I am currently logging the current and temperature values transmitted over CAN to the serial monitor.
- The ADS1115 also features an ALERT pin that can be triggered when a threshold is exceeded, either immediately or after a defined time window. This pin can be used as an interrupt, which is an interesting feature. However, it can only be assigned to one channel at a time. For this reason, I am currently managing threshold checks directly within the `getTemperature()` and `getCurrent()` methods.
- After the latest update, current and temperature values are printed during each update cycle in the `SmartMotor` class for debugging purposes. The current thresholds are now set to 5 A, 4 A, and 3 A, while the temperature threshold is 50 °C.
- The `safe_mode` variable indicates whether a safety mechanism is active and, if so, which one. If `safe_mode` is set to 1 (overheating), the reference passed to the PID is zeroed. If it is set to 2 (overcurrent), the reference value is scaled down (through  `division_factor`) based on the severity of the current:
    - slightly over 3 A → ¼ of the original reference
    - slightly over 4 A → ⅓ of the original reference
    - above 5 A → ½ of the original reference
- All these cases raise the warning flag, with longer persistence required at lower thresholds. Both `getTemperature()` and `getCurrent()` include logic to lower the flag once values return below a safe margin, to prevent mode switching due to small oscillations near the threshold.
- **All current and temperature limits are indicative and will need to be adjusted based on real-world testing.**



> ⚠️ **This code has not been tested** because after uploading, the USB is no longer recognized in BOOTSEL mode. Commenting out `sendFeedback()` makes it work.

 



