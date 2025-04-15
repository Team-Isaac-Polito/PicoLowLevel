# Work In Progress – Motors Monitoring: Current and Temperature

## Current State
- [x] Tested current sensor and thermistor individually
- [x] Added new methods to the `SmartMotor` class (`getCurrent` and `getTemperature`)
- [x] Transmitting data from the new methods via CAN
- [x] Possible adjustments after on-field testing
- [ ] Set current and temperature limits and stop motors when exceeded
- [x] Add external ADC
- [ ] Implement error codes for different scenarios
- [ ] Possible adjustments after on-field testing

## Notes
- A redundant `analogReadResolution` was necessary in `SmartMotor` because `analogRead` in the methods of `SmartMotor` did not use the resolution set in the `.ino` file.
- I am printing the current and temperature values sent over CAN to the serial monitor. Currently, they are coming from the same sensors referring to the same motor.
> ⚠️ **This code is not tested** because, after the upload, the USB is no longer recognized in BOOTSEL mode. The reason for this needs to be investigated.

 



