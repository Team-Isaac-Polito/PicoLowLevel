# Work In Progress – Motors Monitoring: Current and Temperature

## Current State
- [x] Tested current sensor and thermistor individually
- [x] Added new methods to the `SmartMotor` class (`getCurrent` and `getTemperature`)
- [x] Transmitting data from the new methods via CAN
- [x] Possible adjustments after on-field testing
- [ ] Set current and temperature limits and stop motors when exceeded
- [ ] Add external ADC
- [ ] Implement error codes for different scenarios
- [ ] Possible adjustments after on-field testing

## Notes
- A redundant `analogReadResolution` was necessary in `SmartMotor` because `analogRead` in the methods of `SmartMotor` did not use the resolution set in the `.ino` file.
- I am printing the current and temperature values sent over CAN to the serial monitor. Currently, they are coming from the same sensors referring to the same motor.
- Currently, I am testing only one motor, so the Raspberry Pi Pico's ADC is sufficient. Since 4 ADC channels are needed (one thermistor and one current sensor for each motor), the idea is to use an external ADC. One possibility is the ADS1115, which is I2C, has 4 single-ended channels, and a comparator. There is an ALERT output pin that is interesting because it activates if the analog value exceeds a certain threshold.



