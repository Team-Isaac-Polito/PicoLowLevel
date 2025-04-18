# Work In Progress – Motors Monitoring: Current and Temperature

## Current State
- [x] Tested current sensor and thermistor individually
- [x] Added new methods to the `SmartMotor` class (`getCurrent` and `getTemperature`)
- [x] Transmitting data from the new methods via CAN
- [x] Possible adjustments after on-field testing
- [x] Set current and temperature limits and stop motors when exceeded
- [x] Add external ADC (ADS1115)
- [ ] Implement error codes for different scenarios
- [ ] Possible adjustments after on-field testing

## Notes
- A redundant `analogReadResolution` was necessary in `SmartMotor` because `analogRead` in the methods of `SmartMotor` did not use the resolution set in the `.ino` file. Probably it will be removed if I find out this is unnecessary with ADS1115's methods.
- I am currently logging the current and temperature values transmitted over CAN to the serial monitor.
- At the moment, warnings are triggered when the current exceeds 5 A for 100 consecutive readings (equivalent to 1 second), and when the temperature exceeds 60 °C for 5 consecutive readings (5 seconds). These thresholds will be fine-tuned based on field testing.
- The ADS1115 also features an ALERT pin that can signal when a threshold is exceeded, either immediately or after a specified time window. It can be used as an interrupt, which is an interesting feature. However, it can only be associated with one channel at a time. For this reason, I am currently handling threshold checks directly within the `getTemperature()` and `getCurrent()` methods.

> ⚠️ **This code has not been tested** because after uploading, the USB is no longer recognized in BOOTSEL mode. Commenting out `sendFeedback()` makes it work.

 



