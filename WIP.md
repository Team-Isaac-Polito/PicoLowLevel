# Work In Progress – Motors Monitoring: Current and Temperature

## Current State
- [x] Tested current sensor and thermistor individually
- [x] Added new methods to the `SmartMotor` class (`getCurrent` and `getTemperature`)
- [x] Transmitting data from the new methods via CAN
- [ ] Possible adjustments after on-field testing
- [ ] Set current and temperature limits and stop motors when exceeded
- [ ] Implement error codes for different scenarios
- [ ] Possible adjustments after on-field testing

## Notes
- The values returned by the thermistor (70°C) during the individual test are questionable: need to retest. Also, keep in mind that the safe limit for PLA is 60°C.


