# PicoLowLevel

Low-level firmware for the **Rese.Q MK2** modular snake-like rescue robot, running on **Raspberry Pi Pico W** boards inside each module.

> **Firmware version:** LL v0.2 (Dynamixel-only traction)
> **Board revision:** mk_v2.2
> **Documentation:** [docs.teamisaac.it](https://docs.teamisaac.it/doc/picolowlevel-WnfPCgwOM6)

---

## Overview

Each Rese.Q module contains a Pico W that communicates with the high-level stack (`reseq_ros2` on Jetson Orin) over CAN bus. The firmware handles:

- **Dynamixel motor control** - traction and arm joints via `Dynamixel_ll` library
- **CAN bus communication** - setpoint reception and feedback transmission (MCP2515 + TJA1050)
- **Sensor reading** - absolute encoders (AMS AS5048B), battery voltage monitoring
- **OLED display** - status GUI on SH1106 64x128 display
- **Safety** - motor timeout, battery low-voltage warning

## Module Configurations

The firmware is compiled with a module identifier that enables/disables features via preprocessor defines in `mod_config.h`:

| Module | CAN ID | Features |
|--------|--------|----------|
| `MK2_MOD1` | `0x21` | Head module - robotic arm (6 Dynamixel motors) |
| `MK2_MOD2` | `0x22` | Middle module - yaw encoder, active joint |
| `MK2_MOD3` | `0x23` | Tail module - yaw encoder, active joint |

Build all three with `make compile_all`, or a single module with `make compile MODULE_DEFINE=MK2_MOD1`.

---

## Repository Structure

```
PicoLowLevel/
+-- README.md              This file
+-- START.md               Environment setup guide (Arduino-CLI, toolchain)
+-- TODO.md                Feature tracker
+-- docs/
|   +-- getting-started.md     Detailed build system guide (Makefile reference)
+-- PicoLowLevel/          Main firmware sketch
|   +-- PicoLowLevel.ino       Entry point
|   +-- Makefile                Build/upload/flash commands
|   +-- TODO.md                 Test notes and open items
|   +-- include/                Header files (definitions, CAN IDs, mod_config)
|   +-- lib/                    Custom libraries (13 total)
+-- dxl_get_position/       Arm motor position reader utility
    +-- dxl_get_position.ino
    +-- README.md
    +-- Makefile
    +-- include/
    +-- lib/
```

---

## Getting Started

See [START.md](START.md) for full environment setup instructions. In summary:

1. Install **Arduino-CLI** and add it to PATH
2. Install the **RP2040 core** (Earlephilhower arduino-pico)
3. Install **GnuWin32 Make** and **MinGW-w64** (Windows)
4. Install the required Arduino library: `Adafruit SH110X`

Then:

```bash
cd PicoLowLevel
make compile                    # Build for MK2_MOD1 (default)
make upload bootsel             # Flash via BOOTSEL mode
make compile_all                # Build all three module variants
```

For the full Makefile reference, see [docs/getting-started.md](docs/getting-started.md).

---

## Arm Motor Zeroing Procedure

When arm motors are disconnected and reconnected, their reference position resets. Before using the main firmware on Module 1 (`MK2_MOD1`):

1. Flash the `dxl_get_position` sketch (see [dxl_get_position/README.md](dxl_get_position/README.md))
2. Open a serial monitor at 115200 baud
3. Read the printed motor position values
4. Copy them into the `MODC_ARM_INIT()` function in `PicoLowLevel/PicoLowLevel.ino`
5. Recompile and flash the main firmware

---

## CAN Bus Protocol

Communication between the Pico and the Jetson uses CAN frames. Key message IDs are defined in `include/definitions.h`. Data is **little-endian**.

For the full protocol specification, see the [CAN Bus Protocol](https://docs.teamisaac.it/doc/can-bus-protocol-t40e2NOEqp) page on the team wiki.

---

## Libraries

| Library | Description |
|---------|-------------|
| `AbsoluteEncoder` | AMS AS5048B I2C absolute encoder driver |
| `Battery` | Battery voltage reading and low-voltage detection |
| `Can` | CAN bus wrapper around MCP2515 SPI controller |
| `Debug` | Serial debug output with verbosity levels |
| `debug_log` | Additional debug logging utilities |
| `Display` | SH1106 OLED display GUI |
| `Dynamixel_ll` | Low-level Dynamixel protocol (XL/XM series, 2 Mbps UART) |
| `DynamixelSerial` | Higher-level Dynamixel serial wrapper |
| `Motor` | Generic motor abstraction (PWM + direction) |
| `PID` | PID controller |
| `SmartMotor` | Motor + encoder + PID combined abstraction |
| `TractionEncoder` | Quadrature encoder reader (48 ppr, PIO-based) |
| `WebManagement` | WiFi web interface for configuration and OTA |

---

## Contributing

All development happens on the `develop` branch. Create feature branches from `develop` and submit pull requests back to it. The `main` branch contains tested, stable firmware only.

---

## Links

- [Team Isaac Documentation](https://docs.teamisaac.it)
- [PicoLowLevel Wiki Page](https://docs.teamisaac.it/doc/picolowlevel-WnfPCgwOM6)
- [LL v0.2 Architecture](https://docs.teamisaac.it/doc/ll-v02-rO7ew05CEo)
- [CAN Bus Protocol](https://docs.teamisaac.it/doc/can-bus-protocol-t40e2NOEqp)
- [MK2 Pico Versions](https://docs.teamisaac.it/doc/mk2-pico-version-lrrgftOA7j)
