# STM32F103 Vehicle ECU Simulation Node

> A compact STM32F103C8T6 ECU-side firmware for a vehicle-electronics demo.
> It publishes BMS/motor state over CAN and accepts driving-mode commands from
> the companion [STM32F407 Vehicle Dashboard](https://github.com/zhiziranyi/stm32f407-vehicle-dashboard).

## What this project demonstrates

- 500 kbit/s CAN node design on an STM32F103C8T6.
- BMS-state simulation and periodic vehicle/motor status reporting.
- Open-loop SVPWM motor-control path with enable/sleep safety signals.
- AS5600 magnetic-encoder interface for rotor/position acquisition.
- Separation between gateway-originated drive-mode commands and local
  actuator/state handling.

## System architecture

```text
STM32F407 dashboard / gateway
          │  CAN, 500 kbit/s
          ▼
STM32F103 ECU node ──> BMS state simulation / motor status reports
          │
          ├──> PWM U/V/W + enable/sleep ──> external motor driver
          └──> I2C ──> AS5600 magnetic encoder
```

This firmware is an engineering demonstration node. Motor power hardware,
current sensing, and fault protection must be implemented by the external
driver/power stage before it is used with a real motor.

## Hardware interface

| Function | STM32F103 pin(s) | Notes |
| --- | --- | --- |
| CAN | PB8 (RX), PB9 (TX) | 500 kbit/s through a 3.3 V CAN transceiver |
| PWM phases | PA6, PA7, PB0 | Open-loop SVPWM output to motor driver inputs |
| Driver enable / sleep | PA4, PA5 | Keep the power stage disabled during wiring changes |
| AS5600 I²C | PB6 (SCL), PB7 (SDA) | Shared 3.3 V ground and pull-ups as required |
| Debug/flash | Board UART/boot pins | See `platformio.ini` for the selected upload method |

Never drive a motor directly from an STM32 pin. Use an isolated or suitably
protected power stage, keep logic/power grounds intentional, and verify CANH,
CANL, and termination before transmitting.

## Build and flash

```powershell
cd C:\Users\13957\Documents\PlatformIO\Projects\cheji103
C:\Users\13957\.platformio\penv\Scripts\platformio.exe run -e genericSTM32F103C8
```

Target: `genericSTM32F103C8`, STM32Cube framework. Keep generated `.pio/`
artifacts out of commits; they are rebuildable from source.

## Bench bring-up sequence

1. Build the project and flash with the configured PlatformIO upload method.
2. Start with the motor driver unpowered or disabled; confirm the ECU boots.
3. Attach a CAN transceiver and confirm the bus is set to 500 kbit/s.
4. Bring up the F407 dashboard/gateway and observe the ECU status frames.
5. Verify a driving-mode command changes the ECU-side target-state behavior.
6. Connect the AS5600 and validate its I²C response before enabling any motor
   power stage.
7. Only then test PWM output on a scope or logic analyzer.

## Verification and handover

The project keeps the reproducible details outside the README:

- [Hardware wiring, CAN identifiers and power precautions](docs/HANDOVER.md)
- [Build and bench-validation checklist](docs/VALIDATION.md)

Expected evidence includes a successful build, valid CAN traffic with the F407
peer, BMS/motor state changes, encoder response, and a safe disabled-to-enabled
motor-driver sequence.

## Repository layout

```text
src/       ECU application, CAN and PWM control implementation
include/   Pin mapping and public interfaces
lib/       Project-local support code
docs/      Handover and validation procedures
test/      Test assets
```

## Resume description

**STM32F103 vehicle ECU simulation node** — Implemented a 500 kbit/s CAN node
combining BMS-state simulation, motor-status reporting, AS5600 position input,
and open-loop SVPWM drive control; accepted driving-mode commands from an
STM32F407 gateway while preserving a safe external-driver interface.
