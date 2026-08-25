# STM32F103 Vehicle ECU

Companion ECU firmware for a vehicle-electronics demonstration. It provides a
CAN 500 kbps node, BMS-state simulation and an open-loop SVPWM motor-control
path for the STM32F103C8T6.

## Build

```powershell
platformio run -e genericSTM32F103C8
```

## Key interfaces

- CAN: PB8/PB9
- Motor PWM: PA6/PA7/PB0
- Motor enable / sleep: PA4/PA5
- AS5600 I2C: PB6/PB7

`cheji407` is the complementary STM32F407 dashboard/gateway project.

## Documentation and validation

- [Hardware wiring, CAN identifiers and power precautions](docs/HANDOVER.md)
- [Build and bench-validation checklist](docs/VALIDATION.md)

## Resume project description

**STM32F103 vehicle ECU simulation node**: implemented a 500 kbps CAN node that combines BMS-state simulation, motor-status reporting and open-loop SVPWM drive control; accepts vehicle driving-mode commands from an STM32F407 gateway and changes target motor speed accordingly.
