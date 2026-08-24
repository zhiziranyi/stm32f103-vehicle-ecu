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
