# Project Handover

## Build and upload

- Build: `platformio run -e genericSTM32F103C8`
- Upload protocol currently configured: ST-Link

## Integration

This ECU exchanges CAN data with the STM32F407 dashboard project. Keep CAN IDs
and baud-rate changes coordinated between both repositories.
