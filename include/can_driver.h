#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H
#include <stdint.h>
#include "stm32f1xx_hal.h"
void BSP_CAN_Init(void);
int BSP_CAN_SendMsg(uint32_t id, const uint8_t *d, uint8_t n);
CAN_HandleTypeDef *BSP_CAN_GetHandle(void);
#endif
