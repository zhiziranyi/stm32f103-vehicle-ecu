#ifndef MOTOR_CTRL_H
#define MOTOR_CTRL_H
#include <stdint.h>
int  Motor_Init(void);
void Motor_RunFOC(void);
void Motor_SetSpeed(float rpm);
float Motor_GetSpeed(void);
uint8_t Motor_GetTemp(void);
uint8_t Motor_GetFault(void);
void Motor_SendCANStatus(void);
#endif
