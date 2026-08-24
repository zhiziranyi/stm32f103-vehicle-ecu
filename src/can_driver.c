#include "can_driver.h"
#include "stm32f1xx_hal.h"
static CAN_HandleTypeDef gh;
void HAL_CAN_MspInit(CAN_HandleTypeDef *h){
 GPIO_InitTypeDef s={0}; __HAL_RCC_CAN1_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_AFIO_CLK_ENABLE();
 AFIO->MAPR=(AFIO->MAPR&~(3U<<13))|(2U<<13);
 s.Pin=GPIO_PIN_8;s.Mode=GPIO_MODE_INPUT;s.Pull=GPIO_PULLUP;HAL_GPIO_Init(GPIOB,&s);
 s.Pin=GPIO_PIN_9;s.Mode=GPIO_MODE_AF_PP;s.Speed=GPIO_SPEED_FREQ_HIGH;HAL_GPIO_Init(GPIOB,&s);
}
void BSP_CAN_Init(void){
 gh.Instance=CAN1;uint32_t a=HAL_RCC_GetPCLK1Freq();
 if(a>=36000000U){gh.Init.Prescaler=6;gh.Init.TimeSeg1=CAN_BS1_8TQ;gh.Init.TimeSeg2=CAN_BS2_3TQ;}
 else{gh.Init.Prescaler=4;gh.Init.TimeSeg1=CAN_BS1_11TQ;gh.Init.TimeSeg2=CAN_BS2_4TQ;}
 gh.Init.Mode=CAN_MODE_NORMAL;gh.Init.SyncJumpWidth=CAN_SJW_1TQ;
 gh.Init.TimeTriggeredMode=DISABLE;gh.Init.AutoBusOff=ENABLE;gh.Init.AutoRetransmission=ENABLE;
 gh.Init.ReceiveFifoLocked=DISABLE;gh.Init.TransmitFifoPriority=DISABLE;
 HAL_CAN_Init(&gh);CAN_FilterTypeDef f={0};f.FilterBank=0;f.FilterMode=CAN_FILTERMODE_IDMASK;
 f.FilterScale=CAN_FILTERSCALE_32BIT;f.FilterFIFOAssignment=CAN_FILTER_FIFO0;f.FilterActivation=ENABLE;
 HAL_CAN_ConfigFilter(&gh,&f);HAL_CAN_Start(&gh);
}
int BSP_CAN_SendMsg(uint32_t id,const uint8_t*d,uint8_t n){
 CAN_TxHeaderTypeDef h={0};uint32_t m;if(n>8)return-1;
 h.StdId=id;h.IDE=CAN_ID_STD;h.RTR=CAN_RTR_DATA;h.DLC=n;
 return HAL_CAN_AddTxMessage(&gh,&h,(uint8_t*)d,&m)==HAL_OK?0:-2;
}
CAN_HandleTypeDef *BSP_CAN_GetHandle(void){return &gh;}
