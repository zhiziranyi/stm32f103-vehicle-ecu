/**
 * F103 — 电机(开环SVPWM) + BMS + 接收F407驾驶模式指令
 * PA6/PA7/PB0=Mini PWM, PA4=EN, PA5=nSLEEP
 * PB6/7=AS5600, PB8/9=CAN
 */
#include "stm32f1xx_hal.h"
#include "can_driver.h"
#include "motor_ctrl.h"

static void clk(void) {
    RCC_OscInitTypeDef o={0}; RCC_ClkInitTypeDef c={0}; HAL_StatusTypeDef r;
    o.OscillatorType=RCC_OSCILLATORTYPE_HSE; o.HSEState=RCC_HSE_ON;
    o.PLL.PLLState=RCC_PLL_ON; o.PLL.PLLSource=RCC_PLLSOURCE_HSE; o.PLL.PLLMUL=RCC_PLL_MUL9;
    r=HAL_RCC_OscConfig(&o);
    if(r!=HAL_OK){o.OscillatorType=RCC_OSCILLATORTYPE_HSI; o.HSIState=RCC_HSI_ON;
     o.HSICalibrationValue=RCC_HSICALIBRATION_DEFAULT;
     o.PLL.PLLSource=RCC_PLLSOURCE_HSI_DIV2; o.PLL.PLLMUL=RCC_PLL_MUL16; HAL_RCC_OscConfig(&o);}
    c.ClockType=RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    c.SYSCLKSource=RCC_SYSCLKSOURCE_PLLCLK; c.AHBCLKDivider=RCC_SYSCLK_DIV1;
    c.APB1CLKDivider=RCC_HCLK_DIV2; c.APB2CLKDivider=RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&c,FLASH_LATENCY_2);
}
void SysTick_Handler(void) { HAL_IncTick(); }

/* BMS */
static uint8_t  g_soc=80;
static uint32_t g_stk=0;
static void bms_send(void) {
    static uint32_t last=0;
    if(HAL_GetTick()-last<1000) return; last=HAL_GetTick();
    if(++g_stk>=60){g_stk=0; if(g_soc>15) g_soc--;}
    uint16_t v=3700; int16_t c=150;
    uint8_t d[8]={(uint8_t)(v>>8),(uint8_t)v,(uint8_t)(c>>8),(uint8_t)c,g_soc,75,65,0};
    BSP_CAN_SendMsg(0x100,d,8);
}

/* 接收F407驾驶模式 (CAN 0x300 整车状态, byte1=驾驶模式) */
static void check_can_cmd(void) {
    CAN_RxHeaderTypeDef rh;
    uint8_t d[8];
    while(HAL_CAN_GetRxFifoFillLevel(BSP_CAN_GetHandle(),CAN_RX_FIFO0)>0) {
        if(HAL_CAN_GetRxMessage(BSP_CAN_GetHandle(),CAN_RX_FIFO0,&rh,d)==HAL_OK) {
            if(rh.StdId==0x300 && rh.DLC>=2) {
                uint8_t mode=d[1];
                if(mode<=2) { float s[]={300,600,1000}; Motor_SetSpeed(s[mode]); }
            }
        }
    }
}

int main(void) {
    uint32_t last_mot=0;
    HAL_Init(); clk(); BSP_CAN_Init();
    if(Motor_Init()==0) Motor_SetSpeed(300);
    while(1) {
        Motor_RunFOC();
        bms_send();
        check_can_cmd();
        if(HAL_GetTick()-last_mot>=100) { last_mot=HAL_GetTick(); Motor_SendCANStatus(); }
    }
}
