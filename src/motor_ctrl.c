/**
 * motor_ctrl.c — F103 开环速度控制 (参考SimpleFOC velocity_openloop)
 *
 * 原理: 不依赖编码器换向, 直接累加电角度 = target_speed * dt * 极对数
 *       磁场旋转 → 转子像同步电机一样跟随
 *
 * PA6/PA7/PB0=TIM3 PWM, PA4=EN, PA5=nSLEEP, PB6/7=I2C, PB8/9=CAN
 */
#include "motor_ctrl.h"
#include "can_driver.h"
#include "stm32f1xx_hal.h"
#include <math.h>

#define PP        7
#define V_BUS     12.0f
#define V_DRIVE   3.0f     /* 开环驱动电压 */
#define ARR_VAL   2879     /* 72M/2880=25kHz */

static TIM_HandleTypeDef htim3;
static I2C_HandleTypeDef  hi2c1;

static float g_speed=0, g_target=0, g_elec=0;
static float g_last_a=0, g_last_t=0;
static int   g_aligned=0;
static uint32_t g_align_s=0;

/* ---- Mini EN/nSLEEP ---- */
static void drv_on(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g={0}; g.Pin=GPIO_PIN_4|GPIO_PIN_5;
    g.Mode=GPIO_MODE_OUTPUT_PP; g.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA,&g);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
}

/* ---- I2C AS5600 ---- */
static void i2c_init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef s={0}; s.Pin=GPIO_PIN_6|GPIO_PIN_7;
    s.Mode=GPIO_MODE_AF_OD; s.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB,&s);
    hi2c1.Instance=I2C1; hi2c1.Init.ClockSpeed=400000;
    hi2c1.Init.DutyCycle=I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1=0; hi2c1.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static uint16_t as5600_read(void) {
    uint8_t b[2]={0}, r=0x0C;
    HAL_I2C_Mem_Read(&hi2c1,0x36<<1,r,I2C_MEMADD_SIZE_8BIT,b,2,5);
    return ((uint16_t)b[0]<<8)|b[1];
}

/* ---- TIM3 PWM ---- */
static void pwm_init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef s={0}; s.Mode=GPIO_MODE_AF_PP; s.Speed=GPIO_SPEED_FREQ_HIGH;
    s.Pin=GPIO_PIN_6; HAL_GPIO_Init(GPIOA,&s);
    s.Pin=GPIO_PIN_7; HAL_GPIO_Init(GPIOA,&s);
    s.Pin=GPIO_PIN_0; HAL_GPIO_Init(GPIOB,&s);
    htim3.Instance=TIM3; htim3.Init.Prescaler=0; htim3.Init.Period=ARR_VAL;
    htim3.Init.CounterMode=TIM_COUNTERMODE_UP; htim3.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload=TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_PWM_Init(&htim3);
    TIM_OC_InitTypeDef oc={0}; oc.OCMode=TIM_OCMODE_PWM1; oc.Pulse=ARR_VAL/2;
    oc.OCPolarity=TIM_OCPOLARITY_HIGH; oc.OCFastMode=TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3,&oc,TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim3,&oc,TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim3,&oc,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
}

/* ---- SVPWM ---- */
static void set_pwm(float a, float b) {
    float u=a, v=-0.5f*a+0.8660254f*b, w=-0.5f*a-0.8660254f*b;
    float m=(fminf(fminf(u,v),w)+fmaxf(fmaxf(u,v),w))*0.5f;
    u-=m; v-=m; w-=m;
    int du=(int)((u*0.5f+0.5f)*ARR_VAL), dv=(int)((v*0.5f+0.5f)*ARR_VAL), dw=(int)((w*0.5f+0.5f)*ARR_VAL);
    if(du<0)du=0; if(du>ARR_VAL)du=ARR_VAL;
    if(dv<0)dv=0; if(dv>ARR_VAL)dv=ARR_VAL;
    if(dw<0)dw=0; if(dw>ARR_VAL)dw=ARR_VAL;
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,du);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,dv);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,dw);
}

/* ---- 开环FOC (参考SimpleFOC velocity_openloop) ---- */
void Motor_RunFOC(void) {
    uint16_t raw=as5600_read();
    float a=(float)raw*(2.0f*3.14159265f)/4096.0f;
    uint32_t now=HAL_GetTick();

    /* 速度测量 (仅供显示) */
    float da=a-g_last_a; if(da>3.14159265f)da-=6.2831853f; if(da<-3.14159265f)da+=6.2831853f;
    float dt=(float)(now-(uint32_t)g_last_t)*0.001f;
    if(g_last_t<1.0f){g_last_a=a;g_last_t=(float)now;}  /* 首次记录 */
    else if(dt>0.0001f && dt<0.5f) {
        g_speed=da/dt; g_last_a=a; g_last_t=(float)now;
    }

    /* 对齐: 施加直流让转子归位 */
    if(!g_aligned) {
        if(now-g_align_s<1500) { set_pwm(0.25f,0); return; } /* 3V@A相 */
        g_elec=0; g_aligned=1;
    }

    /* 开环: 累加电角度, 用独立时间戳 */
    static float g_elec_t=0;
    float dt2=(float)(now-(uint32_t)g_elec_t)*0.001f;
    if(g_elec_t<1.0f || dt2>0.5f) dt2=0.001f; /* 首次/大跳跃用1ms */
    if(dt2>0.0001f) {
        g_elec += g_target * dt2 * (float)PP;
        if(g_elec > 6.2831853f) g_elec -= 6.2831853f;
        if(g_elec < 0.0f) g_elec += 6.2831853f;
        g_elec_t=(float)now;
    }

    /* SVPWM: 电压矢量超前90°(q轴) */
    float th = g_elec + 1.5707963f;
    float amp = V_DRIVE / V_BUS;
    set_pwm(amp*cosf(th), amp*sinf(th));
}

/* ---- API ---- */
int Motor_Init(void) {
    drv_on(); i2c_init(); pwm_init();
    uint8_t b[2]={0}, r=0x0C;
    if(HAL_I2C_Mem_Read(&hi2c1,0x36<<1,r,I2C_MEMADD_SIZE_8BIT,b,2,5)!=HAL_OK) return -1;
    g_align_s=HAL_GetTick(); g_aligned=0; g_target=0;
    return 0;
}

void Motor_SetSpeed(float rpm) { g_target=rpm*(2.0f*3.14159265f)/60.0f; }
float Motor_GetSpeed(void)      { return g_speed*60.0f/(2.0f*3.14159265f); }
uint8_t Motor_GetTemp(void)     { return 85; }
uint8_t Motor_GetFault(void)    { return 0; }

void Motor_SendCANStatus(void) {
    int16_t r=(int16_t)Motor_GetSpeed();
    uint8_t d[8]={0}; d[0]=(uint8_t)(r>>8); d[1]=(uint8_t)r; d[2]=85; d[3]=0;
    BSP_CAN_SendMsg(0x200,d,4);
}
