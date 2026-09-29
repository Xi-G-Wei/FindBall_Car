#include "stm32f10x.h"
#include "PWM.h"

/**
  * @brief  TIM3 四路 PWM 初始化（PA6 / PA7 / PB0 / PB1）
  * @param  无
  * @retval 无
  * @note   72MHz / (4) / (900) = 20kHz
  */
void PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    //打开时钟：TIM3 在 APB1，GPIO 在 APB2
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    //初始化GPIO：复用推挽输出（AF_PP），写成 Out_PP 会没有波形
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;      //TIM3_CH1 / CH2
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;      //TIM3_CH3 / CH4
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    //选择内部时钟
    TIM_InternalClockConfig(TIM3);

    //配置时基单元
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 4 - 1;            //预分频 4，计数时钟 18MHz
    TIM_TimeBaseInitStructure.TIM_Period = 900 - 1;             //ARR：一个周期计数 900 次
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;        //高级定时器才用，TIM3 写 0 即可
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    //配置输出比较单元
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;                          //CCR 初值 0 -> 上电静止

    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    TIM_OC3Init(TIM3, &TIM_OCInitStructure);
    TIM_OC4Init(TIM3, &TIM_OCInitStructure);

    //开启预装载：CCR/ARR 在更新事件时统一生效，运行中改占空比不会出毛刺
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    //启动定时器
    TIM_Cmd(TIM3, ENABLE);
}

/**
  * @brief  设置指定通道的占空比
  * @param  ch   通道号 1 ~ 4
  * @param  duty 0 ~ PWM_MAX_DUTY
  * @retval 无
  */
void PWM_SetDuty(uint8_t ch, uint16_t duty)
{
    if (duty > PWM_MAX_DUTY)                                //限幅：越界会被当成满占空比
    {
        duty = PWM_MAX_DUTY;
    }

    switch (ch)
    {
        case 1: TIM_SetCompare1(TIM3, duty); break;
        case 2: TIM_SetCompare2(TIM3, duty); break;
        case 3: TIM_SetCompare3(TIM3, duty); break;
        case 4: TIM_SetCompare4(TIM3, duty); break;
        default: break;
    }
}
