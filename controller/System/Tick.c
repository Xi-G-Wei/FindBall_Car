#include "stm32f10x.h"
#include "Tick.h"

static volatile uint32_t tick_ms = 0;

/**
  * @brief  启动 1ms 系统心跳（TIM2 更新中断）
  * @param  无
  * @retval 无
  */
void Tick_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_InternalClockConfig(TIM2);

    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Prescaler = TICK_PSC;
    TIM_TimeBaseInitStructure.TIM_Period = TICK_ARR;
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    //先清一次标志位再开中断，避免初始化时误进一次
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM2, ENABLE);
}

/**
  * @brief  读取当前毫秒计数
  */
uint32_t Tick_Get(void)
{
    return tick_ms;         //32 位对齐读写是单条指令，不用关中断保护
}

/**
  * @brief  TIM2 更新中断：中断里只做计数，活儿都留给主循环
  */
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
    {
        tick_ms++;
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    }
}
