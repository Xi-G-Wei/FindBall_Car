#ifndef __TICK_H__
#define __TICK_H__

#include "stm32f10x.h"

/* ============================ 系统时间基准 ============================
 * TIM2 每 1ms 中断一次，中断里只做一件事：tick_ms++
 *
 * 怎么用 —— 所有"等一段时间"的地方都改成这样写：
 *
 *     static uint32_t last = 0;
 *     ...
 *     if (Tick_Get() - last >= 500)   //走到这里说明过了 500ms
 *     {
 *         last = Tick_Get();
 *         //该做的事
 *     }
 *
 * 永远用"减法 >= 间隔"判断，别写成 last + 500 < Tick_Get()：
 * 计数器 49.7 天回绕一次，加法会算错，减法不会
 *
 * 为什么不用 SysTick：Delay.c 把 SysTick 当延时器反复重写 LOAD/VAL/CTRL，
 * 毫秒计数放那儿会被延时调用打乱，所以另开 TIM2（TIM3 已被 PWM 占用）
 * ===================================================================
 */

/* TIM2 挂在 APB1 上，定时器时钟 72MHz */
#define TICK_PSC        (72 - 1)        //72MHz / 72 = 1MHz 计数时钟
#define TICK_ARR        (1000 - 1)      //数 1000 次 = 1ms

void Tick_Init(void);                   //开机调一次，越早越好

/* 开机以来的毫秒数（32 位，约 49.7 天回绕一次） */
uint32_t Tick_Get(void);

#endif
