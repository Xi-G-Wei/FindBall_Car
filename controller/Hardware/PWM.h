#ifndef __PWM_H__
#define __PWM_H__

#include "stm32f10x.h"

/* PWM 底层驱动：TIM3 四路输出
 *   CH1 = PA6   CH2 = PA7   CH3 = PB0   CH4 = PB1
 *
 * 频率 20kHz（72MHz / 4 / 900），占空比分辨率 1/900 ≈ 0.11%
 * 20kHz 在人耳听觉之外，且对 DRV8701 + TPH1R403NL 这种分立 H 桥很轻松
 */
#define PWM_MAX_DUTY    900         /* 占空比满量程，对应 100% */

void PWM_Init(void);

/* 设置占空比
 *   ch  : 通道号 1 ~ 4
 *   duty: 0 ~ PWM_MAX_DUTY，超出会被限幅
 */
void PWM_SetDuty(uint8_t ch, uint16_t duty);

#endif
