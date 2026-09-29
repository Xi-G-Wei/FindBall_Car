#ifndef __SELFTEST_H__
#define __SELFTEST_H__

#include "stm32f10x.h"

/* ==================== 上电自检剧本 ====================
 *
 * 注意：这是"剧本"，不是"库"。
 *
 *   它自己不控制任何硬件，只是把 motor.h 的动作按顺序播一遍，
 *   用来验证接线和转向对不对（直行 → 右转 → 左转 → 后退 → 循环）。
 *
 *   >>> 要开车请用 motor.h（Motor_Forward / Motor_Drive / ...），别调这个 <<<
 *
 * 用法：开机调一次 SelfTest_Init()，然后主循环里持续调 SelfTest_Task(now)。
 *
 * 注意：它会一直占着电机，和串口跟踪逻辑不能同时开，二选一。
 * ==================================================== */

/* ---------------- 调参速查 ---------------- */

/* 整套动作的速度（千分比）
 *   第一次上车先给 300，确认四个轮子方向对了再往上加
 */
#define SELFTEST_SPEED      300

/* 转向量：和 SELFTEST_SPEED 的比值越大，弯越急
 *   现在 350/300 —— 内侧轮会倒转，是很急的弯（自检要看得明显，可以接受）
 *   想缓一点就减到 150
 */
#define SELFTEST_STEER      350

/* ---------------- 接口 ---------------- */

void SelfTest_Init(void);               //启动剧本（要先调 Motor_Init）

/* 在 main 循环里持续调用，now 传 Tick_Get() 的当前值 */
void SelfTest_Task(uint32_t now);

#endif
