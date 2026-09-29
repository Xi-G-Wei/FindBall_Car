#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32f10x.h"

/* ========================= 电机 + 底盘动作 =========================
 *
 * 【开车只用这个文件】常用动作就这五个：
 *
 *     Motor_Forward(速度)          前进
 *     Motor_Back(速度)             后退
 *     Motor_Spin(转向量)           原地转      正 = 右转，负 = 左转
 *     Motor_Drive(油门, 转向)      边走边转    转向 0 = 直行，正 = 右，负 = 左
 *     Motor_Stop()                 软停
 *
 *   速度 / 转向量范围都是 -1000 ~ +1000，1000 = 100% 占空比
 *
 *   这些都是"异步"的：调完立刻返回，实际加减速由 Motor_Task 的斜坡做，
 *   所以车永远不会突然窜出去。同一个动作重复调一万遍也没事（幂等）。
 *
 *   （想走一整套顺序动作的剧本，看 selftest.h —— 那个不是开车用的库）
 * ------------------------------------------------------------------
 * 硬件：DRV8701 的 PH/EN 模式
 *       每个电机 = 1 路 EN(PWM 调占空比) + 1 路 PH(方向电平)
 *       nSLEEP 四个驱动共用，必须为高电平才输出
 *
 * 数据流 —— "平稳"就来自这条链路：
 *
 *      Motor_xxx()  ──▶  motor_tgt[]   目标速度
 *                           │
 *                           │  Motor_Task() 每 5ms 做三件事：
 *                           │    ① 死区补偿  ② 换向保护  ③ 斜率限制
 *                           ▼
 *                       motor_cur[]   实际输出（不会跳变）
 *                           │
 *                           ▼
 *                        PWM 占空比 + 方向脚
 * ===================================================================
 */

/* 电机数量与编号，按原理图：FL 左前 / FR 右前 / BL 左后 / BR 右后 */
#define MOTOR_COUNT         4

#define MOTOR_FL            0       //M1 左前
#define MOTOR_FR            1       //M2 右前
#define MOTOR_BL            2       //M3 左后
#define MOTOR_BR            3       //M4 右后

/* ---------------- 参数速查：想调手感就改这三处 ---------------- */

#define MOTOR_SPEED_MAX     1000    //满速值，对应 100% 占空比

/* ① 节拍周期(ms)：Motor_Task 的调用周期，main 里按它喂节拍
 *    斜坡时间按这个周期算，改了要一起改主循环
 */
#define MOTOR_TICK_MS       5

/* ② 斜坡步长：每次 Motor_Task 最多变多少（千分比）
 *    调大 -> 起步猛、响应快、电流冲击大
 *    调小 -> 起步柔、电流小，但感觉"肉"
 *    40：1000 / 40 × 5ms = 125ms 从 0 走到满速
 */
#define MOTOR_RAMP_STEP     40

/* ③ 死区补偿：占空比低于这个值电机不转，所以非零指令直接抬到这里
 *    调大 -> 起步有力，但低速段会"一跳"
 *    调小 -> 低速细腻，但太小会嗡嗡响却不转
 *    70/1000 = 7%，按你的电机和电池电压实调
 */
#define MOTOR_MIN_DUTY      70

/* ---------------- 接口分三层：平时只用第 3 层 ---------------- */

/* 第 1 层：初始化和节拍 */
void Motor_Init(void);              //开机调一次
void Motor_Task(void);              //所有速度变化都靠它生效，必须每 5ms 调一次

/* 第 2 层：单个电机（做特殊动作、以后接闭环才用） */
void Motor_SetSpeed(uint8_t id, int16_t speed);   //写目标速度，不会立即生效
int16_t Motor_GetSpeed(uint8_t id);               //读的是期望值，不是实测转速
void Motor_Enable(uint8_t on);                    //全局使能(nSLEEP)，0 = 断电滑行
uint8_t Motor_IsEnabled(void);

/* 第 3 层：底盘动作（写逻辑就用这层，只需要给速度） */

/* 核心接口：油门 + 转向，一个式子覆盖所有移动
 *   throttle -1000 ~ +1000   正 = 前进，负 = 后退
 *   steer    -1000 ~ +1000   正 = 右转，负 = 左转，0 = 直行
 *
 *   Drive( 1000,    0)   直行
 *   Drive(    0, 1000)   原地右转        Drive(   0, -1000)   原地左转
 *   Drive(  700,  350)   前进中右转(弧线) Drive( 700,  -350)   前进中左转(弧线)
 *   Drive( -700,  350)   后退中右转       Drive(-700,  -350)   后退中左转
 *
 *   弯的急缓看 steer 的大小：
 *     steer = 0            直行
 *     steer < throttle     内侧轮还在往前转 —— 平滑弧线，跟踪用这个
 *     steer = throttle     内侧轮停住
 *     steer > throttle     内侧轮倒转 —— 很急的弯（自检用的 350/300 就是这种）
 */
void Motor_Drive(int16_t throttle, int16_t steer);

/* 常用动作的快捷方式，内部都是调 Motor_Drive
 * 注意：这几个只是"最常用的几种"。弧线、斜走等组合直接用 Motor_Drive 传参数，
 *       不需要新的函数 —— 方向由转向量的正负号决定（正=右，负=左）
 */
void Motor_Forward(int16_t speed);      // = Drive(speed, 0)      四轮前进
void Motor_Back(int16_t speed);         // = Drive(-speed, 0)     四轮后退
void Motor_Spin(int16_t steer);         // = Drive(0, steer)      原地转：正=右转，负=左转
void Motor_Stop(void);                  // 四轮软停：目标清零，斜坡平滑减到 0
void Motor_RunFor(uint16_t ms);         // 阻塞版（只在自检里用），期间持续喂节拍

#endif
