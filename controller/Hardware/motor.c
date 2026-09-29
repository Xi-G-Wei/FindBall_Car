#include "stm32f10x.h"
#include "Delay.h"
#include "PWM.h"
#include "motor.h"

/* ==================== 硬件配置表（接线后只改这里） ==================== */

/* 方向脚 PH：四个电机的方向 GPIO
 * 排法 = 车上的实际位置，顺序：左前 / 右前 / 左后 / 右后
 * 2026-09-22 逐个点测过：板子上原来的 FL/FR/BL/BR 和车的实际位置对不上，
 * 已按实测重新排好，这样 Motor_Drive 的左右判断才是对的
 * 注意这四个脚跨了 GPIOA 和 GPIOB，所以端口要跟引脚一起记
 */
static GPIO_TypeDef * const motor_ph_port[MOTOR_COUNT] = {
    GPIOB,              //左前 -> PB9
    GPIOA,              //右前 -> PA1
    GPIOB,              //左后 -> PB6
    GPIOA               //右后 -> PA0
};

static const uint16_t motor_ph_pin[MOTOR_COUNT] = {
    GPIO_Pin_9,         //左前 -> PB9
    GPIO_Pin_1,         //右前 -> PA1
    GPIO_Pin_6,         //左后 -> PB6
    GPIO_Pin_0          //右后 -> PA0
};

/* EN 走 TIM3 的哪一路（CH1~CH4 对应 PA6 / PA7 / PB0 / PB1） */
static const uint8_t motor_pwm_ch[MOTOR_COUNT] = {
    1,                  //左前 -> CH1 / PA6
    3,                  //右前 -> CH3 / PB0
    2,                  //左后 -> CH2 / PA7
    4                   //右后 -> CH4 / PB1
};

/* 某个电机接线接反了，把对应项改成 1，软件翻相，不用拆线 */
static const uint8_t motor_invert[MOTOR_COUNT] = { 0, 0, 0, 0 };

/* 电机配平（千分比）：补偿四个电机的转速差异，让它们跑一样快
 *   1000 = 不修正（默认）
 *   900  = 这颗压掉 10% 出力（某颗明显偏快时用它）
 *   1100 = 这颗多给 10%（某颗偏慢时用它）
 *
 * 顺序：{ 左前, 右前, 左后, 右后 }
 *
 * 现在左后（下标 2）偏快，压到 900 试试 —— 还不够就把数继续往下调
 *
 * 注意：调的是"输出占空比"，所以不影响 Motor_Drive 的转向计算和斜坡时间
 */
static const uint16_t motor_trim[MOTOR_COUNT] = {
    1000,               //左前
    1000,               //右前
    540,                //左后 —— 偏快
    1000                //右后
};

/* 全局使能 nSLEEP */
#define MOTOR_NSLEEP_PORT   GPIOB
#define MOTOR_NSLEEP_PIN    GPIO_Pin_5     //PB5

/* ===================================================================== */

static int16_t motor_tgt[MOTOR_COUNT];      //目标速度（带符号）
static int16_t motor_cur[MOTOR_COUNT];      //当前实际输出（斜率限制的中间值）
static uint8_t motor_on = 0;                //nSLEEP 状态

/**
  * @brief  电机层初始化：PWM + 方向脚 + nSLEEP
  * @param  无
  * @retval 无
  */
void Motor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t i;

    PWM_Init();                                         //TIM3 + PA6/PA7/PB0/PB1

    //GPIOA 也要开：方向脚 PA1/PA0 在上面（GPIOB 的时钟 PWM_Init 里已经开了）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    //方向脚：普通推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    for (i = 0; i < MOTOR_COUNT; i++)
    {
        GPIO_InitStructure.GPIO_Pin = motor_ph_pin[i];
        GPIO_Init(motor_ph_port[i], &GPIO_InitStructure);
        GPIO_ResetBits(motor_ph_port[i], motor_ph_pin[i]);  //默认方向 0
    }

    //nSLEEP：先拉低（驱动关断）再初始化状态，避免上电瞬间乱输出
    GPIO_InitStructure.GPIO_Pin = MOTOR_NSLEEP_PIN;
    GPIO_Init(MOTOR_NSLEEP_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(MOTOR_NSLEEP_PORT, MOTOR_NSLEEP_PIN);

    for (i = 0; i < MOTOR_COUNT; i++)
    {
        motor_tgt[i] = 0;
        motor_cur[i] = 0;
        PWM_SetDuty(motor_pwm_ch[i], 0);
    }

    Motor_Enable(1);                                    //四路占空比都是 0，使能也不会窜车
    motor_on = 1;
}

/**
  * @brief  全局使能：对应 DRV8701 的 nSLEEP
  * @param  on 0=关断（电机滑行），1=使能
  * @retval 无
  */
void Motor_Enable(uint8_t on)
{
    uint8_t i;

    if (on)
    {
        GPIO_SetBits(MOTOR_NSLEEP_PORT, MOTOR_NSLEEP_PIN);
    }
    else
    {
        //关断前先把目标清零，重新使能时不会突然窜出去
        for (i = 0; i < MOTOR_COUNT; i++)
        {
            motor_tgt[i] = 0;
            motor_cur[i] = 0;
            PWM_SetDuty(motor_pwm_ch[i], 0);
        }
        GPIO_ResetBits(MOTOR_NSLEEP_PORT, MOTOR_NSLEEP_PIN);
    }

    motor_on = on ? 1 : 0;
}

/**
  * @brief  读取全局使能状态
  */
uint8_t Motor_IsEnabled(void)
{
    return motor_on;
}

/**
  * @brief  设置目标速度，实际输出由 Motor_Task() 做斜率限制后给出
  * @param  id    电机编号 MOTOR_FL / MOTOR_FR / MOTOR_BL / MOTOR_BR
  * @param  speed -1000 ~ +1000，正数=正转
  */
void Motor_SetSpeed(uint8_t id, int16_t speed)
{
    if (id >= MOTOR_COUNT)
    {
        return;
    }

    if (speed > MOTOR_SPEED_MAX)
    {
        speed = MOTOR_SPEED_MAX;
    }
    if (speed < -MOTOR_SPEED_MAX)
    {
        speed = -MOTOR_SPEED_MAX;
    }

    motor_tgt[id] = speed;
}

/**
  * @brief  读取某个电机当前的实际输出速度（带符号）
  */
int16_t Motor_GetSpeed(uint8_t id)
{
    if (id >= MOTOR_COUNT)
    {
        return 0;
    }

    return motor_cur[id];
}

/**
  * @brief  电机节拍任务：死区补偿 + 斜率限制 + 换向保护，然后输出到 PWM
  * @note   必须周期性调用（建议 5ms），运动指令靠它生效
  */
void Motor_Task(void)
{
    uint8_t i;

    for (i = 0; i < MOTOR_COUNT; i++)
    {
        int16_t t = motor_tgt[i];
        int16_t c = motor_cur[i];
        int32_t mag;
        uint16_t duty;

        //死区补偿：非零指令抬到能转起来的最小占空比
        if (t > 0 && t < MOTOR_MIN_DUTY)
        {
            t = MOTOR_MIN_DUTY;
        }
        if (t < 0 && t > -MOTOR_MIN_DUTY)
        {
            t = -MOTOR_MIN_DUTY;
        }

        //换向保护：目标与当前方向相反时先熄火，减到 0 下一次再反向加速
        //否则等于瞬间反接全压，H 桥电流冲击很大
        if ((t > 0 && c < 0) || (t < 0 && c > 0))
        {
            t = 0;
        }

        //斜率限制
        if (c < t)
        {
            c += MOTOR_RAMP_STEP;
            if (c > t)
            {
                c = t;
            }
        }
        else if (c > t)
        {
            c -= MOTOR_RAMP_STEP;
            if (c < t)
            {
                c = t;
            }
        }
        motor_cur[i] = c;

        //方向脚：只在有输出时改写；输出为 0 时保持原状（此时 EN=0，电机不动）
        if (c != 0)
        {
            uint8_t forward = (c > 0) ? 1 : 0;
            if (motor_invert[i])
            {
                forward = forward ? 0 : 1;
            }

            if (forward)
            {
                GPIO_SetBits(motor_ph_port[i], motor_ph_pin[i]);
            }
            else
            {
                GPIO_ResetBits(motor_ph_port[i], motor_ph_pin[i]);
            }
        }

        //占空比输出：先按配平千分比缩放（补偿四个电机的转速差异），再换算成 PWM 刻度
        //取绝对值是因为占空比没有正负，方向由 PH 脚表达
        mag = (c >= 0) ? c : -c;                        //实际输出幅值 0~1000
        mag = mag * motor_trim[i] / 1000;               //按配平缩放
        duty = (uint16_t)(mag * PWM_MAX_DUTY / MOTOR_SPEED_MAX);
        PWM_SetDuty(motor_pwm_ch[i], duty);
    }
}

/* ==================== 底盘动作指令 ====================
 * 这些函数都只干一件事：改目标速度（motor_tgt[]）
 * 真正的加减速由 Motor_Task 的斜坡完成，所以它们永远不会让车"窜"
 * 直行 / 后退 / 原地转 / 弧线，全都是 Motor_Drive 的特例
 * ==================================================== */

/**
  * @brief  取绝对值（内部小工具）
  */
static int16_t Abs16(int16_t v)
{
    return (v >= 0) ? v : -v;
}

/**
  * @brief  差速行驶：油门 + 转向（所有移动动作的基础）
  * @param  throttle -1000 ~ +1000，正数 = 前进
  * @param  steer    -1000 ~ +1000，0 = 直行，正数 = 右转
  * @retval 无
  * @note   左轮 = throttle + steer     右轮 = throttle - steer
  */
void Motor_Drive(int16_t throttle, int16_t steer)
{
    int16_t left  = throttle + steer;
    int16_t right = throttle - steer;
    int16_t peak  = (Abs16(left) > Abs16(right)) ? Abs16(left) : Abs16(right);

    //超量程时左右一起等比缩小，保住两边比例
    //（若各自单独限幅，"外轮满速 + 内轮半速"会被压成两边都满速，弧线就变直行了）
    if (peak > MOTOR_SPEED_MAX)
    {
        left  = (int16_t)((int32_t)left  * MOTOR_SPEED_MAX / peak);
        right = (int16_t)((int32_t)right * MOTOR_SPEED_MAX / peak);
    }

    Motor_SetSpeed(MOTOR_FL, left);
    Motor_SetSpeed(MOTOR_BL, left);
    Motor_SetSpeed(MOTOR_FR, right);
    Motor_SetSpeed(MOTOR_BR, right);
}

/* ---------- 下面三个是常用动作，内部都是调 Motor_Drive ---------- */

void Motor_Forward(int16_t speed)
{
    Motor_Drive(speed, 0);      //两边同速 -> 直行
}

void Motor_Back(int16_t speed)
{
    Motor_Drive(-speed, 0);     //两边同速反向 -> 后退
}

void Motor_Spin(int16_t steer)
{
    //油门给 0，只有转向 -> 车绕自己中心转
    //注意：滑移转向很磨轮、电流也大，能用弧线过的弯就别原地转
    Motor_Drive(0, steer);
}

/**
  * @brief  四轮软停：目标清零，由 Motor_Task 的斜坡平滑减到 0
  */
void Motor_Stop(void)
{
    uint8_t i;

    for (i = 0; i < MOTOR_COUNT; i++)
    {
        Motor_SetSpeed(i, 0);
    }
}

/**
  * @brief  阻塞运行 ms 毫秒，期间持续喂 Motor_Task
  * @param  ms 持续时间，建议传 MOTOR_TICK_MS 的整数倍
  * @note   单次调用最多阻塞 ms 毫秒；主循环里靠它把动作撑开
  */
void Motor_RunFor(uint16_t ms)
{
    while (ms >= MOTOR_TICK_MS)
    {
        Motor_Task();
        Delay_ms(MOTOR_TICK_MS);
        ms -= MOTOR_TICK_MS;
    }
}
