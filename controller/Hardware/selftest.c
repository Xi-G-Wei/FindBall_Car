#include "stm32f10x.h"
#include "motor.h"
#include "selftest.h"

/* ==================== 自检剧本 ====================
 *   ① 停一拍   0.5s
 *   ② 直行     1.5s
 *   ③ 弧线右转 1.2s      <- 边走边转，不是原地转
 *   ④ 软停     0.8s
 *   ⑤ 弧线左转 1.2s      <- 和 ③ 镜像，两项挨着放，好对比左右
 *   ⑥ 软停     0.8s
 *   ⑦ 原地右转 1s
 *   ⑧ 软停     0.8s
 *   ⑨ 原地左转 1s        <- 和 ⑦ 镜像
 *   ⑩ 软停     0.8s
 *   ⑪ 后退     1.5s
 *   ⑫ 软停     1.5s  ---> 回到 ② 循环
 *
 * 改动作顺序、时长，只动下面 selftest_seq[] 表，其他代码不用碰。
 *
 * 左右转共用 selftest.h 里的 SELFTEST_STEER，方向靠正负号：正=右，负=左
 * 如果某一侧明显转不动，先查那侧的电机接线/方向脚，别急着加大转向量
 * ================================================ */

/* ==================== 状态定义 ==================== */

typedef enum {
    ST_IDLE = 0,        //上电先停一拍
    ST_FORWARD,         //直行
    ST_ARC_RIGHT,       //前进中右转（弧线）
    ST_STOP_1,          //软停
    ST_ARC_LEFT,        //前进中左转（弧线）
    ST_STOP_2,          //软停
    ST_SPIN_RIGHT,      //原地右转
    ST_STOP_3,          //软停
    ST_SPIN_LEFT,       //原地左转
    ST_STOP_4,          //软停
    ST_BACK,            //后退
    ST_STOP_5,          //软停后回到直行，自动循环
    ST_COUNT
} SelfTestState;

/* 一个状态 = 进入时做什么 + 保持多久 + 时间到跳去哪 */
typedef struct {
    void (*enter)(void);    //进入本状态时执行的动作，NULL 表示什么都不做
    uint16_t hold_ms;       //保持时长(ms)
    uint8_t next;           //时间到之后的下一个状态
} SelfTestStep;

/* ==================== 各状态的进入动作 ====================
 * 进入某个状态时执行一次 —— 只"发指令"，不等待、不阻塞
 * 实际加减速由 Motor_Task 的斜率限制完成
 * ====================================================== */

static void ST_EnterForward(void)   { Motor_Forward(SELFTEST_SPEED); }
static void ST_EnterArcRight(void)  { Motor_Drive(SELFTEST_SPEED,  SELFTEST_STEER); }
static void ST_EnterArcLeft(void)   { Motor_Drive(SELFTEST_SPEED, -SELFTEST_STEER); }
static void ST_EnterStop(void)      { Motor_Stop(); }
static void ST_EnterSpinRight(void) { Motor_Spin( SELFTEST_STEER); }
static void ST_EnterSpinLeft(void)  { Motor_Spin(-SELFTEST_STEER); }
static void ST_EnterBack(void)      { Motor_Back(SELFTEST_SPEED); }

/* ==================== 状态表（整个剧本就在这张表） ====================
 * 一行 = 一个状态：进入时做什么 | 保持多久 | 时间到之后去哪
 * 想改动作顺序、时长，只动这张表，SelfTest_Task 的逻辑不用碰
 * ================================================================== */
static const SelfTestStep selftest_seq[ST_COUNT] = {
    /* ①停一拍    */  { 0,                  500,  ST_FORWARD    },
    /* ②直行      */  { ST_EnterForward,    1500, ST_ARC_RIGHT  },
    /* ③弧线右转  */  { ST_EnterArcRight,   1200, ST_STOP_1     },
    /* ④软停      */  { ST_EnterStop,       800,  ST_ARC_LEFT   },
    /* ⑤弧线左转  */  { ST_EnterArcLeft,    1200, ST_STOP_2     },
    /* ⑥软停      */  { ST_EnterStop,       800,  ST_SPIN_RIGHT },
    /* ⑦原地右转  */  { ST_EnterSpinRight,  1000, ST_STOP_3     },
    /* ⑧软停      */  { ST_EnterStop,       800,  ST_SPIN_LEFT  },
    /* ⑨原地左转  */  { ST_EnterSpinLeft,   1000, ST_STOP_4     },
    /* ⑩软停      */  { ST_EnterStop,       800,  ST_BACK       },
    /* ⑪后退      */  { ST_EnterBack,       1500, ST_STOP_5     },
    /* ⑫软停      */  { ST_EnterStop,       1500, ST_FORWARD    },   //回到 ② 循环
};

static SelfTestState st_state;
static uint32_t      st_enter_ms;       //进入当前状态的时刻

/* ==================== 状态机本体 ==================== */

/**
  * @brief  切换状态：记时刻 + 执行进入动作
  * @note   记时刻代替 Delay —— 这就是"不阻塞"的关键
  */
static void ST_Goto(SelfTestState s, uint32_t now)
{
    st_state = s;
    st_enter_ms = now;

    if (selftest_seq[s].enter)
    {
        selftest_seq[s].enter();
    }
}

/**
  * @brief  初始化剧本，停在第一个状态
  * @note   要先调 Motor_Init()
  */
void SelfTest_Init(void)
{
    ST_Goto(ST_IDLE, 0);
}

/**
  * @brief  剧本节拍：只判断"当前状态待够时间了没"，跑一遍就返回
  * @param  now Tick_Get() 的当前值
  */
void SelfTest_Task(uint32_t now)
{
    const SelfTestStep *step = &selftest_seq[st_state];

    //无符号减法：计数器回绕时这个判断依然正确
    if (step->hold_ms != 0 && (now - st_enter_ms) >= step->hold_ms)
    {
        ST_Goto((SelfTestState)step->next, now);
    }
}
