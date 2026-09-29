#include "stm32f10x.h"                  // Device header
#include "Tick.h"						//1ms 系统心跳
#include "motor.h"						//电机驱动
#include "selftest.h"					//上电自检剧本
#include "USART3.h"						//摄像头串口(PB10/PB11)：收球的坐标

/* ============================ 主循环 ============================
 *   Tick    ：1ms 中断里自增，给下面所有判断提供时间
 *   Motor   ：每 5ms 喂一次 Motor_Task()，电机的斜坡/换向保护靠它
 *   USART3  ：把收到的坐标解析出来，决定车往哪走
 *                摄像头走 USART3(PB10/PB11)：对照原理图核对过，
 *                板子上引出来的那组串口就是 USART3
 *   SelfTest：上电自检剧本（和跟踪二选一，跟踪时必须关掉）
 *
 * 以后加按键、OLED、循迹，就在这个 while 里再挂一个 if。
 * 但绝对不能写 Delay_ms() —— 卡住主循环会让电机节拍变慢、动作发顿
 * ================================================================
 */

/* ==================== 跟踪参数（调手感）====================
 *
 * AIM_X    车正前方在摄像头画面里的 x 坐标。球跑到这个位置才算"车对着它"。
 *          不是画面中心 400 摄像头装歪一点，"画面中间"就不等于"车正前方"。
 *          标定办法：把球放在"车往前开、框正好扫到它"的位置，
 *                    看 K230 屏幕左上角绿圈旁边那个数字，就是 AIM_X。
 *
 * DEADZONE 允许的横向误差。球和 AIM_X 的差在这个范围内就算对正 → 直行。
 *          调大 → 直行多、转弯少，但可能歪着开过去撞不到球
 *          调小 → 对得准，但检测一抖就左右来回摆
 *
 * TRACK_SPEED / TRACK_STEER  跟踪时的前进速度 / 转向量
 *             STEER 必须 < SPEED 否则内侧轮会倒转、还会反复触发换向保护。
 *             转弯时内轮速度 = SPEED - STEER = 350，要明显大于电机死区。
 *             600/250 是架空空载实测的起步值，落地加负载后必须重调。
 *
 * SEARCH_STEER  找不到球时原地右转的转速（正值 = 右转）
 * ================================================================== */
#define AIM_X            400
#define DEADZONE         30
#define TRACK_SPEED      900
#define TRACK_STEER      400
#define SEARCH_STEER     400

/* ==================== 收不到坐标时 ============
 *
 * K230 只管"看见球就报坐标，看不见就闭嘴" 丢球之后怎么走由这里决定
 * 判断依据只有一个：距离上次收到坐标过了多久（lost_ms）。
 *
 *         0 ──── LOST_DEAD_MS ──── LOST_FORWARD_MS ────▶ 时间
 *         │           │                  │
 *       正常跟踪    刚丢几帧          球真没了
 *                  什么都不做        继续直行冲过去    超出 → 原地右转找
 *                  （当抖动）
 *
 * LOST_DEAD_MS    抖动容忍。K230 的框偶尔会闪一下丢掉一两帧
 *                 这么短的时间里不动方向盘，免得车跟着抖
 *                 调大 → 更稳，但球真丢了反应变慢
 *
 * LOST_FORWARD_MS  球从画面下边沿消失后【继续直行】多久（毫秒）
 *                 这段是拿去"冲过去把球扫进框"的
 *                 球在视野里消失时车还没开到它跟前，不冲这一段就永远收不到
 *                 调大 → 冲得远，球容易进框，但可能冲过头、撞墙、掉下桌
 *                 调小 → 收不到球就开始转圈找，白跑一趟
 *                    这是个【距离】不是时间！LOST_FORWARD_MS × TRACK_SPEED
 *                    就是"冲多远"。架空(空载)和落地(带负载)差得很远
 *                    落地后必须实测。TRACK_SPEED 越大冲得越远。
 * ========================================================================= */
#define LOST_DEAD_MS     300

/*球消失后【直行】的秒数 —— 就想改这个数
 * 5000 = 5 秒。想让车冲得久一点就调大，冲得太远会撞墙/掉下桌就调小 */
#define LOST_FORWARD_MS  2000

int main(void)
{
    uint32_t last_motor_tick = 0;
    uint32_t last_rx_tick    = 0;       //最后一次收到坐标的时刻
    uint32_t lost_ms         = 0;       //距离上次收到坐标过了多久
    uint8_t  msg;
    int16_t  ball_x = 0;
    int16_t  ball_y = 0;

    Tick_Init();                        //时间基准最先起，后面都用它计时
    Motor_Init();
    USART3_Init();                      //摄像头串口（必须放在 Tick_Init 之后）

    /* 上电时还没见过球，应该直接去找，而不是先傻往前冲一段
     * 把"上次收到坐标的时刻"往回拨 LOST_FORWARD_MS
     * 等价于告诉后面的判断"球早就丢了" → 一上电就进入原地右转搜索
     * （unsigned 减法在溢出时是回绕的，now - last_rx_tick 照样算得对）*/
    last_rx_tick = Tick_Get() - LOST_FORWARD_MS;
    SelfTest_Init();

    

    /* ==================== 方向测试 ====================
     *    而且会真的驱动电机。跑跟踪必须把它注释掉，
     *    要重测方向时再临时打开（记得把下面的跟踪逻辑一起关掉）。
     * 轮子架空看方向；落地要有空间，或者先架空
     * ================================================ */
    // /* ① 前进 */         Motor_Drive( 600,    0);  Motor_RunFor(2000);  Motor_Stop();  Motor_RunFor(800);
    // /* ② 后退 */         Motor_Drive(-600,    0);  Motor_RunFor(2000);  Motor_Stop();  Motor_RunFor(800);
    // /* ③ 原地右转 */     Motor_Drive(   0,  600);  Motor_RunFor(2000);  Motor_Stop();  Motor_RunFor(800);
    // /* ④ 原地左转 */     Motor_Drive(   0, -600);  Motor_RunFor(2000);  Motor_Stop();  Motor_RunFor(800);
    // /* ⑤ 前进右弧线 */   Motor_Drive( 600,  250);  Motor_RunFor(2000);  Motor_Stop();  Motor_RunFor(800);
    // /* ⑥ 前进左弧线 */   Motor_Drive( 600, -250);  Motor_RunFor(2000);  Motor_Stop();  Motor_RunFor(800);
    // /* ⑦ 后退右弧线 */   Motor_Drive(-600,  250);  Motor_RunFor(2000);  Motor_Stop();  Motor_RunFor(800);
    // /* ⑧ 后退左弧线 */   Motor_Drive(-600, -250);  Motor_RunFor(2000);  Motor_Stop();  Motor_RunFor(800);

    while (1)
    {
        uint32_t now = Tick_Get();

        /* 电机节拍：每 5ms 喂一次，斜率限制按这个周期算 */
        if (now - last_motor_tick >= MOTOR_TICK_MS)
        {
            last_motor_tick = now;
            Motor_Task();
        }

        /* 自检剧本：跑一遍就返回，不阻塞
         它和跟踪逻辑抢方向盘（两边都往 motor_tgt[] 写）
         调试跟踪时必须保持注释状态 */
        //SelfTest_Task(now);

        /* ---------------- 摄像头串口：收坐标 ---------------- */
        lost_ms = now - last_rx_tick;       /* 距离上次看见球，过了多久 */

        msg = USART3_Poll(&ball_x, &ball_y);

        if (msg == UART3_MSG_COORD)
        {
            /* 看见球了。刷新计时，再按横向偏差决定往哪拐 */
            last_rx_tick = now;
            lost_ms = 0;

            /* 球偏右 → 右弧线；球偏左 → 左弧线；对正了 → 直行前进
             * "对正"是直行前进，不是停车 —— 球框拖在车后面，
             *  得开过去才能把它扫进框里 */
            if (ball_x > AIM_X + DEADZONE)
            {
                Motor_Drive(TRACK_SPEED, TRACK_STEER);
            }
            else if (ball_x < AIM_X - DEADZONE)
            {
                Motor_Drive(TRACK_SPEED, -TRACK_STEER);
            }
            else
            {
                Motor_Drive(TRACK_SPEED, 0);
            }
        }
        else if (lost_ms < LOST_DEAD_MS)
        {
            /* 刚丢几帧（K230 的框偶尔会闪一下）—— 什么都不做，保持上一条指令。
             * 这里要是跟着改方向，车就会跟着检测一起抖 */
        }
        else if (lost_ms < LOST_FORWARD_MS)
        {
            /* 球从画面下边沿消失 → 继续直行冲过去。
             * 这一段就是"收球"：球在视野里消失时车还没开到它跟前，
             * 不冲这一段就永远收不到 */
            Motor_Drive(TRACK_SPEED, 0);
        }
        else
        {
            /* 冲了 LOST_FORWARD_MS 还是没再看见球 → 原地右转，找下一颗。
             * 到这里车就再也不会停了：要么在跟球、要么在冲、要么在转着找 */
            Motor_Spin(SEARCH_STEER);
        }
    }
}
