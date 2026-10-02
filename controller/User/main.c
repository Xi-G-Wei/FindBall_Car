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
 * TRACK_SPEED      跟踪时的前进速度
 *                  调大 → 追得快，但配合转向更容易冲过头（见下面转向那段）
 *
 * SEARCH_STEER  找不到球时原地右转的转速（正值 = 右转）
 * ================================================================== */
#define AIM_X            400
#define DEADZONE         30
#define TRACK_SPEED      800
#define SEARCH_STEER     300

/* ==================== 转向力度（治"拐弯过头"）====================
 *      转向量和偏差成正比，偏差小了自动收力：
 *      偏差大  →  转得猛   （快速对准）
 *      偏差小  →  转得轻   （慢慢贴上去，不会冲过）
 *      偏差≈0  →  不转，直行
 *
 * 实际算的式子：
 *      偏差 = ball_x - AIM_X
 *      偏差在 ±DEADZONE 以内 → 当作 0（检测抖动，别理它）
 *      超出的部分 × STEER_GAIN → 就是转向量，再限幅到 ±TRACK_STEER_MAX
 *
 * TRACK_STEER_MAX  打得最猛时的转向量（原来 TRACK_STEER 的角色）
 *                  调大 → 大偏差时更果断，但收尾容易过头
 *                  调小 → 稳，但球跑到边上的时候绕大圈
 *                  别超过 TRACK_SPEED，否则内侧轮会倒转
 *
 * STEER_GAIN       偏差放大多少倍
 *                  调大 → 同样的偏差转得更猛（收尾变急、更容易过头）
 *                  调小 → 转得温柔（不容易过头，但大偏差时反应肉）
 *                  换算出"什么时候开始满舵"：
 *                      满舵偏差 = TRACK_STEER_MAX ÷ STEER_GAIN
 *                      现在 = 400 ÷ 2 = 200 像素
 *                      也就是球偏出中心 200 像素以上，都按满舵转
 * ============================================================== */
#define TRACK_STEER_MAX    400
#define STEER_GAIN         2

/* ==================== 搜索时给右后轮补力矩 ====================
 *
 * 球筐挂在车尾，后轮被压得很重。原地右转时左轮往前、右轮往后，
 * 右后轮一边扛着筐的重量、一边还要往后拖，阻力是四个轮子里最大的 ——
 * SEARCH_STEER 那点占空比推不动它，轮子堵转，车就转不起来。
 *
 * 所以搜索时不走 Motor_Spin（它四个轮子给一样大），改用第 2 层的
 * Motor_SetSpeed 一个个写：左右还是对称转向，只把【右后轮】单独放大。
 *
 * SEARCH_BOOST_BR  右后轮多给多少占空比
 *                  0    → 和别的轮子一样，退回原来 Motor_Spin 的行为
 *                  150  → 起步建议值，右后轮 = 300 + 150 = 450
 *                  加到"能转起来"为止，别一次加太猛
 *
 *    堵转 = 电流很大 = 发热。如果加到 600 还是不动，说明不是"给得不够"，
 *    是负载已经超过这个电机的力矩上限了 —— 再加会烧电机或者烧 DRV8701。
 *    那种情况该动机械：球筐往前挪一点，或者在车头加点配重。
 * ========================================================== */
#define SEARCH_BOOST_BR  150

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
#define LOST_FORWARD_MS  1500
#define MANUAL_SPEED     800    /*手动行进速度*/
#define MANUAL_TURN      500    /*原地转向量*/
#define MANUAL_LOST_MS   500    /*未收到指令超时时间500ms  会自动停车*/

int main(void)
{
    uint32_t last_motor_tick = 0;
    uint32_t last_rx_tick = 0;       //最后一次收到坐标的时刻
    uint32_t lost_ms = 0;       //距离上次收到坐标过了多久
    uint32_t last_cmd_tick = 0;     //最后一次收到指令时间  
    uint8_t  msg;
    uint8_t manual = 0;     //0=自动  1=手动  上电默认自动
    uint8_t cmd = 0;        //UART3_Poll带出的指令码
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
    USART3_SendStr("Hello");
    

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

        msg = USART3_Poll(&ball_x, &ball_y,&cmd);
        //用于调试
        // if (msg != UART3_MSG_NONE)
        //   {
        //       USART3_SendStr("msg=");
        //       USART3_SendNum(msg);
        //       USART3_SendStr("\n");
        //   }

        if(msg==UART3_MSG_CMD)
        {
            //更新时间
            last_cmd_tick=now;
            if(cmd==ROBOT_CMD_MANUAL)
            {
                manual=1;
                Motor_Stop();
            }
            else if(cmd==ROBOT_CMD_AUTO)
            {
                manual=0;
                last_rx_tick=now-LOST_FORWARD_MS;
            }
            else if(manual)
            {
                switch(cmd)
                {
                    case ROBOT_CMD_FWD:     Motor_Drive(MANUAL_SPEED,0);   break;
                    case ROBOT_CMD_BACK:    Motor_Drive(-MANUAL_SPEED,0);  break;
                    case ROBOT_CMD_LEFT:    Motor_Drive(0,-MANUAL_TURN);   break;
                    case ROBOT_CMD_RIGHT:   Motor_Drive(0,MANUAL_TURN);    break;
                    case ROBOT_CMD_STOP:    Motor_Stop();                  break;
                    default: break;
                }
            }
        }
        else if(manual)
        {
           if(now-last_cmd_tick>=MANUAL_LOST_MS)
           {
            Motor_Stop();
           }
        }
        else if(msg == UART3_MSG_COORD)
        {
            /* 看见球了。刷新计时，再按横向偏差决定往哪拐 */
            last_rx_tick = now;
            lost_ms = 0;

            /* ---- 比例转向：偏差越大转得越猛 ----
             * 算法见文件上面 TRACK_STEER_MAX / STEER_GAIN 那段说明。
             * 球偏右 → err 正 → 右弧线；偏左 → err 负 → 左弧线；
             * 对正了 → 转向量算出来就是 0 → 直行前进
             *
             *   "对正"是直行前进，不是停车 —— 球框拖在车后面，
             *    得开过去才能把它扫进框里 */
            int16_t err = (int16_t)(ball_x - AIM_X);

            /* 死区：偏差很小的时候当没看见，免得到处跟着检测抖动一起抖。
             * 注意是【减掉】死区而不是"直接归零" —— 直接归零的话，
             * 偏差从 DEADZONE 跨出去的那一下转向量会从 0 跳到一大截，
             * 车会一顿一顿的 */
            if      (err >  DEADZONE) { err = (int16_t)(err - DEADZONE); }
            else if (err < -DEADZONE) { err = (int16_t)(err + DEADZONE); }
            else                      { err = 0; }

            int32_t steer = (int32_t)err * STEER_GAIN;

            if      (steer >  TRACK_STEER_MAX) { steer =  TRACK_STEER_MAX; }
            else if (steer < -TRACK_STEER_MAX) { steer = -TRACK_STEER_MAX; }

            Motor_Drive(TRACK_SPEED, (int16_t)steer);
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
             * 到这里车就再也不会停了：要么在跟球、要么在冲、要么在转着找
             *
             * 没用 Motor_Spin(SEARCH_STEER)：右后轮被球筐压得堵转，
             * 单独给它多加一点占空比（原因见 SEARCH_BOOST_BR 那段说明）。
             *   左轮往前、右轮往后 —— 符号弄反车就往左转了 */
            Motor_SetSpeed(MOTOR_FL,  SEARCH_STEER);
            Motor_SetSpeed(MOTOR_BL,  SEARCH_STEER);
            Motor_SetSpeed(MOTOR_FR, -SEARCH_STEER);
            Motor_SetSpeed(MOTOR_BR, -(SEARCH_STEER + SEARCH_BOOST_BR));
        }
    }
}
