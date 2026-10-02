#ifndef __USART3_H__
#define __USART3_H__

#include "stm32f10x.h"

#define USART3_Rx_BufSize 64

/* ==================== 摄像头串口（PB10/PB11，115200 8N1）====================
 *
 *
 *       K230                STM32 本板        
 *       ────────────────    ──────────────    
 *       TX (IO3)   ───────▶ PB11              
 *       RX (IO4)   ◀─────── PB10              
 *       GND        ───────── GND              
 *
 *
 *   为什么用 USART3 而不是 USART1：对照原理图核对过，**板子上引出来的
 *    那组串口（H4 排针）接的就是 USART3 的 PB10/PB11**。USART1 的 PA9/PA10
 *    另有去处（板载 CH32），外接设备挂上去会打架。
 *
 *  两条不能改的语义：
 *   1. 没数据 ≠ 命令。"没收到"本身就是信息（球没了），
 *      但绝不能把它当成某个具体指令去执行
 *   2. 乱码/半个包不能当有效数据（会让车朝假方向冲出去）
 * ==================================================================== */

/* USART3_Poll 的返回值 */
#define UART3_MSG_NONE    0   /* 还没攒够一整行 —— 什么都不做 */
#define UART3_MSG_COORD   1   /* 收到坐标，此时 x/y 有效 */
#define UART3_MSG_CMD     2   /* 收到指令 此时cmd有效*/
#define UART3_MSG_BAD     3   /* 收到一整行但格式不对（乱码）—— 什么都不做 */ 

/*手动遥控指令*/
#define ROBOT_CMD_STOP      0
#define ROBOT_CMD_FWD       1
#define ROBOT_CMD_BACK      2
#define ROBOT_CMD_LEFT      3
#define ROBOT_CMD_RIGHT     4
#define ROBOT_CMD_MANUAL    5
#define ROBOT_CMD_AUTO      6

/* 必须放在 Tick_Init() 之后 —— 中断优先级分组是在 Tick_Init 里设的*/
void USART3_Init(void);

/* 取一个已收到的字节：返回 1 = 取到了，0 = 缓冲空（非阻塞） */
uint8_t USART3_ReadByte(uint8_t *data);

/* 在主循环里反复调用：把环形缓冲里的字节吃干净，攒够一整行就解析。
 * 返回最近一行的解析结果，x/y 只在返回 UART3_MSG_COORD 时有效。
 * 一次调用里若收到多行，只保留最后一行的结果（旧命令已经过期了）。*/
uint8_t USART3_Poll(int16_t *x, int16_t *y,uint8_t *cmd);

/* ---- 发（阻塞）：本口是收为主的，这几个是调试打印用的 ----
 *  这几个刻意不用 printf：本工程没开 microLIB，printf 会走"半主机"模式
 *    把字符发给调试器，没接调试器时直接卡死在这儿。
 *
 *  用的时候注意：PB10 接的是 K230 的 RX，发出去的东西 K230 不读（它只写）。
 *    想用串口助手看，得把 K230 拔了、USB-TTL 接 PB10/PB11。
 */
void USART3_SendByte(uint8_t data);     /* 发一个字节 */
void USART3_SendStr(const char *s);     /* 发一个字符串 */
void USART3_SendNum(int32_t v);         /* 发一个十进制整数，支持负数 */

#endif
