#include "stm32f10x.h"
#include "USART3.h"

static volatile uint8_t  Rx_Buf[USART3_Rx_BufSize];
static volatile uint16_t Rx_Head = 0;
static volatile uint16_t Rx_Tail = 0;

/* ---------------- 行缓冲：攒够一整行才解析 ----------------
 * 为什么必须攒行：坐标是变长的（"5,7" 4 字节 / "412,238" 8 字节），
 * 读一个字节根本判断不出它是不是一条完整消息 —— 必须等到 '\n' 才敢动手。
 * 只在主循环（USART3_Poll）里用，中断不碰它。*/
#define Line_BufSize 16
static char    Line_Buf[Line_BufSize];
static uint8_t Line_Len = 0;

void USART3_Init(void)
{
    /* USART3 挂在 APB1 上（USART1 在 APB2），别抄错总线！
     *    GPIO 的时钟永远在 APB2，不管外设挂哪条总线 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    /* PB10 = TX，复用推挽输出 */
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* PB11 = RX，浮空输入（K230 是推挽输出在驱动它，这里不需要上下拉） */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* PB10/PB11 就是 USART3 的默认引脚，不需要 AFIO 重映射 */

    USART_InitTypeDef USART_InitStructure;
    USART_InitStructure.USART_BaudRate            = 115200;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity              = USART_Parity_No;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;

    USART_Init(USART3, &USART_InitStructure);

    /* 开启串口中断 */
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    /* 初始化 NVIC */
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel                   = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART3, ENABLE);
}

/* 中断只干一件事：把字节塞进环形缓冲，立刻退出。
 * 解析放到主循环做 —— 中断里待久了会拖慢 Tick 和电机节拍。*/
void USART3_IRQHandler(void)
{
    if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
    {
        uint8_t  Data = (uint8_t)USART_ReceiveData(USART3);
        uint16_t Next = (Rx_Head + 1) & (USART3_Rx_BufSize - 1);

        if(Next != Rx_Tail)             /* 满了就丢，不覆盖还没读走的旧数据 */
        {
            Rx_Buf[Rx_Head] = Data;
            Rx_Head = Next;
        }
    }
}

uint8_t USART3_ReadByte(uint8_t *Data)
{
    if(Rx_Head == Rx_Tail)
    {
        return 0;
    }

    *Data = Rx_Buf[Rx_Tail];
    Rx_Tail = (Rx_Tail + 1) & (USART3_Rx_BufSize - 1);
    return 1;
}

/**
  * @brief  把攒好的一行拆成坐标
  * @param  x, y  传出：解析出的坐标（只在返回 UART3_MSG_COORD 时有意义）
  * @return UART3_MSG_COORD / UART3_MSG_BAD
  *
  *   绝对不能用 atoi() 它对乱码返回 0，看起来就像"球在 (0,0)"，
  *    主控会朝一个假方向冲出去。所以这里逐字符校验，一个非法字符就整行作废。
  *
  */
static uint8_t Parse_Line(int16_t *x, int16_t *y,uint8_t *cmd)
{
    uint8_t i;
    uint8_t seen_comma = 0;
    int32_t vx = 0, vy = 0;

    if(Line_Buf[0] == '\0')
    {
        return UART3_MSG_BAD;                   /* 空行 */
    }

    //收到指令
    if(Line_Buf[0]=='M')
    {
        //判断格式错误出现乱码
        if(Line_Buf[1] == '\0' || Line_Buf[2] != '\0')
        {
            return UART3_MSG_BAD;
        }
        switch(Line_Buf[1])
        {
            case 'f': *cmd=ROBOT_CMD_FWD;    break;
            case 'b': *cmd=ROBOT_CMD_BACK;   break;
            case 'l': *cmd=ROBOT_CMD_LEFT;   break;
            case 'r': *cmd=ROBOT_CMD_RIGHT;  break;
            case 's': *cmd=ROBOT_CMD_STOP;   break;
            case 'M': *cmd=ROBOT_CMD_MANUAL; break;
            case 'A': *cmd=ROBOT_CMD_AUTO;   break;
            default:   return UART3_MSG_BAD;
        }
        return UART3_MSG_CMD;
    }

    /* ---- 坐标："数字,数字" ---- */
    for(i = 0; Line_Buf[i] != '\0'; i++)
    {
        char c = Line_Buf[i];

        if(c == ',')
        {
            if(seen_comma)
            {
                return UART3_MSG_BAD;           /* 出现两个逗号 */
            }
            seen_comma = 1;
        }
        else if(c >= '0' && c <= '9')
        {
            if(seen_comma)
            {
                vy = vy * 10 + (c - '0');
                if(vy > 9999) return UART3_MSG_BAD;   /* 位数太多 = 不对 */
            }
            else
            {
                vx = vx * 10 + (c - '0');
                if(vx > 9999) return UART3_MSG_BAD;
            }
        }
        else
        {
            return UART3_MSG_BAD;               /* 非数字非逗号 = 乱码 */
        }
    }

    if(!seen_comma)
    {
        return UART3_MSG_BAD;                   /* 没有逗号，不是坐标 */
    }

    *x = (int16_t)vx;
    *y = (int16_t)vy;
    return UART3_MSG_COORD;
}

/**
  * @brief  主循环里反复调用：吃干净环形缓冲，攒够一行就解析
  * @param  x, y  传出：最近一次坐标（只在返回 UART3_MSG_COORD 时有效）
  * @return 最近一行的解析结果，见 USART3.h 里那几个 UART3_MSG_xxx
  *
  * 一次调用里如果收了好几行，只留 最后一行 的结果 ——
  * 前面的命令已经过期了，拿旧的去开车等于用历史位置导航。
  */
uint8_t USART3_Poll(int16_t *x, int16_t *y,uint8_t *cmd)
{
    uint8_t b;
    uint8_t result = UART3_MSG_NONE;

    while(USART3_ReadByte(&b))
    {
        if(b == '\n' || b == '\r')
        {
            if(Line_Len > 0)
            {
                Line_Buf[Line_Len] = '\0';
                result = Parse_Line(x, y,cmd);      /* 覆盖式，只留最新 */
                Line_Len = 0;
            }
            /* Line_Len == 0 说明是 "\r\n" 里的第二个字符，忽略 */
        }
        else if(Line_Len < Line_BufSize - 1)
        {
            Line_Buf[Line_Len++] = (char)b;
        }
        else
        {
            /* 攒到上限还没等到换行 = 乱码 / 波特率不对 / 线接错
             * 丢弃重来。不丢的话缓冲会被写爆，之后永远解析不出东西 */
            Line_Len = 0;
        }
    }

    return result;
}

/**
  * @brief  发一个字节
  * @note   阻塞式（等 TXE）。本口以收为主，这是给调试打印留的
  */
void USART3_SendByte(uint8_t Data)
{
    USART_SendData(USART3, Data);
    while(USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
}

/**
  * @brief  发一个字符串（阻塞）
  * @note   调试打印用，比一个个发字节方便。
  *         刻意不用 printf：本工程没开 microLIB，printf 走半主机模式，
  *         没接调试器时直接卡死。
  */
void USART3_SendStr(const char *s)
{
    while(*s != '\0')
    {
        USART3_SendByte((uint8_t)*s);
        s++;
    }
}

/**
  * @brief  发一个十进制整数（阻塞），支持负数
  * @note   调试打印用 坐标都是正数，负号是给以后别的调试信息留的
  */
void USART3_SendNum(int32_t v)
{
    char    tmp[12];
    uint8_t i = 0;

    if(v < 0)
    {
        USART3_SendByte('-');
        v = -v;
    }

    if(v == 0)
    {
        USART3_SendByte('0');
        return;
    }

    while(v > 0 && i < sizeof(tmp))
    {
        tmp[i++] = (char)('0' + (v % 10));
        v /= 10;
    }

    while(i > 0)                    /* 低位先算出来，所以倒着发回去 */
    {
        USART3_SendByte((uint8_t)tmp[--i]);
    }
}
