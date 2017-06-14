#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "serialManager.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32f10x.h"

void uart1_printf(char *fmt, ...);
void uart2_printf(char *fmt, ...);

static void initUart1Peri(void)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
    USART_DeInit(USART1);  //复位串口1
    //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9

    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10
    //USART 初始化设置
    USART_InitStructure.USART_BaudRate = 57600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART1, &USART_InitStructure); //初始化串口

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ; //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3

    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启接收中断

    USART_Cmd(USART1, ENABLE);	//使能串口
}

static void initUart2Peri()
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB2Periph_GPIOA, ENABLE);
    USART_DeInit(USART2);  //复位串口2

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA2

    //USART2_RX	  PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA3

    //USART 初始化设置
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

    USART_Init(USART2, &USART_InitStructure); //初始化串口

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ; //抢占优先级3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级4

    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
    NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断

    USART_Cmd(USART2, ENABLE);	//使能串口
}


void USART1_IRQHandler(void)                	//串口1中断服务程序
{
    u8 Res;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断
    {
        Res = USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
    
    }
}

void USART2_IRQHandler(void)                	//串口2中断服务程序
{
    u8 Res;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
    {
        Res = USART_ReceiveData(USART2); //(USART1->DR);	//读取接收到的数据
    }
}

void serialMngInit()
{
    initUart1Peri();
    initUart2Peri();
}

void sendString_uart2(char *str)
{
    while(*str)
    {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
        USART_SendData(USART2, *str++);
    }
}

void sendString_uart1(char *str)
{
    while(*str)
    {
        while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
        USART_SendData(USART1, *str++);
    }
}

void uart1_printf(char *fmt, ...)
{
    va_list ap;
    char string[32];
    va_start(ap, fmt);
    vsprintf(string, fmt, ap);
    sendString_uart1(string);
    va_end(ap);
}

void uart2_printf(char *fmt, ...)
{
    va_list ap;
    char string[32];
    va_start(ap, fmt);
    vsprintf(string, fmt, ap);
    sendString_uart2(string);
    va_end(ap);
}

void uart2write(char *str, int len)
{
    while(len--)
    {
        USART_SendData(USART2, *str++);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET);
    }
}

int fputc(int ch, FILE *f)
{
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    USART_SendData(USART1, (uint8_t) ch);
    return ch;
}
