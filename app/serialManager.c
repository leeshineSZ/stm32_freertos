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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
    USART_DeInit(USART1);  //��λ����1
    //USART1_TX   PA.9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA9

    //USART1_RX	  PA.10
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA10
    //USART ��ʼ������
    USART_InitStructure.USART_BaudRate = 57600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART1, &USART_InitStructure); //��ʼ������

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ; //��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3

    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//���������ж�

    USART_Cmd(USART1, ENABLE);	//ʹ�ܴ���
}

static void initUart2Peri()
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB2Periph_GPIOA, ENABLE);
    USART_DeInit(USART2);  //��λ����2

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
    GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA2

    //USART2_RX	  PA.3
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
    GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA3

    //USART ��ʼ������
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
    USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
    USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

    USART_Init(USART2, &USART_InitStructure); //��ʼ������

    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2 ; //��ռ���ȼ�3
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�4

    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
    NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�

    USART_Cmd(USART2, ENABLE);	//ʹ�ܴ���
}


void USART1_IRQHandler(void)                	//����1�жϷ������
{
    u8 Res;
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�
    {
        Res = USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
    
    }
}

void USART2_IRQHandler(void)                	//����2�жϷ������
{
    u8 Res;
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //�����ж�
    {
        Res = USART_ReceiveData(USART2); //(USART1->DR);	//��ȡ���յ�������
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
