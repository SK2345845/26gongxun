#ifndef __USART_H
#define __USART_H

#include "board.h"
#include "fifo.h"

/**********************************************************
***	Emm_V5.0�����ջ���������
***	��д���ߣ�ZHANGDATOU
***	����֧�֣��Ŵ�ͷ�ջ��ŷ�
***	�Ա����̣�https://zhangdatou.taobao.com
***	CSDN���ͣ�http s://blog.csdn.net/zhangdatou666
***	qq����Ⱥ��262438510
**********************************************************/

extern __IO bool rxFrameFlag;
extern __IO uint8_t rxCmd[FIFO_SIZE];
extern __IO uint8_t rxCount;

#define USART1_RX_BUF_SIZE 64
extern __IO uint8_t usart1_rx_buf[USART1_RX_BUF_SIZE];
extern __IO uint8_t usart1_rx_head;
extern __IO uint8_t usart1_rx_tail;

#define USART_EXT_RX_BUF_SIZE 256
extern __IO uint8_t uart5_rx_buf[USART_EXT_RX_BUF_SIZE];
extern __IO uint16_t uart5_rx_head;
extern __IO uint16_t uart5_rx_tail;
extern __IO uint8_t usart6_rx_buf[USART_EXT_RX_BUF_SIZE];
extern __IO uint16_t usart6_rx_head;
extern __IO uint16_t usart6_rx_tail;

void usart_SendCmd(__IO uint8_t *cmd, uint8_t len);
void usart_SendByte(uint16_t data);

// USART1调试串口：PA9-TX、PA10-RX，115200 8N1
void usart1_SendByte(uint16_t data);
void usart1_SendString(const char *str);
uint8_t usart1_ReadByte(uint8_t *data);

// 扩展串口：USART2 PA2/PA3、UART4 PC10/PC11、UART5 PC12/PD2、USART6 PG14/PG9
void usart2_SendByte(uint16_t data);
void uart4_SendByte(uint16_t data);
void uart5_SendByte(uint16_t data);
void usart6_SendByte(uint16_t data);
void usart2_SendString(const char *str);
void uart4_SendString(const char *str);
void uart5_SendString(const char *str);
void usart6_SendString(const char *str);
void uart4_SendData(const uint8_t *data, uint16_t len);
void uart5_SendData(const uint8_t *data, uint16_t len);
void usart6_SendData(const uint8_t *data, uint16_t len);
uint8_t uart5_ReadByte(uint8_t *data);
uint8_t usart6_ReadByte(uint8_t *data);

#endif
