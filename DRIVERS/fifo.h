#ifndef __FIFO_H
#define __FIFO_H

#include "stm32f4xx.h"

/**********************************************************
***	Emm_V5.0ջ
***	дߣZHANGDATOU
***	֧֣Ŵͷջŷ
***	Ա̣https://zhangdatou.taobao.com
***	CSDNͣ https://blog.csdn.net/zhangdatou666
***	qqȺ262438510
**********************************************************/

#define FIFO_SIZE 128
typedef struct
{
	uint16_t buffer[FIFO_SIZE];
	__IO uint8_t ptrWrite;
	__IO uint8_t ptrRead;
} FIFO_t;

extern __IO FIFO_t rxFIFO;

void fifo_initQueue(void);
void fifo_enQueue(uint16_t data);
uint16_t fifo_deQueue(void);
bool fifo_isEmpty(void);
uint16_t fifo_queueLength(void);

#endif
