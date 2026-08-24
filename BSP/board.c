#include "board.h"

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	编写作者：ZHANGDATOU
***	技术支持：张大头闭环伺服
***	淘宝店铺：https://zhangdatou.taobao.com
***	CSDN博客：https://blog.csdn.net/zhangdatou666
***	qq交流群：262438510
**********************************************************/

/**
	* @brief   配置NVIC控制器
	* @param   无
	* @retval  无
	*/
void nvic_init(void)
{	
	// 4bit抢占优先级位
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5; // FreeRTOS安全中断优先级
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

/**
	*	@brief		外设时钟初始化
	*	@param		无
	*	@retval		无
	*/
void clock_init(void)
{
	// 使能GPIOA、GPIOB、GPIOD外设时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOD, ENABLE);

	// 使能USART3外设时钟 (APB1总线)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
}

/**
	* @brief   初始化USART3 (PD8-TX, PD9-RX)
	* @param   无
	* @retval  无
	*/
void usart_init(void)
{
/**********************************************************
***	初始化USART3引脚: PD8 - TX, PD9 - RX
**********************************************************/
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;					/* 复用推挽输出 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					//上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// 将PD8、PD9复用为USART3 (AF7)
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

/**********************************************************
***	初始化USART3
**********************************************************/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART3, &USART_InitStructure);

/**********************************************************
***	清除USART3中断
**********************************************************/
	USART3->SR; USART3->DR;
	USART_ClearITPendingBit(USART3, USART_IT_RXNE);

/**********************************************************
***	使能USART3中断
**********************************************************/	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

/**********************************************************
***	使能USART3
**********************************************************/
	USART_Cmd(USART3, ENABLE);
}

/**
	*	@brief		板载初始化
	*	@param		无
	*	@retval		无
	*/
void board_init(void)
{
	nvic_init();
	clock_init();
	usart_init();
}
