#include "board.h"
#include "usart.h"

/**********************************************************
***     Emm_V5.0步进闭环控制例程
***     编写作者：ZHANGDATOU
***     技术支持：张大头闭环伺服
***     淘宝店铺：https://zhangdatou.taobao.com
***     CSDN博客：https://blog.csdn.net/zhangdatou666
***     qq交流群：262438510
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
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_Init(&NVIC_InitStructure);
        NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
        NVIC_Init(&NVIC_InitStructure);
        NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
        NVIC_Init(&NVIC_InitStructure);
        NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
        NVIC_Init(&NVIC_InitStructure);
}

/**
        *       @brief          外设时钟初始化
        *       @param          无
        *       @retval         无
        */
void clock_init(void)
{
        // 使能串口使用的GPIO外设时钟
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
                               RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD |
                               RCC_AHB1Periph_GPIOG, ENABLE);

        // 使能USART1、USART2、USART3、UART4、UART5、USART6外设时钟
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_UART4 |
                               RCC_APB1Periph_UART5 | RCC_APB1Periph_USART3, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
}

/**
        * @brief   初始化USART3 (PD8-TX, PD9-RX)
        * @param   无
        * @retval  无
        */
void usart_init(void)
{
USART_InitTypeDef USART_InitStructure;

/**********************************************************
*** 初始化USART1调试串口: PA9 - TX, PA10 - RX
**********************************************************/
        GPIO_InitTypeDef  GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        // 将PA9、PA10复用为USART1 (AF7)
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

        USART_InitStructure.USART_BaudRate = 115200;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
        USART_Init(USART1, &USART_InitStructure);
        USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
        USART_Cmd(USART1, ENABLE);

        // USART2：PA2-TX、PA3-RX，串口屏，9600 8N1
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
        USART_InitStructure.USART_BaudRate = 9600;
        USART_Init(USART2, &USART_InitStructure);
        USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
        USART_Cmd(USART2, ENABLE);

        // UART4：PC10-TX、PC11-RX，雷达，460800 8N1
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);
        USART_InitStructure.USART_BaudRate = 460800;
        USART_Init(UART4, &USART_InitStructure);
        USART_Cmd(UART4, ENABLE);

        // UART5：PC12-TX、PD2-RX，扫码模块，9600 8N1
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
        GPIO_Init(GPIOD, &GPIO_InitStructure);
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);
        USART_InitStructure.USART_BaudRate = 9600;
        USART_Init(UART5, &USART_InitStructure);
        USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
        USART_Cmd(UART5, ENABLE);

        // USART6：PG14-TX、PG9-RX，鲁班猫，115200 8N1
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_9;
        GPIO_Init(GPIOG, &GPIO_InitStructure);
        GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);
        GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);
        USART_InitStructure.USART_BaudRate = 115200;
        USART_Init(USART6, &USART_InitStructure);
        USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
        USART_Cmd(USART6, ENABLE);

/**********************************************************
***     初始化USART3引脚: PD8 - TX, PD9 - RX
**********************************************************/
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;                                    /* 复用推挽输出 */
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;                                    //上拉
        GPIO_Init(GPIOD, &GPIO_InitStructure);

        // 将PD8、PD9复用为USART3 (AF7)
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource8, GPIO_AF_USART3);
        GPIO_PinAFConfig(GPIOD, GPIO_PinSource9, GPIO_AF_USART3);

/**********************************************************
***     初始化USART3
**********************************************************/
        USART_InitStructure.USART_BaudRate = 115200;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_No;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
        USART_Init(USART3, &USART_InitStructure);

/**********************************************************
***     清除USART3中断
**********************************************************/
        USART3->SR; USART3->DR;
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);

/**********************************************************
***     使能USART3中断
**********************************************************/     
        USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
        USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);

/**********************************************************
***     使能USART3
**********************************************************/
        USART_Cmd(USART3, ENABLE);
}

/**
        *       @brief          板载初始化
        *       @param          无
        *       @retval         无
        */
void board_init(void)
{
        nvic_init();
        clock_init();
        usart_init();
        // 上电立即输出，确认USART1调试串口初始化和接线正常
        usart1_SendString("[SYS] USART1 ready\r\n");
}
