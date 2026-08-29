#include "usart.h"

/**********************************************************
***     Emm_V5.0步进闭环控制例程
***     编写作者：ZHANGDATOU
***     技术支持：张大头闭环伺服
***     淘宝店铺：https://zhangdatou.taobao.com
***     CSDN博客：https://blog.csdn.net/zhangdatou666
***     qq交流群：262438510
**********************************************************/

__IO bool rxFrameFlag = false;
__IO uint8_t rxCmd[FIFO_SIZE] = {0};
__IO uint8_t rxCount = 0;
__IO uint8_t usart1_rx_buf[USART1_RX_BUF_SIZE] = {0};
__IO uint8_t usart1_rx_head = 0;
__IO uint8_t usart1_rx_tail = 0;
__IO uint8_t uart5_rx_buf[USART_EXT_RX_BUF_SIZE] = {0};
__IO uint16_t uart5_rx_head = 0;
__IO uint16_t uart5_rx_tail = 0;
__IO uint8_t usart2_rx_buf[USART_EXT_RX_BUF_SIZE] = {0};
__IO uint16_t usart2_rx_head = 0;
__IO uint16_t usart2_rx_tail = 0;
__IO uint8_t usart6_rx_buf[USART_EXT_RX_BUF_SIZE] = {0};
__IO uint16_t usart6_rx_head = 0;
__IO uint16_t usart6_rx_tail = 0;

/**
        * @brief   USART1接收中断函数
        * @param   无
        * @retval  无
        */
void USART1_IRQHandler(void)
{
        if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
        {
                uint8_t next = (uint8_t)((usart1_rx_head + 1) % USART1_RX_BUF_SIZE);
                uint8_t data = (uint8_t)USART1->DR;

                if(next != usart1_rx_tail)
                {
                        usart1_rx_buf[usart1_rx_head] = data;
                        usart1_rx_head = next;
                }
                USART_ClearITPendingBit(USART1, USART_IT_RXNE);
        }
}

void UART5_IRQHandler(void)
{
        if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
        {
                uint16_t next = (uint16_t)((uart5_rx_head + 1) % USART_EXT_RX_BUF_SIZE);
                uint8_t data = (uint8_t)UART5->DR;
                if(next != uart5_rx_tail)
                {
                        uart5_rx_buf[uart5_rx_head] = data;
                        uart5_rx_head = next;
                }
                USART_ClearITPendingBit(UART5, USART_IT_RXNE);
        }
}

void USART2_IRQHandler(void)
{
        if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
        {
                uint16_t next = (uint16_t)((usart2_rx_head + 1) % USART_EXT_RX_BUF_SIZE);
                uint8_t data = (uint8_t)USART2->DR;
                if(next != usart2_rx_tail)
                {
                        usart2_rx_buf[usart2_rx_head] = data;
                        usart2_rx_head = next;
                }
                USART_ClearITPendingBit(USART2, USART_IT_RXNE);
        }
}

void USART6_IRQHandler(void)
{
        if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET)
        {
                uint16_t next = (uint16_t)((usart6_rx_head + 1) % USART_EXT_RX_BUF_SIZE);
                uint8_t data = (uint8_t)USART6->DR;
                if(next != usart6_rx_tail)
                {
                        usart6_rx_buf[usart6_rx_head] = data;
                        usart6_rx_head = next;
                }
                USART_ClearITPendingBit(USART6, USART_IT_RXNE);
        }
}

/**
        * @brief   USART3中断函数
        * @param   无
        * @retval  无
        */
void USART3_IRQHandler(void)
{
        __IO uint16_t i = 0;

/**********************************************************
***     串口接收中断
**********************************************************/
        if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
        {
                // 未完成一帧数据接收，数据进入缓冲队列
                fifo_enQueue((uint8_t)USART3->DR);

                // 清除串口接收中断
                USART_ClearITPendingBit(USART3, USART_IT_RXNE);
        }

/**********************************************************
***     串口空闲中断
**********************************************************/
        else if(USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
        {
                // 先读SR再读DR，清除IDLE中断
                USART3->SR; USART3->DR;

                // 提取一帧数据命令
                rxCount = fifo_queueLength(); for(i=0; i < rxCount; i++) { rxCmd[i] = fifo_deQueue(); }

                // 一帧数据接收完成，置位帧标志位
                rxFrameFlag = true;
        }
}

/**
        * @brief   USART发送多个字节
        * @param   无
        * @retval  无
        */
void usart_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
        __IO uint8_t i = 0;
        
        for(i=0; i < len; i++) { usart_SendByte(cmd[i]); }
}

/**
        * @brief   USART发送一个字节
        * @param   无
        * @retval  无
        */
void usart_SendByte(uint16_t data)
{
        __IO uint16_t t0 = 0;
        
        USART3->DR = (data & (uint16_t)0x01FF);

        while(!(USART3->SR & USART_FLAG_TXE))
        {
                ++t0; if(t0 > 8000)     {       return; }
        }
}

/**
        * @brief   USART1调试串口发送一个字节
        * @param   data: 待发送数据
        * @retval  无
        */
void usart1_SendByte(uint16_t data)
{
        __IO uint16_t t0 = 0;

        USART1->DR = (data & (uint16_t)0x01FF);

        while(!(USART1->SR & USART_FLAG_TXE))
        {
                ++t0; if(t0 > 8000) { return; }
        }
}

/**
        * @brief   USART1调试串口发送字符串
        * @param   str: 以'\\0'结尾的字符串
        * @retval  无
        */
void usart1_SendString(const char *str)
{
        while(*str != '\0')
        {
                usart1_SendByte((uint16_t)*str);
                str++;
        }
}

static void usart_ext_SendByte(USART_TypeDef *uart, uint16_t data)
{
        __IO uint16_t t0 = 0;

        uart->DR = data & (uint16_t)0x01FF;
        while(!(uart->SR & USART_FLAG_TXE))
        {
                ++t0; if(t0 > 8000) { return; }
        }
}

static void usart_ext_SendString(USART_TypeDef *uart, const char *str)
{
        while(*str != '\0')
        {
                usart_ext_SendByte(uart, (uint16_t)*str++);
        }
}

void usart2_SendByte(uint16_t data) { usart_ext_SendByte(USART2, data); }
void uart4_SendByte(uint16_t data) { usart_ext_SendByte(UART4, data); }
void uart5_SendByte(uint16_t data) { usart_ext_SendByte(UART5, data); }
void usart6_SendByte(uint16_t data) { usart_ext_SendByte(USART6, data); }
void usart2_SendString(const char *str) { usart_ext_SendString(USART2, str); }
void uart4_SendString(const char *str) { usart_ext_SendString(UART4, str); }
void uart5_SendString(const char *str) { usart_ext_SendString(UART5, str); }
void usart6_SendString(const char *str) { usart_ext_SendString(USART6, str); }

void usart2_SendData(const uint8_t *data, uint16_t len)
{
        while(len-- > 0) { usart_ext_SendByte(USART2, *data++); }
}

void uart5_SendData(const uint8_t *data, uint16_t len)
{
        while(len-- > 0) { usart_ext_SendByte(UART5, *data++); }
}

void usart6_SendData(const uint8_t *data, uint16_t len)
{
        while(len-- > 0) { usart_ext_SendByte(USART6, *data++); }
}

void uart4_SendData(const uint8_t *data, uint16_t len)
{
        while(len-- > 0) { usart_ext_SendByte(UART4, *data++); }
}

uint8_t usart2_ReadByte(uint8_t *data)
{
        if(usart2_rx_head == usart2_rx_tail) { return 0; }
        *data = usart2_rx_buf[usart2_rx_tail];
        usart2_rx_tail = (uint16_t)((usart2_rx_tail + 1) % USART_EXT_RX_BUF_SIZE);
        return 1;
}

uint8_t uart5_ReadByte(uint8_t *data)
{
        if(uart5_rx_head == uart5_rx_tail) { return 0; }
        *data = uart5_rx_buf[uart5_rx_tail];
        uart5_rx_tail = (uint16_t)((uart5_rx_tail + 1) % USART_EXT_RX_BUF_SIZE);
        return 1;
}

uint8_t usart6_ReadByte(uint8_t *data)
{
        if(usart6_rx_head == usart6_rx_tail) { return 0; }
        *data = usart6_rx_buf[usart6_rx_tail];
        usart6_rx_tail = (uint16_t)((usart6_rx_tail + 1) % USART_EXT_RX_BUF_SIZE);
        return 1;
}

/**
        * @brief   USART1读取一个接收字节
        * @param   data: 接收数据存放地址
        * @retval  1:读取成功，0:暂无数据
        */
uint8_t usart1_ReadByte(uint8_t *data)
{
        if(usart1_rx_head == usart1_rx_tail)
        {
                return 0;
        }

        *data = usart1_rx_buf[usart1_rx_tail];
        usart1_rx_tail = (uint8_t)((usart1_rx_tail + 1) % USART1_RX_BUF_SIZE);
        return 1;
}
