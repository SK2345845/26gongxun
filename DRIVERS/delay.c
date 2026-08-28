#include "delay.h"

/**
	* @brief   Î¢ÃëÑÓÊ±£¨´¿¼Ä´æÆ÷ÂÖÑ¯¼ÆÊý£¬²»¿ªÆô SysTick ÖÐ¶Ï£¬·ÀÖ¹Óë FreeRTOS ³åÍ»µ¼ÖÂ HardFault£©
	* @param   us: ÑÓÊ±Î¢ÃëÊý
	* @retval  ÎÞ
	*/
void delay_us(__IO uint16_t us)
{
	uint16_t i;

	SysTick->LOAD = (SystemCoreClock / 1000000) - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

	for(i = 0; i < us; i++)
	{
		while(!((SysTick->CTRL) & SysTick_CTRL_COUNTFLAG_Msk));
	}	 
	 
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

/**
	* @brief   ºÁÃëÑÓÊ±£¨´¿¼Ä´æÆ÷ÂÖÑ¯¼ÆÊý£¬²»¿ªÆô SysTick ÖÐ¶Ï£¬·ÀÖ¹Óë FreeRTOS ³åÍ»µ¼ÖÂ HardFault£©
	* @param   ms: ÑÓÊ±ºÁÃëÊý
	* @retval  ÎÞ
	*/
void delay_ms(__IO uint16_t ms)
{
	uint16_t i;

	SysTick->LOAD = (SystemCoreClock / 1000) - 1;
	SysTick->VAL = 0;
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

	for(i = 0; i < ms; i++)
	{
		while(!((SysTick->CTRL) & SysTick_CTRL_COUNTFLAG_Msk));
	}	 
	 
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}
