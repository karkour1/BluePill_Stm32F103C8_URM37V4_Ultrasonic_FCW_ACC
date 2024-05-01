/*
 * Stm32_F103C6_TIMERS_driver.c
 *
 *  Created on: Feb 22, 2024
 *  Author: Abdullah Karkour
 */

//-----------------------------
//Includes
//-----------------------------
#include "Stm32_F103C6_TIMERS_driver.h"

/*
 * =======================================================================================
 * 							Generic Variables
 * =======================================================================================
 */
// To Keep the OverFlow Numbers Of Timers in Calculate time Period
uint32_t OverFlow_Number[3]  ;

// CallBack Function
void (*P_IRQ_CallBack_Fun)(void);

// Global Pointer To the TIMERx
TIMER_TypeDef* G_TIMERx ;

// Delay Flag
uint8_t Delay_Flag = 0 ;
/*
 * =======================================================================================
 * 							Generic Function
 * =======================================================================================
 */
void TIMER_Enable(TIMER_TypeDef* TIMERx)
{
	//Bit 4 DIR: Direction , 0: Counter used as up counter
	TIMERx->CR1 &= ~(1<<4);

	/*Bit 2 URS: Update request source
		1: Only counter overflow/underflow generates an update interrupt or DMA request if
		   enabled.
	*/
	TIMERx->CR1 |=(1<<2);

	/*Bit 0 UG: Update generation
		This bit can be set by software, it is automatically cleared by hardware.
		0: No action
		1: Re-initialize the counter and generates an update of the registers
	*/
	TIMERx->EGR |=(1<<0);

	/*Bit 0 UIE: Update interrupt enable
		0: Update interrupt disabled.
		1: Update interrupt enabled.
	 */
	TIMERx->DIER |=(1<<0);

	// Enable NVIC
	if(TIMERx == TIMER2)
	{
		NVIC_IRQ28_TIM2_ENABLE;

	}else if(TIMERx == TIMER3)
	{
		NVIC_IRQ29_TIM3_ENABLE;

	}else if(TIMERx == TIMER4)
	{
		NVIC_IRQ30_TIM4_ENABLE;
	}

	/*Bit 0 CEN: Counter enable
		0: Counter disabled
		1: Counter enabled
	*/
	TIMERx->CR1 |=(1<<0);
}

void IRQ_Delay(void)
{
	// Clear Bit 0 UIF: Update interrupt flag
	G_TIMERx->SR &= ~(1<<0);

	// Set Delay Flag
	Delay_Flag = 1 ;

}

void IRQ_Count_Time(void)
{
	// Clear Bit 0 UIF: Update interrupt flag
	G_TIMERx->SR &= ~(1<<0);

	// Increment the overflow number
	if(G_TIMERx == TIMER2)
	{
		OverFlow_Number[0]++;

	}else if(G_TIMERx == TIMER3)
	{
		OverFlow_Number[1]++;

	}else if(G_TIMERx == TIMER4)
	{
		OverFlow_Number[2]++;
	}

}

/*
 * =======================================================================================
 * 									APIS
 * =======================================================================================
 */
void MCAL_TIMER_Delay(TIMER_TypeDef* TIMERx ,uint32_t time ,uint32_t unit)
{
	G_TIMERx = TIMERx;
	// Timer off
	TIMERx->CR1 &= ~(1<<0);

	//Calculate AAR_REG value  , PSC_REG value
	if((TIMER_CLK * time * unit)<= 65500 )
	{
		// Set the pre_load Value
		TIMERx->ARR = (TIMER_CLK * time * unit);

		// Set Prescaler =1
		TIMERx->PSC = 1 ;

	}
	else
	{
		if(((TIMER_CLK * time * unit)%65500)==0)
		{
			// Set the pre_load Value
			TIMERx->ARR = 65500;

			// Set Prescaler =1
			TIMERx->PSC = ((TIMER_CLK * time * unit)/65500) ;
		}
		else
		{
			// Set Prescaler =1
			TIMERx->PSC = ((TIMER_CLK * time * unit)/65500)+1 ;

			// Set the pre_load Value
			TIMERx->ARR = ((TIMER_CLK * time * unit)/TIMERx->PSC);

		}

	}


	// Set IRQ Callback
	P_IRQ_CallBack_Fun = IRQ_Delay ;

	// Enable Timer
	TIMER_Enable(TIMERx);

	Delay_Flag = 0;
	// wait the delay period
	while (Delay_Flag == 0);


	// Disable Timer
	/*Bit 0 CEN: Counter enable
		0: Counter disabled
		1: Counter enabled
	*/
	TIMERx->CR1 &= ~(1<<0);
}
void MCAL_TIMER_Start_Calculate_Time(TIMER_TypeDef* TIMERx )
{
	// Timer off
	TIMERx->CR1 &= ~(1<<0);


	//Set AAR_REG value  , PSC_REG value
	TIMERx->ARR = 65500;
	TIMERx->PSC = 1 ;

	// Set IRQ Callback
	P_IRQ_CallBack_Fun = IRQ_Count_Time ;

	// Enable Timer
	TIMER_Enable(TIMERx);
}
uint32_t MCAL_TIMER_Get_Time(TIMER_TypeDef* TIMERx )
{
	// Disable Timer
	/*Bit 0 CEN: Counter enable
		0: Counter disabled
		1: Counter enabled
	*/
	TIMERx->CR1 &= ~(1<<0);

	// Calculate Time
	uint32_t time ;
	if(G_TIMERx == TIMER2)
	{
		time = (((OverFlow_Number[0]*65500)+ TIMERx->CNT)/8);

	}else if(G_TIMERx == TIMER3)
	{
		time = (((OverFlow_Number[1]*65500)+ TIMERx->CNT)/8);

	}else if(G_TIMERx == TIMER4)
	{
		time = (((OverFlow_Number[2]*65500)+ TIMERx->CNT)/8);
	}

	return time ;

}





/*
 * =======================================================================================
 * 									ISR
 * =======================================================================================
 */
void TIM2_IRQHandler(void)
{
	P_IRQ_CallBack_Fun();
}
void TIM3_IRQHandler(void)
{
	P_IRQ_CallBack_Fun();
}
void TIM4_IRQHandler(void)
{
	P_IRQ_CallBack_Fun();
}
