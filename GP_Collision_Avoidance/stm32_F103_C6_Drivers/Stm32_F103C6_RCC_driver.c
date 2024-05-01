/*
 * Stm32_F103C6_RCC_driver.c
 *
 *  Created on: Feb 19, 2024
 *  Author: Abdullah Karkour
 */

//-----------------------------
//Includes
//-----------------------------
#include "Stm32_F103C6_RCC_driver.h"

/*
 * =======================================================================================
 * 							Generic Variables
 * =======================================================================================
 */
//Bits 7:4 HPRE[3:0]: AHB prescaler
//Set and cleared by software to control AHB clock division factor.
//0xxx: SYSCLK not divided
//1000: SYSCLK divided by 2
//1001: SYSCLK divided by 4
//1010: SYSCLK divided by 8
//1011: SYSCLK divided by 16
//1100: SYSCLK divided by 64
//1101: SYSCLK divided by 128
//1110: SYSCLK divided by 256
//1111: SYSCLK divided by 512
const uint8_t AHBPrescTable[16U] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

//PPRE1[2:0]: APB Low-speed prescaler (APB1)
//0xx: HCLK not divided
//100: HCLK divided by 2
//101: HCLK divided by 4
//110: HCLK divided by 8
//111: HCLK divided by 16
const uint8_t APBPrescTable[8U] =  {0, 0, 0, 0, 1, 2, 3, 4}; //Shift 1 right == multiply by 2


/*
 * =======================================================================================
 * 									APIS
 * =======================================================================================
 */
uint32_t MCAL_RCC_GET_SYSCLK_Freq()
{
/*  Bits 3:2 SWS: System clock switch status
	Set and cleared by hardware to indicate which clock source is used as system clock.
	00: HSI oscillator used as system clock
	01: HSE oscillator used as system clock
	10: PLL used as system clock
	11: not applicable
*/
	switch(((RCC->CFGR)>>2)& 0b11)
	{
	case 0:

		return HSI_RC_Clk ;
		break ;

	case 1:

		//to do need to calculate  it //HSE User Should Specify it
		return HSE_Clock ;
		break ;

	case 2:

		//to do need to calculate  it PLLCLK and PLLMUL & PLL Source MUX
		return 16000000 ;
		break ;

	}
}
uint32_t MCAL_RCC_GET_HCLK_Freq()
{
	return (MCAL_RCC_GET_SYSCLK_Freq()  >> AHBPrescTable[(((RCC->CFGR)>>4)& 0xF)]);
}
uint32_t MCAL_RCC_GET_PCLK1_Freq()
{
	return (MCAL_RCC_GET_SYSCLK_Freq()  >> APBPrescTable[(((RCC->CFGR)>>8)& 0b111)]);
}
uint32_t MCAL_RCC_GET_PCLK2_Freq()
{
	return (MCAL_RCC_GET_SYSCLK_Freq()  >> APBPrescTable[(((RCC->CFGR)>>11)& 0b111)]);
}
