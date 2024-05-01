/*
 * Stm32_F103C6_RCC_driver.h
 *
 *  Created on: Feb 19, 2024
 *  Author: Abdullah Karkour
 */

#ifndef INC_STM32_F103C6_RCC_DRIVER_H_
#define INC_STM32_F103C6_RCC_DRIVER_H_

//-----------------------------
//	Includes
//-----------------------------
#include <stdint.h>
#include "Stm32_F103X6.h"




//-----------------------------
//	Macros define
//-----------------------------
#define  HSE_Clock			(uint32_t)16000000
#define  HSI_RC_Clk			(uint32_t)8000000

//-----------------------------
//	APIS
//-----------------------------
uint32_t MCAL_RCC_GET_SYSCLK_Freq();
uint32_t MCAL_RCC_GET_HCLK_Freq();
uint32_t MCAL_RCC_GET_PCLK1_Freq();
uint32_t MCAL_RCC_GET_PCLK2_Freq();

#endif /* INC_STM32_F103C6_RCC_DRIVER_H_ */
