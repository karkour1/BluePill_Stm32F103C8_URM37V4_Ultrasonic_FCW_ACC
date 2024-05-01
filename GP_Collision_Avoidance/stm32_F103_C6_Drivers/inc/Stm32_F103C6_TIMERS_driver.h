/*
 * Stm32_F103C6_TIMERS_driver.h
 *
 *  Created on: Feb 22, 2024
 *  Author: Abdullah Karkour
 */

#ifndef INC_STM32_F103C6_TIMERS_DRIVER_H_
#define INC_STM32_F103C6_TIMERS_DRIVER_H_

//-----------------------------
//	Includes
//-----------------------------
#include <stdint.h>
#include "Stm32_F103X6.h"
#include "Stm32_F103C6_RCC_driver.h"


//-----------------------------
//	Macros define
//-----------------------------

// Timer Clock
#define TIMER_CLK				8  // The default clock to Timer peripherals(MHZ)

// TIMER UNITS
#define TIMER_MICRO_SEC			1
#define TIMER_MILL_SEC			1000
#define TIMER_SEC				1000000

//-----------------------------
//	APIS
//-----------------------------
void MCAL_TIMER_Delay(TIMER_TypeDef* TIMERx ,uint32_t time ,uint32_t unit);
void MCAL_TIMER_Start_Calculate_Time(TIMER_TypeDef* TIMERx );
uint32_t MCAL_TIMER_Get_Time(TIMER_TypeDef* TIMERx );

#endif /* INC_STM32_F103C6_TIMERS_DRIVER_H_ */
