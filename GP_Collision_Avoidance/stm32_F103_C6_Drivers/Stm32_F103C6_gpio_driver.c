/*
 * Stm32_F103C6_gpio_driver.c
 * Created on: Jan 5, 2024
 * Author: Abdullah karkour
 */


//-----------------------------
//Includes
//-----------------------------
#include "Stm32_F103C6_gpio_driver.h"

/**================================================================
 * @Fn			-MCAL_GPIO_Init
 * @brief 		-Initializes the GPIOX PINY according to the specified parameters in the PinConfig
 * @param [in] 	-GPIOx: where x can be (A..E depending on device used) to select the GPIO peripheral
 * @param [in] 	-PinConfig pointer to a GPIO_PinConfig_t structure that contains
 *         		 the configuration information for the specified GPIO PIN.
 * @retval 		-none
 * Note			-Stm32F103C6 MCU has GPIO A,B,C,D,E Modules
 * 				 But LQFP48 Package has only GPIO A,B,PART of C/D exported as external PINS from the MCU
 */
void MCAL_GPIO_Init(GPIO_TypeDef *GPIOX ,  GPIO_PinConfig_t *PinConfig)
{
	//Port configuration register low (GPIOx_CRL) Configure PINS from 0 >>> 7
	//Port configuration register High (GPIOx_CRH) Configure PINS from 8 >>> 15
	volatile uint32_t* configregister ;
	uint8_t PIN_Config = 0 ;

	configregister = (PinConfig->GPIO_PinNumber < GPIO_PIN_8)? &GPIOX->CRL : &GPIOX->CRH ;

	//Get the position of configuration pins in CRL or CRH
	// CRL (position = pin number * 4)
	// CRH (position = (pinNumber-8)*4)
	uint8_t CRLH_Position;
	CRLH_Position = (PinConfig->GPIO_PinNumber < GPIO_PIN_8)? (PinConfig->GPIO_PinNumber * 4): ((PinConfig->GPIO_PinNumber -8) * 4) ;

	//clear CNF8[1:0] MODE8[1:0]
	(*configregister) &= ~(0xf << CRLH_Position);

	//if the pin is output
	if((PinConfig->GPIO_Mode > GPIO_MODE_INPUT_PD) && (PinConfig->GPIO_Mode != GPIO_MODE_AF_INPUT))
	{
		//Set  CNF8[1:0] MODE8[1:0]
		PIN_Config = ((((PinConfig->GPIO_Mode - 4)<<2)|(PinConfig->GPIO_Output_Speed)) & 0x0f);
	}
	//else the pin is input
	else
	{
		if((PinConfig->GPIO_Mode == GPIO_MODE_Analog)||(PinConfig->GPIO_Mode == GPIO_MODE_INPUT_FLO))
		{
			//Set  CNF8[1:0] MODE8[1:0]00
			PIN_Config = ((((PinConfig->GPIO_Mode)<<2)|0x0) & 0x0f);
		}
		else if ((PinConfig->GPIO_Mode == GPIO_MODE_AF_INPUT))
		{
			//Set  CNF8[1:0] MODE8[1:0]00
			PIN_Config = ((((GPIO_MODE_INPUT_FLO)<<2)|0x0) & 0x0f);
		}
		else //PU PD Input
		{
			//Set  CNF8[1:0] MODE8[1:0]00
			PIN_Config = ((((GPIO_MODE_INPUT_PU)<<2)|0x0) & 0x0f);

			if (PinConfig->GPIO_Mode == GPIO_MODE_INPUT_PU)
			{
				//PxODR = 1 Input pull-up :Table 20. Port bit configuration table
				GPIOX->ODR |= (1<<PinConfig->GPIO_PinNumber);
			}
			else
			{
				//PxODR = 0 Input pull-down :Table 20. Port bit configuration table
				GPIOX->ODR &= ~(1<<PinConfig->GPIO_PinNumber);
			}
		}
	}
	// write on the CRL or CRH
	(*configregister) |=  ( (PIN_Config) << CRLH_Position);

}

/**================================================================
 * @Fn			-MCAL_GPIO_DeInit
 * @brief 		-reset all the GPIO Registers
 * @param [in] 	-GPIOx: where x can be (A..E depending on device used) to select the GPIO peripheral
 * @retval 		-none
 * Note			-none
 */
void MCAL_GPIO_DeInit(GPIO_TypeDef *GPIOX )
{
	//or you can use the reset Controller
	//APB2 peripheral reset register (RCC_APB2RSTR)
	//Set and cleared by software.

	if(GPIOX == GPIOA)
	{
		RCC->APB2RSTR |=(1<<2);
		RCC->APB2RSTR &= ~(1<<2);
	}
	else if(GPIOX == GPIOB)
	{
		RCC->APB2RSTR |=(1<<3);
		RCC->APB2RSTR &= ~(1<<3);
	}
	else if(GPIOX == GPIOC)
	{
		RCC->APB2RSTR |=(1<<4);
		RCC->APB2RSTR &= ~(1<<4);
	}
	else if(GPIOX == GPIOD)
	{
		RCC->APB2RSTR |=(1<<5);
		RCC->APB2RSTR &= ~(1<<5);
	}
	else if(GPIOX == GPIOE)
	{
		RCC->APB2RSTR |=(1<<6);
		RCC->APB2RSTR &= ~(1<<6);
	}
}

/**================================================================
 * @Fn			-MCAL_GPIO_ReadPin
 * @brief 		-Read Specific PIN
 * @param [in] 	-GPIOx: where x can be (A..E depending on device used) to select the GPIO peripheral
 * @param [in] 	-PinNumber: Set Pin Number according @ref GPIO_PINS_define
 * @retval 		-the input pin value (two values based on @ref GPIO_PIN_state )
 * Note			-none
 */
uint8_t MCAL_GPIO_ReadPin (GPIO_TypeDef *GPIOX,uint8_t PinNumber )
{
	uint8_t bitstatus ;
	if(((GPIOX->IDR)&(1<<PinNumber))!= (uint32_t)GPIO_PIN_RESET)
	{
		bitstatus = GPIO_PIN_SET ;
	}
	else
	{
		bitstatus = GPIO_PIN_RESET ;
	}
	return bitstatus ;
}

/**================================================================
 * @Fn					-MCAL_GPIO_ReadPort
 * @brief 				-read the input port VALUE
 * @param [in] 			-GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
 * @retval 				-the input port VALUE
 * Note					-none
 */
uint16_t MCAL_GPIO_ReadPort (GPIO_TypeDef *GPIOX)
{
	uint16_t port_value ;
	port_value = (uint16_t)GPIOX->IDR ;
	return port_value ;

}

/**================================================================
 * @Fn					-MCAL_GPIO_WritePin
 * @brief 				-write on specific input pin
 * @param [in] 			-GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
 *@param [in] 			-PinNumber:  specifies the port bit to read. Set by @ref GPIO_PINS_define
 *@param [in] 			-Value: Pin Value
 * @retval 				-none
 * Note					-none
 */
void MCAL_GPIO_WritePin (GPIO_TypeDef *GPIOX,uint8_t PinNumber , uint8_t Value)
{
	if(Value != GPIO_PIN_RESET)
	{
		GPIOX->ODR |= (1<<PinNumber);
	}
	else
	{
		GPIOX->ODR &= ~(1<<PinNumber);
	}

}

/**================================================================
 * @Fn					-MCAL_GPIO_WritePort
 * @brief 				-write on output port
 * @param [in] 			-GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
 * @retval 				-none
 * Note					-none
 */
void MCAL_GPIO_WritePort (GPIO_TypeDef *GPIOX, uint16_t Value)
{
	GPIOX->ODR = (uint32_t)Value ;

}

/**================================================================
 * @Fn					-MCAL_GPIO_TogglePin
 * @brief 				-Toggles the specified GPIO pin
 * @param [in] 			-GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
 * @param [in] 			-PinNumber: specifies the port bit to read. Set by @ref GPIO_PINS_define
 * @retval 				-none
 * Note					-none
 */
void MCAL_GPIO_TogglePin (GPIO_TypeDef *GPIOX,uint8_t PinNumber )
{
	GPIOX->ODR ^= (1<<PinNumber);
}

/**================================================================
 * @Fn					-MCAL_GPIO_LockPin
 * @brief 				-The locking mechanism allows the IO configuration to be frozen
 * @param [in] 			-GPIOx: where x can be (A..G depending on device used) to select the GPIO peripheral
 * @param [in] 			-PinNumber: specifies the port bit to read. Set by @ref GPIO_PINS_define
 * @retval 				-Ok if pin Config is locked Or ERROR if pin  not locked  (OK & ERROR are defined @ref GPIO_RETURN_LOCK
 * Note					-none
 */

uint8_t MCAL_GPIO_LockPin (GPIO_TypeDef *GPIOX,uint8_t PinNumber )
{
	//	Bit 16 LCKK[16]: Lock key
	//	This bit can be read anytime. It can only be modified using the Lock Key Writing Sequence.
	//	0: Port configuration lock key not active
	//	1: Port configuration lock key active. GPIOx_LCKR register is locked until the next reset.
	//	LOCK key writing sequence:
	//	Write 1
	//	Write 0
	//	Write 1
	//	Read 0
	//	Read 1 (this read is optional but confirms that the lock is active)
	//	Note: During the LOCK Key Writing sequence, the value of LCK[15:0] must not change.
	//	Any error in the lock sequence will abort the lock.
	//	Bits 15:0 LCKy: Port x Lock bit y (y= 0 .. 15)
	//	These bits are read write but can only be written when the LCKK bit is 0.
	//	0: Port configuration not locked
	//	1: Port configuration locked

	//Set LCKK[16]
	volatile uint32_t tmp = (1<<16) ;
	//Set the LCKy
	tmp |= (1<<PinNumber) ;

	//	Write 1
	GPIOX->LCKR = tmp ;
	//	Write 0
	GPIOX->LCKR = (1<<PinNumber) ;
	//	Write 1
	GPIOX->LCKR = tmp ;

	//	Read 0
	tmp = GPIOX->LCKR  ;
	//	Read 1 (this read is optional but confirms that the lock is active)
	if ( (uint32_t) (GPIOX->LCKR  & 1<<16 ))
	{
		return GPIO_RETURN_LOCK_Enabled ;
	}else
	{
		return GPIO_RETURN_LOCK_ERROR ;

	}

}
