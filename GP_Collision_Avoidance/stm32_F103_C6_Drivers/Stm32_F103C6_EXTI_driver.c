/*
 * Stm32_F103C6_EXTI_driver.c
 * Created on: Feb 12, 2024
 * Author: Abdullah karkour
 */



//-----------------------------
//Includes
//-----------------------------
#include "Stm32_F103C6_EXTI_driver.h"

//-----------------------------
//Generic Variables
//-----------------------------

void(*GP_IRQ_CallBack[15])(void);

//-----------------------------
//Generic Macros
//-----------------------------

#define AFIO_GPIO_EXTI_Mapping(x)		(	(x==GPIOA)?0:\
											(x==GPIOB)?1:\
											(x==GPIOC)?2:\
											(x==GPIOD)?3:0  )
//-----------------------------
//Generic Functions
//-----------------------------
void Enable_NVIC(uint16_t IRQ)
{
	switch(IRQ)
	{
	case 0 :
		NVIC_IRQ6_EXTI0_ENABLE;
		break;
	case 1 :
		NVIC_IRQ7_EXTI1_ENABLE;
		break;
	case 2 :
		NVIC_IRQ8_EXTI2_ENABLE;
		break;
	case 3 :
		NVIC_IRQ8_EXTI2_ENABLE;
		break;
	case 4 :
		NVIC_IRQ9_EXTI3_ENABLE;
		break;
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
		NVIC_IRQ23_EXTI5_9_ENABLE;
		break;
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
		NVIC_IRQ40_EXTI10_15_ENABLE;
		break;
	}
}

void Disable_NVIC(uint16_t IRQ)
{
	switch(IRQ)
	{
	case 0 :
		NVIC_IRQ6_EXTI0_DISABLE;
		break;
	case 1 :
		NVIC_IRQ7_EXTI1_DISABLE;
		break;
	case 2 :
		NVIC_IRQ8_EXTI2_DISABLE;
		break;
	case 3 :
		NVIC_IRQ8_EXTI2_DISABLE;
		break;
	case 4 :
		NVIC_IRQ9_EXTI3_DISABLE;
		break;
	case 5:
	case 6:
	case 7:
	case 8:
	case 9:
		NVIC_IRQ23_EXTI5_9_DISABLE;
		break;
	case 10:
	case 11:
	case 12:
	case 13:
	case 14:
	case 15:
		NVIC_IRQ40_EXTI10_15_DISABLE;
		break;
	}
}

void Update_EXTI(EXTI_PinConfig_t* EXTI_cfg)
{
	// Configure GPIO AF Input --> Floating Input
	GPIO_PinConfig_t GPIO_Cfg ;
	GPIO_Cfg.GPIO_PinNumber = EXTI_cfg->EXTI_Pin.GPIO_Pin;
	GPIO_Cfg.GPIO_Mode = GPIO_MODE_INPUT_FLO;
	MCAL_GPIO_Init(EXTI_cfg->EXTI_Pin.GPIO_Port, &GPIO_Cfg);
	//================================================================

	// Update the AFIO to Route EXTI Line with Port A,B,C,D
	uint8_t AFIO_EXTICR_Index    = (EXTI_cfg->EXTI_Pin.EXTI_InputLineNumber /4);
	uint8_t AFIO_EXTICR_Position = (EXTI_cfg->EXTI_Pin.EXTI_InputLineNumber %4)*4;

	//Clear Four bit
	AFIO->EXTICR[AFIO_EXTICR_Index] &= ~(0xF << AFIO_EXTICR_Position);

	AFIO->EXTICR[AFIO_EXTICR_Index] |= ((AFIO_GPIO_EXTI_Mapping(EXTI_cfg->EXTI_Pin.GPIO_Port) & 0xF) << AFIO_EXTICR_Position);
	//================================================================

	// Update Raising or Falling Trigger
	//DisableRaising or Falling Trigger
	EXTI->RTSR &= ~(1<<EXTI_cfg->EXTI_Pin.EXTI_InputLineNumber);
	EXTI->FTSR &= ~(1<<EXTI_cfg->EXTI_Pin.EXTI_InputLineNumber);

	if(EXTI_cfg->Trigger_case == EXTI_TRIGGER_RAISING)
	{
		EXTI->RTSR |= (1<<EXTI_cfg->EXTI_Pin.EXTI_InputLineNumber);
	}
	else if(EXTI_cfg->Trigger_case == EXTI_TRIGGER_FALLING)
	{
		EXTI->FTSR |= (1<<EXTI_cfg->EXTI_Pin.EXTI_InputLineNumber);
	}
	else if(EXTI_cfg->Trigger_case == EXTI_TRIGGER_RaisingAndFalling)
	{
		EXTI->RTSR |= (1<<EXTI_cfg->EXTI_Pin.EXTI_InputLineNumber);
		EXTI->FTSR |= (1<<EXTI_cfg->EXTI_Pin.EXTI_InputLineNumber);
	}
	//================================================================

	//Update the IRQ CallBack Function
	GP_IRQ_CallBack[EXTI_cfg->EXTI_Pin.EXTI_InputLineNumber] = EXTI_cfg->P_IRQ_CallBack ;
	//================================================================

	// Enable or Disable IRQ EXTI and NVIC
	if(EXTI_cfg->IRQ_EN == EXTI_IRQ_ENABLE)
	{
		EXTI->IMR |= (1<<EXTI_cfg->EXTI_Pin.EXTI_InputLineNumber);
		Enable_NVIC(EXTI_cfg->EXTI_Pin.EXTI_InputLineNumber);
	}
	else
	{
		EXTI->IMR &= ~(1<<EXTI_cfg->EXTI_Pin.EXTI_InputLineNumber);
		Disable_NVIC(EXTI_cfg->EXTI_Pin.EXTI_InputLineNumber);
	}
}
/**================================================================
 * @Fn			-MCAL_EXTI_DeInit
 * @brief 		-Reset the register of EXTI and NVIC Interrupt Controller
 * @retval 		-none
 * Note			-none
 */
void MCAL_EXTI_GPIO_DeInit(void)
{
	//Reset EXTI Register
	EXTI->IMR   = 0x00000000;
	EXTI->EMR   = 0x00000000;
	EXTI->RTSR  = 0x00000000;
	EXTI->FTSR  = 0x00000000;
	EXTI->SWIER = 0x00000000;
	//cleared by writing a ‘1’ into the bit
	EXTI->PR    = 0xFFFFFFFF;

	//Disable EXTI IRQ From NVIC
	NVIC_IRQ6_EXTI0_DISABLE ;
	NVIC_IRQ7_EXTI1_DISABLE;
	NVIC_IRQ8_EXTI2_DISABLE ;
	NVIC_IRQ9_EXTI3_DISABLE;
	NVIC_IRQ10_EXTI4_DISABLE ;
	NVIC_IRQ23_EXTI5_9_DISABLE;
	NVIC_IRQ40_EXTI10_15_DISABLE;
}
/**================================================================
 * @Fn			-MCAL_EXTI_GPIO_Init
 * @brief 		-Initializes the EXTI From Specific GPIO PIN And specify the Mask/Trigger Condition and specify the IRQ CallBack Function
 * @param [in] 	-EXTI_cfg Set by @ref EXTI_define , EXTI_Trigger_define , EXTI_IRQ_define .
 * @retval 		-none
 * Note			-Stm32F103C6 MCU has GPIO A,B,C,D,E Modules
 * 				 But LQFP48 Package has only GPIO A,B,PART of C/D exported as external PINS from the MCU
 * 				-Also Mandatory to Enable RCC Clock for AFIO and GPIO
 */
void MCAL_EXTI_GPIO_Init(EXTI_PinConfig_t* EXTI_cfg)
{
	Update_EXTI(EXTI_cfg);
}

/**================================================================
 * @Fn			-MCAL_EXTI_GPIO_Update
 * @brief 		-Initializes the EXTI From Specific GPIO PIN And specify the Mask/Trigger Condition and specify the IRQ CallBack Function
 * @param [in] 	-EXTI_cfg Set by @ref EXTI_define , EXTI_Trigger_define , EXTI_IRQ_define .
 * @retval 		-none
 * Note			-Stm32F103C6 MCU has GPIO A,B,C,D,E Modules
 * 				 But LQFP48 Package has only GPIO A,B,PART of C/D exported as external PINS from the MCU
 * 				-Also Mandatory to Enable RCC Clock for AFIO and GPIO
 */
void MCAL_EXTI_GPIO_Update(EXTI_PinConfig_t* EXTI_cfg)
{
	Update_EXTI(EXTI_cfg);
}



//-----------------------------
//ISR Functions
//-----------------------------
void EXTI0_IRQHandler(void)
{
	// Clear Pending Pin cleared by writing a ‘1’ into the bit
	EXTI->PR |= (1<<0);

	GP_IRQ_CallBack[0]();
}
void EXTI1_IRQHandler(void)
{
	// Clear Pending Pin cleared by writing a ‘1’ into the bit
	EXTI->PR |= (1<<1);

	GP_IRQ_CallBack[1]();
}
void EXTI2_IRQHandler(void)
{
	// Clear Pending Pin cleared by writing a ‘1’ into the bit
	EXTI->PR |= (1<<2);

	GP_IRQ_CallBack[2]();
}
void EXTI3_IRQHandler(void)
{
	// Clear Pending Pin cleared by writing a ‘1’ into the bit
	EXTI->PR |= (1<<3);

	GP_IRQ_CallBack[3]();
}
void EXTI4_IRQHandler(void)
{
	// Clear Pending Pin cleared by writing a ‘1’ into the bit
	EXTI->PR |= (1<<4);

	GP_IRQ_CallBack[4]();
}
void EXTI9_5_IRQHandler(void)
{
	if(EXTI->PR & 1<<5){ EXTI->PR |= (1<<5); GP_IRQ_CallBack[5]();}
	else if(EXTI->PR & 1<<6){ EXTI->PR |= (1<<6); GP_IRQ_CallBack[6]();}
	else if(EXTI->PR & 1<<7){ EXTI->PR |= (1<<7); GP_IRQ_CallBack[7]();}
	else if(EXTI->PR & 1<<8){ EXTI->PR |= (1<<8); GP_IRQ_CallBack[8]();}
	else if(EXTI->PR & 1<<9){ EXTI->PR |= (1<<9); GP_IRQ_CallBack[9]();}
}
void EXTI15_10_IRQHandler(void)
{
	if(EXTI->PR & 1<<10){ EXTI->PR |= (1<<10); GP_IRQ_CallBack[10]();}
	else if(EXTI->PR & 1<<11){ EXTI->PR |= (1<<11); GP_IRQ_CallBack[11]();}
	else if(EXTI->PR & 1<<12){ EXTI->PR |= (1<<12); GP_IRQ_CallBack[12]();}
	else if(EXTI->PR & 1<<13){ EXTI->PR |= (1<<13); GP_IRQ_CallBack[13]();}
	else if(EXTI->PR & 1<<14){ EXTI->PR |= (1<<14); GP_IRQ_CallBack[14]();}
	else if(EXTI->PR & 1<<15){ EXTI->PR |= (1<<15); GP_IRQ_CallBack[15]();}
}






