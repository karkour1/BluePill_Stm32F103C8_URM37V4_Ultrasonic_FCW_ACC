/*
 *  Stm32_F103C6_UART_driver.c
 *  Created on: Feb 19, 2024
 *  Author: Abdullah Karkour
 */

/*
 * =======================================================================================
 * 							includes
 * =======================================================================================
 */
#include "Stm32_F103C6_UART_driver.h"

/*
 * =======================================================================================
 * 							Generic Variables
 * =======================================================================================
 */
UART_PinConfig_t G_UART_Config [3];


/*
 * =======================================================================================
 * 							APIs Supported by "MCAL UART DRIVER"
 * =======================================================================================
 */

/**================================================================
 * @Fn				-MCAL_UART_Init
 * @brief 			- Initializes UART (Supported feature ASYNCH. Only)
 * @param [in] 		- USARTx: where x can be (1..3 depending on device used)
 * @param [in] 		- UART_Config: All UART Configuration EXTI_PinConfig_t
 * @retval 			-none
 * Note				-Support for Now Asynch mode & Clock 8 MHZ S
 */
void MCAL_UART_Init (USART_TypeDef *USARTx, UART_PinConfig_t* UART_Config)
{
	uint32_t pclk ;

	//	enable the Clock for given USART peripheral
	if(USARTx == USART1)
	{
		G_UART_Config[0].Payload_Length = UART_Config->Payload_Length ;
		G_UART_Config[0].HwFlowCtl = UART_Config->HwFlowCtl ;
		G_UART_Config[0].Parity = UART_Config->Parity ;
		G_UART_Config[0].P_IRQ_CallBack = UART_Config->P_IRQ_CallBack ;
		RCC_USART1_CLK_EN();

	}else if(USARTx == USART2)
	{
		G_UART_Config[1].Payload_Length = UART_Config->Payload_Length ;
		G_UART_Config[1].HwFlowCtl = UART_Config->HwFlowCtl ;
		G_UART_Config[1].Parity = UART_Config->Parity ;
		G_UART_Config[1].P_IRQ_CallBack = UART_Config->P_IRQ_CallBack ;
		RCC_USART2_CLK_EN();

	}else if(USARTx == USART3)
	{
		G_UART_Config[2].Payload_Length = UART_Config->Payload_Length ;
		G_UART_Config[2].HwFlowCtl = UART_Config->HwFlowCtl ;
		G_UART_Config[2].Parity = UART_Config->Parity ;
		G_UART_Config[2].P_IRQ_CallBack = UART_Config->P_IRQ_CallBack ;
		RCC_USART3_CLK_EN();
	}

	//  Enable USART Module
	//	USART_CR1  Bit 13 UE: USART enable
	USARTx->CR1 |= (1<<13);

	//	Enable USART Tx and Rx engines according to the USART_Mode configuration item
	//	USART_CR1 Bit 3 TE: Transmitter enable & Bit 2 RE: Receiver enable
	USARTx->CR1 |= UART_Config->Mode ;

	// PAYLOAD Width
	// USARTx->CR1  Bit 12 M: Word length
	USARTx->CR1 |= UART_Config->Payload_Length ;

	//Configuration of parity control bit fields
	// USARTx->CR1 	Bit 10 PCE: Parity control enable     Bit 9 PS: Parity selection
	USARTx->CR1 |= UART_Config->Parity ;

	//configure the number of stop bits
	//USART_CR2  Bits 13:12 STOP: STOP bits
	USARTx->CR2 |= UART_Config->StopBits ;

	//USART hardware flow control
	//USART_CR3  Bit 9 CTSE: CTS enable   Bit 8 RTSE: RTS enable
	USARTx->CR3 |= UART_Config->HwFlowCtl ;

	//Configuration of BRR(Baudrate register)
	//PCLK1 for USART2, 3
	//PCLK2 for USART1
	if ( USARTx == USART1 )
	{
		pclk = MCAL_RCC_GET_PCLK1_Freq();
	}
	else
	{
		pclk = MCAL_RCC_GET_PCLK2_Freq();
	}

	USARTx->BRR = UART_BRR_Register(pclk, UART_Config->BaudRate) ;

	//ENABLE / DISABLE Interrupt
	//USART_CR1
	if (UART_Config->IRQ_Enable  != UART_IRQ_Enable_NONE)
	{
		USARTx->CR1 |= (UART_Config->IRQ_Enable) ;
		//		Enable NVIC For USARTx IRQ
		if ( USARTx == USART1 )
			NVIC_IRQ37_USART1_ENABLE ;

		else if ( USARTx == USART2 )
			NVIC_IRQ38_USART2_ENABLE ;

		else if ( USARTx == USART3 )
			NVIC_IRQ39_USART3_ENABLE ;
	}
}

/**================================================================
 * @Fn				-MCAL_UART_DEInit
 * @brief 			- DEInitializes UART (Supported feature ASYNCH. Only)
 * @param [in] 		- USARTx: where x can be (1..3 depending on device used)
 * @retval 			-none
 * Note				-Reset the Model By RCC
 */
void MCAL_UART_DeInit (USART_TypeDef *USARTx)
{
	// Disable NVIC and Reset the USART Registers
	if ( USARTx == USART1 )
	{
		RCC_USART1_RESET();
		NVIC_IRQ37_USART1_DISABLE ;

	}

	else if ( USARTx == USART2 )
	{
		RCC_USART2_RESET();
		NVIC_IRQ38_USART2_DISABLE ;


	}

	else if ( USARTx == USART3 )
	{
		RCC_USART3_RESET();
		NVIC_IRQ39_USART3_DISABLE ;

	}
}

/**================================================================
 * @Fn				-MCAL_UART_GPIO_Set_Pins
 * @brief 			- intialize GPIO Pins
 * @param [in] 		- USARTx: where x can be (1..3 depending on device used)
 * @retval 			-none
 * Note				-Should enable the corresponding ALT  & GPIO  in RCC clock Also called after MCAL_UART_Init()
 */
void MCAL_UART_GPIO_Set_Pins (USART_TypeDef *USARTx)
{
	GPIO_PinConfig_t PinCfg ;

	if ( USARTx == USART1 )
	{
		//PA9 TX
		//PA10 RX
		//PA11 CTS
		//PA12 RTS

		//PA9 TX
		PinCfg.GPIO_PinNumber = GPIO_PIN_9;
		PinCfg.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PP;
		PinCfg.GPIO_Output_Speed = GPIO_SPEED_10M;
		MCAL_GPIO_Init(GPIOA, &PinCfg);

		//PA10 RX
		PinCfg.GPIO_PinNumber = GPIO_PIN_10;
		PinCfg.GPIO_Mode = GPIO_MODE_AF_INPUT;
		MCAL_GPIO_Init(GPIOA, &PinCfg);

		if (G_UART_Config[0].HwFlowCtl  ==UART_HwFlowCtl_CTS  || G_UART_Config[0].HwFlowCtl  ==UART_HwFlowCtl_RTS_CTS  ){
			//PA11 CTS
			PinCfg.GPIO_PinNumber = GPIO_PIN_11;
			PinCfg.GPIO_Mode = GPIO_MODE_INPUT_FLO;
			MCAL_GPIO_Init(GPIOA, &PinCfg);
		}


		if (G_UART_Config[0].HwFlowCtl  ==UART_HwFlowCtl_RTS || G_UART_Config[0].HwFlowCtl  ==UART_HwFlowCtl_RTS_CTS  )
		{
			//PA12 RTS
			PinCfg.GPIO_PinNumber = GPIO_PIN_12;
			PinCfg.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PP;
			PinCfg.GPIO_Output_Speed = GPIO_SPEED_10M;
			MCAL_GPIO_Init(GPIOA, &PinCfg);

		}
	}
	else if ( USARTx == USART2 )
	{
		//PA2 TX
		//PA3 RX
		//PA0 CTS
		//PA1 RTS


		//PA2 TX
		PinCfg.GPIO_PinNumber = GPIO_PIN_2;
		PinCfg.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PP;
		PinCfg.GPIO_Output_Speed = GPIO_SPEED_10M;
		MCAL_GPIO_Init(GPIOA, &PinCfg);

		//PA3 RX
		PinCfg.GPIO_PinNumber = GPIO_PIN_3;
		PinCfg.GPIO_Mode = GPIO_MODE_AF_INPUT;
		MCAL_GPIO_Init(GPIOA, &PinCfg);

		if (G_UART_Config[1].HwFlowCtl  ==UART_HwFlowCtl_CTS  || G_UART_Config[1].HwFlowCtl  ==UART_HwFlowCtl_RTS_CTS  )
		{
			//PA0 CTS
			PinCfg.GPIO_PinNumber = GPIO_PIN_0;
			PinCfg.GPIO_Mode = GPIO_MODE_INPUT_FLO;
			MCAL_GPIO_Init(GPIOA, &PinCfg);
		}

		if (G_UART_Config[1].HwFlowCtl  ==UART_HwFlowCtl_RTS || G_UART_Config[1].HwFlowCtl  ==UART_HwFlowCtl_RTS_CTS  )
		{
			//PA1 RTS
			PinCfg.GPIO_PinNumber = GPIO_PIN_1;
			PinCfg.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PP;
			PinCfg.GPIO_Output_Speed = GPIO_SPEED_10M;
			MCAL_GPIO_Init(GPIOA, &PinCfg);
		}
	}
	else if ( USARTx == USART3 )
	{
		//PB10 TX
		//PB11 RX
		//PB13 CTS
		//PA14 RTS



		//PB10 TX
		PinCfg.GPIO_PinNumber = GPIO_PIN_10;
		PinCfg.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PP;
		PinCfg.GPIO_Output_Speed = GPIO_SPEED_10M;
		MCAL_GPIO_Init(GPIOB, &PinCfg);

		//PB11 RX
		PinCfg.GPIO_PinNumber = GPIO_PIN_11;
		PinCfg.GPIO_Mode = GPIO_MODE_AF_INPUT;
		MCAL_GPIO_Init(GPIOB, &PinCfg);

		if (G_UART_Config[2].HwFlowCtl  ==UART_HwFlowCtl_CTS  || G_UART_Config[2].HwFlowCtl  ==UART_HwFlowCtl_RTS_CTS  )
		{
			//PB13 CTS
			PinCfg.GPIO_PinNumber = GPIO_PIN_13;
			PinCfg.GPIO_Mode = GPIO_MODE_INPUT_FLO;
			MCAL_GPIO_Init(GPIOB, &PinCfg);
		}
		if (G_UART_Config[2].HwFlowCtl  ==UART_HwFlowCtl_RTS || G_UART_Config[2].HwFlowCtl  ==UART_HwFlowCtl_RTS_CTS  )
		{
			//PA14 RTS
			PinCfg.GPIO_PinNumber = GPIO_PIN_14;
			PinCfg.GPIO_Mode = GPIO_MODE_OUTPUT_AF_PP;
			PinCfg.GPIO_Output_Speed = GPIO_SPEED_10M;
			MCAL_GPIO_Init(GPIOB, &PinCfg);

		}
	}
}

/*********************************************************************
 * @fn      		  - MCAL_UART_SendData
 *
 * @brief             -Send Buffer on UART
 *
 * @param [in] 		- USARTx: where x can be (1..3 depending on device used)
 * @param[in]         -pTxBuffer Buffer
 * @param[in]         -PollingEn   Enable pooling or disable it
 *
 * @return            -

 * @Note              - Should initialize UART First
 * 		//			When transmitting with the parity enabled (PCE bit set to 1 in the USART_CR1 register),
			//			the value written in the MSB (bit 7 or bit 8 depending on the data length) has no effect
			//			because it is replaced by the parity.
			//			When receiving with the parity enabled, the value read in the MSB bit is the received parity
			//			bit

 */

void MCAL_UART_SendData	(USART_TypeDef *USARTx, uint16_t *pTxBuffer,enum Polling_mechism PollingEn )
{
	// wait until TXE flag is set in the SR
	if (PollingEn == enable)
		while(! (USARTx->SR & 1<<7 ) );

	//Check the USART_WordLength item for 9BIT or 8BIT in a frame
	if ( USARTx == USART1 )
	{
		if(G_UART_Config[0].Payload_Length == UART_Payload_Length_9B)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			USARTx->DR = (*pTxBuffer & (uint16_t)0x01FF);

		}else if (G_UART_Config[0].Payload_Length == UART_Payload_Length_8B)
		{
			//if 8BIT, load the DR with 1bytes
			USARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
		}

	}

	else if ( USARTx == USART2 )
	{
		if(G_UART_Config[1].Payload_Length == UART_Payload_Length_9B)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			USARTx->DR = (*pTxBuffer & (uint16_t)0x01FF);

		}else if (G_UART_Config[1].Payload_Length == UART_Payload_Length_8B)
		{
			//if 8BIT, load the DR with 1bytes
			USARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
		}


	}

	else if ( USARTx == USART3 )
	{
		if(G_UART_Config[2].Payload_Length == UART_Payload_Length_9B)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			USARTx->DR = (*pTxBuffer & (uint16_t)0x01FF);

		}else if (G_UART_Config[2].Payload_Length == UART_Payload_Length_8B)
		{
			//if 8BIT, load the DR with 1bytes
			USARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);
		}

	}

}

/*********************************************************************
 * @fn      		  - MCAL_UART_ReceiveData
 *
 * @brief             -Receive Buffer on UART
 *
 * @param [in] 		- USARTx: where x can be (1..3 depending on device used)
 * @param[in]         -pTxBuffer Buffer
 * @param[in]         -PollingEn   Enable pooling or dsable it
 *
 * @return            -

 * @Note              - Should initialize UART First
 * 		//			When transmitting with the parity enabled (PCE bit set to 1 in the USART_CR1 register),
			//			the value written in the MSB (bit 7 or bit 8 depending on the data length) has no effect
			//			because it is replaced by the parity.
			//			When receiving with the parity enabled, the value read in the MSB bit is the received parity
			//			bit

 */
void MCAL_UART_ReceiveData	(USART_TypeDef *USARTx, uint16_t *pRxBuffer ,enum Polling_mechism PollingEn )
{
	//Loop over until "Len" number of bytes are transferred
	//wait until RXNE(Read data register not empty) flag is set in the SR
	if (PollingEn == enable)
		while( ! (USARTx->SR & 1<<5 ));

	//Check the USART_WordLength item for 9BIT or 8BIT in a frame
	if ( USARTx == USART1 )
	{
		if(G_UART_Config[0].Payload_Length == UART_Payload_Length_9B)
		{
			if(G_UART_Config[0].Parity == UART_Parity__NONE)
			{
				//no parity So all 9bit are considered data
				*pRxBuffer = (uint16_t)(USARTx->DR);
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (uint16_t)( USARTx->DR  & (uint8_t)0xFF );
			}

		}else if (G_UART_Config[0].Payload_Length == UART_Payload_Length_8B)
		{
			if(G_UART_Config[0].Parity == UART_Parity__NONE)
			{
				//no parity So all 8bit are considered data
				*pRxBuffer = (uint16_t)(USARTx->DR & 0xFF);
			}
			else
			{
				//Parity is used, so, 7bits will be of user data and 1 bit is parity
				*pRxBuffer = (uint16_t)( USARTx->DR  & (uint8_t)0x7F );
			}
		}

	}

	else if ( USARTx == USART2 )
	{
		if(G_UART_Config[1].Payload_Length == UART_Payload_Length_9B)
		{
			if(G_UART_Config[1].Parity == UART_Parity__NONE)
			{
				//no parity So all 9bit are considered data
				*pRxBuffer = (uint16_t)(USARTx->DR);
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (uint16_t)( USARTx->DR  & (uint8_t)0xFF );
			}

		}else if (G_UART_Config[1].Payload_Length == UART_Payload_Length_8B)
		{
			if(G_UART_Config[1].Parity == UART_Parity__NONE)
			{
				//no parity So all 8bit are considered data
				*pRxBuffer = (uint16_t)(USARTx->DR & 0xFF);
			}
			else
			{
				//Parity is used, so, 7bits will be of user data and 1 bit is parity
				*pRxBuffer = (uint16_t)( USARTx->DR  & (uint8_t)0x7F );
			}
		}


	}

	else if ( USARTx == USART3 )
	{
		if(G_UART_Config[2].Payload_Length == UART_Payload_Length_9B)
		{
			if(G_UART_Config[2].Parity == UART_Parity__NONE)
			{
				//no parity So all 9bit are considered data
				*pRxBuffer = (uint16_t)(USARTx->DR);
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (uint16_t)( USARTx->DR  & (uint8_t)0xFF );
			}

		}else if (G_UART_Config[2].Payload_Length == UART_Payload_Length_8B)
		{
			if(G_UART_Config[2].Parity == UART_Parity__NONE)
			{
				//no parity So all 8bit are considered data
				*pRxBuffer = (uint16_t)(USARTx->DR & 0xFF);
			}
			else
			{
				//Parity is used, so, 7bits will be of user data and 1 bit is parity
				*pRxBuffer = (uint16_t)( USARTx->DR  & (uint8_t)0x7F );
			}
		}

	}

}

void MCAL_UART_WAIT_TC (USART_TypeDef *USARTx )
{
	// wait till TC flag is set in the SR
	while( ! (USARTx->SR & 1<<6 ));
}


//ISR
void USART1_IRQHandler (void)
{

	G_UART_Config[0].P_IRQ_CallBack () ;

}

void USART2_IRQHandler (void)
{
	G_UART_Config[1].P_IRQ_CallBack () ;

}
void USART3_IRQHandler (void)
{
	G_UART_Config[2].P_IRQ_CallBack () ;


}





