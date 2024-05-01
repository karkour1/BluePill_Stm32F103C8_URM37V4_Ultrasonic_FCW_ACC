/*
 *  Stm32_F103C6_UART_driver.H
 *  Created on: Feb 19, 2024
 *  Author: Abdullah Karkour
 */


#ifndef INC_STM32_F103C6_UART_DRIVER_H_
#define INC_STM32_F103C6_UART_DRIVER_H_

/*
 * =======================================================================================
 * 							includes
 * =======================================================================================
 */
#include "Stm32_F103X6.h"
#include "Stm32_F103C6_gpio_driver.h"
#include "Stm32_F103C6_RCC_driver.h"

//-----------------------------
//User type definitions (structures)
//-----------------------------
typedef struct
{
	uint8_t 		 Mode ;     		 // Specifies the UART mode (TX,RX)Enable/Disable.
							     	 	 // This parameter must be set based on @ref UART_Mode_define

	uint32_t 		 BaudRate ;  		 // Specifies the Baud Rate of the communication
								 	 	 // This parameter can be a value of  @ref UART_BaudRate_define

	uint8_t 		 Payload_Length;	 // Specifies the number of data bits transmitted or received in a frame.
									     // This parameter must be set based on @ref UART_Payload_Length_define

	uint8_t 		 Parity ;	 		//Specifies the parity mode.
										//@ref UART_Parity_define


	uint8_t 		StopBits ;			//Specifies the number of stop bits transmitted
										//@ref UART_StopBits_define

	uint8_t 		HwFlowCtl ;			//Specifies whether the hardware flow control mode is enabled or disabled
										//@ref UART_HwFlowCtl_define


	uint8_t			IRQ_Enable;			//enable or disable UART IRQ TX/RX
										//@ref UART_IRQ_Enable_define , you can select two or three parameters EX.UART_IRQ_Enable_TXE |UART_IRQ_Enable_TC


	void(* P_IRQ_CallBack)(void) ;		//Set the C Function() which will be called once the IRQ  Happen

}UART_PinConfig_t;



enum Polling_mechism{
	enable ,
	disable
};

/*
 * =======================================================================================
 * 							Generic Macros
 * =======================================================================================
 */

//BaudRate Calculation
//USARTDIV = fclk /  Baudrate
//DIV_Mantissa = Integer Part (USARTDIV /16)
//DIV_Fraction = Modulus of  (USARTDIV %16)

#define USARTDIV(_PCLK_, _BAUD_)							(uint32_t) (_PCLK_/(_BAUD_ ))
#define Mantissa(_PCLK_, _BAUD_)							(uint32_t) (USARTDIV(_PCLK_, _BAUD_)/16)
#define DIV_Fraction(_PCLK_, _BAUD_)						(uint32_t) (USARTDIV(_PCLK_, _BAUD_)%16)
#define UART_BRR_Register(_PCLK_, _BAUD_)					(( Mantissa (_PCLK_, _BAUD_) ) <<4 )|( (DIV_Fraction(_PCLK_, _BAUD_)) & 0xF )

//-----------------------------
//Macros Configuration References
//-----------------------------

//@ref UART_Mode_define
#define UART_Mode_RX			(uint32_t)(1<<2) //Bit 2 RE: Receiver enable
#define UART_Mode_TX 			(uint32_t)(1<<3) //Bit 3 TE: Transmitter enable
#define UART_Mode_TX_RX			(uint32_t)((1<<3)|(1<<2))

//@ref UART_BaudRate_define
#define UART_BaudRate_2400			2400
#define UART_BaudRate_9600 			9600
#define UART_BaudRate_19200			19200
#define UART_BaudRate_57600			57600
#define UART_BaudRate_115200		115200
#define UART_BaudRate_230400		230400
#define UART_BaudRate_460800		460800
#define UART_BaudRate_921600		921600
#define UART_BaudRate_2250000		2250000
#define UART_BaudRate_4500000		4500000

// @ref UART_Payload_Length_define
#define UART_Payload_Length_8B                  ((uint32_t)(0))
#define UART_Payload_Length_9B                  (uint32_t)(1<<12)

//@ref UART_Parity_define
#define UART_Parity__NONE                    (uint32_t)(0)
#define UART_Parity__EVEN                    (uint32_t)(1<<10)
#define UART_Parity__ODD                     (uint32_t)(1<<10 | 1<<9)

//@ref UART_StopBits_define
#define UART_StopBits__half						(uint32_t)(1<<12)
#define UART_StopBits__1						(uint32_t)(0)
#define UART_StopBits__1_half					(uint32_t)(3<<12)
#define UART_StopBits__2						(uint32_t)(2<<12)

//@ref UART_HwFlowCtl_define
#define UART_HwFlowCtl_NONE						(uint32_t)(0)
#define UART_HwFlowCtl_RTS						(uint32_t)(1<<8)
#define UART_HwFlowCtl_CTS						(uint32_t)(1<<9)
#define UART_HwFlowCtl_RTS_CTS					(uint32_t)(3<<8)

//@ref UART_IRQ_Enable_define
/*
 * Can Set more than one Interrupt by OR them with each other
 * EX: to Enable TXE and TC
 * IRQ_Enable = (UART_IRQ_Enable_TXE | UART_IRQ_Enable_TC)
 * */
#define UART_IRQ_Enable_NONE					(uint32_t)(0)
#define UART_IRQ_Enable_TXE						(uint32_t)(1<<7)// Transmit data register empty ,TXE interrupt enable
#define UART_IRQ_Enable_TC						(uint32_t)(1<<6)// Transmission complete interrupt enable
#define UART_IRQ_Enable_RXNEIE					(uint32_t)(1<<5)// Received data ready to be read , RXNE interrupt enable
#define UART_IRQ_Enable_PE						(uint32_t)(1<<8)// Parity error , PE interrupt enable


/*
 *
 *
 * =======================================================================================
 * 							APIs Supported by "MCAL UART DRIVER"
 * =======================================================================================
 */


/* Initialization/de-initialization functions  **********************************/


void MCAL_UART_Init (USART_TypeDef *USARTx, UART_PinConfig_t* UART_Config);
void MCAL_UART_DeInit (USART_TypeDef *USARTx);


void MCAL_UART_GPIO_Set_Pins (USART_TypeDef *USARTx);


void MCAL_UART_SendData	(USART_TypeDef *USARTx, uint16_t *pTxBuffer,enum Polling_mechism PollingEn );
void MCAL_UART_ReceiveData	(USART_TypeDef *USARTx, uint16_t *pRxBuffer ,enum Polling_mechism PollingEn );

void MCAL_UART_WAIT_TC (USART_TypeDef *USARTx ) ;


#endif /* INC_STM32_F103C6_UART_DRIVER_H_ */
