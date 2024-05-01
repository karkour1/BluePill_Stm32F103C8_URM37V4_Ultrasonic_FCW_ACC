/*
 * Stm32_F103C6_gpio_driver.h
 * Created on: Jan 5, 2024
 * Author: Abdullah karkour
 */

#ifndef INC_STM32_F103C6_GPIO_DRIVER_H_
#define INC_STM32_F103C6_GPIO_DRIVER_H_
//-----------------------------
//Includes
//-----------------------------
#include "Stm32_F103X6.h"

//-----------------------------
//User type definitions (structures)
//-----------------------------
typedef struct
{
	uint8_t GPIO_PinNumber ;      // Specifies the GPIO pins to be configured.
							     // This parameter must be set based on @ref GPIO_PINS_define
	uint8_t GPIO_Mode ;            // Specifies the operating mode for the selected pins
								 // This parameter can be a value of @ref GPIO_MODE_define
	uint8_t GPIO_Output_Speed ;    // Specifies the speed for the selected pins
								// This parameter can be a value of @ref GPIO_SPEED_define

}GPIO_PinConfig_t;
//-----------------------------
//Macros Configuration References
//-----------------------------

//@ref GPIO_PIN_state
#define GPIO_PIN_SET    	1
#define GPIO_PIN_RESET      0

//@ref GPIO_RETURN_LOCK
#define GPIO_RETURN_LOCK_Enabled    	1
#define GPIO_RETURN_LOCK_ERROR     	    0

//@ref GPIO_PINS_define
#define GPIO_PIN_0                 ((uint8_t)0x00)  /* Pin 0 selected    */
#define GPIO_PIN_1                 ((uint8_t)0x01)  /* Pin 1 selected    */
#define GPIO_PIN_2                 ((uint8_t)0x02)  /* Pin 2 selected    */
#define GPIO_PIN_3                 ((uint8_t)0x03)  /* Pin 3 selected    */
#define GPIO_PIN_4                 ((uint8_t)0x04)  /* Pin 4 selected    */
#define GPIO_PIN_5                 ((uint8_t)0x05)  /* Pin 5 selected    */
#define GPIO_PIN_6                 ((uint8_t)0x06)  /* Pin 6 selected    */
#define GPIO_PIN_7                 ((uint8_t)0x07)  /* Pin 7 selected    */
#define GPIO_PIN_8                 ((uint8_t)0x08)  /* Pin 8 selected    */
#define GPIO_PIN_9                 ((uint8_t)0x09)  /* Pin 9 selected    */
#define GPIO_PIN_10                ((uint8_t)0x0A)  /* Pin 10 selected   */
#define GPIO_PIN_11                ((uint8_t)0x0B)  /* Pin 11 selected   */
#define GPIO_PIN_12                ((uint8_t)0x0C)  /* Pin 12 selected   */
#define GPIO_PIN_13                ((uint8_t)0x0D)  /* Pin 13 selected   */
#define GPIO_PIN_14                ((uint8_t)0x0E)  /* Pin 14 selected   */
#define GPIO_PIN_15                ((uint8_t)0x0F)  /* Pin 15 selected   */
#define GPIO_PIN_All               ((uint8_t)0xFF)  /* All pins selected */

#define GPIO_PIN_MASK              0x0000FFFFu /* PIN mask for assert test */

//@ref GPIO_MODE_define
//0: Analog mode
//1: Floating input (reset state)
//2: Input with pull-up
//3:  pull-down
//4: General purpose output push-pull
//5: General purpose output Open-drain
//6: Alternate function output Push-pull
//7: Alternate function output Open-drain
//8: Alternate function  INPUT
#define  GPIO_MODE_Analog						 0x00000000u   //Analog mode
#define  GPIO_MODE_INPUT_FLO                     0x00000001u   //Floating input
#define  GPIO_MODE_INPUT_PU                      0x00000002u   //Input with pull-up
#define  GPIO_MODE_INPUT_PD                      0x00000003u   //Input with pull-down
#define  GPIO_MODE_OUTPUT_PP                     0x00000004u   //General purpose output push-pull
#define  GPIO_MODE_OUTPUT_OD                     0x00000005u   //General purpose output Open-drain
#define  GPIO_MODE_OUTPUT_AF_PP                  0x00000006u   //Alternate function output Push-pull
#define  GPIO_MODE_OUTPUT_AF_OD                  0x00000007u   //Alternate function output Open-drain
#define  GPIO_MODE_AF_INPUT                      0x00000008u   //Alternate function INPUT

//@ref GPIO_SPEED_define
//1: Output mode, max speed 10 MHz.
//2: Output mode, max speed 2 MHz.
//3: Output mode, max speed 50 MHz

#define GPIO_SPEED_10M		0x00000001u  //Output mode, max speed 10 MHz.
#define GPIO_SPEED_2M		0x00000002u  //Output mode, max speed 2 MHz.
#define GPIO_SPEED_50M		0x00000003u  //Output mode, max speed 50 MHz.
/*@ref Module_REF_NAME_define
*/
/*
* ===============================================
* APIs Supported by "MCAL GPIO DRIVER"
* ===============================================
*/

void MCAL_GPIO_Init(GPIO_TypeDef *GPIOX ,  GPIO_PinConfig_t *PinConfig);
void MCAL_GPIO_DeInit(GPIO_TypeDef *GPIOX );

uint8_t MCAL_GPIO_ReadPin (GPIO_TypeDef *GPIOX,uint8_t PinNumber );
uint16_t MCAL_GPIO_ReadPort (GPIO_TypeDef *GPIOX);

void MCAL_GPIO_WritePin (GPIO_TypeDef *GPIOX,uint8_t PinNumber , uint8_t Value);
void MCAL_GPIO_WritePort (GPIO_TypeDef *GPIOX, uint16_t Value);

void MCAL_GPIO_TogglePin (GPIO_TypeDef *GPIOX,uint8_t PinNumber );

uint8_t MCAL_GPIO_LockPin (GPIO_TypeDef *GPIOX,uint8_t PinNumber );

#endif /* INC_STM32_F103C6_GPIO_DRIVER_H_ */
