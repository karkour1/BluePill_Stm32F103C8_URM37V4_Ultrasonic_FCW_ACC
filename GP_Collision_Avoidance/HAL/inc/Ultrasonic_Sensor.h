/*
 *  Ultrasonic_Sensor.h (URM37 V4.0 US Driver)
 *  Created on: Feb 21, 2024
 *  Author: Eng.ABDULLAH KARKOUR
 */

#ifndef INC_ULTRASONIC_SENSOR_H_
#define INC_ULTRASONIC_SENSOR_H_

#include "stdint.h"
#include "Stm32_F103C6_TIMERS_driver.h"
#include "Stm32_F103C6_UART_driver.h"
#include "Stm32_F103C6_EXTI_driver.h"
#include "Stm32_F103C6_gpio_driver.h"
#include "Stm32_F103X6.h"

/*
 * =======================================================================================
 * 									MCROS
 * =======================================================================================
 */

// The Mode Of URM37 V4.0 Ultrasonic Sensor
typedef enum {
	Serial_Passive_Mode ,
	PWM_Output_in_Trigger_Mode ,
	TF_LUNA_LIDAR_MODE ,
	UART_With_Microcontroller
}URM37_US_mode_t;

/*
 * =======================================================================================
 * 							APIs Supported by "Ultrasonic Sensor DRIVER"
 * =======================================================================================
 */
/**================================================================
 * @Fn				-HAL_US_Init
 * @brief 			- Initializes the Sensor Configuration
 * @param [in] 		- Sensor_Mode: The Mode Of URM37 V4.0 Ultrasonic Sensor
 * @param [in] 		- USARTx: The USARTx (x-> 1,2,3) which data transmit within it .
 * @retval 			-none
 * Note				-in Serial_Passive_Mode you must select the USART that you use correct
 */
void HAL_US_Init(URM37_US_mode_t Sensor_Mode , USART_TypeDef* USARTx , TIMER_TypeDef* TIMERx);
/**================================================================
 * @Fn				- HAL_US_GET_DISTANCE_Serial_Passive_Mode
 * @brief 			- By serial, you have all authority to access to the sensor such as:
 * 					  ultrasonic distance measurement, temperature measurement, the distance changes, automatic measurement intervals set
 * @param [in] 		- US_distance: Pointer to the Place which distance Store in
 * @retval 			-none
 * Note				-none
 */
void HAL_US_GET_DISTANCE_Serial_Passive_Mode(uint16_t* US_distance);
/**================================================================
 * @Fn				- HAL_US_GET_DISTANC_PWM_Output_in_Trigger_Mode
 * @brief 			- the detected distance will be output in the form of low level pulse via PWM from the ECHO pin.
 * 					  Every 50US pulses represent 1 centimeter. In this way, we can read the distance
 * @param [in] 		- US_distance: Pointer to the Place which distance Store in
 * @retval 			-none
 * Note				-none
 */
void HAL_US_GET_DISTANC_PWM_Output_in_Trigger_Mode(uint16_t* US_distance);
/**================================================================
 * @Fn				- HAL_US_GET_Distance_TF_Luna_Lidar
 * @brief 			- Get Distance Using TFLuna Lidar Sensor using UART Serial interface with The Sensor
 * @param [in] 		- TF_distance: Pointer to the Place which distance Store in
 * @retval 			-none
 * Note				-none
 */
void HAL_US_GET_Distance_TF_Luna_Lidar(uint16_t* TF_distance);
/**================================================================
 * @Fn				- Hal_US_GET_distance
 * @brief 			- Get Distance by interface with another microcontroller using UART Communication
 * @param [in] 		- TF_distance: Pointer to the Place which distance Store in
 * @retval 			-none
 * Note				-none
 */
void Hal_US_GET_distance(uint16_t* TF_distance);
/**================================================================
 * @Fn				- HAL_US_GET_relativeAndFollowing_volcity
 * @brief 			- Get relative velocity and velocity of the following vechile
 * @param [in] 		- V_rel: Pointer to the Place which relative velocity Store in
 * @param [in] 		- Actual_Volicty: Pointer to the Place which Actual_Volicty of vechile Store in
 * @param [in] 		- V_f: Pointer to the Place which  the following vechile Store in
 * @retval 			-none
 * Note				-none
 */
void HAL_US_GET_relativeAndFollowing_volcity(uint32_t* V_rel,uint32_t* Actual_Volicty , uint32_t* V_f);

#endif /* INC_ULTRASONIC_SENSOR_H_ */
