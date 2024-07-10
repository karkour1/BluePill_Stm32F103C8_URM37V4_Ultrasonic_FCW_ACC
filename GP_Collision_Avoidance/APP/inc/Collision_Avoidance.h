/*
 * Collision_Avoidance.h
 *
 *  Created on: Feb 21, 2024
 *      Author: Hello
 */

#ifndef INC_COLLISION_AVOIDANCE_H_
#define INC_COLLISION_AVOIDANCE_H_

/*
 * =======================================================================================
 * 									Includes"
 * =======================================================================================
 */
#include "Ultrasonic_Sensor.h"
#include "Stm32_F103C6_TIMERS_driver.h"
#include "Stm32_F103X6.h"

/*
 * =======================================================================================
 * 									Macros"
 * =======================================================================================
 */
#define Vehicle_situation_constant_Speed_Acceleration		0
#define Vehicle_situation_normal_deceleration				1
#define Vehicle_situation_emergency_deceleration			2

#define tbc			150  // the time between pressing the brake pedal of the following vehicle and the braking effect
#define tbr         450  // deceleration time of the following vehicle
#define thum 		1200 // the driver response time of the following vehicle
#define D0  		50  // predetermined safe distance between the two vehicles, which is defined as 2 m.
#define ahmax       600    // the maximum deceleration of the following vehicle
#define afmax       600    // the maximum deceleration of the front vehicle

#define Warning_ON		1
#define Warning_OFF		0

#define ACC_ON			1
#define ACC_OFF			0

#define EMB_ON			1
#define EMB_OFF			0
/*
 * =======================================================================================
 * 							APIs Supported by "Collision Avoidance DRIVER"
 * =======================================================================================
 */

/**================================================================
 * @Fn				-APP_FCA_Detecte_Collision_Status
 * @brief 			- Detect the Collision System Action According the Distance between vehicle and the object
 * @param [in] 		-Actual_distance : pointer the Distance between vehicle and the object
 * @retval 			-none
 * Note				-none
 */
void APP_FCA_Detecte_Collision_Status();
/**================================================================
 * @Fn				-APP_FCA_Calculte_Threshold
 * @brief 			- Calculate the Critical Distance
 * @retval 			-none
 * Note				-none
 */
void APP_FCA_Calculte_Threshold();

/**================================================================
 * @Fn				-APP_FCA_Get_Actual_Speed_deceleration
 * @brief 			- Receive the Actual velocity and deceleration
 * @param [in] 		-none
 * @retval 			-none
 * Note				-none
 */
void APP_FCA_Get_Actual_Speed_deceleration();

void APP_FCA_Init(USART_TypeDef* USARTx);
void APP_FCA_Set_Warning(uint8_t W_State);
void APP_FCA_Set_ACC(uint8_t ACC_State);
void APP_FCA_Set_Emergency_Brake(uint8_t EMB_State);
#endif /* INC_COLLISION_AVOIDANCE_H_ */
