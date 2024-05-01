/**
 ******************************************************************************
 * @file           : main.c
 * @author         : EL 3anteeel Team
 * @brief          : Collision Avoidance program body
 ******************************************************************************
 * @attention
 *
 *
 * This software component is licensed by EL 3anteeel Group under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * =======================================================================================
 * 									Includes"
 * =======================================================================================
 */
#include "Collision_Avoidance.h"


/*
 * =======================================================================================
 * 							Generic Variables
 * =======================================================================================
 */
uint16_t Actual_distance ;

 uint32_t V_F ; 	// Front vehicle speed
 uint32_t V_h ; 	// Following Vehicle speed
 uint32_t v_rel; 	// the relative speed between two vehicle
 uint32_t a_h ;		// the deceleration of the following vehicle
 uint32_t a_f ; 	// the deceleration of the front vehicle

 uint32_t Vf_temp ;	// Temp Front vehicle speed

 uint32_t threshold ; // the critical distance

 uint8_t situation ;

 /*
  * =======================================================================================
  * 							Generic Function
  * =======================================================================================
  */

 /*
   * The proposed FCW model considered four situations:
   * the front vehicle traveled at a constant speed, acceleration, normal deceleration, and emergency deceleration.
   */
 void Get_Vehicles_Situation()
 {
	 // at point one
	 APP_FCA_Get_Actual_Speed_deceleration();
	 HAL_US_GET_relativeAndFollowing_volcity(&v_rel, &V_h, &Vf_temp);

	 MCAL_TIMER_Delay(TIMER2, 1,TIMER_SEC);

	 // at point two
	 APP_FCA_Get_Actual_Speed_deceleration();
	 HAL_US_GET_relativeAndFollowing_volcity(&v_rel, &V_h, &V_F);

	 // determine the situation
     if      (V_F >= Vf_temp) 	situation = Vehicle_situation_constant_Speed_Acceleration;
	 else if (V_F < Vf_temp)
		 {
		 	 situation = Vehicle_situation_normal_deceleration;
		 	 a_f = ( Vf_temp - V_F );
		 }
	 else if (V_F == 0 ) 	situation = Vehicle_situation_emergency_deceleration;

 }


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
void APP_FCA_Detecte_Collision_Status()
{
	// Get Threshold
	APP_FCA_Calculte_Threshold();

	// Get Actual Distance
	HAL_US_GET_DISTANCE_Serial_Passive_Mode(&Actual_distance);

	if(Actual_distance <= 2*threshold)
	{
		// Collision Warning only
		APP_FCA_Set_Warning(Warning_ON);
	}
	else if(Actual_distance <= (3*threshold)/2)
	{
		// Collision Warning and ACC
		APP_FCA_Set_Warning(Warning_ON);
		APP_FCA_Set_ACC(ACC_ON);
	}
	else if(Actual_distance < threshold)
	{
		// Warning and Emergency brake (SW interrupt)
		APP_FCA_Set_Warning(Warning_ON);
		APP_FCA_Set_Emergency_Brake(EMB_ON);
	}
	else
	{
		// Warning , ACC and Emergency brake  OFF
		APP_FCA_Set_Warning(Warning_OFF);
		APP_FCA_Set_ACC(ACC_OFF);
		APP_FCA_Set_Emergency_Brake(EMB_OFF);
	}
}
/**================================================================
 * @Fn				-APP_FCA_Calculte_Threshold
 * @brief 			- Calculate the Critical Distance
 * @param [in] 		- threshold: Pointer to the Place which Critical distance Store in
 * @retval 			-none
 * Note				-none
 */
void APP_FCA_Calculte_Threshold()
{
	// Get the Situation of Vehicles
	Get_Vehicles_Situation();

	if(situation == Vehicle_situation_constant_Speed_Acceleration)
	{
		threshold = ((v_rel *(tbc +(tbr/2)+thum))/1000)+(((V_h*V_h)-(V_F*V_F))/(2*a_h))-((V_F*v_rel)/a_h)+D0;
	}
	else if(situation == Vehicle_situation_normal_deceleration)
	{
		threshold = ((V_h*V_h)/(2*a_h))-((V_F*V_F)/(2*a_f))+(((tbc+thum)*V_h)/1000)+((v_rel*(tbr/2))/1000)+D0;
	}
	else if (situation == Vehicle_situation_emergency_deceleration)
	{
		threshold = ((V_h*V_h)/(2*ahmax))-((V_F*V_F)/(2*afmax))+(((tbc+thum)*V_h)/1000)+((v_rel*(tbr/2))/1000)+D0;
	}

	//threshold += (v_rel * Ttran); //for the delay in measurements
}

/**================================================================
 * @Fn				-Get_Actual_Speed_deceleration
 * @brief 			- Receive the Actual velocity and deceleration
 * @param [in] 		-none
 * @retval 			-none
 * Note				-none
 */
void APP_FCA_Get_Actual_Speed_deceleration()
{
	 V_h = 500 ;
	 a_h = 30 ;
}
void APP_FCA_Set_Warning(uint8_t W_State)
{
	if (W_State == Warning_ON)
	{
		MCAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		MCAL_TIMER_Delay(TIMER2, 3, TIMER_SEC);
		MCAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		MCAL_TIMER_Delay(TIMER2, 3, TIMER_SEC);
	}
}
void APP_FCA_Set_ACC(uint8_t ACC_State)
{
	if (ACC_State == ACC_ON)
	{
		MCAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		MCAL_TIMER_Delay(TIMER2, 1, TIMER_SEC);
		MCAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		MCAL_TIMER_Delay(TIMER2, 1, TIMER_SEC);
	}
}
void APP_FCA_Set_Emergency_Brake(uint8_t EMB_State)
{
	if( EMB_State==EMB_ON )
	{
		MCAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
		MCAL_TIMER_Delay(TIMER2, 500, TIMER_MILL_SEC);
		MCAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
		MCAL_TIMER_Delay(TIMER2, 500, TIMER_MILL_SEC);
	}
}
