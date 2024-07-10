/**
 ******************************************************************************
 * @file           : main.c
 * @author         : ABDULLAH KARKOUR
 * @brief          : Collision Avoidance program body
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
 uint32_t a_h  ;		// the deceleration of the following vehicle
 uint32_t a_f ; 	// the deceleration of the front vehicle


 uint32_t Vf_temp  ;	// Temp Front vehicle speed

 uint32_t V_time ;

 uint32_t threshold ; // the critical distance
 uint32_t G_threshold = 100 ;
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
	// MCAL_TIMER_Start_Stop_Calculate_Time(TIMER4, start);

	 // at point two
	 APP_FCA_Get_Actual_Speed_deceleration();
	 HAL_US_GET_relativeAndFollowing_volcity(&v_rel, &V_h, &V_F);
	 V_time = MCAL_TIMER_Get_Time(TIMER3);
	// MCAL_TIMER_Start_Stop_Calculate_Time(TIMER4, stop);

	 // determine the situation
     if      (V_F >= Vf_temp)
    	 {
    	 	 situation = Vehicle_situation_constant_Speed_Acceleration;
    	 }
	 else if (V_F < Vf_temp)
		 {
		 	 situation = Vehicle_situation_normal_deceleration;
		 	 a_f = ((( Vf_temp - V_F ) * 1000000)/ V_time);
		 }
	 else if (V_F == 0 ) 	situation = Vehicle_situation_emergency_deceleration;

 }


/*
 * =======================================================================================
 * 							APIs Supported by "Collision Avoidance DRIVER"
 * =======================================================================================
 */
 void APP_FCA_Init(USART_TypeDef* USARTx)
 {
	 // Configure USART which used to get Speed and Acceleration
		UART_PinConfig_t uart_cfg;
		uart_cfg.BaudRate = UART_BaudRate_115200;
		uart_cfg.HwFlowCtl = UART_HwFlowCtl_NONE;
		uart_cfg.IRQ_Enable = UART_IRQ_Enable_NONE;
		uart_cfg.Mode = UART_Mode_TX_RX;
		uart_cfg.Parity = UART_Parity__NONE;
		uart_cfg.Payload_Length = UART_Payload_Length_8B ;
		uart_cfg.StopBits = UART_StopBits__1 ;
		MCAL_UART_Init(USARTx, &uart_cfg);
		MCAL_UART_GPIO_Set_Pins(USARTx);

		 // Configure Pins of Warning
		GPIO_PinConfig_t PinCfg ;
		PinCfg.GPIO_PinNumber = GPIO_PIN_12;
		PinCfg.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
		PinCfg.GPIO_Output_Speed = GPIO_SPEED_50M;
		MCAL_GPIO_Init(GPIOA, &PinCfg);
		MCAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

		PinCfg.GPIO_PinNumber = GPIO_PIN_11;
		PinCfg.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
		PinCfg.GPIO_Output_Speed = GPIO_SPEED_50M;
		MCAL_GPIO_Init(GPIOA, &PinCfg);
		MCAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

 }
 /**================================================================
  * @Fn				-APP_FCA_Detecte_Collision_Status
  * @brief 			- Detect the Collision System Action According the Distance between vehicle and the object
  * @param [in] 	-Actual_distance : pointer the Distance between vehicle and the object
  * @retval 		-none
  * Note			-none
  */
void APP_FCA_Detecte_Collision_Status()

{
	// Get Threshold
	APP_FCA_Calculte_Threshold();

	// Get Actual Distance
	Hal_US_GET_distance(&Actual_distance);

	if((Actual_distance < (threshold/3)) && (V_h != 0))
	{
		// Warning and Emergency brake (SW interrupt)
		APP_FCA_Set_Warning(Warning_ON);
		APP_FCA_Set_Emergency_Brake(EMB_ON);
	}
	else if((Actual_distance <= (3*(threshold/3))/2)&& (V_h != 0))
	{
		// Collision Warning and ACC
		APP_FCA_Set_Warning(Warning_ON);
		APP_FCA_Set_ACC(ACC_ON);
		APP_FCA_Set_Emergency_Brake(EMB_OFF);
	}
	else if((Actual_distance <= 2*(threshold/3)) && (V_h != 0) && (Actual_distance < G_threshold))
	{
		// Collision Warning only
		APP_FCA_Set_Warning(Warning_ON);
		APP_FCA_Set_Emergency_Brake(EMB_OFF);
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
		threshold = ((v_rel *(tbc +(tbr/2)+thum))/1000)+(((V_h*V_h)-(V_F*V_F))/(2*a_h))-((V_F*v_rel)/a_h)+ D0;
	}
	else if(situation == Vehicle_situation_normal_deceleration)
	{
		threshold = ((V_h*V_h)/(2*a_h))-((V_F*V_F)/(2*a_f))+(((tbc+thum)*V_h)/1000)+((v_rel*(tbr/2))/1000)+D0;
	}
	else if (situation == Vehicle_situation_emergency_deceleration)
	{
		threshold = ((V_h*V_h)/(2*ahmax))-((V_F*V_F)/(2*afmax))+(((tbc+thum)*V_h)/1000)+((v_rel*(tbr/2))/1000)+D0;
	}
	if(threshold >1000)
	{
		threshold = 1000;
	}

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
	uint16_t data ;
	MCAL_UART_ReceiveData(USART2, &data, enable);

	// Speed ... MSB 1
	// ACC   ... MSB 0

	if((data >>7))
	{
		V_h = (10*(data & 0x7F));
	}
	else if((data >>7) == 0)
	{
		a_h = (data / 100) ;
	}
}

// Function to predict future position of a vehicle
void predict_position( uint32_t *Predict_distance)
{
    *Predict_distance = V_h * 2 ;
}

// Function to predict future position of an obstacle
void predict_obstacle_position(uint32_t *obstacle) {
    // For simplicity, assuming constant velocity
	uint16_t  obstacle_position ;
	HAL_US_GET_DISTANCE_Serial_Passive_Mode(&obstacle_position);
	HAL_US_GET_relativeAndFollowing_volcity(&v_rel, &V_h, &V_F);
	*obstacle = V_F *2 ;
}

void APP_FCA_Set_Warning(uint8_t W_State)
{
	if (W_State == Warning_ON)
	{
		MCAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	}
	else
	{
		MCAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	}
}
void APP_FCA_Set_ACC(uint8_t ACC_State)
{
	if (ACC_State == ACC_ON)
	{
		//ACC ON

	}
}
void APP_FCA_Set_Emergency_Brake(uint8_t EMB_State)
{
	if( EMB_State==EMB_ON )
	{
		// Stop Car
		//MCAL_TIMER_Generate_PWM(TIMER3, CH_1, 75 , 500);
		MCAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
	}
	else
	{
		//MCAL_TIMER_Generate_PWM(TIMER3, CH_1, 0 , 500);
		MCAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

	}
}

