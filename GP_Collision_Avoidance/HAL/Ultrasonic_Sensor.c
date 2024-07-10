/*
 *  Ultrasonic_Sensor.c (URM37 V4.0 US Driver)
 *  Created on: Feb 21, 2024
 *  Author: Eng.ABDULLAH KARKOUR
 */

/*
 * =======================================================================================
 * 								INCLUDES
 * =======================================================================================
 */
#include "Ultrasonic_Sensor.h"


/*
 * =======================================================================================
 * 							Generic Variables
 * =======================================================================================
 */
EXTI_PinConfig_t G_EXTI_Confg ;
USART_TypeDef* G_USARTx ;

uint8_t TF_data[9];
uint32_t check ;

uint32_t US_time ;
uint16_t US_EN_Dis_cmd[4] =  {0x22,0x00,0x00,0x22};
uint8_t Recieve_Flag = 0 ;
uint16_t previous_distance , current_distance ;
uint32_t time ;

void US_CB(void)
{
	US_time = MCAL_TIMER_Get_Time(TIMER2);
	Recieve_Flag = 1 ;

	G_EXTI_Confg.IRQ_EN = EXTI_IRQ_DISABLE;
	MCAL_EXTI_GPIO_Update(&G_EXTI_Confg);
}

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
void HAL_US_Init(URM37_US_mode_t Sensor_Mode , USART_TypeDef* USARTx , TIMER_TypeDef* TIMERx)
{
	if (Sensor_Mode == PWM_Output_in_Trigger_Mode)
	{
		// Configuration of EXTI0PA0
		G_EXTI_Confg.EXTI_Pin = EXTI0PA0;
		G_EXTI_Confg.Trigger_case = EXTI_TRIGGER_FALLING;
		G_EXTI_Confg.P_IRQ_CallBack= US_CB;
		G_EXTI_Confg.IRQ_EN = EXTI_IRQ_DISABLE;
		MCAL_EXTI_GPIO_Update(&G_EXTI_Confg);

		// Configuration of Trig as OutPut
		GPIO_PinConfig_t PinCfg ;
		PinCfg.GPIO_PinNumber = GPIO_PIN_1;
		PinCfg.GPIO_Mode = GPIO_MODE_OUTPUT_PP;
		PinCfg.GPIO_Output_Speed = GPIO_SPEED_50M;
		MCAL_GPIO_Init(GPIOA, &PinCfg);

	}
	else if ((Sensor_Mode == Serial_Passive_Mode) || (Sensor_Mode == TF_LUNA_LIDAR_MODE) || (Sensor_Mode == UART_With_Microcontroller))
	{
		G_USARTx = USARTx ;
		UART_PinConfig_t uart_cfg;
		uart_cfg.BaudRate = UART_BaudRate_115200 ;
		uart_cfg.HwFlowCtl = UART_HwFlowCtl_NONE;
		uart_cfg.IRQ_Enable = UART_IRQ_Enable_NONE;
		uart_cfg.Mode = UART_Mode_TX_RX;
		uart_cfg.Parity = UART_Parity__NONE;
		uart_cfg.Payload_Length = UART_Payload_Length_8B ;
		uart_cfg.StopBits = UART_StopBits__1 ;
		MCAL_UART_Init(USARTx, &uart_cfg);
		MCAL_UART_GPIO_Set_Pins(USARTx);
	}

	// Start Timers which used in the APP
	MCAL_TIMER_Start_Stop_Calculate_Time(TIMERx, start);

}
/**================================================================
 * @Fn				- HAL_US_GET_DISTANCE_Serial_Passive_Mode
 * @brief 			- By serial, you have all authority to access to the sensor such as:
 * 					  ultrasonic distance measurement, temperature measurement, the distance changes, automatic measurement intervals set
 * @param [in] 		- US_distance: Pointer to the Place which distance Store in
 * @retval 			-none
 * Note				-none
 */
void HAL_US_GET_DISTANCE_Serial_Passive_Mode(uint16_t* US_distance)
{
	// Send Command To Get The Distance , Enable 16 bit distance reading
	uint8_t i ;
	for(i=0 ; i<4 ; i++)
	{
		MCAL_UART_SendData(G_USARTx, &US_EN_Dis_cmd[i], enable);
	}

	/* Receive The Distance Data , 16 bit distance reading
	 * Return data format will be: 0x22＋High(distance)＋Low(distance) SUM. When the reading is invalid
	 * it returns 0x22 0xFF 0xFF SUM
	 */
	uint16_t Disdance_Value[4] ;
	for(i=0 ; i<4 ; i++)
	{
		MCAL_UART_ReceiveData(G_USARTx, &Disdance_Value[i], enable);
	}

	if(!((Disdance_Value[1]==0xFF)&&(Disdance_Value[2]==0xFF)))
		*US_distance = ((Disdance_Value[1]<<8)|(Disdance_Value[2]));
}
/**================================================================
 * @Fn				- HAL_US_GET_DISTANC_PWM_Output_in_Trigger_Mode
 * @brief 			- the detected distance will be output in the form of low level pulse via PWM from the ECHO pin.
 * 					  Every 50US pulses represent 1 centimeter. In this way, we can read the distance
 * @param [in] 		- US_distance: Pointer to the Place which distance Store in
 * @retval 			-none
 * Note				-none
 */
void HAL_US_GET_DISTANC_PWM_Output_in_Trigger_Mode(uint16_t* US_distance)
{
	G_EXTI_Confg.IRQ_EN = EXTI_IRQ_ENABLE;
	MCAL_EXTI_GPIO_Update(&G_EXTI_Confg);

	MCAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	//MCAL_TIMER_Delay(TIMER2, 10, TIMER_MICRO_SEC);
	MCAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

	MCAL_TIMER_Start_Stop_Calculate_Time(TIMER2, start);

	while (!Recieve_Flag);
	Recieve_Flag=0;

	if (US_time <= 50000)
	*US_distance = (US_time/50) ;

	MCAL_TIMER_Delay(TIMER2, 200, TIMER_MILL_SEC);
}

/**================================================================
 * @Fn				- HAL_US_GET_Distance_TF_Luna_Lidar
 * @brief 			- Get Distance Using TFLuna Lidar Sensor using UART Serial interface with The Sensor
 * @param [in] 		- TF_distance: Pointer to the Place which distance Store in
 * @retval 			-none
 * Note				-none
 */
void HAL_US_GET_Distance_TF_Luna_Lidar(uint16_t* TF_distance)
{
	uint8_t i ;

	MCAL_UART_ReceiveData(G_USARTx, &TF_data[0], enable);
	if(TF_data[0] == 0x59 )
	{
		MCAL_UART_ReceiveData(G_USARTx, &TF_data[1], enable);
		if(TF_data[1] == 0x59 )
		{
			for(i= 2 ; i<9 ; i++)
			{
				MCAL_UART_ReceiveData(G_USARTx, &TF_data[i], enable);
			}

			check = TF_data[0] + TF_data[1] + TF_data[2] + TF_data[3] + TF_data[4] + TF_data[5] + TF_data[6] + TF_data[7];
			if (TF_data[8] == (check & 0xFF))
			{
				//verify the received data as per protocol
				*TF_distance = (TF_data[2] + (TF_data[3] << 8)) ;  //calculate distance value
			}
		}
	}
}
/**================================================================
 * @Fn				- Hal_US_GET_distance
 * @brief 			- Get Distance by interface with another microcontroller using UART Communication
 * @param [in] 		- TF_distance: Pointer to the Place which distance Store in
 * @retval 			-none
 * Note				-none
 */
void Hal_US_GET_distance(uint16_t* TF_distance)
{
	MCAL_UART_ReceiveData(G_USARTx, TF_distance, enable);
	*TF_distance *=10 ;
}

/**================================================================
 * @Fn				- HAL_US_GET_relativeAndFollowing_volcity
 * @brief 			- Get relative velocity and velocity of the following vehicle
 * @param [in] 		- V_rel: Pointer to the Place which relative velocity Store in
 * @param [in] 		- Actual_Volicty: Pointer to the Place which Actual_Volicty of vehicle Store in
 * @param [in] 		- V_f: Pointer to the Place which  the Front vehicle Store in
 * @retval 			-none
 * Note				-none
 */
void HAL_US_GET_relativeAndFollowing_volcity(uint32_t* V_rel,uint32_t* Actual_Volicty , uint32_t* V_f)
{

	// Get Distance1
	//HAL_US_GET_DISTANCE_Serial_Passive_Mode(&distance1);

	// Start Delay
	//MCAL_TIMER_Delay(TIMER2, 30 , TIMER_MILL_SEC);

	time = MCAL_TIMER_Get_Time(TIMER3);
	if(time >= 300)
	{
		//Stop time
		MCAL_TIMER_Start_Stop_Calculate_Time(TIMER3, stop);

		// Get distance2
		Hal_US_GET_distance(&current_distance);

		// find the relative velocity
		*V_rel = (current_distance > previous_distance)? (((current_distance - previous_distance)*1000000)/time) :  (((previous_distance - current_distance)*1000000)/time);

		// velocity of the following vehicle
		*V_f = (current_distance > previous_distance)?  (*V_rel + *Actual_Volicty) : (*Actual_Volicty - *V_rel) ;

		previous_distance = current_distance ;

		//Reset timer
		MCAL_TIMER_Start_Stop_Calculate_Time(TIMER3, start);
	}
}



