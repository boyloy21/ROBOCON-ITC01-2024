/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tim.h"
#include "adc.h"
//#include "time.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRIG_PIN GPIO_PIN_8
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_9
#define ECHO_PORT GPIOC
#define TRIG2_PIN GPIO_PIN_10
#define TRIG2_PORT GPIOB
#define ECHO2_PIN GPIO_PIN_11
#define ECHO2_PORT GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t Distance_Uldra  = 0;
uint16_t Distance_Uldra2 = 0;
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint32_t pMillis2;
uint32_t Value11 = 0;
uint32_t Value22 = 0;
uint8_t button =0;
uint16_t ADC2_read[2];
int gg;
uint8_t ball_aready = 0;
uint8_t check_threeball = 0;
uint8_t check_ballinside = 0;
uint16_t pwm_servo;
uint8_t degree;
/* USER CODE END Variables */
/* Definitions for Button_Task */
osThreadId_t Button_TaskHandle;
const osThreadAttr_t Button_Task_attributes = {
  .name = "Button_Task",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for HRS_04_Task */
osThreadId_t HRS_04_TaskHandle;
const osThreadAttr_t HRS_04_Task_attributes = {
  .name = "HRS_04_Task",
  .stack_size = 500 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
float Map(float Input, float Min_Input, float Max_Input, float Min_Output,
		float Max_Output) {

	return (float) ((Input - Min_Input) * (Max_Output - Min_Output)
			/ (Max_Input - Min_Input) + Min_Output);
}
void servo_rotation(uint16_t deg) // degree input from 0 to 180
{
	pwm_servo = Map(deg,0,180,1900,9600);
	TIM4->CCR1 = pwm_servo;
}
//void servo_rotation(uint16_t deg) // degree input from 0 to 180
//{
//	pwm_servo = map(deg,0,180,1900,9600);
//	TIM4->CCR1 = pwm_servo;
//}
/* USER CODE END FunctionPrototypes */

void Button_Init(void *argument);
void HRS_04_Init(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Button_Task */
  Button_TaskHandle = osThreadNew(Button_Init, NULL, &Button_Task_attributes);

  /* creation of HRS_04_Task */
  HRS_04_TaskHandle = osThreadNew(HRS_04_Init, NULL, &HRS_04_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Button_Init */
/**
  * @brief  Function implementing the Button_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Button_Init */
void Button_Init(void *argument)
{
  /* USER CODE BEGIN Button_Init */
	 HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  /* Infinite loop */
  for(;;)
  {
		if (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14))) {
			button = 1;
			servo_rotation(90);
			osDelay(1000);
			servo_rotation(0);
		}
		else if (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15)))
		{
			button = 2;
			servo_rotation(90);
			osDelay(1000);
			servo_rotation(0);
		}
		else {
			button = 0;
		}
    osDelay(10);
  }
  /* USER CODE END Button_Init */
}

/* USER CODE BEGIN Header_HRS_04_Init */
/**
* @brief Function implementing the HRS_04_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HRS_04_Init */
void HRS_04_Init(void *argument)
{
  /* USER CODE BEGIN HRS_04_Init */
	/*** Configure ULDRASONIC ***/
//		HAL_TIM_Base_Start(&htim1);
//		HAL_GPIO_WritePin(TRIG2_PORT, TRIG2_PIN, GPIO_PIN_RESET); // pull the TRIG pin low
  /* Infinite loop */
  for(;;)
  {

	  if (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))) {
		  ball_aready = 1;
	  		}

	  		else {
	  			ball_aready = 0;
	  		}
	  if (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1)) || (!(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10))))
			  {
		  	  	  check_threeball = 1;
			  }
	  else
	  {
		  check_threeball = 0;
	  }
//	  if ((HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_13)))
//	  			 	  {
//	  			 		  check_ballinside = 1;
//	  			 	  }
//	  			 	  else
//	  			 	  {
//	  			 		  check_ballinside = 0;
//	  			 	  }
    osDelay(50);
  }
  /* USER CODE END HRS_04_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

