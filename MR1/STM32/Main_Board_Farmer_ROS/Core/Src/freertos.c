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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t Position;
uint8_t state;
int k;
float motor_air;
uint8_t catch;
extern uint8_t Pick;
extern uint8_t Store;
extern uint8_t Prepare;
/* USER CODE END Variables */
/* Definitions for Concept_Task */
osThreadId_t Concept_TaskHandle;
const osThreadAttr_t Concept_Task_attributes = {
  .name = "Concept_Task",
  .stack_size = 1200 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Store_Task */
osThreadId_t Store_TaskHandle;
const osThreadAttr_t Store_Task_attributes = {
  .name = "Store_Task",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Prepare_Task */
osThreadId_t Prepare_TaskHandle;
const osThreadAttr_t Prepare_Task_attributes = {
  .name = "Prepare_Task",
  .stack_size = 1000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Concept_Init(void *argument);
void Store_Init(void *argument);
void Prepare_Init(void *argument);

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
  /* creation of Concept_Task */
  Concept_TaskHandle = osThreadNew(Concept_Init, NULL, &Concept_Task_attributes);

  /* creation of Store_Task */
  Store_TaskHandle = osThreadNew(Store_Init, NULL, &Store_Task_attributes);

  /* creation of Prepare_Task */
  Prepare_TaskHandle = osThreadNew(Prepare_Init, NULL, &Prepare_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Concept_Init */
/**
  * @brief  Function implementing the Concept_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Concept_Init */
void Concept_Init(void *argument)
{
  /* USER CODE BEGIN Concept_Init */
  /* Infinite loop */
  for(;;)
  {
	  	if (Pick ==1 )
	  		  {
	  		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0); //grip
	  		 HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0);

			 // Slide
			motor_air = -500;
			osDelay(1200);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 1);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1); //Push
			state = 0;
	  		  }
	  		  else if (Pick == 2 && state == 0)
	  		  {
	  			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 1); //grip
	  			  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 1);
	  			  osDelay(500);
	  			motor_air = 800;
	  			osDelay(2000);
	  			motor_air = -800;
	  			osDelay(650);
	  		    motor_air = 0;
	  			state =1;
	  		  }

	  		  else if (Pick == 3 && state == 1)
	  		  {

	  			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0); // releas
	  				state = 2;
	  		  }
	  		  else if (Pick == 4 && state == 2)
	  		  {
	  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
	  			state = 3;
	  		  }
	  		  else if (Pick == 5 && state == 3)
	  		  {

	  			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0);
	  			state = 0;
	  		  }


  }
  /* USER CODE END Concept_Init */
}

/* USER CODE BEGIN Header_Store_Init */
/**
* @brief Function implementing the Store_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Store_Init */
void Store_Init(void *argument)
{
  /* USER CODE BEGIN Store_Init */
  /* Infinite loop */
  for(;;)
  {
	  if (Store == 1)
	  {
//		  osDelay(2000);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
//		  osDelay(1000);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
	  }
//	  else if (Store == 0)
//		{
//		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
//		  osDelay(500);
//		}
	  k++;

  }
  /* USER CODE END Store_Init */
}

/* USER CODE BEGIN Header_Prepare_Init */
/**
* @brief Function implementing the Prepare_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Prepare_Init */
void Prepare_Init(void *argument)
{
  /* USER CODE BEGIN Prepare_Init */
  /* Infinite loop */
  for(;;)
  {
	  if (Prepare == 1)  // Prepare rice
	  	  		  {
	  	  			motor_air = -500;
	  	  			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 0); //grip
	  	  		    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 0);
	  	  			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, 0);
	  	  			osDelay(1500);
	  	  			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, 1); //grip
	  	  		    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, 1);
	  	  		  }

//    osDelay(1);
  }
  /* USER CODE END Prepare_Init */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

