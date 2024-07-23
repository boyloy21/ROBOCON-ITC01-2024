/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
PIDController MPID;
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI 3.1456
#define H 0.62
#define r 0.005
//#define CPR 64  // Motor bro Ra
//#define CPR 1300 // Motor bro Phana Motor 24V
//#define CPR2 1308  // Motor 12V
#define Sample_time 10 // ms
#define pi 3.14
//** Motor Shooting Bro Ra **//
//#define kp 0.03072
//#define ki 0.7483


//** Motor Shooting Bro Phana **//
#define CPR 946
#define kp 1.0510
#define ki 204.0816
//#define kp 3.6312  // Motor 24V
//#define ki 249.3097 // Motor 12V
//#define kp2 0.4122
//#define ki2 101.8
#define kd 0
#define Motor1 0
#define Motor2 1

// shooting ball
#define m 0.35 //[kg]
#define g 9.81 // [m/s^2]
#define b 1.225
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//CAN
uint8_t RxData[8];
uint8_t TxData[8];
uint32_t TxMailbox;
int flag = 0;
uint8_t cntt;
float RxData1 = 0;
float RxData2 = 0;
float RxData3 = 0;
float RxData4 = 0;
// MOTOR
uint16_t count[2]; // count pulse from encoder
uint16_t new_count[2];
uint8_t count_state[2];
uint16_t diff[2]; // difference between count and new_count in a sample time
float speedM[2];
float rdps[2];
float Motor1_speed;
float Motor2_speed;

// Shooter
float pwm1;
float pwm2;
float h;
float V1;
float V;
float theta;
float V_shooter;
float V_shooter2;
uint16_t V1_out;
uint16_t V2_out;
int x;
uint8_t goal_ball;
float Vx;
float Vy;
float Omega;
float X;
float Y;
float Yaw;
float X_end;
float laserX;
float laserY;
uint8_t state = 0;
uint8_t ball_pur = 0;
uint8_t check_ballinside;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float Motors_RPS( int j,float SampleTime, float N_round)
{

	new_count[Motor2] = TIM3->CNT;
	new_count[Motor1] = TIM4->CNT;
	count_state[Motor2] = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
	count_state[Motor1] = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);

	if (count_state[j])
	{
		if (new_count[j] <= count[j])
		{ // Check for counter underflow
			diff[j] = count[j] - new_count[j];
		}
		else
		{
			diff[j] = (65536 - new_count[j]) + count[j];
		}
		speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime)*-1;
	}
	else
	{
		if (new_count[j] >= count[j])
		{ // Check for counter overflow
			diff[j] = new_count[j] - count[j];
		}
		else
		{
			diff[j] = (65536 - count[j]) + new_count[j];
		}
		speedM[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime);
	}

	rdps[j] = -2.0f * pi * speedM[j];
	count[j] = new_count[j];

	return rdps[j];
}
float map(float Input, float Min_Input, float Max_Input, float Min_Output,
		float Max_Output) {

	return (float) ((Input - Min_Input) * (Max_Output - Min_Output)
			/ (Max_Input - Min_Input) + Min_Output);
}


void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, RxData);
		cntt++;
		while (cntt - 100 > 0) {
			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			cntt = 0;
		}

	if (RxHeader.StdId == 0x409) {
//		RxData1 = (RxData[0]<<8 | RxData[1]);
		RxData2 = (RxData[1]<<8 | RxData[2]);
		RxData3 = (RxData[3]<<8 | RxData[4]);
//		laserX = map(RxData2,0,65535,0.0,10.0);
//		laserY = map(RxData3,0,65535,0.0,10.0);
		check_ballinside = RxData[7];
		laserX = ((RxData2*0.08502) + 1.727)/100;
		laserY = ((RxData3*0.154) - 2.169)/100;
		flag = 1;
	}
	if (RxHeader.StdId == 0x222) {
				state = RxData[0];
				ball_pur = RxData[1];
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	PID_Init(&MPID, 2);
	MPID.T = 0.01; // T = 10ms
	MPID.limMax = 1000;
	MPID.limMin = -1000;
	MPID.limMaxInt = 1000;
	MPID.limMinInt = -1000;
	MPID.tau = 0; // for Kd
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_CAN_Start(&hcan);

  	HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  	TxHeader.DLC = 1;
  	TxHeader.IDE = CAN_ID_STD;
  	TxHeader.RTR = CAN_RTR_DATA;
  	TxHeader.StdId = 0x217; //Slave1
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		X = Vx*cosf(Yaw) - Vy*sinf(Yaw);
		Y = Vx*sinf(Yaw) + Vy*cosf(Yaw);
		Yaw = Omega;
		if ((laserX >= 0.02 && laserX <= 0.06 && (state == 17))
				|| (ball_pur == 1) || (check_ballinside==1 &&state == 6)) {
			V_shooter2 = -60;
		} else {
			V_shooter2 = 0.0;
		}

		if (state == 5 || state == 6 || state == 16 || state == 17 || (check_ballinside==1 && state == 6)) {
			V_shooter = -40;
		} else {
			V_shooter = 0.0;
		}



//		V_shooter2 = -40;
//		V_shooter = -40;
		Motor1_speed = Motors_RPS(Motor1, Sample_time, CPR); // Motor24V
		Motor2_speed = Motors_RPS(Motor2, Sample_time, CPR); // Motor12V
		pwm1 = PID(&MPID, V_shooter, Motor1_speed, kp, ki, kd, Motor1); //Motor24V
		pwm2 = PID(&MPID, V_shooter2, Motor2_speed, kp, ki, kd, Motor2); //Motor12V

		// Control Motor
		if (pwm2 > 10){
			TIM1->CCR1 = pwm2;
			TIM1->CCR2 = 0;
		}
		else if (pwm2 < -10){
			TIM1->CCR1 = 0;
			TIM1->CCR2 = -1 * pwm2;
		}
		else{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
		}
		if (pwm1 > 10){
			TIM1->CCR3 = 0;
			TIM1->CCR4 = pwm1;
		} else if (pwm1 < -10) {
			TIM1->CCR3 = -1 * pwm1;
			TIM1->CCR4 = 0;
		} else {
			TIM1->CCR3 = 0;
			TIM1->CCR4 = 0;
		}
//		V1_out = map(Motor1_speed, -50, 50, 0, 65535);
//		V2_out = map(Motor2_speed, -50, 50, 0, 65535);
		//		//


//		if (flag == 1) {
//			HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
//			flag = 0;
//		}

	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
