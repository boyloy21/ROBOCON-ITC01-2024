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
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"bno055.h"
#include"bno055_stm32.h"
#include "math.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;



typedef struct{
	uint32_t counter;
	uint32_t new_counter;
	uint8_t counter_status;
	float speed;
	float rdps;
	double distant;
}Encoder;

Encoder encoderXR;
Encoder encoderXL;
Encoder encoderY;
bno055_vector_t Q;
typedef struct {
	double Roll;
	double Pitch;
	double Yaw;
}Rotation;
Rotation Angle;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define b 0.5
#define Ly 0.2
#define PI 3.14159
#define r 0.03 //[Radius m]
#define CPR_XR 800
#define CPR_XL 800
#define CPR_Y 1400
#define C   2*PI*r //[circumference  C=2*Pi*R]
#define sampling_time  10 //[10ms]
#define sampleTime  0.01
#define dt 0.01
#define RotaryXR 0
#define RotaryXL 1
#define RotaryY 2
//Wheel
#define R 0.05
#define lx 0.165  //m
#define ly 0.225  //m
#define d1 0.05
#define d2 0.15
#define X 0
#define Y 1
#define YAW 2
#define alpha 0.8


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float Vx;
float Vy;
float Omega;
float vy;

//Variable of Calculate Position Robot from Rotary encoder wheel
extern float rdps[3];
float output_Vx;
float output_Vy;
float output_Omega;
float WR;
float WL;
float WY;
float V;
float Omega;

float Vx_enR;
float Vy_enR;
float Vy_enR1;
float Vy_enR2;
float X_old_enR;
float Y_old_enR;
float Yaw_old_IMU;
float yaw_old_enR;
float X_enR;
float Y_enR;
float yaw_enR;
float wheel_velocity_encoder[3];
float P_enR[3];
float old_count[3];
uint16_t count[3]; // count pulse from encoder
uint16_t new_count[3];
uint8_t count_state[3];
uint16_t diff[3]; // difference between count and new_count in a sample time
float SpeedR[3];
float rdps[3];
double position;

// IMU
double siny_cosp;
double cosy_cosp;
double angle; // Angle in degrees
double radians;
float theta;
float phi;
//----------CAN----------------//
uint8_t TxData[8];
uint32_t TxMailbox;
uint16_t rotaryX = 0;
uint16_t rotaryY = 0;
uint16_t imu = 0;
uint16_t laser = 0;
uint16_t j;

uint8_t RxData[8];
//uint8_t TxData[8];
//uint32_t TxMailbox;
float RxData1 = 0;
float RxData2 = 0;
float RxData3 = 0;
float RxData4 = 0;
float V1 = 0; // target speed of motor1
float V2 = 0; // target speed of motor2
float V3 = 0; // target speed of motor1
float V4 = 0; // target speed of motor2

uint8_t datacheck = 0;
uint8_t cntt;
float V_back;
float V1_back;
float V2_back;
float V3_back;
float V4_back;

//NRF
int c;
//Varible to read Encoder
float v;
uint8_t nowA[4];
uint8_t nowB[4];
uint8_t lastA[4];
uint8_t lastB[4];
uint8_t dir[4];
uint16_t cnt[4];
uint16_t Enc_count[4];
volatile int32_t encoder_position = 0;
volatile uint32_t last_time = 0;
volatile float omega = 0.0;
uint16_t pwm_putball1;
uint16_t pwm_putball2;
float speed_shooter1;
float speed_shooter2;

// DATA to sensor
float thetaR = 0.0;
float thetaL = 0.0;
float thetaY = 0.0;

// Laser
extern float laserx;
extern float lasery;
float distance;
uint16_t LaserX;
uint8_t LaserY;
uint8_t state;
uint8_t ball_aready;
int jj;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
float map(float Input, float Min_Input , float Max_Input ,float Min_Output, float Max_Output){

	return (float) ((Input - Min_Input) * (Max_Output - Min_Output) / (Max_Input - Min_Input) + Min_Output);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);
	cntt++;
	while (cntt - 100 > 0) {
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
		cntt = 0;
	}

	if (RxHeader.StdId == 0x111) {
//			distance = map(RxData[0], 0, 255, 0.0, 20.0);
			RxData1 = (RxData[0] << 8) | RxData[1];
			distance = map(RxData1, 0, 65535, 0.0, 100.0);
	}
	else if (RxHeader.StdId == 0x222) {
			state = RxData[0];
	}
	else if (RxHeader.StdId == 0x409)
	{
		ball_aready = RxData[0];
	}
}
void read_encoder(Encoder *enc, TIM_HandleTypeDef* timer,uint16_t cpr){
	enc->new_counter = __HAL_TIM_GET_COUNTER(timer);
	enc->counter_status = __HAL_TIM_IS_TIM_COUNTING_DOWN(timer);
	int16_t count_change = enc->new_counter - enc->counter;
	if(enc->counter_status && count_change <0){
		count_change += 65536;
	}else if (!enc->counter_status && count_change > 0){
		count_change -= 65536;
	}
	enc->counter = enc->new_counter;
	enc->counter_status = (count_change >=0);
	enc->speed = (float)count_change*1000.0f/(cpr * sampling_time);
	enc->rdps = (float)count_change*2*PI*1000.0f/(cpr * sampling_time);
}

//float encoder(int i) {
//	if (nowA[i] != lastA[i]) {
//		lastA[i] = nowA[i];
//		if (lastA[i] == 0) {
//			if (nowB[i] == 0) {
//				dir[i] = 0;
//				cnt[i]--;
//			} else {
//				dir[i] = 1;
//				cnt[i]++;
//			}
//		} else {
//			if (nowB[i] == 1) {
//				dir[i] = 0;
//				cnt[i]--;
//			} else {
//				dir[i] = 1;
//				cnt[i]++;
//			}
//		}
//	}
//	if (nowB[i] != lastB[i]) {
//		lastB[i] = nowB[i];
//		if (lastB[i] == 0) {
//			if (nowA[i] == 1) {
//				dir[i] = 0;
//				cnt[i]--;
//			} else {
//				dir[i] = 1;
//				cnt[i]++;
//			}
//		} else {
//			if (nowA[i] == 0) {
//				dir[i] = 0;
//				cnt[i]--;
//			} else {
//				dir[i] = 1;
//				cnt[i]++;
//			}
//		}
//	}
//	return cnt[i];
//}
//float Rotary_RPS( int j,float SampleTime, float N_round)
//{
//	new_count[RotaryXR] = Enc_count[0];
//	new_count[RotaryXL] = Enc_count[1];
//	new_count[RotaryY] = Enc_count[2];
//
//	count_state[RotaryXR] = !dir[0];
//	count_state[RotaryXL] = !dir[1];
//	count_state[RotaryY] = !dir[2];
//
//	if (count_state[j])
//	{
//		if (new_count[j] <= count[j])
//		{ // Check for counter underflow
//			diff[j] = count[j] - new_count[j];
//		}
//		else
//		{
//			diff[j] = (65536 - new_count[j]) + count[j];
//		}
//		SpeedR[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime)*-1;
//	}
//	else
//	{
//		if (new_count[j] >= count[j])
//		{ // Check for counter overflow
//			diff[j] = new_count[j] - count[j];
//		}
//		else
//		{
//			diff[j] = (65536 - count[j]) + new_count[j];
//		}
//		SpeedR[j] = (float)diff[j] * 1000.0f / (N_round * SampleTime);
//	}
//
//	rdps[j] = -2.0f * PI * SpeedR[j];
//	count[j] = new_count[j];
//
//	return rdps[j];
//}
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
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_ADC2_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  // CAN _Transmition
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	TxHeader.DLC = 8; // data length
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = 0x407; //Id 0x7FF

	// TIMER Internal clock
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim12);
	//*** SENSOR FEEDBACK ***//
	// IMU
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();

	// Read Rotary encoder
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	// IRFilter
//	IIRFilter_Init(&IR, alpha);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3) {
		read_encoder(&encoderXR, &htim5, CPR_XR);
		read_encoder(&encoderXL, &htim4, CPR_XL);
		read_encoder(&encoderY, &htim2, CPR_Y);

		WL = (double) encoderXR.rdps * r;
		WR = (double) encoderXL.rdps * r;
		WY = (double) encoderY.rdps * r;
		V = (WR + WL) / 2;
		Omega = (WR - WL) / b;
//		vy = WY - Omega * Ly;
//				phi = phi + Omega * dt;
//				  	  		if (phi > 2*PI || phi <-2*PI)
//				  	  		{
//				  	  			phi = 0.0;
//				  	  		}
//				  	  		if (phi >= PI)
//				  	  		{
//				  	  			phi -= 2*PI;
//				  	  		}
//				  	  		else if (phi <= -PI)
//				  	  		{
//				  	  			phi += 2*PI;
//				  	  		}
//				  	  	theta = phi;
		rotaryX = map(V, -100.0, 100.0, 0, 65535);
		rotaryY = map(WY, -100.0, 100.0, 0, 65535);
		imu = map(theta, -3.14159, 3.14159, 0, 65535);
		TxData[0] = ((rotaryX & 0xFF00) >> 8);
		TxData[1] = (rotaryX & 0x00FF);
		TxData[2] = ((rotaryY & 0xFF00) >> 8);
		TxData[3] = (rotaryY & 0x00FF);
		TxData[4] = ((imu & 0xFF00) >> 8);
		TxData[5] = (imu & 0x00FF);
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		c++;

	}
	if (htim->Instance == TIM12) {

		//**** Position From Rotary encoder and IMU   *****//
		Q = bno055_getVectorQuaternion();
		// yaw (z-axis rotation)
		siny_cosp = 2 * (Q.w * Q.z + Q.x * Q.y);
		cosy_cosp = 1 - 2 * (Q.y * Q.y + Q.z * Q.z);
		Angle.Yaw = atan2(siny_cosp, cosy_cosp);
		theta = Angle.Yaw; // radians]

		if ((distance <= 0.35 && distance > 0.02) && ball_aready == 0
				&& state == 5) {
			TIM1->CCR4 = 0;
			TIM1->CCR3 = 1000;
		} else if ((ball_aready == 1 || state > 5 || (distance > 0.35 && state == 5))) {
			TIM1->CCR4 = 600;
			TIM1->CCR3 = 0;
		} else {
			TIM1->CCR4 = 0;
			TIM1->CCR3 = 0;
		}

		jj++;

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
