/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	double dist;
	double ang;
	double degA[45];
	double degB[45];
	double degC[45];
	double degD[45];
	double degE[45];
	double degF[45];
	double degG[45];
	double degH[45];
}lidar_new_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LIDAR_AVG_OFFSET 0
#define RECEIVER_SIZE 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//--------- LIDAR VARIABEL ---------//
uint8_t buf_rec[30];
uint8_t buf_descriptor[100];
float distance_f = 0.0;
float angle_f = 0.0;
uint8_t start_scan_flag = 0;
uint8_t quality = 0;
double degA[45];
double degB[45];
double degC[45];
double degD[45];
double degE[45];
double degF[45];
double degG[45];
double degH[45];
double AVG[8];
long sumDegA = 0, sumDegB = 0, sumDegC = 0, sumDegD = 0, sumDegE = 0, sumDegF = 0, sumDegG=0, sumDegH=0;
int countA = 0, countB=0, countC=0, countD=0, countE=0, countF=0, countG=0, countH=0;

uint8_t is_match = 0x00;
uint8_t is_done = 0x01;
lidar_new_t lidar;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	
	if(is_match >= 0x01){
			//-------------------- STARTING POINT -------------//
					if((buf_rec[1]&0x01)&((buf_rec[0]^(buf_rec[0]>>1))&0x01)){
						
						
						//-------------- READ PROPERTY --------------//
						angle_f = ((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0;
						distance_f = ((buf_rec[3] | (buf_rec[4]<<8))/4.0)/10;
						quality = buf_rec[0]>>2;
						start_scan_flag = buf_rec[0] & 0x1;
						
						// Read 45 deg data
						// memperoleh seperempat data sudut 45
						if((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) >= (0) && ((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) < (45) )){
							degA[(int)floor((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0))] = ((buf_rec[3] | (buf_rec[4]<<8))/4.0)/10;
						}
						// memperoleh seperempat data sudut 90
						else if((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) >= (45) && ((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) < (90) )){
							degB[(int)floor((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0)-(45))] = ((buf_rec[3] | (buf_rec[4]<<8))/4.0)/10;
						}
						// memperoleh seperempat data sudut 135
						else if((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) >= (90) && ((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) < (135) )){
							degC[(int)floor((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0)-(90))] = ((buf_rec[3] | (buf_rec[4]<<8))/4.0)/10;
						}
						// memperoleh seperempat data sudut 180
						else if((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) >= (135) && ((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) < (180) )){
							degD[(int)floor((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0)-(135))] = ((buf_rec[3] | (buf_rec[4]<<8))/4.0)/10;
						}
						// memperoleh seperempat data sudut 225
						else if((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) >= (180) && ((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) < (225) )){
							degE[(int)floor((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0)-(180))] = ((buf_rec[3] | (buf_rec[4]<<8))/4.0)/10;
						}
						// memperoleh seperempat data sudut 270
						else if((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) >= (225) && ((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) < (270) )){
							degF[(int)floor((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0)-(225))] = ((buf_rec[3] | (buf_rec[4]<<8))/4.0)/10;
						}
						// memperoleh seperempat data sudut 315
						else if((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) >= (270) && ((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) < (315) )){
							degG[(int)floor((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0)-(270))] = ((buf_rec[3] | (buf_rec[4]<<8))/4.0)/10;
						}
						// memperoleh seperempat data sudut 360
						else if((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) >= (315) && ((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0) < (360) )){
							degH[(int)floor((((buf_rec[1] >> 1) | (buf_rec[2] << 7))/64.0)-(315))] = ((buf_rec[3] | (buf_rec[4]<<8))/4.0)/10;
						}

						// Start to read AVG
						is_match = 0x03;
				}
		}
	
	
		//---------------------- CHECK HEADER MESSAGE ------------------------//
		if(is_match == 0x00){
			for(int l = 0; l < 7; l++){
				
				if(buf_rec[l+0]==0xa5 && buf_rec[l+1]==0x5a && buf_rec[l+2]==0x05 && buf_rec[l+3]==0x00 && buf_rec[l+4]==0x00 && buf_rec[l+5]==0x40 && buf_rec[l+6]==0x81){
					for(uint8_t i =0; i < 7; i++){
						buf_descriptor[i] = buf_rec[l+i];
					}
				
					is_match = 0x01;
				}
			}
		}
		//---------------- GET AVERAGE DATA ----------------/

		
		//---------------- RECEIVE CALLBACK DMA ----------------/
		HAL_UART_Receive_DMA(&huart1, buf_rec, RECEIVER_SIZE);
}

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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	//========================= START LIDAR CONFIG ========================================//
	// Motor Start
	TIM1->CCR1 = (100*2000/100);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	uint8_t rplidar_stop_msg[] = {0xa5,0x25};
	
	// Stop Command
	HAL_UART_Transmit(&huart1, rplidar_stop_msg, 2, 100);
	HAL_Delay(20);
	HAL_Delay(1000);
	
	// Start Scan
	uint8_t rplidar_scan_msg[] = {0xA5, 0x20};
	HAL_UART_Transmit(&huart1, rplidar_scan_msg, 2,100);
	HAL_UART_Receive_DMA(&huart1, buf_rec, 7);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		sumDegA = 0, sumDegB = 0, sumDegC = 0, sumDegD = 0, sumDegE = 0, sumDegF = 0, sumDegG=0, sumDegH=0;
		countA = 0, countB=0, countC=0, countD=0, countE=0, countF=0, countG=0, countH=0;
			for(int i =0; i < 45; i++){
						if(degA[i] > 0.1){
							sumDegA += degA[i];
							countA++;
						}
						if(degB[i] > 0.1){
							sumDegB += degB[i];
							countB++;
						}
						if(degC[i] > 0.1){
							sumDegC += degC[i];
							countC++;
						}
						if(degD[i] > 0.1){
							sumDegD += degD[i];
							countD++;
						}
						if(degE[i] > 0.1){
							sumDegE += degE[i];
							countE++;
						}
						if(degF[i] > 0.1){
							sumDegF += degF[i];
							countF++;
						}
						if(degG[i] > 0.1){
							sumDegG += degG[i];
							countG++;
						}
						if(degH[i] > 0.1){
							sumDegH += degH[i];
							countH++;
						}
			}
			
			AVG[0] = sumDegA/countA;
			AVG[1] = sumDegB/countB;
			AVG[2] = sumDegC/countC;
			AVG[3] = sumDegD/countD;
			AVG[4] = sumDegE/countE;
			AVG[5] = sumDegF/countF;
			AVG[6] = sumDegG/countG;
			AVG[7] = sumDegH/countH;
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

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1818-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
