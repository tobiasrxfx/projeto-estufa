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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DHT.h"

#include "stdio.h"
#include "string.h"
#include "TSL2561.h"
#define TSL2561_ADDR 0x39

#include <ssd1306.h>
#include <ssd1306_fonts.h>

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
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
TSL2561 tslSensor;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


DHT_DataTypedef DHT11_Data;
float Temperature, Humidity;
uint8_t flag_janela = 0;
int8_t flag_telas = 0;
int8_t flag_rele = 0;
int8_t flag_rele_ativar = 0;

/* Callback para a interrupção do temporizador TIM7 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	static int posicao_janela = 1000;

	if (htim->Instance == TIM7)
	{
		uint16_t size;
		uint8_t Data[256];

		/*SENSOR DE LUMINOSIDADE BEGIN*/
		float luxValue;

		/*Testa se o Sensor de Luminosidade estar disponível*/
		HAL_I2C_IsDeviceReady(&hi2c1, TSL2561_ADDR, 5, HAL_MAX_DELAY);
		/* Recebe o valor medido em Lux  */
		if (TSL2561_GetLux(&tslSensor, &luxValue) == HAL_OK)
		{
			/* Mostra o valor se válido */
			size = sprintf((char *)Data, "Light: %u lux\n", (uint16_t)luxValue);
			HAL_UART_Transmit(&huart2, (uint8_t*)Data, size, HAL_MAX_DELAY);
		}

		/*SENSOR DE LUMINOSIDADE END*/

		/*CONTROLE DE JANELA BEGIN*/

		if(!flag_janela) /*Testa se está em modo de controle automático*/
		{
			if(luxValue<100) /*janela aberta 90°*/
			{
				for(;posicao_janela<2000;posicao_janela++)
				{
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, posicao_janela);
					for(int i = 0; i <8000;i++);
				}
			}else if(luxValue<300) /*janela para 45°*/
			{
				if(posicao_janela < 1500)
				{
					for(;posicao_janela<1500;posicao_janela++)
					{
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, posicao_janela);
						for(int i = 0; i <8000;i++);
					}
				}
				else
				{
					for(;posicao_janela>1500;posicao_janela--)
					{
						__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, posicao_janela);
						for(int i = 0; i <8000;i++);
					}
				}
			}else /*janela fechada*/
			{
				for(;posicao_janela>1000;posicao_janela--)
				{
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, posicao_janela);
					for(int i = 0; i <8000;i++);
				}
			}
		}

		/*CONTROLE DE JANELA END*/

		/*SENSOR DE TEMPERATURA E UMIDADE BEGIN*/

//	    DHT_GetData(&DHT11_Data);
//	    Temperature = (float) DHT11_Data.Temperature /10.0;
//	    Humidity = (float) DHT11_Data.Humidity /10.0;
//
//		/* Mostra o valor se válido */
//		size = sprintf((char *)Data, "Temp: %0.2f -- Umi: %0.2f\n", Temperature, Humidity);
//		HAL_UART_Transmit(&huart2, (uint8_t*)Data, size, HAL_MAX_DELAY);

		/*SENSOR DE TEMPERATURA E UMIDADE END*/
	}
	if (htim->Instance == TIM14)
	{
		/*CONTROLE MANUAL DA JANELA START*/
		if(flag_janela)
		{
			uint8_t val_adc;
			HAL_ADC_Start(&hadc1); // start the adc
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			val_adc = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1); // stop adc

			/*calcula a posição atual pelo potenciometro*/
			posicao_janela = 2000 - val_adc*1000/256;
			/*atualiza a posição da janela*/
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, posicao_janela);
		}
		/*CONTROLE MANUAL DA JANELA END*/
	}

}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//
//	/*Botão para controle manual dos motores de irrigação*/
//	if (GPIO_Pin == Button_Irrigacao_Pin) {
//		/*Inverte o estado do relé*/
//		HAL_GPIO_TogglePin(Motor_Irrigacao_GPIO_Port, Motor_Irrigacao_Pin);
//		HAL_GPIO_TogglePin(Motor_Irrigacao_2_GPIO_Port, Motor_Irrigacao_2_Pin);
//	}
//	/*Botão para ativar ou desativar a o controle manual da janela*/
//	if (GPIO_Pin == Button_Janela_Pin) {
//		uint8_t val_adc;
//		HAL_ADC_Start(&hadc1); // start the adc
//		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//		val_adc = HAL_ADC_GetValue(&hadc1);
//		HAL_ADC_Stop(&hadc1); // stop adc
//
//		if(!flag_janela)
//		{
//			if(val_adc < 15) flag_janela = !flag_janela;
//		}else flag_janela = !flag_janela;
//	}
//}


void tela_controle_janela()
{

	ssd1306_Fill(Black);

	ssd1306_SetCursor(35, 5);

	ssd1306_WriteString("AutoEstufa", Font_6x8, White);

	ssd1306_DrawRectangle(0, 0, 127, 15, White);

	ssd1306_DrawRectangle(0, 17, 127, 63, White);

	ssd1306_SetCursor(10, 19);

	ssd1306_WriteString("Controle da janela ", Font_6x8, White);

	ssd1306_Line(67, 60, 96, 60, White);
	ssd1306_Line(67, 59, 96, 59, White);

	ssd1306_UpdateScreen();
}



void tela_controle_irrigacao()
{

	ssd1306_Fill(Black);

	ssd1306_SetCursor(35, 5);

	ssd1306_WriteString("AutoEstufa", Font_6x8, White);

	ssd1306_DrawRectangle(0, 0, 127, 15, White);

	ssd1306_DrawRectangle(0, 17, 127, 63, White);

	ssd1306_SetCursor(10, 19);

	ssd1306_WriteString("Controle da irrig ", Font_6x8, White);

	ssd1306_Line(99, 60, 123, 60, White);
	ssd1306_Line(99, 59, 123, 59, White);

	ssd1306_UpdateScreen();
}

void tela_dados()
{

	ssd1306_Fill(Black);

	ssd1306_SetCursor(35, 5);

	ssd1306_WriteString("AutoEstufa", Font_6x8, White);

	ssd1306_DrawRectangle(0, 0, 127, 15, White);

	ssd1306_DrawRectangle(0, 17, 127, 63, White);

	ssd1306_SetCursor(4, 19);

	ssd1306_WriteString("Temp.: 25 *C", Font_6x8, White);

	ssd1306_SetCursor(4, 29);

	ssd1306_WriteString("Umi. : 63%", Font_6x8, White);

	ssd1306_SetCursor(4, 39);

	ssd1306_WriteString("Lum. : 200lux", Font_6x8, White);

	ssd1306_Line(35, 60, 64, 60, White);
	ssd1306_Line(35, 59, 64, 59, White);

	ssd1306_UpdateScreen();
}

void tela_menu()
{

	ssd1306_Fill(Black);

	ssd1306_SetCursor(35, 5);

	ssd1306_WriteString("AutoEstufa", Font_6x8, White);

	ssd1306_DrawRectangle(0, 0, 127, 15, White);

	ssd1306_DrawRectangle(0, 17, 127, 63, White);

	ssd1306_SetCursor(35, 19);

	ssd1306_WriteString("Bem vindo!", Font_6x8, White);

	ssd1306_SetCursor(10, 29);

	ssd1306_WriteString("Pressione qualquer", Font_6x8, White);

	ssd1306_SetCursor(45, 39);

	ssd1306_WriteString("botao!", Font_6x8, White);

	ssd1306_Line(3, 60, 32, 60, White);
	ssd1306_Line(3, 59, 32, 59, White);

	ssd1306_UpdateScreen();
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {


	/*Botão para controle manual dos motores de irrigação*/
	if (GPIO_Pin == Button_Irrigacao_Pin) {
		flag_telas++;
		if (flag_telas >= 4){
			flag_telas = 0; // Tela Inicial
		}
	}

	/*Botão para ativar ou desativar a o controle manual da janela*/
	if (GPIO_Pin == Button_Janela_Pin) {
		flag_telas--;
		if (flag_telas < 0){
			flag_telas = 3; // Ultima tela
		}
	}

	if (GPIO_Pin == Botao_Placa_Pin  && flag_rele == 1) {
		if (flag_rele_ativar == 0){
			flag_rele_ativar = 1;
		} else {
			flag_rele_ativar = 0;
		}
	}

	switch (flag_telas) {
		case 0:
			tela_menu();
			break;
		case 1:
			tela_dados();
			break;
		case 2:
			tela_controle_janela();

			// Controle manual do motor
			uint8_t val_adc;
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			val_adc = HAL_ADC_GetValue(&hadc1);
			HAL_ADC_Stop(&hadc1);

			if(!flag_janela)
			{
				if(val_adc < 15) flag_janela = !flag_janela;
			}else flag_janela = !flag_janela;

			break;
		case 3:
			tela_controle_irrigacao();
			flag_rele = 1;
			if (flag_rele_ativar == 1){
				HAL_GPIO_TogglePin(Motor_Irrigacao_GPIO_Port, Motor_Irrigacao_Pin);
				HAL_GPIO_TogglePin(Motor_Irrigacao_2_GPIO_Port, Motor_Irrigacao_2_Pin);
			}

		default:
			break;
	}

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM7_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  ssd1306_Init();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  HAL_ADC_Start(&hadc1);

  TSL2561_Init(&tslSensor, &hi2c1, TSL2561_ADDR);

  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim14);


  tela_menu();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//HAL_Delay(5000);
	//tela_dados();

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
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
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 42000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 8000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 84-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Motor_Irrigacao_Pin|Motor_Irrigacao_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DHT11_GPIO_Port, DHT11_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Botao_Placa_Pin */
  GPIO_InitStruct.Pin = Botao_Placa_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Botao_Placa_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_Janela_Pin Button_Irrigacao_Pin */
  GPIO_InitStruct.Pin = Button_Janela_Pin|Button_Irrigacao_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Motor_Irrigacao_Pin Motor_Irrigacao_2_Pin */
  GPIO_InitStruct.Pin = Motor_Irrigacao_Pin|Motor_Irrigacao_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT11_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(DHT11_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
