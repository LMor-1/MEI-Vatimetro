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
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SAMPLES_AMOUNT 	500
#define VOLTAGE_SAMPLES 250
#define CURRENT_SAMPLES 250
#define VOLTAGE_COMP	6.432
#define	CURRENT_COMP	15185.18
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static uint32_t	DMA_samples_buffer [SAMPLES_AMOUNT] = {0};

static uint32_t	instant_power [VOLTAGE_SAMPLES] = {0.0};

typedef enum{
	SPLIT_SAMPLES,
    ADECUATE_SAMPLES,
    CALCULATE,
    MOVING_AVERAGE
}filterState;

typedef struct{
  uint32_t voltage_samples [VOLTAGE_SAMPLES];      //PA3
  double	adequate_voltage_samples [VOLTAGE_SAMPLES];
  uint32_t	average_val_voltage;
  double	effective_voltage;
}VoltageParams;

typedef struct{
  uint32_t current_samples [CURRENT_SAMPLES];        //PA5
  double adequate_current_samples [CURRENT_SAMPLES];
  uint32_t	average_val_current;
  double	effective_current;
}CurrentParams;

typedef struct{
  uint32_t active_power;
  double	apparent_power;
  double	reactive_power;
  float  power_factor;
}PowerParams;

VoltageParams voltage_params = {0};
CurrentParams current_params = {0};
PowerParams power_data_acquired = {0};

volatile bool conversion_is_complete = false;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc); //Este es el callback para la rutina de servicio de la interrupción
														                            //Dentro de HAL_ADC_IRQHandler

uint32_t getAverage (uint32_t* samples_buff);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t getAverage (uint32_t* samples_buff){

	static uint32_t avg_value = 0;

	for (uint32_t i=0; i<VOLTAGE_SAMPLES; i++){
		avg_value += samples_buff[i];
	}
	return (uint32_t)(avg_value/VOLTAGE_SAMPLES);

}
double getRMS (double* samples_buff){

	static uint32_t aux_value = 0;
	static double	rms_value = 0.0;

	for(uint32_t i=0; i<VOLTAGE_SAMPLES; i++){
		aux_value += samples_buff[i]*samples_buff[i];
	}
	rms_value = sqrt(aux_value/VOLTAGE_SAMPLES);

	return rms_value;
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
  SystemInit();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)DMA_samples_buffer, SAMPLES_AMOUNT);
  HAL_TIM_Base_Start_IT(&htim3);
  filterState = SPLIT_SAMPLES;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	if(conversion_is_complete){
		switch (filterState){

			case SPLIT_SAMPLES:
				for(uint16_t i=0; i<SAMPLES_AMOUNT; i++){
					uint16_t k=i/2;
					((i%2)==0)? (voltage_params.voltage_samples[k]=DMA_samples_buffer[i]) : (current_params.current_samples[k]=DMA_samples_buffer[i]);
				}
	
				filterState = ADECUATE_SAMPLES;
				break;
	
			case ADECUATE_SAMPLES:
				for(uint16_t i=0; i<VOLTAGE_SAMPLES; i++){
					voltage_params.adequate_voltage_samples[i]=(voltage_params.voltage_samples[i]-voltage_params.average_val_voltage)*VOLTAGE_COMP;
					current_params.adequate_current_samples[i]=(current_params.current_samples[i]-current_params.average_val_current)*CURRENT_COMP;
				}
	
				filterState = CALCULATE;
				break;	
	
			case CALCULATE:
				//S: apparent power
				//Q: reactive power
				//P: active power
				//FDP: arc cosine of P/S
	
				voltage_params.average_val_voltage = getAverage(voltage_params.voltage_samples);
				current_params.average_val_current = getAverage(current_params.current_samples);
	
				for(uint16_t i=0; i<VOLTAGE_SAMPLES; i++)
					instant_power[i] = voltage_params.adequate_voltage_samples[i]*current_params.adequate_current_samples[i];
	
				power_data_acquired.active_power = getAverage(instant_power);	//V*I*cos(phi)
	
				voltage_params.effective_voltage = getRMS (voltage_params.adequate_voltage_samples);
				current_params.effective_current = getRMS (current_params.adequate_current_samples);
				power_data_acquired.apparent_power = current_params.effective_current*voltage_params.effective_voltage;
	
				power_data_acquired.reactive_power = sqrt(power_data_acquired.apparent_power*power_data_acquired.apparent_power - power_data_acquired.active_power*power_data_acquired.active_power);
				
				power_data_acquired.power_factor = acosf(power_data_acquired.active_power/power_data_acquired.apparent_power);

				filterState = MOVING_AVERAGE;
				break;
	
			case MOVING_AVERAGE:
			
				filterState = SPLIT_SAMPLES;
				conversion_is_complete = false;
				HAL_TIM_Base_Start(&htim3);
				break;
		
	   		default:
				break;
   		}

	}
	
    
    
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3360;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : USER_Btn_Pin PC4 */
  GPIO_InitStruct.Pin = USER_Btn_Pin|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void deInitVariables (void){

	for(uint8_t i=0; i<VOLTAGE_SAMPLES; i++){
		voltage_params.voltage_samples[i] =0;
		current_params.current_samples [i]	= 0;
		voltage_params.adequate_voltage_samples [i] = 0;
		current_params.adequate_current_samples [i] = 0;
		instant_power [i] = 0;
	}
	voltage_params.average_val_voltage = 0;
	voltage_params.effective_voltage = 0;
	current_params.average_val_current = 0;
	current_params.effective_current = 0;
	power_data_acquired.active_power = 0;
	power_data_acquired.apparent_power = 0;
	power_data_acquired.reactive_power = 0;
	power_data_acquired.power_factor = 0;

	return;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){

	//HAL_ADC_MspDeInit(&hadc1);
	//Detener el timer3 que dispara al ADC para evitar el DeInit y luego el Init nuevamente
	HAL_TIM_Base_Stop(&htim3);  
	deInitVariables();
	conversion_is_complete = true;

	//for(uint16_t i=0; i<SAMPLES_AMOUNT; i++){
	//	uint16_t k=i/2;
	//	((i%2)==0)? (voltage_params.voltage_samples[k]=DMA_samples_buffer[i]) : (current_params.current_samples[k]=DMA_samples_buffer[i]);
	//}
	//voltage_params.average_val_voltage = getAverage(voltage_params.voltage_samples);
	//current_params.average_val_current = getAverage(current_params.current_samples);
	//
	//for(uint16_t i=0; i<VOLTAGE_SAMPLES; i++){
	//	voltage_params.adequate_voltage_samples[i]=(voltage_params.voltage_samples[i]-voltage_params.average_val_voltage)*VOLTAGE_COMP;
	//	current_params.adequate_current_samples[i]=(current_params.current_samples[i]-current_params.average_val_current)*CURRENT_COMP;
	//	instant_power[i] = voltage_params.adequate_voltage_samples[i]*current_params.adequate_current_samples[i];
	//}
	//
	//power_data_acquired.active_power = getAverage(instant_power);	//V*I*cos(phi)
	//
	//voltage_params.effective_voltage = getRMS (voltage_params.adequate_voltage_samples);
	//current_params.effective_current = getRMS (current_params.adequate_current_samples);
	//
	//power_data_acquired.apparent_power = current_params.effective_current*voltage_params.effective_voltage;
	//
	//power_data_acquired.reactive_power = sqrt(power_data_acquired.apparent_power*power_data_acquired.apparent_power - power_data_acquired.active_power*power_data_acquired.active_power);

	//HAL_TIM_Base_Start(&htim3);

	//MX_ADC1_Init();
	return;
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
