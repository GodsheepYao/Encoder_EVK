/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 多摩川编码器原始数据接收数组
uint8_t tamaga_encoder_data[6] = {0};
// ADC采样值
__ALIGN_BEGIN uint16_t ADC_Value[14] __ALIGN_END;
// 多摩川编码器原始值
uint32_t encoder_abs = 0.0f;
// 多摩川编码器电角度，转子侧位置，输出轴位置数据，单位：度
float encoder_elec_angle = 0.0f, encoder_rotor_pos = 0.0f, encoder_output_pos = 0.0f;
// 正余弦角度，单位：rad
float sincos_angle = 0.0f, serial_angle = 0.0f;
// 电机电角度，转子侧位置，输出轴位置数据，单位：度
float motor_elec_angle = 0.0f, motor_rotor_pos = 0.0f, motor_output_pos = 0.0f;
/*********************** 以下需要根据电机实际情况配置 ***********************/
// 电机极对数
float motor_pole_pairs = 21.0f;
// 电机减速比
float motor_gear_ratio = 12.0f;
// 最大值宏定义在main.h
float s_gain_ = 2.0f / (float)(SIN_MAX_VALUE - SIN_MIN_VALUE);
float s_offset_ = (float)(SIN_MAX_VALUE + SIN_MIN_VALUE) / 2.0f;
float c_gain_ = 2.0f / (float)(COS_MAX_VALUE - COS_MIN_VALUE);
float c_offset_ = (float)(COS_MAX_VALUE + COS_MIN_VALUE) / 2.0f;
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
  MX_UART4_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // 启动串口DMA中断接收多摩川编码器
  HAL_RS485Ex_Init(&huart4, UART_DE_POLARITY_HIGH, 0, 0);
  HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
  HAL_NVIC_DisableIRQ(DMA1_Channel2_IRQn);
  HAL_UART_Receive_DMA(&huart4, tamaga_encoder_data, sizeof(tamaga_encoder_data));
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
  
  // 多摩川编码器设零
  for (uint8_t i = 0; i < 15; i++) {
     HAL_UART_Transmit_DMA(&huart4, (uint8_t[]){0xC2}, 1);
     HAL_Delay(1);
  }
  
  // 多通道同步采集线性霍尔数据
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
  HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t*)ADC_Value, 1);
    
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);	
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV6;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
