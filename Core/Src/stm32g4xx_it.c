/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */
// Wrap value to range.
// With default rounding mode (round to nearest),
// the result will be in range -y/2 to y/2
inline float wrap_pm(float x, float y) {
#ifdef FPU_FPV4
    float intval = (float)round_int(x / y);
#else
    float intval = nearbyintf(x / y);
#endif
    return x - intval * y;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern UART_HandleTypeDef huart4;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_TIM16_IRQn 1 */

  /* USER CODE END TIM1_UP_TIM16_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt / UART4 wake-up interrupt through EXTI line 34.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
   if(__HAL_UART_GET_FLAG(&huart4,UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(&huart4,UART_IT_IDLE)) {
      __HAL_UART_CLEAR_FLAG(&huart4,UART_FLAG_IDLE);
      __HAL_UART_CLEAR_FLAG(&huart4,USART_ISR_CTSIF);

      HAL_UART_DMAStop(&huart4);
      if (tamaga_encoder_data[0] == 0x02) {
          encoder_abs = (((uint32_t)tamaga_encoder_data[4] << 16) | 
                                ((uint32_t)tamaga_encoder_data[3] << 8) | 
                                ((uint32_t)tamaga_encoder_data[2]));
          encoder_elec_angle = wrap_pm((float)encoder_abs / 8388607.0f * 360.0f * motor_gear_ratio * motor_pole_pairs, 360.0f);
          encoder_rotor_pos = wrap_pm((float)encoder_abs / 8388607.0f * 360.0f * motor_gear_ratio, 360.0f);
          encoder_output_pos = wrap_pm((float)encoder_abs / 8388607.0f * 360.0f, 360.0f);
      }
      HAL_UART_Receive_DMA(&huart4, tamaga_encoder_data, sizeof(tamaga_encoder_data));
  }
  /* USER CODE END UART4_IRQn 0 */
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM1) {
        // 定时器1更新回调，串口请求多摩川编码器数据
        HAL_UART_Transmit_DMA(&huart4, (uint8_t[]){0x02}, 1);
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1)
    {
        const uint16_t sin_cnt = ADC_Value[1];
        const uint16_t cos_cnt = ADC_Value[0];
         // 归一化
        float mod_sin = (sin_cnt - s_offset_) * s_gain_;
        float mod_cos = (cos_cnt - c_offset_) * c_gain_;
        // 滤波
        static float sin_filter = 0.0f;
        static float cos_filter = 0.0f;
        UTILS_LP_FAST(sin_filter, mod_sin, 0.9f);
        UTILS_LP_FAST(cos_filter, mod_cos, 0.9f);
        // 相位差补偿，保证相位差90度，只改cos相位，
        float sin_corrected = sin_filter;
        float cos_corrected = (cos_filter + sin_filter);
        // 反正切
        sincos_angle = utils_fast_atan2(sin_corrected, cos_corrected);
        // 连续化多圈
        static float last_angle = 0.0f;
        static bool initialized = false;
        if (!initialized) {
            last_angle = sincos_angle;  // 第一次运行时设置值
            initialized = true;
        }
        // 增量累加，单位：电角度rad
        float diff = utils_angle_difference_rad(sincos_angle, last_angle);
        last_angle = sincos_angle;
        serial_angle += diff;
        
        motor_elec_angle = wrap_pm(sincos_angle / (float)(2.0f * M_PI) * 360.0f, 360.0f);
        motor_rotor_pos = wrap_pm(serial_angle / (float)(2.0f * M_PI * motor_pole_pairs) * 360.0f, 360.0f);
        motor_output_pos = wrap_pm(serial_angle / (float)(2.0f * M_PI * motor_pole_pairs * motor_gear_ratio) * 360.0f, 360.0f);
    }
}
/* USER CODE END 1 */
