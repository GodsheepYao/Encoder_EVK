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
float tamagawa_elec_rad = 0;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern TIM_HandleTypeDef htim1;
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
  * @brief This function handles DMA1 channel4 global interrupt.
  */
void DMA1_Channel4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */

  /* USER CODE END DMA1_Channel4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_spi1_rx);
  /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */

  /* USER CODE END DMA1_Channel4_IRQn 1 */
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
        __HAL_UART_CLEAR_IDLEFLAG(&huart4);
        HAL_UART_DMAStop(&huart4);
        if (tamaga_encoder_data[0] == 0x02) {
            // 多摩川原始值
            tamagawa_abs = (((uint32_t)tamaga_encoder_data[4] << 16) | 
                                ((uint32_t)tamaga_encoder_data[3] << 8) | 
                                ((uint32_t)tamaga_encoder_data[2]));
            static float last_angle = 0.0f;
            static bool initialized = false;
            if (!initialized) {
                last_angle = tamagawa_abs;
                initialized = true;
            }
                    // 连续化多圈，增量累加
            float diff = utils_angle_difference_custom(tamagawa_abs, last_angle, MAIN_ENCODER_CPR);
            last_angle = tamagawa_abs;
            tamagawa_serial_abs += diff;
            // 编码器端输出，单位：度
            tamagawa_elec_angle = wrap_pm((float)tamagawa_serial_abs / 8388608.0f * 360.0f * motor_gear_ratio * motor_pole_pairs, 360.0f);
            tamagawa_rotor_pos = wrap_pm((float)tamagawa_serial_abs / 8388608.0f * 360.0f * motor_gear_ratio, 360.0f);
            tamagawa_output_pos = wrap_pm((float)tamagawa_abs / 8388608.0f * 360.0f, 360.0f);
            
            tamagawa_elec_rad = wrap_pm((float)tamagawa_serial_abs / 8388608.0f * 2.0f * M_PI * motor_gear_ratio * motor_pole_pairs, 2.0f * M_PI);
//            for(uint8_t i = 0; i < 100; i++) {
//                HAL_GPIO_WritePin(magnetic_spi_GPIO_Port, magnetic_spi_Pin, GPIO_PIN_SET);
//            }
//            HAL_GPIO_WritePin(magnetic_spi_GPIO_Port, magnetic_spi_Pin, GPIO_PIN_RESET);
            // 多摩川和正余弦的差值
            tamagawa_sincos_diff = utils_angle_difference(tamagawa_output_pos, sincos_output_pos);
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
        // 正余弦上电偏置
        static float offset = 0.0f;
        // 反正切
        sincos_angle = utils_fast_atan2(sin_filter, cos_filter);
        static float last_angle = 0.0f;
        static bool initialized = false;
        if (!initialized) {
            offset = sincos_angle;
            last_angle = sincos_angle;
            initialized = true;
        }
        sincos_angle -= offset; 
        // 连续化多圈，增量累加，单位：电角度rad
        float diff = utils_angle_difference_rad(sincos_angle, last_angle);
        last_angle = sincos_angle;
        serial_angle += diff;
        // 正余弦输出，单位：度
        sincos_elec_angle = wrap_pm(sincos_angle / (float)(2.0f * M_PI) * 360.0f, 360.0f);
        sincos_rotor_pos = wrap_pm(serial_angle / (float)(2.0f * M_PI * motor_pole_pairs) * 360.0f, 360.0f);
        sincos_output_pos = wrap_pm(serial_angle / (float)(2.0f * M_PI * motor_pole_pairs * motor_gear_ratio) * 360.0f, 360.0f);
    }
}

float filter_diff = 0;
float main_gear_map = 0, sec_gear_map = 0;
float pos_combined = 0, encoder_gear = 0;
float encoder_gear_diff = 0;
float magnetic_elec_sin = 0;
float magnetic_elec_cos = 0;
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1) {
        // 主磁编原始值
        main_magnetic_abs = (INVERT_MAIN_DIRECTION) ? MAIN_ENCODER_CPR - spi1_rx_buffer : spi1_rx_buffer;
        // 主磁编上电偏置
        static float offset = 0.0f;
        static float last_angle = 0.0f;
        static bool initialized = false;
        if (!initialized) {
            offset = main_magnetic_abs;
            last_angle = main_magnetic_abs;
            initialized = true;
        }
        main_magnetic_abs -= offset;
        // 连续化多圈，增量累加
        float diff = utils_angle_difference_custom(main_magnetic_abs, last_angle, MAIN_ENCODER_CPR);
        last_angle = main_magnetic_abs;
        magnetic_serial_abs += diff;
        // 主磁编输出，单位：度
        magnetic_elec_angle = wrap_pm((float)(magnetic_serial_abs) / MAIN_ENCODER_CPR * motor_pole_pairs * 360.0f, 360.0f);
        magnetic_rotor_pos = wrap_pm((float)(main_magnetic_abs) / MAIN_ENCODER_CPR * 360.0f, 360.0f);
        magnetic_output_pos = wrap_pm((float)(magnetic_serial_abs) / (MAIN_ENCODER_CPR * motor_gear_ratio) * 360.0f, 360.0f);
        float magnetic_elec_rad = wrap_pm((float)(magnetic_serial_abs) / MAIN_ENCODER_CPR * motor_pole_pairs * 2.0f * M_PI, 2.0f * M_PI);
        magnetic_elec_sin = utils_fast_sin(magnetic_elec_rad);
        magnetic_elec_cos = utils_fast_cos(magnetic_elec_rad);
        // for(uint8_t i = 0; i < 100; i++) {
        //     HAL_GPIO_WritePin(magnetic_spi_GPIO_Port, magnetic_spi_Pin, GPIO_PIN_SET);
        // }
        // HAL_GPIO_WritePin(magnetic_spi_GPIO_Port, magnetic_spi_Pin, GPIO_PIN_RESET);
        
        // 和主磁编的差值
        tamagawa_magnetic_diff = utils_angle_difference(tamagawa_rotor_pos, magnetic_rotor_pos);
        sincos_magnetic_diff = utils_angle_difference(sincos_output_pos, magnetic_output_pos);
        // 重复定位精度测试使用，使用低通滤波数值做记录
        UTILS_LP_FAST(filter_diff, tamagawa_magnetic_diff, 0.005f);
        
        // 副编码器原始值
        sec_magnetic_abs = (INVERT_SEC_DIRECTION) ? SEC_ENCODER_CPR - spi2_rx_buffer : spi2_rx_buffer;
        // 主副编码器原始值映射齿数（0~65536映射到0~齿数）
        main_gear_map = utils_map(main_magnetic_abs, 0, MAIN_ENCODER_CPR, 0, MAIN_GEAR_RATIO);
        sec_gear_map = utils_map(sec_magnetic_abs, 0, SEC_ENCODER_CPR, 0, SEC_GEAR_RATIO);
        // 主副齿差值计算
        float sec_main_diff = sec_gear_map - main_gear_map;
        // 主副齿差值过圈修正
        if (sec_main_diff < 0) {
            sec_main_diff += SEC_GEAR_RATIO;
        }
        // 两齿合成角度互质的圈数=（(b-a）modB）+a/A，其中a,b是角度映射的齿数，A，B为最大齿数
        float pos_combined_origin = fmod(sec_main_diff, SEC_GEAR_RATIO) + main_gear_map / MAIN_GEAR_RATIO;
        static float offset_ = 0.0f;
        static bool initialized_ = false;
        if (!initialized_) {
            offset_ = pos_combined_origin;
            initialized_ = true;
        }
        // 上电置零
        pos_combined_origin -= offset_;
        pos_combined = wrap_pm(pos_combined_origin, SEC_GEAR_RATIO);

        encoder_gear = wrap_pm((float)tamagawa_abs / 8388607.0f * SEC_GEAR_RATIO, SEC_GEAR_RATIO);
        // 两齿合成角度和多摩川编码器差值，单位：输出轴度
        encoder_gear_diff = (pos_combined - encoder_gear) / SEC_GEAR_RATIO * 360.0f;
        if (fabs(encoder_gear_diff) > (SEC_GEAR_RATIO / 2.0f)) {
            encoder_gear_diff -= copysign(SEC_GEAR_RATIO, encoder_gear_diff);
        }
    }
}
/* USER CODE END 1 */
